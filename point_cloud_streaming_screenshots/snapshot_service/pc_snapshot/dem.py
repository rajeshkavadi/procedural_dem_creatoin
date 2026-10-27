"""Incremental DEM accumulation + headless heatmap rendering.

What "Get DEM" returns
----------------------
The service merges **every** point-cloud message it receives into one fixed
elevation grid, in place (the grid is never rebuilt from historical points).
The grid therefore always holds "everything seen since the service started",
i.e. from the initial position of playback up to the instant the user clicks
**Get DEM**, which renders that accumulated grid to a top-down PNG heatmap.

Reuse of the existing incremental DEM
-------------------------------------
The grid/filter core is the *same* ROS-free code the `dem_mapping` ROS 2
package uses (`ws/src/dem_mapping/dem_mapping/{elevation_grid,ground_
filters}.py`) and is imported from the workspace at runtime — there is exactly
one implementation of the algorithm in this repository.

Coordinate frame
----------------
This stack does no scan registration, so points are accumulated in the raw
**sensor frame** (origin at the LiDAR, Z up). The shared filters are therefore
handed a context that describes the sensor's own pose:

    ctx = {"pose_xyz": (0, 0, 0), "pose_z": 0.0, "stamp": t}

so `radial_range` gates range around the sensor (and conveniently drops the
``(0, 0, 0)`` filler points the driver emits for no-return beams) and
`height_band` gates elevation around the sensor height.
"""

from __future__ import annotations

import logging
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from .config import Config

log = logging.getLogger(__name__)

__all__ = ["DemAccumulator", "DemCapture", "DemRenderError",
           "import_dem_mapping", "dem_mapping_path", "render_dem"]

# ws/src/dem_mapping (…/point_cloud_streaming_screenshots/snapshot_service/
# pc_snapshot/dem.py -> repo root three levels up).
DEFAULT_MAPPING_PATH = Path(__file__).resolve().parents[3] / "ws" / "src" / "dem_mapping"

_ElevationGrid = None
_build_filter = None
_dem_mapping_error: Optional[str] = None


def dem_mapping_path(explicit: str = "") -> list:
    """Candidate sys.path entries that may contain the `dem_mapping` package."""
    candidates = []
    for raw in (explicit, os.environ.get("PCSNAP_DEM_MAPPING_PATH", ""),
                str(DEFAULT_MAPPING_PATH)):
        if raw:
            p = Path(raw).expanduser()
            if p.is_dir():
                candidates.append(str(p))
    return candidates


def import_dem_mapping(explicit: str = "") -> bool:
    """Import the ROS-free incremental DEM core. Returns True on success.

    Failure is *not* fatal: the service starts without a DEM and reports
    `dem.enabled = false` in /api/health (Get DEM then returns 409 with the
    reason).
    """
    global _ElevationGrid, _build_filter, _dem_mapping_error
    if _ElevationGrid is not None and _build_filter is not None:
        return True
    last_error = "dem_mapping package not found"
    for path in dem_mapping_path(explicit):
        if path not in sys.path:
            sys.path.insert(0, path)
        try:
            from dem_mapping.elevation_grid import ElevationGrid
            from dem_mapping.ground_filters import build_filter
        except Exception as exc:            # import error inside the package
            last_error = f"{path}: {exc}"
            continue
        _ElevationGrid, _build_filter = ElevationGrid, build_filter
        _dem_mapping_error = None
        return True
    _dem_mapping_error = last_error
    log.error("DEM disabled: %s (looked in %s)", last_error,
              ", ".join(dem_mapping_path(explicit)) or "<no candidate>")
    return False


@dataclass(frozen=True)
class DemCapture:
    """One immutable snapshot of the accumulated grid (render input)."""

    elevation: np.ndarray   # (ny, nx) float32, NaN = NoData, read-only
    valid: np.ndarray       # (ny, nx) bool, read-only
    stats: dict             # grid statistics (cells, z range, coverage, ...)
    meta: dict              # accumulation window + filter settings

    @property
    def valid_cells(self) -> int:
        return int(self.stats.get("valid_cells", 0))


class DemRenderError(Exception):
    """The DEM could not be turned into an image (e.g. nothing accumulated)."""


class DemAccumulator:
    """Thread-safe incremental elevation grid fed by the subscriber thread.

    add()     -> subscriber side, called for every decoded message
    capture() -> request side, returns an immutable copy for rendering
    status()  -> cheap health block for /api/health
    """

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._lock = threading.Lock()
        self._grid = None
        self._filters = ()
        self._enabled = False
        self._reason: Optional[str] = None

        self._messages = 0
        self._messages_empty = 0
        self._points_in = 0
        self._points_used = 0
        self._first_stamp: Optional[float] = None
        self._last_stamp: Optional[float] = None
        self._first_walltime: Optional[float] = None
        self._last_walltime: Optional[float] = None
        self._last_error: Optional[str] = None
        self._status_cache: Optional[dict] = None

        if not cfg.dem_enabled:
            self._reason = "DEM accumulation disabled (--no-dem)"
            return
        if not import_dem_mapping(cfg.dem_mapping_path):
            self._reason = f"dem_mapping package unavailable: {_dem_mapping_error}"
            return
        try:
            self._build(cfg)
        except Exception as exc:
            self._reason = f"DEM init failed: {exc}"
            log.exception("DEM init failed")
            return
        self._enabled = True

    # ------------------------------------------------------------------
    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def reason(self) -> Optional[str]:
        return self._reason

    def _build(self, cfg: Config) -> None:
        extent = float(cfg.dem_extent)
        self._grid = _ElevationGrid(
            resolution=float(cfg.dem_resolution),
            size_x=extent, size_y=extent,
            origin_x=-extent / 2.0, origin_y=-extent / 2.0,
            aggregation=cfg.dem_aggregation)

        # Sensor-frame context: origin of every range/height measurement is
        # the LiDAR itself (see module docstring).
        self._ctx = {"pose_xyz": (0.0, 0.0, 0.0), "pose_z": 0.0}

        max_range = float(cfg.dem_clip_range_m) or float("inf")
        radial = _build_filter("radial_range",
                               min_range=float(cfg.dem_min_range_m),
                               max_range=max_range)
        ground = _build_filter(cfg.dem_ground_filter,
                               z_min=float(cfg.dem_z_min),
                               z_max=float(cfg.dem_z_max),
                               below=float(cfg.dem_band_below),
                               above=float(cfg.dem_band_above))
        subsample = _build_filter("max_points",
                                  max_points=int(cfg.dem_max_points_per_scan))
        # Colour map validated once so a typo fails at start-up, not per click.
        import matplotlib.cm as cm
        try:
            cm.get_cmap(cfg.dem_colormap)
        except Exception as exc:
            raise ValueError(f"unknown dem_colormap {cfg.dem_colormap!r}: {exc}")
        self._filters = (radial, ground, subsample)
        log.info("DEM: %dx%d cells @ %.2fm, agg=%s, filter=%s, clip=%s m",
                 self._grid.nx, self._grid.ny, self._grid.resolution,
                 cfg.dem_aggregation, cfg.dem_ground_filter,
                 cfg.dem_clip_range_m or "off")

    # ------------------------------------------------------- subscriber side
    def add(self, points: np.ndarray, ros_stamp: float = 0.0) -> bool:
        """Merge one decoded cloud into the grid. Never raises."""
        if not self._enabled:
            return False
        try:
            return self._add(points, ros_stamp)
        except Exception as exc:              # a bad message must not kill ROS
            with self._lock:
                if str(exc) != self._last_error:
                    self._last_error = str(exc)
                    log.warning("DEM accumulation failed: %s", exc)
                self._messages += 1
                self._messages_empty += 1
                self._status_cache = None
            return False

    def _add(self, points: np.ndarray, ros_stamp: float) -> bool:
        pts = np.asarray(points, dtype=np.float32)
        n_in = int(pts.shape[0])
        now = time.time()
        if n_in == 0:
            with self._lock:
                self._messages += 1
                self._messages_empty += 1
                self._bump_window(ros_stamp, now)
                self._status_cache = None
            return False

        # Filtering (and the GIL release inside NumPy) happens outside the
        # grid lock; only the in-place merge serialises with other updates.
        ctx = dict(self._ctx, stamp=ros_stamp)
        for filt in self._filters:
            pts = filt(pts, ctx)
            if pts.shape[0] == 0:
                break
        if pts.shape[0] == 0:
            # Every point was rejected by a filter: still a received scan, and
            # the offered points must show up in points_in.
            with self._lock:
                self._messages += 1
                self._messages_empty += 1
                self._points_in += n_in
                self._bump_window(ros_stamp, now)
                self._status_cache = None
            return False

        with self._lock:
            res = self._grid.update(pts[:, :2], pts[:, 2])
            self._messages += 1
            self._points_in += n_in
            self._points_used += int(res.get("accepted", 0))
            self._bump_window(ros_stamp, now)
            self._status_cache = None
        return True

    def _bump_window(self, ros_stamp: float, walltime: float) -> None:
        """Caller must hold the lock."""
        if self._first_stamp is None:
            self._first_stamp = float(ros_stamp)
            self._first_walltime = walltime
        if self._last_stamp is None or float(ros_stamp) >= self._last_stamp:
            self._last_stamp = float(ros_stamp)
        self._last_walltime = walltime

    # --------------------------------------------------------- request side
    def capture(self) -> Optional[DemCapture]:
        """Immutable view of the grid accumulated so far (None if disabled)."""
        if not self._enabled:
            return None
        with self._lock:
            state = self._grid.copy_state()
            meta = self._meta_locked()
        elev, valid, stats = self._grid.evaluate(state)
        elev.setflags(write=False)
        valid.setflags(write=False)
        meta.update(stats)
        return DemCapture(elevation=elev, valid=valid, stats=stats, meta=meta)

    def empty(self) -> bool:
        if not self._enabled:
            return False
        with self._lock:
            return self._grid.total_updates == 0

    def _meta_locked(self) -> dict:
        cfg = self.cfg
        g = self._grid
        return {
            "kind": "dem",
            "topic": cfg.topic,
            "frame": "sensor",
            "resolution_m": g.resolution,
            "grid_shape": [g.ny, g.nx],
            "extent_m": [g.nx * g.resolution, g.ny * g.resolution],
            "origin": [g.origin_x, g.origin_y],
            "aggregation": g.aggregation,
            "scans_accumulated": self._messages,
            "scans_empty": self._messages_empty,
            "points_in": self._points_in,
            "points_used": self._points_used,
            "first_scan_stamp": self._first_stamp,
            "last_scan_stamp": self._last_stamp,
            "window_start_walltime": self._first_walltime,
            "window_end_walltime": self._last_walltime,
            "clip_range_m": cfg.dem_clip_range_m,
            "min_range_m": cfg.dem_min_range_m,
            "ground_filter": cfg.dem_ground_filter,
            "image_size": [cfg.dem_image_width, cfg.dem_image_height],
            "colormap": cfg.dem_colormap,
            "last_error": self._last_error,
        }

    def status(self) -> dict:
        """Health block for /api/health (cheap: cached between updates)."""
        base = {"enabled": self._enabled, "reason": self._reason}
        if not self._enabled:
            return base
        with self._lock:
            if self._status_cache is None:
                meta = self._meta_locked()
                elev, valid, stats = self._grid.evaluate(self._grid.copy_state())
                self._status_cache = {
                    **base,
                    "topic": meta["topic"],
                    "resolution_m": meta["resolution_m"],
                    "grid_shape": meta["grid_shape"],
                    "origin": meta["origin"],
                    "aggregation": meta["aggregation"],
                    "scans_accumulated": meta["scans_accumulated"],
                    "scans_empty": meta["scans_empty"],
                    "points_in": meta["points_in"],
                    "points_used": meta["points_used"],
                    "valid_cells": stats["valid_cells"],
                    "coverage_m2": stats["coverage_m2"],
                    "z_min": stats["z_min"],
                    "z_max": stats["z_max"],
                    "observations": stats["observations"],
                    "first_scan_stamp": meta["first_scan_stamp"],
                    "last_scan_stamp": meta["last_scan_stamp"],
                    "last_error": meta["last_error"],
                }
            return dict(self._status_cache)


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
def _span(seconds: Optional[float]) -> str:
    if not seconds:
        return "0s"
    seconds = float(seconds)
    if seconds < 60:
        return f"{seconds:.0f}s"
    if seconds < 3600:
        return f"{int(seconds // 60)}m{int(seconds % 60):02d}s"
    return f"{int(seconds // 3600)}h{int((seconds % 3600) // 60):02d}m"


def render_dem(capture: DemCapture, cfg: Config, path: Path) -> float:
    """Render the accumulated grid to a PNG heatmap. Returns render seconds.

    Uses a standalone matplotlib `Figure` + Agg canvas (no pyplot state, no
    global backend switch), exactly like the Matplotlib fallback renderer, so
    it is safe from any worker thread.
    """
    t0 = time.time()
    valid = capture.valid
    if not valid.any():
        raise DemRenderError("the accumulated DEM contains no valid cells")

    try:
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.cm import get_cmap
        from matplotlib.figure import Figure
        from matplotlib.colors import Normalize
    except Exception as exc:
        raise DemRenderError(f"matplotlib unavailable: {exc}")

    stats = capture.stats
    elev = capture.elevation
    ny, nx = elev.shape
    res = float(stats["resolution_m"])
    ox, oy = (float(v) for v in stats["origin"])

    # Crop to the observed area (+ a small margin) so the map is not mostly
    # empty grid, and compute the matching world extent for the axes.
    rows = np.flatnonzero(valid.any(axis=1))
    cols = np.flatnonzero(valid.any(axis=0))
    pad = 2
    r0, r1 = max(0, rows[0] - pad), min(ny - 1, rows[-1] + pad)
    c0, c1 = max(0, cols[0] - pad), min(nx - 1, cols[-1] + pad)
    sub = elev[r0:r1 + 1, c0:c1 + 1]
    sub_valid = valid[r0:r1 + 1, c0:c1 + 1]
    x0, x1 = ox + c0 * res, ox + (c1 + 1) * res
    y0, y1 = oy + r0 * res, oy + (r1 + 1) * res

    masked = np.ma.masked_where(~sub_valid, sub)
    z_min, z_max = stats.get("z_min"), stats.get("z_max")
    if z_min is None or z_max is None or z_max <= z_min:
        z_min, z_max = float(np.nanmin(sub)), float(np.nanmax(sub))
        if not np.isfinite(z_min) or not np.isfinite(z_max) or z_max <= z_min:
            z_min, z_max = 0.0, 1.0

    w, h = int(cfg.dem_image_width), int(cfg.dem_image_height)
    bg = "#0e1117"
    fig = Figure(figsize=(w / 100.0, h / 100.0), dpi=100, facecolor=bg)
    FigureCanvasAgg(fig)
    # left/bottom/right leave room for tick labels; top holds the title.
    ax = fig.add_axes([0.075, 0.09, 0.815, 0.79])
    ax.set_facecolor("#0b0e14")
    for spine in ax.spines.values():
        spine.set_color("#263041")
    ax.tick_params(colors="#8b96a8", labelsize=8)

    try:
        cmap = get_cmap(cfg.dem_colormap).copy()
    except Exception:
        cmap = get_cmap(cfg.dem_colormap)
    cmap.set_bad("#0b0e14")                  # NoData cells stay panel-dark

    im = ax.imshow(masked, origin="lower", extent=[x0, x1, y0, y1],
                   aspect="equal", cmap=cmap,
                   norm=Normalize(vmin=z_min, vmax=z_max))
    ax.axhline(0.0, color="#8b96a8", lw=0.6, alpha=0.35, ls=":")
    ax.axvline(0.0, color="#8b96a8", lw=0.6, alpha=0.35, ls=":")
    ax.plot([0.0], [0.0], marker="+", color="#ffffff", ms=12, mew=1.6)
    ax.annotate("sensor (0, 0)", xy=(0.0, 0.0), xytext=(6, 6),
                textcoords="offset points", color="#dbe2ee", fontsize=8)
    ax.set_xlabel("X — sensor frame (m)", color="#8b96a8", fontsize=9)
    ax.set_ylabel("Y — sensor frame (m)", color="#8b96a8", fontsize=9)

    cax = fig.add_axes([0.90, 0.09, 0.018, 0.79])
    cbar = fig.colorbar(im, cax=cax)
    cbar.set_label("Elevation (m)", color="#8b96a8", fontsize=9)
    cbar.ax.yaxis.set_tick_params(color="#8b96a8", labelsize=8)
    for t in cbar.ax.get_yticklabels():
        t.set_color("#8b96a8")

    meta = capture.meta
    start, end = meta.get("window_start_walltime"), meta.get("window_end_walltime")
    window = _span((end - start) if (start and end) else None)
    scans = meta.get("scans_accumulated", 0)
    title = (f"DEM · {scans} scans · {stats['valid_cells']:,} cells · "
             f"z {z_min:.2f} … {z_max:.2f} m")
    subtitle = (f"{meta.get('topic', '?')} · {res:g} m cells · "
                f"agg={meta.get('aggregation')} · filter={meta.get('ground_filter')} · "
                f"accumulated {window} since service start")
    fig.text(0.075, 0.965, title, color="#dbe2ee", fontsize=12,
             fontweight="bold", va="top")
    fig.text(0.075, 0.925, subtitle, color="#8b96a8", fontsize=8.5, va="top")

    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(str(path), format="png", facecolor=bg)
    except Exception as exc:
        raise DemRenderError(f"failed to write DEM image: {exc}")
    return round(time.time() - t0, 3)
