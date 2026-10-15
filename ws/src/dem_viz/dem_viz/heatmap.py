"""Live Python heatmap visualisation for the incremental DEM.

Displays, while playback is running:

* the elevation grid as a colour-mapped heatmap with a colour bar in metres,
* NoData cells transparent,
* the drone/vehicle trajectory (and current position),
* playback progress and DEM processing statistics.

Works with any matplotlib backend that is available (Qt/Tk); when no display
is reachable it falls back to a non-interactive backend and (optionally)
writes PNG snapshots instead.
"""

import json
import os
import threading
import time

import numpy as np


def pick_backend():
    """Choose an interactive backend if a display exists, else Agg."""
    import matplotlib
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        matplotlib.use('Agg')
        return 'Agg'
    for backend in ('Qt5Agg', 'TkAgg'):
        try:
            matplotlib.use(backend)
            return backend
        except Exception:
            continue
    matplotlib.use('Agg')
    return 'Agg'


class DemHeatmap:
    """Owns the matplotlib figure; updates are triggered from the ROS timer."""

    def __init__(self, node, cmap='terrain', title='Incremental DEM'):
        import matplotlib.pyplot as plt
        from matplotlib.colors import Normalize

        self._node = node
        self._lock = threading.Lock()
        self._elev = None
        self._origin = (0.0, 0.0)
        self._res = 0.2
        self._traj = []            # list of (x, y)
        self._current = None       # (x, y)
        self._stats = {}
        self._replay = {}
        self._last_draw = 0.0
        self._frames_saved = 0
        self._save_dir = node.get_parameter('save_dir').value \
            if hasattr(node, 'get_parameter') else ''

        self.fig, self.ax = plt.subplots(figsize=(9, 8))
        self.fig.canvas.manager.set_window_title(title) \
            if hasattr(self.fig.canvas.manager, 'set_window_title') else None
        self.im = None
        self.cbar = None
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_aspect('equal', adjustable='box')
        self.title = self.ax.set_title(title)
        self.norm = Normalize(vmin=0.0, vmax=1.0)
        self._traj_line, = self.ax.plot([], [], color='black', lw=1.2,
                                        label='trajectory')
        self._cur_dot, = self.ax.plot([], [], 'o', color='red', ms=7,
                                      label='vehicle')
        self._info = self.fig.text(0.02, 0.02, '', fontsize=9,
                                   family='monospace',
                                   va='bottom')
        self.ax.legend(loc='upper right')

    # ----------------------------------------------------------- data in
    def set_elevation(self, elev, origin, resolution):
        with self._lock:
            self._elev = np.asarray(elev, dtype=np.float32)
            self._origin = (float(origin[0]), float(origin[1]))
            self._res = float(resolution)

    def set_trajectory(self, xy, current=None):
        with self._lock:
            if xy is not None and len(xy):
                self._traj = np.asarray(xy, dtype=np.float64)
            if current is not None:
                self._current = tuple(current)

    def set_stats(self, d):
        with self._lock:
            self._stats = d

    def set_replay(self, d):
        with self._lock:
            self._replay = d

    # ---------------------------------------------------------- rendering
    def draw(self, force=False, min_interval=0.2):
        now = time.monotonic()
        if not force and now - self._last_draw < min_interval:
            return False
        with self._lock:
            if self._elev is None:
                self._last_draw = now
                return False
            elev = self._elev
            origin = self._origin
            res = self._res
            traj = self._traj
            cur = self._current
            stats = dict(self._stats)
            replay = dict(self._replay)

        ny, nx = elev.shape
        ext = (origin[0], origin[0] + nx * res,
               origin[1], origin[1] + ny * res)

        masked = np.ma.masked_invalid(elev)
        if masked.count():
            vmin = float(masked.min())
            vmax = float(masked.max())
            if vmax - vmin < 1e-6:
                vmax = vmin + 1e-3
            self.norm.vmin, self.norm.vmax = vmin, vmax
        else:
            vmin = vmax = 0.0

        if self.im is None:
            self.im = self.ax.imshow(masked, origin='lower', extent=ext,
                                     cmap='terrain', norm=self.norm,
                                     interpolation='nearest')
            self.cbar = self.fig.colorbar(self.im, ax=self.ax, shrink=0.85,
                                          pad=0.02)
            self.cbar.set_label('Elevation (m)')
        else:
            self.im.set_data(masked)
            self.im.set_extent(ext)
            self.im.set_clim(vmin, vmax)

        # observed mapping area (axes follow the grid extent as it grows)
        valid_count = np.ma.masked_invalid(elev).count()
        if valid_count:
            self.ax.set_xlim(ext[0] - 5, ext[1] + 5)
            self.ax.set_ylim(ext[2] - 5, ext[3] + 5)

        if traj is not None and len(traj):
            self._traj_line.set_data(traj[:, 0], traj[:, 1])
        if cur is not None:
            self._cur_dot.set_data([cur[0]], [cur[1]])

        # info text: playback progress + processing stats
        prog = replay.get('progress', 0.0)
        bar_len = 30
        filled = int(round(prog * bar_len))
        bar = '#' * filled + '-' * (bar_len - filled)
        grid = stats.get('grid', {})
        proc = stats.get('processing_ms', {}).get('total', {})
        lines = [
            f"playback [{bar}] {100.0 * prog:5.1f}%  "
            f"state={replay.get('state', '-')} rate={replay.get('rate', '-')}x",
            f"bag t={replay.get('bag_time_s', 0.0):.1f}s/"
            f"{replay.get('bag_duration_s', 0.0):.1f}s  "
            f"msgs={replay.get('messages_published', 0)}",
            f"scans={stats.get('clouds_received', 0)}  "
            f"input={stats.get('input_hz', 0.0):.1f} Hz  "
            f"proc total={proc.get('mean', 0.0):.1f} ms "
            f"(p95 {proc.get('p95', 0.0):.1f} ms)",
            f"cells={grid.get('valid_cells', 0)}  "
            f"coverage={grid.get('coverage_m2', 0.0):.0f} m2  "
            f"elev [{grid.get('z_min')}, {grid.get('z_max')}] m"
            if grid.get('z_min') is not None else
            f"cells={grid.get('valid_cells', 0)}",
            f"dropped(no pose)={stats.get('clouds_dropped_no_pose', 0)}",
        ]
        self._info.set_text('\n'.join(lines))
        self.title.set_text(
            f"Incremental DEM  ({nx}x{ny} @ {res} m, "
            f"{grid.get('aggregation', '?')} aggregation)")

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        self._last_draw = now

        if self._save_dir:
            self._maybe_save(elev, prog)
        return True

    def _maybe_save(self, elev, prog):
        # one snapshot every ~5% of playback
        bucket = int(prog * 20)
        if bucket == getattr(self, '_last_bucket', -1):
            return
        self._last_bucket = bucket
        try:
            os.makedirs(self._save_dir, exist_ok=True)
            path = os.path.join(
                self._save_dir,
                f'dem_progress_{self._frames_saved:03d}_'
                f'{int(prog * 100):03d}pct.png')
            self.fig.savefig(path, dpi=110)
            self._frames_saved += 1
            self._node.get_logger().info(f'saved snapshot {path}')
        except Exception as exc:
            self._node.get_logger().warn(f'could not save snapshot: {exc}')

    def close(self):
        import matplotlib.pyplot as plt
        plt.close(self.fig)
