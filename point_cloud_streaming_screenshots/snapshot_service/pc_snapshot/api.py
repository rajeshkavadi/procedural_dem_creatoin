"""FastAPI HTTP application.

Endpoints:
    GET  /api/health
    POST /api/snapshot
    GET  /api/snapshot/{snapshot_id}/{view}.png
    GET  /api/snapshot/{snapshot_id}/download.zip
    POST /api/dem                     (render the incremental DEM so far)
    GET  /api/dem/{dem_id}/dem.png
    GET  /api/dem/{dem_id}/download.zip
    GET  /api/stream                (live MJPEG-style point cloud stream)
    GET  /                       (static browser frontend)

The app depends only on a SnapshotStore and a RenderCoordinator, so it can be
unit-tested without ROS or a bag. The DEM is optional: when no DemAccumulator
is wired in, /api/health reports it as disabled and POST /api/dem returns 409.
"""

from __future__ import annotations

import logging
import platform
import time
from pathlib import Path
from typing import Optional

import anyio
from fastapi import Depends, FastAPI, Header, HTTPException, Query, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import (FileResponse, JSONResponse, Response,
                               StreamingResponse)
from fastapi.staticfiles import StaticFiles

from . import __version__
from .config import VIEW_NAMES, Config
from .dem import DemAccumulator, DemRenderError
from .jobs import (JobBusyError, RenderCoordinator, SnapshotNotFound,
                   DemCoordinator, build_zip)
from .renderer import RenderError
from .ros_subscriber import RosCloudListener
from .store import SnapshotStore
from .stream import BOUNDARY, StreamManager, mjpeg_generator

log = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).resolve().parent.parent / "static"


def create_app(cfg: Config, store: SnapshotStore,
               coordinator: RenderCoordinator,
               listener: Optional[RosCloudListener] = None,
               stream: Optional[StreamManager] = None,
               dem: Optional[DemAccumulator] = None,
               dem_coordinator: Optional[DemCoordinator] = None) -> FastAPI:
    app = FastAPI(title="Remote LiDAR Snapshot Service", version=__version__)
    app.state.cfg = cfg
    app.state.store = store
    app.state.coordinator = coordinator
    app.state.listener = listener
    app.state.dem = dem
    app.state.dem_coordinator = dem_coordinator
    if dem is not None and dem_coordinator is None:
        log.warning("DEM accumulator given without a DemCoordinator; "
                    "POST /api/dem will fail until one is provided")
    # A StreamManager renders on-demand only (threads start when a client
    # connects), so creating a default one here is safe even in tests.
    app.state.stream = stream if stream is not None else StreamManager(cfg, store)

    app.add_middleware(
        CORSMiddleware,
        allow_origins=cfg.cors_origins,
        allow_methods=["GET", "POST"],
        allow_headers=["*"],
    )

    # ------------------------------------------------------------------
    # Optional token auth (empty token = disabled, intended for a private
    # Tailscale network only).
    # ------------------------------------------------------------------
    def require_token(x_api_token: Optional[str] = Header(default=None),
                      token: Optional[str] = Query(default=None)) -> None:
        if not cfg.api_token:
            return
        if x_api_token == cfg.api_token or token == cfg.api_token:
            return
        raise HTTPException(status_code=401, detail="invalid or missing API token")

    # ------------------------------------------------------------------
    @app.get("/api/health")
    def health() -> dict:
        store_health = store.health()
        listener_alive = listener.alive if listener is not None else False
        return {
            "status": "ok",
            "service": "pc-snapshot",
            "version": __version__,
            "ros": {
                "listener_alive": listener_alive,
                "listener_start_error":
                    listener.start_error if listener is not None else None,
                "connected": listener_alive and store_health["message_received"],
            },
            "point_cloud": store_health,
            "render": {
                "renderer": cfg.renderer,
                "image_size": [cfg.image_width, cfg.image_height],
                "max_render_points": cfg.max_render_points,
                "max_concurrent_renders": cfg.max_concurrent_renders,
            },
            "stream": app.state.stream.status(),
            "dem": (dem.status() if dem is not None
                    else {"enabled": False,
                          "reason": "DEM not configured for this service"}),
            "host": {"platform": platform.platform(),
                     "python": platform.python_version()},
            "time": time.time(),
        }

    @app.post("/api/snapshot")
    async def snapshot(request: Request, _: None = Depends(require_token)):
        age = store.age_seconds()
        if age is None:
            raise HTTPException(
                status_code=409,
                detail=(f"no point cloud has been received yet on topic "
                        f"'{cfg.topic}'. Start bag playback or the sensor "
                        f"driver first."))
        if age > cfg.stale_after_sec:
            msg = (f"latest point cloud is stale: last message {age:.1f}s ago "
                   f"(threshold {cfg.stale_after_sec:.1f}s). Bag playback may "
                   f"have stopped or the topic is idle.")
            if cfg.stale_policy == "reject":
                raise HTTPException(status_code=409, detail=msg)
            log.warning(msg)

        snap = store.capture()
        if snap is None or snap.point_count == 0:
            raise HTTPException(
                status_code=409,
                detail="captured message contains no usable points")

        # Render in a worker thread; the subscriber keeps running.
        try:
            artifact = await anyio.to_thread.run_sync(coordinator.run, snap)
        except JobBusyError as exc:
            raise HTTPException(status_code=409, detail=str(exc))
        except RenderError as exc:
            log.error("render failed: %s", exc)
            raise HTTPException(status_code=500, detail=f"rendering failed: {exc}")
        except Exception as exc:  # unexpected
            log.exception("snapshot job failed")
            raise HTTPException(status_code=500, detail=f"snapshot job failed: {exc}")

        meta = artifact.meta
        base = str(request.base_url).rstrip("/")
        images = {
            view: f"{base}/api/snapshot/{artifact.snapshot_id}/{view}.png"
            for view in VIEW_NAMES
        }
        warnings = []
        if age > cfg.stale_after_sec:
            warnings.append(f"snapshot rendered from a stale message ({age:.1f}s old)")
        if meta["rendered_point_count"] < meta["valid_point_count"]:
            warnings.append(
                f"downsampled from {meta['valid_point_count']} to "
                f"{meta['rendered_point_count']} points for rendering")
        return {
            "snapshot_id": artifact.snapshot_id,
            "capture_timestamp": meta["capture_timestamp"],
            "ros_message_stamp": meta["ros_message_stamp"],
            "message_age_at_capture_sec": meta["message_age_at_capture_sec"],
            "frame_id": meta["frame_id"],
            "topic": meta["topic"],
            "original_point_count": meta["original_point_count"],
            "valid_point_count": meta["valid_point_count"],
            "rendered_point_count": meta["rendered_point_count"],
            "image_size": meta["image_size"],
            "render_seconds": meta["render_seconds"],
            "views": list(VIEW_NAMES),
            "images": images,
            "download_zip": f"{base}/api/snapshot/{artifact.snapshot_id}/download.zip",
            "warnings": warnings,
        }

    # ------------------------------------------------------------------
    # Incremental DEM: render everything accumulated from the start of
    # playback up to the instant the user clicks "Get DEM".
    # ------------------------------------------------------------------
    @app.post("/api/dem")
    async def get_dem(request: Request, _: None = Depends(require_token)):
        acc: Optional[DemAccumulator] = app.state.dem
        if acc is None:
            raise HTTPException(
                status_code=409,
                detail="DEM is not configured for this service")
        if not acc.enabled:
            raise HTTPException(status_code=409,
                                detail=f"DEM unavailable: {acc.reason}")
        if acc.empty():
            raise HTTPException(
                status_code=409,
                detail=(f"no point cloud has been accumulated yet on topic "
                        f"'{cfg.topic}'. Start bag playback first, wait for "
                        f"the stream to deliver messages, then retry."))
        dem_coord: Optional[DemCoordinator] = app.state.dem_coordinator
        if dem_coord is None:
            raise HTTPException(
                status_code=503,
                detail="DEM renderer is not configured for this service")

        capture = await anyio.to_thread.run_sync(acc.capture)
        if capture is None or not capture.valid.any():
            raise HTTPException(
                status_code=409,
                detail=("the accumulated grid has no valid cells yet — "
                        "the filters may be discarding every point "
                        "(check dem_clip_range_m / dem_ground_filter)"))

        try:
            artifact = await anyio.to_thread.run_sync(dem_coord.run, capture)
        except JobBusyError as exc:
            raise HTTPException(status_code=409, detail=str(exc))
        except DemRenderError as exc:
            log.error("DEM render failed: %s", exc)
            raise HTTPException(status_code=409, detail=f"DEM render failed: {exc}")
        except Exception as exc:  # unexpected
            log.exception("DEM job failed")
            raise HTTPException(status_code=500, detail=f"DEM job failed: {exc}")

        meta = artifact.meta
        base = str(request.base_url).rstrip("/")
        warnings = []
        # Staleness is judged by host receive time (window_end_walltime), the
        # same clock /api/health uses — ROS stamps may be from record time.
        end = capture.meta.get("window_end_walltime")
        if end:
            age = time.time() - float(end)
            if age > cfg.stale_after_sec:
                warnings.append(
                    f"accumulated up to a message {age:.1f}s old "
                    f"(bag playback may have stopped)")
        return {
            "dem_id": artifact.snapshot_id,
            "capture_timestamp": meta["capture_timestamp"],
            "render_seconds": meta["render_seconds"],
            "topic": meta["topic"],
            "frame": meta["frame"],
            "scans_accumulated": meta["scans_accumulated"],
            "points_in": meta["points_in"],
            "points_used": meta["points_used"],
            "window": {
                "start_walltime": meta.get("window_start_walltime"),
                "end_walltime": meta.get("window_end_walltime"),
                "first_scan_stamp": meta.get("first_scan_stamp"),
                "last_scan_stamp": meta.get("last_scan_stamp"),
            },
            "grid": {
                "shape": meta.get("shape") or meta.get("grid_shape"),
                "resolution_m": meta.get("resolution_m"),
                "extent_m": meta.get("extent_m"),
                "origin": meta.get("origin"),
                "aggregation": meta.get("aggregation"),
                "valid_cells": meta.get("valid_cells"),
                "coverage_m2": meta.get("coverage_m2"),
                "observations": meta.get("observations"),
                "out_of_bounds_points": meta.get("out_of_bounds_points"),
                "z_min": meta.get("z_min"),
                "z_max": meta.get("z_max"),
            },
            "filters": {
                "clip_range_m": meta.get("clip_range_m"),
                "min_range_m": meta.get("min_range_m"),
                "ground_filter": meta.get("ground_filter"),
            },
            "image_size": meta.get("image_size"),
            "colormap": meta.get("colormap"),
            "image": f"{base}/api/dem/{artifact.snapshot_id}/dem.png",
            "download_zip": f"{base}/api/dem/{artifact.snapshot_id}/download.zip",
            "warnings": warnings,
        }

    # ------------------------------------------------------------------
    # Live point-cloud stream (multipart/x-mixed-replace, MJPEG-style)
    # ------------------------------------------------------------------
    @app.get("/api/stream")
    async def stream_endpoint(request: Request,
                              view: Optional[str] = Query(default=None),
                              _: None = Depends(require_token)):
        """Live view of the incoming point cloud as JPEG frames.

        Uses the same multipart/x-mixed-replace mechanism as Flask/OpenCV
        camera-streaming tutorials: a plain browser <img> tag displays it,
        no WebSocket/video codec needed. ?view= picks the camera angle.
        """
        if not cfg.stream_enabled:
            raise HTTPException(status_code=404, detail="streaming disabled")
        mgr: StreamManager = app.state.stream
        view_name = view or cfg.stream_view
        if view_name not in VIEW_NAMES:
            raise HTTPException(
                status_code=400,
                detail=f"unknown view {view_name!r}; "
                       f"expected one of {list(VIEW_NAMES)}")

        mgr.add_client(view_name)
        return StreamingResponse(
            mjpeg_generator(mgr, view_name),
            media_type=f"multipart/x-mixed-replace; boundary={BOUNDARY}")

    @app.get("/api/snapshot/{snapshot_id}/download.zip")
    async def get_zip(snapshot_id: str):
        try:
            artifact = coordinator.registry.get(snapshot_id)
        except SnapshotNotFound as exc:
            raise HTTPException(status_code=404, detail=str(exc))
        data = await anyio.to_thread.run_sync(build_zip, artifact)
        return Response(
            content=data, media_type="application/zip",
            headers={"Content-Disposition":
                     f'attachment; filename="snapshot_{snapshot_id}.zip"'})

    @app.get("/api/snapshot/{snapshot_id}/{view}")
    async def get_view(snapshot_id: str, view: str):
        view_name = view[:-4] if view.endswith(".png") else view
        if view_name not in VIEW_NAMES:
            raise HTTPException(
                status_code=404,
                detail=f"unknown view {view!r}; expected one of {list(VIEW_NAMES)}")
        try:
            artifact = coordinator.registry.get(snapshot_id)
        except SnapshotNotFound as exc:
            raise HTTPException(status_code=404, detail=str(exc))
        path = artifact.files.get(view_name)
        if path is None or not path.is_file():
            raise HTTPException(status_code=404, detail="image not found")
        return FileResponse(path, media_type="image/png",
                            filename=f"{view_name}.png")

    @app.get("/api/dem/{dem_id}/download.zip")
    async def get_dem_zip(dem_id: str):
        try:
            artifact = _dem_artifact(dem_id)
        except SnapshotNotFound as exc:
            raise HTTPException(status_code=404, detail=str(exc))
        data = await anyio.to_thread.run_sync(build_zip, artifact)
        return Response(
            content=data, media_type="application/zip",
            headers={"Content-Disposition":
                     f'attachment; filename="dem_{dem_id}.zip"'})

    @app.get("/api/dem/{dem_id}/{name}")
    async def get_dem_image(dem_id: str, name: str):
        name_stem = name[:-4] if name.endswith(".png") else name
        if name_stem != "dem":
            raise HTTPException(
                status_code=404,
                detail=f"unknown DEM file {name!r}; expected 'dem.png'")
        try:
            artifact = _dem_artifact(dem_id)
        except SnapshotNotFound as exc:
            raise HTTPException(status_code=404, detail=str(exc))
        path = artifact.files.get("dem")
        if path is None or not path.is_file():
            raise HTTPException(status_code=404, detail="image not found")
        return FileResponse(path, media_type="image/png", filename="dem.png")

    def _dem_artifact(dem_id: str):
        dem_coord: Optional[DemCoordinator] = app.state.dem_coordinator
        if dem_coord is None:
            raise SnapshotNotFound("DEM renderer is not configured")
        return dem_coord.registry.get(dem_id)

    # ------------------------------------------------------------------
    # Frontend
    # ------------------------------------------------------------------
    if STATIC_DIR.is_dir():
        app.mount("/", StaticFiles(directory=str(STATIC_DIR), html=True),
                  name="static")
    else:  # pragma: no cover
        @app.get("/")
        def index_missing():
            return JSONResponse({"error": "static frontend directory missing"},
                                status_code=500)

    return app
