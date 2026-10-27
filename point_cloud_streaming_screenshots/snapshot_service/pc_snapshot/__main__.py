"""Service entrypoint.

Usage (inside a sourced ROS 2 environment):
    python3 -m pc_snapshot --topic /ouster/points --port 8085

Configuration precedence: CLI > environment (PCSNAP_*) > JSON file > defaults.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from .api import create_app
from .config import Config, VIEW_NAMES
from .dem import DemAccumulator
from .jobs import DemCoordinator, RenderCoordinator, SnapshotRegistry
from .ros_subscriber import RosCloudListener
from .store import SnapshotStore
from .stream import StreamManager


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Remote LiDAR point-cloud snapshot service")
    p.add_argument("--config", help="path to JSON config file")
    p.add_argument("--topic", help="PointCloud2 topic to subscribe to")
    p.add_argument("--host", help="HTTP bind address (default 0.0.0.0)")
    p.add_argument("--port", type=int, help="HTTP port (default 8085)")
    p.add_argument("--renderer", choices=["auto", "open3d", "matplotlib"],
                   help="rendering backend (default auto)")
    p.add_argument("--up-axis", choices=["z", "y"], dest="up_axis",
                   help="vertical axis of the cloud (default z)")
    p.add_argument("--color-mode", choices=["height", "intensity"],
                   dest="color_mode", help="point coloring (default height)")
    p.add_argument("--max-render-points", type=int, dest="max_render_points",
                   help="max points rendered per snapshot (default 300000)")
    p.add_argument("--stale-after", type=float, dest="stale_after_sec",
                   help="seconds before latest message counts as stale (default 5)")
    p.add_argument("--clip-range", type=float, dest="clip_range_m",
                   help="radial clip from sensor origin in meters (default 0 = off)")
    # --- incremental DEM (Get DEM) -------------------------------------
    p.add_argument("--no-dem", action="store_true",
                   help="disable incremental DEM accumulation (Get DEM)")
    p.add_argument("--dem-resolution", type=float, dest="dem_resolution",
                   help="DEM cell size in metres (default 0.25)")
    p.add_argument("--dem-extent", type=float, dest="dem_extent",
                   help="square DEM grid edge in metres, centred on the "
                        "sensor (default 200)")
    p.add_argument("--dem-clip-range", type=float, dest="dem_clip_range_m",
                   help="radial clip for DEM points in metres (default 60)")
    p.add_argument("--dem-aggregation", dest="dem_aggregation",
                   choices=["min", "mean"],
                   help="per-cell aggregation (default min)")
    p.add_argument("--dem-ground-filter", dest="dem_ground_filter",
                   choices=["height_band", "z_range", "none"],
                   help="filter applied before merging (default height_band)")
    p.add_argument("--dem-colormap", dest="dem_colormap",
                   help="matplotlib colormap for the DEM heatmap (default turbo)")
    p.add_argument("--dem-mapping-path", dest="dem_mapping_path",
                   help="directory containing the `dem_mapping` package "
                        "(default: <repo>/ws/src/dem_mapping)")
    p.add_argument("--api-token", dest="api_token",
                   help="optional API token required in X-API-Token header")
    p.add_argument("--stream-fps", type=float, dest="stream_fps",
                   help="live stream target FPS (default 5)")
    p.add_argument("--stream-view", dest="stream_view",
                   choices=list(VIEW_NAMES),
                   help="default live stream camera view (default front)")
    p.add_argument("--no-stream", action="store_true",
                   help="disable the live /api/stream endpoint")
    p.add_argument("--no-ros", action="store_true",
                   help="run without a ROS subscriber (API/testing only)")
    p.add_argument("--work-dir", dest="work_dir",
                   help="directory for rendered snapshot files")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    cfg = Config.load(config_file=args.config, cli_overrides={
        "topic": args.topic,
        "host": args.host,
        "port": args.port,
        "renderer": args.renderer,
        "up_axis": args.up_axis,
        "color_mode": args.color_mode,
        "max_render_points": args.max_render_points,
        "stale_after_sec": args.stale_after_sec,
        "clip_range_m": args.clip_range_m,
        "api_token": args.api_token,
        "stream_fps": args.stream_fps,
        "stream_view": args.stream_view,
        "stream_enabled": (False if args.no_stream else None),
        "dem_enabled": (False if args.no_dem else None),
        "dem_resolution": args.dem_resolution,
        "dem_extent": args.dem_extent,
        "dem_clip_range_m": args.dem_clip_range_m,
        "dem_aggregation": args.dem_aggregation,
        "dem_ground_filter": args.dem_ground_filter,
        "dem_colormap": args.dem_colormap,
        "dem_mapping_path": args.dem_mapping_path,
    })

    store = SnapshotStore(topic=cfg.topic, stale_after_sec=cfg.stale_after_sec)
    work_dir = Path(args.work_dir) if args.work_dir else Path("/tmp/pc_snapshot")
    registry = SnapshotRegistry(work_dir, cfg.max_snapshots,
                                cfg.snapshot_retention_sec)
    coordinator = RenderCoordinator(cfg, registry)
    dem_registry = SnapshotRegistry(work_dir / "dem", cfg.max_snapshots,
                                    cfg.snapshot_retention_sec)
    dem_coordinator = DemCoordinator(cfg, dem_registry)
    dem = DemAccumulator(cfg)
    if not dem.enabled:
        logging.warning("DEM disabled: %s", dem.reason)
    stream = StreamManager(cfg, store)

    listener = None
    if not args.no_ros:
        listener = RosCloudListener(cfg.topic, store, domain_id=cfg.ros_domain_id,
                                    dem=dem)
        try:
            listener.start()
        except Exception as exc:
            logging.error("failed to start ROS subscriber: %s", exc)
            listener = None

    app = create_app(cfg, store, coordinator, listener=listener, stream=stream,
                     dem=dem, dem_coordinator=dem_coordinator)

    import uvicorn
    logging.info("starting HTTP server on %s:%d (topic=%s, renderer=%s)",
                 cfg.host, cfg.port, cfg.topic, cfg.renderer)
    uvicorn.run(app, host=cfg.host, port=cfg.port, log_level="info")
    from .renderer import shutdown_render_thread
    shutdown_render_thread()   # destroy Open3D renderer on its own thread
    if listener is not None:
        listener.stop()
    registry.shutdown()
    dem_registry.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
