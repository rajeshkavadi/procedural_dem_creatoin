"""Service entrypoint.

Usage:
    python -m pc_snapshot --bag /path/to/recording.bag          # replay a bag
    python -m pc_snapshot --bag /path/to/recording.bag --inspect  # list topics
    python -m pc_snapshot                                        # live ROS 1

Configuration precedence: CLI > environment (PCSNAP_*) > JSON file > defaults.

The ``--bag`` path needs no ROS installation: the ROS 1 bag is read directly
with the pure-Python ``rosbags`` library (see ``bag_source.py``). This is the
supported mode on Ubuntu 18.04 / ROS Melodic, whose ``rospy`` is Python 2
only while this service requires Python 3.
"""



import argparse
import logging
import sys
from pathlib import Path

from .api import create_app
from .bag_source import BagCloudSource, BagSourceError, inspect_bag
from .config import Config, VIEW_NAMES
from .dem import DemAccumulator
from .gnss import load_gnss
from .jobs import DemCoordinator, RenderCoordinator, SnapshotRegistry
from .ros_subscriber import Ros1CloudListener
from .store import SnapshotStore
from .stream import StreamManager


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Remote LiDAR point-cloud snapshot service")
    p.add_argument("--config", help="path to JSON config file")
    p.add_argument("--topic", help="PointCloud2 topic to read")
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
    # --- ROS 1 bag replay ----------------------------------------------
    p.add_argument("--bag", dest="bag_path", metavar="PATH",
                   help="ROS 1 .bag file (or directory of split bags) to replay; "
                        "no ROS environment required")
    p.add_argument("--bag-rate", type=float, dest="bag_rate",
                   help="bag playback speed factor (default 1.0)")
    p.add_argument("--no-loop", dest="bag_loop", action="store_false",
                   default=None,
                   help="play the bag once instead of looping (default: loop)")
    p.add_argument("--bag-start-offset", type=float, dest="bag_start_offset_sec",
                   help="seconds to skip into the bag (default 0)")
    p.add_argument("--no-bag-auto-topic", dest="bag_auto_topic",
                   action="store_false", default=None,
                   help="do not auto-select the only PointCloud2 topic when "
                        "--topic is missing from the bag")
    p.add_argument("--gnss", dest="gnss_path", metavar="PATH",
                   help="raw NMEA/gpspipe GNSS log (e.g. ublox7_raw.txt) used "
                        "to report the recording position and annotate the DEM")
    p.add_argument("--inspect", action="store_true",
                   help="print the bag's topics/duration and exit")
    p.add_argument("--ros-master", dest="ros_master_uri",
                   help="ROS 1 master URI for live mode (default: ROS_MASTER_URI)")
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
                   help="run without any point-cloud source (API/testing only)")
    p.add_argument("--work-dir", dest="work_dir",
                   help="directory for rendered snapshot files")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    if args.inspect:
        if not args.bag_path:
            print("error: --inspect requires --bag PATH", file=sys.stderr)
            return 2
        try:
            print(inspect_bag(args.bag_path).describe())
        except BagSourceError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 1
        return 0

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
        "bag_path": args.bag_path,
        "bag_rate": args.bag_rate,
        "bag_loop": args.bag_loop,
        "bag_start_offset_sec": args.bag_start_offset_sec,
        "bag_auto_topic": args.bag_auto_topic,
        "gnss_path": args.gnss_path,
        "ros_master_uri": args.ros_master_uri,
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
    # DEM first: importing dem_mapping puts the shared georef module on
    # sys.path, which the GNSS UTM anchor reuses.
    dem = DemAccumulator(cfg)
    if not dem.enabled:
        logging.warning("DEM disabled: %s", dem.reason)
    gnss = load_gnss(cfg.gnss_path)
    if gnss is not None:
        if gnss.available:
            anchor = gnss.anchor() or {}
            logging.info("GNSS: %d valid fixes from %s, anchor %.6f, %.6f",
                         gnss.fixes_valid, gnss.path, anchor.get("lat", 0.0),
                         anchor.get("lon", 0.0))
        else:
            logging.warning("GNSS unavailable: %s", gnss.reason)
    dem.gnss = gnss
    stream = StreamManager(cfg, store)

    # ------------------------------------------------------------- source
    source = None
    if args.no_ros:
        logging.info("no point-cloud source requested (--no-ros)")
    elif cfg.bag_path:
        source = BagCloudSource(cfg.bag_path, cfg.topic, store, dem=dem,
                                rate=cfg.bag_rate, loop=cfg.bag_loop,
                                start_offset_sec=cfg.bag_start_offset_sec,
                                auto_topic=cfg.bag_auto_topic)
    else:
        source = Ros1CloudListener(cfg.topic, store,
                                   master_uri=cfg.ros_master_uri, dem=dem)

    if source is not None:
        try:
            source.start()
        except Exception as exc:
            # Keep the object: /api/health reports its start_error so the
            # operator sees *why* nothing arrives instead of a bare timeout.
            logging.error("failed to start point-cloud source: %s", exc)
        active = getattr(source, "active_topic", None)
        if active:
            store.topic = active
            if active != cfg.topic:
                logging.info("using bag topic %s", active)

    app = create_app(cfg, store, coordinator, listener=source, stream=stream,
                     dem=dem, dem_coordinator=dem_coordinator, gnss=gnss)

    import uvicorn
    logging.info("starting HTTP server on %s:%d (topic=%s, source=%s, "
                 "renderer=%s)", cfg.host, cfg.port, store.topic,
                 (getattr(source, "kind", None) or "none"), cfg.renderer)
    try:
        uvicorn.run(app, host=cfg.host, port=cfg.port, log_level="info")
    finally:
        from .renderer import shutdown_render_thread
        shutdown_render_thread()   # destroy Open3D renderer on its own thread
        if source is not None:
            source.stop()
        registry.shutdown()
        dem_registry.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
