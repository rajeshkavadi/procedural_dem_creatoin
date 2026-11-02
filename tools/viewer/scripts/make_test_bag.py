#!/usr/bin/env python3
"""Write a synthetic ROS 1 ``.bag`` for end-to-end testing.

The service reads ROS 1 bags directly, so a recording is the only input it
needs. This script produces a small one (ground plane + two walls + a moving
box) with ``sensor_msgs/PointCloud2`` messages, using the same pure-Python
``rosbags`` library the service uses -- no ROS installation required.

Usage:
    python scripts/make_test_bag.py /tmp/test.bag --frames 60 --rate 10
    ./scripts/run_service.sh --bag /tmp/test.bag
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pc_snapshot.synthetic import write_test_bag  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("out", help="output .bag path")
    ap.add_argument("--topic", default="/mid360/livox/lidar")
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--rate", type=float, default=10.0,
                    help="messages per second (default 10)")
    ap.add_argument("--frame-id", default="camera_init")
    ap.add_argument("--no-intensity", action="store_true")
    args = ap.parse_args()

    out = write_test_bag(args.out, topic=args.topic, frames=args.frames,
                         rate=args.rate, frame_id=args.frame_id,
                         include_intensity=not args.no_intensity)
    print(f"wrote {args.frames} messages to {out} "
          f"({args.frames / max(0.1, args.rate):.1f}s, topic {args.topic})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
