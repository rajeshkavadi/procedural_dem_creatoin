"""Synthetic scene + ROS 1 bag writer used by tests and demos.

Kept in the package (rather than only in ``scripts/``) so the test suite can
build a real ``.bag`` fixture without importing a script module. Requires
``rosbags`` only when :func:`write_test_bag` is called.
"""

from __future__ import annotations

import math
import struct
from pathlib import Path
from typing import List, Tuple

Point = Tuple[float, float, float, float]


def build_points(t: float) -> List[Point]:
    """Deterministic outdoor-ish scene: ground plane, two walls, moving box."""
    pts: List[Point] = []
    for i in range(-10, 11):
        for j in range(-10, 11):
            x, y = float(i), float(j)
            z = 0.02 * math.sin(0.3 * x + t) + 0.02 * math.cos(0.3 * y)
            pts.append((x, y, z, 30.0 + 2.0 * z))
    for i in range(0, 41):
        z = i * 0.1
        pts.append((10.0, -10.0 + 0.5 * (i % 20), z, 120.0))
        pts.append((-10.0 + 0.5 * (i % 20), 10.0, z, 90.0))
    cx = 2.0 * math.sin(0.2 * t)
    for dx in (-2.0, 2.0):
        for dy in (-1.0, 1.0):
            for k in range(0, 16):
                pts.append((cx + dx, dy, k * 0.1 + 0.05, 200.0))
    for rx in range(-19, 20):
        for ry in (-9, 9):
            pts.append((cx + rx * 0.1, ry * 0.1, 1.5, 180.0))
    return pts


def write_test_bag(path, topic: str = "/mid360/livox/lidar",
                   frames: int = 60, rate: float = 10.0,
                   frame_id: str = "camera_init",
                   include_intensity: bool = True,
                   start_ns: int = 1_700_000_000_000_000_000):
    """Write *frames* PointCloud2 messages to a ROS 1 ``.bag`` and return it."""
    import numpy as np
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.ROS1_NOETIC)
    PointField = typestore.types["sensor_msgs/msg/PointField"]
    PointCloud2 = typestore.types["sensor_msgs/msg/PointCloud2"]
    Header = typestore.types["std_msgs/msg/Header"]
    Time = typestore.types["builtin_interfaces/msg/Time"]

    out = Path(path).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        out.unlink()

    period_ns = int(1e9 / max(0.1, float(rate)))
    msgtype = "sensor_msgs/msg/PointCloud2"

    with Writer(out) as writer:
        conn = writer.add_connection(topic, msgtype, typestore=typestore)
        for i in range(int(frames)):
            pts = build_points(i * 0.2)
            n = len(pts)
            if include_intensity:
                fields = (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))
                point_step = 16
                buf = b"".join(struct.pack("<ffff", *p) for p in pts)
            else:
                fields = (("x", 0), ("y", 4), ("z", 8))
                point_step = 12
                buf = b"".join(struct.pack("<fff", p[0], p[1], p[2])
                               for p in pts)

            stamp_ns = start_ns + i * period_ns
            header = Header(
                seq=i,
                stamp=Time(sec=stamp_ns // 1_000_000_000,
                           nanosec=stamp_ns % 1_000_000_000),
                frame_id=frame_id)
            msg = PointCloud2(
                header=header,
                height=1,
                width=n,
                fields=[PointField(name=name, offset=off,
                                   datatype=PointField.FLOAT32, count=1)
                        for name, off in fields],
                is_bigendian=False,
                point_step=point_step,
                row_step=point_step * n,
                data=np.frombuffer(buf, dtype=np.uint8),
                is_dense=True,
            )
            writer.write(conn, stamp_ns,
                         typestore.serialize_ros1(msg, msgtype))
    return out
