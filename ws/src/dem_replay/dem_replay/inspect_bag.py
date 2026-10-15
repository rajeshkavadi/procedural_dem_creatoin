#!/usr/bin/env python3
"""Inspect an arbitrary ROS 2 bag before building a pipeline from it.

Prints, for every topic: message type, count, rate, header-stamp range and
monotonicity, frame_id, plus for the first message of sensor topics the
detailed payload layout (PointCloud2 fields, IMU units, GNSS conventions).

Usage:
    ros2 run dem_replay inspect_bag <path-to-bag> [--max-samples N]
"""

import argparse
import json
import sys
from collections import defaultdict

import numpy as np


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def inspect(bag_path, storage_id='sqlite3', max_samples=20):
    import rclpy  # noqa: F401  (needed for serialization)
    from rclpy.serialization import deserialize_message
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    meta = reader.get_metadata()
    duration = meta.duration.nanoseconds / 1e9

    from rosidl_runtime_py.utilities import get_message
    classes = {name: get_message(t) for name, t in topic_types.items()}

    stats = defaultdict(lambda: {
        'count': 0, 'first': None, 'last': None, 'non_monotonic': 0,
        'frames': set(), 'first_msg': None, 'bag_first': None,
        'bag_last': None})
    per_topic_msgs = defaultdict(int)

    while reader.has_next():
        topic, data, bag_ts = reader.read_next()
        s = stats[topic]
        s['count'] += 1
        s['bag_first'] = bag_ts if s['bag_first'] is None else s['bag_first']
        s['bag_last'] = bag_ts
        try:
            msg = deserialize_message(data, classes[topic])
        except Exception as exc:                       # pragma: no cover
            if s['first_msg'] is None:
                s['first_msg'] = f'<deserialisation failed: {exc}>'
            continue
        hdr = getattr(msg, 'header', None)
        if hdr is not None:
            ts = _stamp_ns(hdr.stamp)
            if s['last'] is not None and ts < s['last']:
                s['non_monotonic'] += 1
            if s['first'] is None:
                s['first'] = ts
            s['last'] = ts
            s['frames'].add(hdr.frame_id)
        if s['first_msg'] is None:
            s['first_msg'] = msg

    report = {
        'bag': bag_path,
        'duration_s': round(duration, 3),
        'start_epoch_s': round(meta.starting_time.nanoseconds / 1e9, 6),
        'storage_id': storage_id,
        'topics': {},
    }

    for topic, typ in sorted(topic_types.items()):
        s = stats[topic]
        entry = {
            'type': typ,
            'count': s['count'],
            'rate_hz': round((s['count'] - 1) / duration, 2)
            if duration > 0 else None,
            'frame_ids': sorted(s['frames']),
            'non_monotonic_stamps': s['non_monotonic'],
            'header_first_s': None if s['first'] is None
            else round(s['first'] / 1e9, 6),
            'header_last_s': None if s['last'] is None
            else round(s['last'] / 1e9, 6),
        }
        if entry['header_first_s'] is not None and entry['header_last_s']:
            span = entry['header_last_s'] - entry['header_first_s']
            if span > 0:
                entry['header_rate_hz'] = round((s['count'] - 1) / span, 2)
        msg = s['first_msg']
        if isinstance(msg, bytes) or msg is None:
            entry['payload'] = str(msg)
        elif typ.endswith('PointCloud2'):
            entry['payload'] = {
                'height': msg.height, 'width': msg.width,
                'point_step': msg.point_step, 'is_dense': msg.is_dense,
                'fields': [{'name': f.name, 'offset': f.offset,
                            'datatype': f.datatype, 'count': f.count}
                           for f in msg.fields],
                'datatype_names': {1: 'INT8', 2: 'UINT8', 3: 'INT16',
                                   4: 'UINT16', 5: 'INT32', 6: 'UINT32',
                                   7: 'FLOAT32', 8: 'FLOAT64'},
            }
            # ranges of x/y/z/intensity if float32 at expected offsets
            try:
                arr = np.frombuffer(bytes(msg.data),
                                    dtype=np.uint8).reshape(msg.width,
                                                            msg.point_step)
                for f in msg.fields:
                    if f.datatype == 7:
                        v = arr[:, f.offset:f.offset + 4].copy() \
                            .view(np.float32).ravel()
                        entry['payload'].setdefault('ranges', {})[f.name] = [
                            float(np.nanmin(v)), float(np.nanmax(v))]
            except Exception as exc:
                entry['payload']['range_error'] = str(exc)
        elif typ.endswith('Imu'):
            a = msg.linear_acceleration
            g = msg.angular_velocity
            entry['payload'] = {
                'orientation': [msg.orientation.x, msg.orientation.y,
                                msg.orientation.z, msg.orientation.w],
                'acc_sample': [a.x, a.y, a.z],
                'gyro_sample': [g.x, g.y, g.z],
                'hint': 'mean |acc| ~1 -> units of g, ~9.8 -> m/s^2',
            }
        elif typ.endswith('NavSatFix'):
            entry['payload'] = {
                'lat': msg.latitude, 'lon': msg.longitude, 'alt': msg.altitude,
                'status': msg.status.status,
            }
        elif typ.endswith('PoseStamped') or typ.endswith('Odometry'):
            p = msg.pose.pose.position if typ.endswith('Odometry') \
                else msg.pose.position
            entry['payload'] = {
                'position_sample': [p.x, p.y, p.z],
                'hint': 'values look like metres in a local/projected frame '
                        'if |x|,|y| are small; degrees if |lat|,|lon| like '
                        '45.x / 9.x',
            }
        report['topics'][topic] = entry
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag', help='path to a ROS 2 bag (directory or .db3)')
    ap.add_argument('--storage-id', default='sqlite3')
    ap.add_argument('--max-samples', type=int, default=20)
    ap.add_argument('--json', action='store_true', help='print raw JSON')
    args = ap.parse_args(argv)

    report = inspect(args.bag, args.storage_id, args.max_samples)
    if args.json:
        print(json.dumps(report, indent=2, default=str))
        return 0

    print(f"bag        : {report['bag']}")
    print(f"duration   : {report['duration_s']} s")
    print(f"start time : {report['start_epoch_s']}")
    print(f"storage    : {report['storage_id']}\n")
    hdr = f"{'topic':<28} {'type':<32} {'count':>6} {'rate Hz':>8} " \
          f"{'mono':>5} {'frames':<16} header span (s)"
    print(hdr)
    print('-' * len(hdr))
    for topic, e in report['topics'].items():
        span = ''
        if e['header_first_s'] is not None:
            span = f"{e['header_first_s']} .. {e['header_last_s']}"
            if 'header_rate_hz' in e:
                span += f"  ({e['header_rate_hz']} Hz)"
        mono = 'OK' if e['non_monotonic_stamps'] == 0 else \
            f"FAIL({e['non_monotonic_stamps']})"
        print(f"{topic:<28} {e['type']:<32} {e['count']:>6} "
              f"{e['rate_hz'] or 0:>8.2f} {mono:>5} "
              f"{','.join(e['frame_ids']) or '-':<16} {span}")
    print()
    for topic, e in report['topics'].items():
        print(f"== {topic} ({e['type']})")
        print(json.dumps(e['payload'], indent=2, default=str))
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
