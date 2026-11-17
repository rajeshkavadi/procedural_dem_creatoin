#!/usr/bin/env python
"""Inspect an arbitrary ROS 1 bag before building a pipeline from it.

Prints, for every topic: message type, count, rate, header-stamp range and
monotonicity, frame_id, plus for the first message of sensor topics the
detailed payload layout (PointCloud2 fields, IMU units, GNSS conventions).

Usage:
    rosrun dem_replay inspect_bag <path-to-bag>
"""

from __future__ import division

import argparse
import json
import sys
from collections import defaultdict

import numpy as np


def _stamp_ns(stamp):
    return stamp.to_nsec()


def inspect(bag_path):
    import rosbag

    bag = rosbag.Bag(bag_path, 'r')
    try:
        info = bag.get_type_and_topic_info().topics
        duration = float(bag.get_end_time() - bag.get_start_time())
        start_epoch = float(bag.get_start_time())

        topic_types = dict((name, t.msg_type) for name, t in info.items())
        stats = defaultdict(lambda: {
            'count': 0, 'first': None, 'last': None, 'non_monotonic': 0,
            'frames': set(), 'first_msg': None})
        for topic, msg, _bag_t in bag.read_messages():
            s = stats[topic]
            s['count'] += 1
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
    finally:
        bag.close()

    report = {
        'bag': bag_path,
        'duration_s': round(duration, 3),
        'start_epoch_s': round(start_epoch, 6),
        'storage': 'rosbag (ROS 1)',
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
        if msg is None:
            entry['payload'] = None
        elif typ.endswith('PointCloud2'):
            int_names = {1: 'INT8', 2: 'UINT8', 3: 'INT16', 4: 'UINT16',
                         5: 'INT32', 6: 'UINT32', 7: 'FLOAT32', 8: 'FLOAT64'}
            entry['payload'] = {
                'height': msg.height, 'width': msg.width,
                'point_step': msg.point_step, 'is_dense': msg.is_dense,
                'fields': [{'name': f.name, 'offset': f.offset,
                            'datatype': f.datatype, 'count': f.count,
                            'datatype_name': int_names.get(f.datatype)}
                           for f in msg.fields],
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
                'lat': msg.latitude, 'lon': msg.longitude,
                'alt': msg.altitude, 'status': msg.status.status,
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
        else:
            entry['payload'] = {'repr': repr(msg)[:400]}
        report['topics'][topic] = entry
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag', help='path to a ROS 1 .bag file')
    ap.add_argument('--max-samples', type=int, default=20)
    ap.add_argument('--json', action='store_true', help='print raw JSON')
    args = ap.parse_args(argv)

    report = inspect(args.bag)
    if args.json:
        print(json.dumps(report, indent=2, default=str))
        return 0

    print('bag        : {}'.format(report['bag']))
    print('duration   : {} s'.format(report['duration_s']))
    print('start time : {}'.format(report['start_epoch_s']))
    print('storage    : {}\n'.format(report['storage']))
    hdr = '{:<28} {:<32} {:>6} {:>8} {:>5} {:<16} header span (s)'.format(
        'topic', 'type', 'count', 'rate Hz', 'mono', 'frames')
    print(hdr)
    print('-' * len(hdr))
    for topic, e in report['topics'].items():
        span = ''
        if e['header_first_s'] is not None:
            span = '{} .. {}'.format(e['header_first_s'], e['header_last_s'])
            if 'header_rate_hz' in e:
                span += '  ({} Hz)'.format(e['header_rate_hz'])
        mono = 'OK' if e['non_monotonic_stamps'] == 0 else \
            'FAIL({})'.format(e['non_monotonic_stamps'])
        print('{:<28} {:<32} {:>6} {:>8.2f} {:>5} {:<16} {}'.format(
            topic, e['type'], e['count'], e['rate_hz'] or 0, mono,
            ','.join(e['frame_ids']) or '-', span))
    print()
    for topic, e in report['topics'].items():
        print('== {} ({})'.format(topic, e['type']))
        print(json.dumps(e['payload'], indent=2, default=str))
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
