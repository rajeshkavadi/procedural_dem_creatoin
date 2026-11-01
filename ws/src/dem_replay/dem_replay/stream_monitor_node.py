#!/usr/bin/env python
"""Stream-order / synchronisation verifier (ROS 1 / Melodic).

Subscribe to any number of topics (given as ``topic:type`` pairs with ROS 1
types such as ``sensor_msgs/PointCloud2``; the ``sensor_msgs/msg/PointCloud2``
spelling is accepted too), measure their arrival
statistics and verify that the replayed streams keep the recorded temporal
order:

* per topic: message count, rate, header-stamp monotonicity, first/last stamp,
* global check: the sequence of *header stamps* of a reference topic compared
  against the order of arrival for all monitored topics,
* synchronisation: for every message of the first (reference) topic, the time
  difference to the nearest message of every other topic.

Everything is reported as JSON on ``/stream_monitor/status`` and printed
periodically to the log.  This is the deliverable "verify that the LiDAR,
GNSS and IMU streams are received in the correct temporal order".
"""

from __future__ import division

import json
import threading
import time

import numpy as np
import rospy
from std_msgs.msg import String


class StreamMonitor(object):

    def __init__(self):
        rospy.init_node('stream_monitor')
        self._tol = float(rospy.get_param('~order_tolerance_ms', 250.0)) / 1e3
        self._lock = threading.Lock()
        self._stamps = {}       # topic -> list of header stamps (float s)
        self._arrival = {}      # topic -> list of wall arrival times
        self._bad_order = {}    # topic -> count of non-monotonic stamps
        self._arrival_seq = []  # header stamps in global arrival order

        pairs = [p for p in rospy.get_param(
            '~topics',
            ['/mid360/livox/lidar:sensor_msgs/PointCloud2',
             '/mid360/livox/imu:sensor_msgs/Imu',
             '/gnss_pose:geometry_msgs/PoseStamped']) if p]
        self._topics = []
        from roslib.message import get_message_class
        self._subs = []
        for pair in pairs:
            topic, _, type_str = pair.partition(':')
            if not type_str:
                raise RuntimeError(
                    "topic '{}' must be given as topic/type".format(pair))
            cls = get_message_class(type_str.replace('/msg/', '/'))
            if cls is None:
                raise RuntimeError('unknown message type: {}'.format(type_str))
            self._topics.append(topic)
            self._stamps[topic] = []
            self._arrival[topic] = []
            self._bad_order[topic] = 0
            self._subs.append(rospy.Subscriber(
                topic, cls, self._make_cb(topic), queue_size=1000))

        self._ref = rospy.get_param('~reference_topic',
                                    '/mid360/livox/lidar')
        self._status_pub = rospy.Publisher(
            '/stream_monitor/status', String, queue_size=10)
        period = float(rospy.get_param('~report_period', 5.0))
        if period > 0:
            self._timer = rospy.Timer(rospy.Duration(period),
                                      self._report)
        rospy.loginfo('monitoring %s', self._topics)

    def _make_cb(self, topic):
        def cb(msg):
            hdr = getattr(msg, 'header', None)
            if hdr is None:
                stamp = time.time()
            else:
                stamp = hdr.stamp.to_sec()
            with self._lock:
                lst = self._stamps[topic]
                if lst and stamp < lst[-1]:
                    self._bad_order[topic] += 1
                lst.append(stamp)
                self._arrival[topic].append(time.time())
                self._arrival_seq.append(stamp)
        return cb

    # ------------------------------------------------------------- reporting
    def _report(self, _event=None):
        with self._lock:
            data = self._snapshot()
        msg = String()
        msg.data = json.dumps(data)
        self._status_pub.publish(msg)
        lines = ['stream monitor @ {:.1f}s'.format(data['wall_time'])]
        for t in self._topics:
            s = data['topics'][t]
            lines.append(
                '  {:<26} n={:<6} rate={:7.2f} Hz monotonic={} span={:.2f}s'
                .format(t, s['count'], s['rate_hz'],
                        'OK' if s['non_monotonic'] == 0 else 'FAIL',
                        s['span_s']))
        for t, d in data['sync_to_reference_ms'].items():
            lines.append('  sync {} <-> {}: median |dt| = {:.2f} ms, '
                         'max = {:.2f} ms'.format(
                             self._ref, t, d['median_abs_ms'],
                             d['max_abs_ms']))
        if self._ref in data['topics']:
            lines.append(
                '  global order check: {} (max inversion {} ms within '
                '{} ms tolerance)'.format(
                    'OK' if data['global_order']['ok'] else 'FAIL',
                    data['global_order']['max_inversion_ms'],
                    data['global_order']['tolerance_ms']))
        rospy.loginfo('\n'.join(lines))

    def _snapshot(self):
        now = time.time()
        out = {'wall_time': now, 'topics': {}, 'non_monotonic_total': 0}
        for t in self._topics:
            st = np.asarray(self._stamps[t], dtype=np.float64)
            ar = np.asarray(self._arrival[t], dtype=np.float64)
            n = st.size
            rate = 0.0
            span = 0.0
            if n > 1:
                span = float(st[-1] - st[0])
                rate = (n - 1) / span if span > 0 else 0.0
            out['topics'][t] = {
                'count': int(n),
                'rate_hz': round(rate, 3),
                'first_stamp': float(st[0]) if n else None,
                'last_stamp': float(st[-1]) if n else None,
                'span_s': round(span, 3),
                'non_monotonic': int(self._bad_order[t]),
                'arrival_span_s': round(float(ar[-1] - ar[0]), 3)
                if n > 1 else 0.0,
            }
            out['non_monotonic_total'] += int(self._bad_order[t])

        # synchronisation of every topic against the reference topic
        sync = {}
        if self._ref in self._stamps and self._stamps[self._ref]:
            ref = np.asarray(self._stamps[self._ref], dtype=np.float64)
            for t in self._topics:
                if t == self._ref or not self._stamps[t]:
                    continue
                other = np.asarray(self._stamps[t], dtype=np.float64)
                idx = np.searchsorted(other, ref)
                idx = np.clip(idx, 0, other.size - 1)
                idx0 = np.clip(idx - 1, 0, other.size - 1)
                d = np.minimum(np.abs(other[idx] - ref),
                               np.abs(other[idx0] - ref))
                sync[t] = {
                    'median_abs_ms': round(float(np.median(d)) * 1e3, 3),
                    'max_abs_ms': round(float(np.max(d)) * 1e3, 3),
                }
        out['sync_to_reference_ms'] = sync

        # global temporal order: messages arrive interleaved from the merged
        # time-ordered bag pass, so the merged header-stamp sequence must be
        # non-decreasing up to the (sensor-vs-record clock) tolerance.
        merged = np.asarray(self._arrival_seq, dtype=np.float64)
        max_inversion = 0.0
        if merged.size > 1:
            max_inversion = float(max(0.0, np.max(np.diff(merged) * -1.0)))
        out['global_order'] = {
            'max_inversion_ms': round(max_inversion * 1e3, 3),
            'tolerance_ms': round(self._tol * 1e3, 3),
            'ok': bool(max_inversion <= self._tol),
        }
        return out

    def summary(self):
        """Return the final JSON summary (used by tests)."""
        with self._lock:
            return self._snapshot()


def main():
    StreamMonitor()
    rospy.spin()


if __name__ == '__main__':
    main()
