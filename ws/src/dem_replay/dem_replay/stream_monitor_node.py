#!/usr/bin/env python3
"""Stream-order / synchronisation verifier.

Subscribe to any number of topics (given as ``topic:=ros2/type`` pairs),
measure their arrival statistics and verify that the replayed streams keep
the recorded temporal order:

* per topic: message count, rate, header-stamp monotonicity, first/last stamp,
* global check: the sequence of *header stamps* of a reference topic compared
  against the order of arrival for all monitored topics,
* synchronisation: for every message of the first (reference) topic, the time
  difference to the nearest message of every other topic.

Everything is reported as JSON on ``/stream_monitor/status`` and printed
periodically to the log.  This is the deliverable "verify that the LiDAR,
GNSS and IMU streams are received in the correct temporal order".
"""

import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String


class StreamMonitor(Node):

    def __init__(self):
        super().__init__('stream_monitor')
        self.declare_parameter(
            'topics',
            ['/mid360/livox/lidar:sensor_msgs/msg/PointCloud2',
             '/mid360/livox/imu:sensor_msgs/msg/Imu',
             '/gnss_pose:geometry_msgs/msg/PoseStamped'])
        self.declare_parameter('reference_topic', '/mid360/livox/lidar')
        self.declare_parameter('report_period', 5.0)
        # header stamps of different sensors are allowed to be offset from
        # each other (sensor time vs. record time); the merged arrival order
        # must be non-decreasing within this tolerance
        self.declare_parameter('order_tolerance_ms', 250.0)

        self._tol = float(self.get_parameter('order_tolerance_ms').value) / 1e3
        self._lock = threading.Lock()
        self._stamps = {}       # topic -> list of header stamps (float s)
        self._arrival = {}      # topic -> list of wall arrival times
        self._bad_order = {}    # topic -> count of non-monotonic stamps
        self._arrival_seq = []  # header stamps in global arrival order

        qos = QoSProfile(depth=1000,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        pairs = [p for p in self.get_parameter('topics').value if p]
        self._topics = []
        for pair in pairs:
            topic, _, type_str = pair.partition(':')
            if not type_str:
                raise RuntimeError(
                    f"topic '{pair}' must be given as topic:ros2/type")
            cls = get_message(type_str)
            self._topics.append(topic)
            self._stamps[topic] = []
            self._arrival[topic] = []
            self._bad_order[topic] = 0
            # accumulate: deserialize via generic callback closure
            self.create_subscription(
                cls, topic,
                self._make_cb(topic, cls, type_str), qos)

        self._ref = self.get_parameter('reference_topic').value
        self._status_pub = self.create_publisher(
            String, '/stream_monitor/status', 10)
        period = float(self.get_parameter('report_period').value)
        if period > 0:
            self.create_timer(period, self._report)
        self.get_logger().info(f'monitoring {self._topics}')

    def _make_cb(self, topic, cls, type_str):
        def cb(msg):
            # msg is already the deserialized object
            hdr = getattr(msg, 'header', None)
            if hdr is None:
                stamp = time.time()
            else:
                stamp = hdr.stamp.sec + hdr.stamp.nanosec * 1e-9
            with self._lock:
                lst = self._stamps[topic]
                if lst and stamp < lst[-1]:
                    self._bad_order[topic] += 1
                lst.append(stamp)
                self._arrival[topic].append(time.monotonic())
                self._arrival_seq.append(stamp)
        return cb

    # ------------------------------------------------------------- reporting
    def _report(self):
        with self._lock:
            data = self._snapshot()
        msg = String()
        msg.data = json.dumps(data)
        self._status_pub.publish(msg)
        lines = [f"stream monitor @ {data['wall_time']:.1f}s"]
        for t in self._topics:
            s = data['topics'][t]
            lines.append(
                f"  {t:<26} n={s['count']:<6} rate={s['rate_hz']:7.2f} Hz "
                f"monotonic={'OK' if s['non_monotonic'] == 0 else 'FAIL'} "
                f"span={s['span_s']:.2f}s")
        for t, d in data['sync_to_reference_ms'].items():
            lines.append(f"  sync {self._ref} <-> {t}: "
                         f"median |dt| = {d['median_abs_ms']:.2f} ms, "
                         f"max = {d['max_abs_ms']:.2f} ms")
        if self._ref in data['topics']:
            lines.append(
                f"  global order check: "
                f"{'OK' if data['global_order']['ok'] else 'FAIL'} "
                f"(max inversion {data['global_order']['max_inversion_ms']} ms "
                f"within {data['global_order']['tolerance_ms']} ms tolerance)")
        self.get_logger().info('\n'.join(lines))

    def _snapshot(self):
        now = time.monotonic()
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
                'arrival_span_s': round(float(ar[-1] - ar[0]), 3) if n > 1 else 0.0,
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


def main(args=None):
    rclpy.init(args=args)
    node = StreamMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
