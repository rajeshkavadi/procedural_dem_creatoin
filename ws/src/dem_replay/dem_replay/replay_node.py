#!/usr/bin/env python3
"""Timestamp-preserving ROS 2 bag replay node.

Replays a ROS 2 bag as a live sensor stream:

* messages are read with one merged, time-ordered pass over every selected
  topic (never per-topic chunking), preserving the recorded temporal order,
* original header stamps are preserved verbatim,
* wall-clock pacing is scaled by ``rate`` (0.5x, 1x, 2x, ...),
* an optional ``/clock`` stream maps the recorded time onto the ROS clock so
  downstream nodes may use ``use_sim_time``,
* playback can be paused / resumed / restarted and the rate changed at
  runtime through ROS 2 services and topics,
* a JSON status topic reports progress and per-topic publish counters.

Services (std_srvs/Trigger):
    ~/pause, ~/resume, ~/restart, ~/toggle   (also mirrored on /replay/*)

Topics:
    /replay/rate (std_msgs/Float64)   set playback speed factor
    /replay/status (std_msgs/String)  JSON progress/statistics
    /clock (rosgraph_msgs/Clock)      when ``publish_clock`` is true
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

import rosbag2_py


class ReplayNode(Node):
    """Replays a ROS 2 bag in original temporal order at a configurable rate."""

    # ------------------------------------------------------------------ setup
    def __init__(self):
        super().__init__('replay')

        self.declare_parameter('bag_path', '')
        self.declare_parameter('storage_id', 'sqlite3')
        self.declare_parameter('topics', [''])           # empty -> every topic
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('start_offset', 0.0)      # seconds into the bag
        self.declare_parameter('publish_clock', True)
        self.declare_parameter('status_rate', 5.0)
        self.declare_parameter('loop', False)

        self._bag_path = self.get_parameter('bag_path').value
        if not self._bag_path:
            raise RuntimeError("parameter 'bag_path' is required")

        self._storage_id = self.get_parameter('storage_id').value
        topics = [t for t in self.get_parameter('topics').value if t]
        self._topics = topics or None
        self._rate = float(self.get_parameter('rate').value)
        self._start_offset = float(self.get_parameter('start_offset').value)
        self._publish_clock = bool(self.get_parameter('publish_clock').value)
        self._loop = bool(self.get_parameter('loop').value)

        self._cond = threading.Condition()
        self._state = 'starting'          # playing | paused | finished
        self._restart_req = False
        self._shutdown = False
        self._pause_enter = None
        self._base_wall = time.monotonic()
        self._last_clock = None

        # statistics
        self._t0_bag = None               # first bag timestamp (ns)
        self._t_last_bag = None           # last read bag timestamp (ns)
        self._bag_duration = self._bag_duration_from_metadata()
        self._published = {}
        self._published_total = 0
        self._pushback = None

        self._qos = QoSProfile(depth=100,
                               reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST)
        self._out_pubs = {}
        self._msg_types = {}
        self._available = {}

        for name in ('pause', 'resume', 'restart', 'toggle'):
            self.create_service(Trigger, f'/replay/{name}',
                                getattr(self, f'_srv_{name}'))
            self.create_service(Trigger, f'~/{name}',
                                getattr(self, f'_srv_{name}'))
        self.create_subscription(Float64, '/replay/rate', self._on_rate, 10)
        self.create_subscription(Float64, 'rate', self._on_rate, 10)

        self._status_pub = self.create_publisher(String, '/replay/status', 10)
        self._clock_pub = (self.create_publisher(Clock, '/clock', 10)
                           if self._publish_clock else None)
        status_rate = float(self.get_parameter('status_rate').value)
        if status_rate > 0.0:
            self.create_timer(1.0 / status_rate, self._publish_status)

        self._reader = None
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f'replay: bag={self._bag_path} rate={self._rate}x '
            f'topics={self._topics or "ALL"} '
            f'duration={self._bag_duration:.2f}s')

    # ------------------------------------------------------------- bag helper
    def _open_reader(self):
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=self._bag_path,
                                      storage_id=self._storage_id),
            rosbag2_py.ConverterOptions('', ''))
        available = {t.name: t.type for t in reader.get_all_topics_and_types()}
        if self._topics:
            missing = [t for t in self._topics if t not in available]
            if missing:
                raise RuntimeError(f'topics not in bag: {missing}')
            reader.set_filter(
                rosbag2_py.StorageFilter(topics=list(self._topics)))
        self._available = available
        return reader

    def _bag_duration_from_metadata(self):
        try:
            tmp = rosbag2_py.SequentialReader()
            tmp.open(rosbag2_py.StorageOptions(uri=self._bag_path,
                                               storage_id=self._storage_id),
                     rosbag2_py.ConverterOptions('', ''))
            return tmp.get_metadata().duration.nanoseconds / 1e9
        except Exception:                                      # pragma: no cover
            return 0.0

    def _open(self):
        """(Re)open the bag, honour start_offset, reset pacing state."""
        self._reader = self._open_reader()
        self._t0_bag = None
        self._t_last_bag = None
        self._pushback = None
        self._last_clock = None
        self._base_wall = time.monotonic()

        # skip forward to start_offset while keeping strict ordering
        if self._start_offset > 0.0 and self._reader.has_next():
            topic, data, ts = self._reader.read_next()
            t0 = ts
            offset_ns = self._start_offset * 1e9
            while ts - t0 < offset_ns and self._reader.has_next():
                topic, data, ts = self._reader.read_next()
            if ts - t0 >= offset_ns:
                self._pushback = (topic, data, ts)
        self.get_logger().info('replay (re)started')

    def _publisher_for(self, topic):
        pub = self._out_pubs.get(topic)
        if pub is None:
            cls = get_message(self._available[topic])
            self._msg_types[topic] = cls
            pub = self.create_publisher(cls, topic, self._qos)
            self._out_pubs[topic] = pub
        return pub

    # --------------------------------------------------------------- services
    def _srv_pause(self, _req, res):
        with self._cond:
            if self._state == 'playing':
                self._state = 'paused'
                self._pause_enter = time.monotonic()
            res.success = self._state == 'paused'
            res.message = self._state
            self._cond.notify_all()
        return res

    def _srv_resume(self, _req, res):
        with self._cond:
            if self._state == 'paused':
                self._base_wall += time.monotonic() - self._pause_enter
                self._pause_enter = None
                self._state = 'playing'
            elif self._state == 'finished':
                self._restart_req = True
                self._state = 'playing'
            res.success = self._state == 'playing'
            res.message = self._state
            self._cond.notify_all()
        return res

    def _srv_restart(self, _req, res):
        with self._cond:
            self._restart_req = True
            if self._state == 'finished':
                self._state = 'playing'
            res.success = True
            res.message = 'restarting'
            self._cond.notify_all()
        return res

    def _srv_toggle(self, _req, res):
        with self._cond:
            if self._state == 'playing':
                self._state = 'paused'
                self._pause_enter = time.monotonic()
            else:
                if self._state == 'paused':
                    self._base_wall += time.monotonic() - self._pause_enter
                    self._pause_enter = None
                if self._state == 'finished':
                    self._restart_req = True
                self._state = 'playing'
            res.success = True
            res.message = self._state
            self._cond.notify_all()
        return res

    def _on_rate(self, msg):
        new_rate = float(msg.data)
        if new_rate <= 0.0:
            self.get_logger().warn(f'ignoring invalid rate {new_rate}')
            return
        with self._cond:
            if self._t_last_bag is not None and self._t0_bag is not None:
                elapsed = (self._t_last_bag - self._t0_bag) / 1e9
                # keep the playback position continuous across the rate change
                self._base_wall = time.monotonic() - elapsed / new_rate
            self._rate = new_rate
            self._cond.notify_all()
        self.get_logger().info(f'playback rate set to {new_rate}x')

    # ------------------------------------------------------------ worker loop
    def _run(self):
        while not rclpy.ok():
            time.sleep(0.05)
        with self._cond:
            self._state = ('playing'
                           if self.get_parameter('auto_start').value
                           else 'paused')
            if self._state == 'paused':
                self._pause_enter = time.monotonic()
            self._cond.notify_all()

        while rclpy.ok() and not self._shutdown:
            # ---- (re)start handling ------------------------------------
            with self._cond:
                restart = self._restart_req
                self._restart_req = False
            if restart or self._reader is None:
                self._open()

            # ---- wait while paused -------------------------------------
            with self._cond:
                while self._state == 'paused' and not self._restart_req \
                        and not self._shutdown:
                    self._cond.wait(0.2)
                restart = self._restart_req
                self._restart_req = False
                if restart:
                    continue            # reopen at top of loop
                if self._shutdown:
                    break

            # ---- read one message --------------------------------------
            if self._pushback is not None:
                topic, data, bag_ts = self._pushback
                self._pushback = None
            elif self._reader.has_next():
                topic, data, bag_ts = self._reader.read_next()
            else:
                if self._loop:
                    continue            # reopen at top of loop
                with self._cond:
                    self._state = 'finished'
                self.get_logger().info('replay finished')
                with self._cond:
                    while not self._restart_req and not self._shutdown:
                        self._cond.wait(0.5)
                    if self._shutdown:
                        break
                continue

            if self._t0_bag is None:
                self._t0_bag = bag_ts
                self._base_wall = time.monotonic()

            self._t_last_bag = bag_ts

            # ---- pace against wall clock --------------------------------
            if not self._wait_paced(bag_ts):
                continue                # restart/shutdown requested

            # ---- publish (original message, original stamps) ------------
            cls = self._msg_types.get(topic)
            if cls is None:
                cls = get_message(self._available[topic])
                self._msg_types[topic] = cls
                self._out_pubs[topic] = self.create_publisher(
                    cls, topic, self._qos)
            msg = deserialize_message(data, cls)
            self._out_pubs[topic].publish(msg)

            self._published[topic] = self._published.get(topic, 0) + 1
            self._published_total += 1

            if self._clock_pub is not None:
                clk = Clock()
                clk.clock.sec = bag_ts // 1_000_000_000
                clk.clock.nanosec = bag_ts % 1_000_000_000
                self._clock_pub.publish(clk)
                self._last_clock = clk

    def _wait_paced(self, bag_ts):
        """Sleep until wall clock matches recorded time / rate.

        Returns False when a restart or shutdown interrupted the wait, in
        which case the current message must not be published.
        """
        while True:
            with self._cond:
                if self._restart_req or self._shutdown:
                    return False
                if self._state == 'paused':
                    self._cond.wait(0.1)
                    continue
                rate = self._rate
                base = self._base_wall
            target = base + (bag_ts - self._t0_bag) / 1e9 / rate
            now = time.monotonic()
            if now >= target:
                return True
            time.sleep(min(0.02, target - now))

    # --------------------------------------------------------------- status
    def _publish_status(self):
        with self._cond:
            state = self._state
            rate = self._rate
            t0, tlast = self._t0_bag, self._t_last_bag
            counts = dict(self._published)
            total = self._published_total
            base = self._base_wall
            pause_enter = self._pause_enter
        elapsed = 0.0
        progress = 0.0
        if t0 is not None and tlast is not None:
            elapsed = (tlast - t0) / 1e9
            if self._bag_duration > 0.0:
                progress = min(1.0, elapsed / self._bag_duration)
        now = time.monotonic()
        wall = (now - base) if pause_enter is None else (pause_enter - base)
        status = {
            'state': state,
            'rate': rate,
            'bag_path': self._bag_path,
            'bag_time_s': round(elapsed, 3),
            'bag_duration_s': round(self._bag_duration, 3),
            'progress': round(progress, 4),
            'messages_published': total,
            'per_topic': counts,
            'wall_time_s': round(wall, 3),
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)
        if self._clock_pub is not None and self._last_clock is not None:
            # heartbeat keeps /clock alive while paused or between messages
            self._clock_pub.publish(self._last_clock)

    # -------------------------------------------------------------- shutdown
    def destroy(self):
        with self._cond:
            self._shutdown = True
            self._cond.notify_all()


def main(args=None):
    rclpy.init(args=args)
    node = ReplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
