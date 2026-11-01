#!/usr/bin/env python
"""Timestamp-preserving ROS 1 bag replay node.

Replays a ROS 1 ``.bag`` as a live sensor stream:

* messages are read with one merged, time-ordered pass over every selected
  topic (never per-topic chunking), preserving the recorded temporal order,
* original header stamps are preserved verbatim,
* wall-clock pacing is scaled by ``rate`` (0.5x, 1x, 2x, ...),
* an optional ``/clock`` stream maps the recorded time onto the ROS clock so
  downstream nodes may use ``use_sim_time``,
* playback can be paused / resumed / restarted and the rate changed at
  runtime through services and topics,
* a JSON status topic reports progress and per-topic publish counters.

Services (std_srvs/Trigger):
    ~/pause, ~/resume, ~/restart, ~/toggle   (also mirrored on /replay/*)

Topics:
    /replay/rate (std_msgs/Float64)   set playback speed factor
    /replay/status (std_msgs/String)  JSON progress/statistics
    /clock (rosgraph_msgs/Clock)      when ``publish_clock`` is true
"""

from __future__ import division

import json
import threading
import time

import rosbag
import rospy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger, TriggerResponse


def _now():
    """Wall clock in seconds (``time.monotonic`` does not exist on Py2)."""
    return time.time()


class ReplayNode(object):
    """Replays a ROS 1 bag in original temporal order at a configurable rate."""

    # ------------------------------------------------------------------ setup
    def __init__(self):
        rospy.init_node('replay')

        self._bag_path = rospy.get_param('~bag_path', '')
        if not self._bag_path:
            raise RuntimeError("parameter 'bag_path' is required")

        topics = [t for t in (rospy.get_param('~topics', ['']) or []) if t]
        self._topics = topics or None
        self._rate = float(rospy.get_param('~rate', 1.0))
        self._start_offset = float(rospy.get_param('~start_offset', 0.0))
        self._publish_clock = bool(rospy.get_param('~publish_clock', True))
        self._loop = bool(rospy.get_param('~loop', False))
        self._auto_start = bool(rospy.get_param('~auto_start', True))

        self._cond = threading.Condition()
        self._state = 'starting'          # playing | paused | finished
        self._restart_req = False
        self._shutdown = False
        self._pause_enter = None
        self._base_wall = _now()
        self._last_clock = None

        # statistics
        self._t0_bag = None               # first bag timestamp (s)
        self._t_last_bag = None           # last read bag timestamp (s)
        self._bag_duration = self._bag_duration_from_bag()
        self._published = {}
        self._published_total = 0
        self._pushback = None

        self._bag = None
        self._out_pubs = {}
        self._available = {}
        self._given_types = {}

        for name in ('pause', 'resume', 'restart', 'toggle'):
            cb = getattr(self, '_srv_' + name)
            rospy.Service('/replay/' + name, Trigger, cb)
            rospy.Service('~/' + name, Trigger, cb)
        self._sub_rate = rospy.Subscriber(
            '/replay/rate', Float64, self._on_rate, queue_size=10)
        self._sub_rate_local = rospy.Subscriber(
            'rate', Float64, self._on_rate, queue_size=10)

        self._status_pub = rospy.Publisher(
            '/replay/status', String, queue_size=10)
        self._clock_pub = (rospy.Publisher('/clock', Clock, queue_size=10)
                           if self._publish_clock else None)
        status_rate = float(rospy.get_param('~status_rate', 5.0))
        if status_rate > 0.0:
            self._status_timer = rospy.Timer(
                rospy.Duration(1.0 / status_rate), self._publish_status)

        self._worker = threading.Thread(target=self._run)
        self._worker.daemon = True
        self._worker.start()

        rospy.on_shutdown(self.destroy)
        rospy.loginfo(
            'replay: bag=%s rate=%sx topics=%s duration=%.2fs',
            self._bag_path, self._rate, self._topics or 'ALL',
            self._bag_duration)

    # ------------------------------------------------------------- bag helper
    def _open_reader(self):
        bag = rosbag.Bag(self._bag_path, 'r')
        info = bag.get_type_and_topic_info().topics
        available = dict((name, t.msg_type) for name, t in info.items())
        if self._topics:
            missing = [t for t in self._topics if t not in available]
            if missing:
                bag.close()
                raise RuntimeError('topics not in bag: {}'.format(missing))
        self._available = available
        return bag

    def _bag_duration_from_bag(self):
        try:
            info = rosbag.Bag(self._bag_path, 'r')
            try:
                return float(info.get_end_time() - info.get_start_time())
            finally:
                info.close()
        except Exception:                                      # pragma: no cover
            return 0.0

    def _open(self):
        """(Re)open the bag, honour start_offset, reset pacing state."""
        if self._bag is not None:
            try:
                self._bag.close()
            except Exception:
                pass
        self._bag = self._open_reader()
        self._t0_bag = None
        self._t_last_bag = None
        self._pushback = None
        self._last_clock = None
        self._base_wall = _now()

        # messages are read through a generator so start_offset can be
        # applied without dropping the first message after the offset
        start = self._bag.get_start_time() + self._start_offset
        self._reader = self._bag.read_messages(
            topics=self._topics, start_time=rospy.Time.from_sec(start))
        rospy.loginfo('replay (re)started')

    def _publisher_for(self, topic, msg_type):
        pub = self._out_pubs.get(topic)
        if pub is None:
            from roslib.message import get_message_class
            cls = get_message_class(msg_type)
            if cls is None:
                raise RuntimeError('unknown message type {} on {}'.format(
                    msg_type, topic))
            pub = rospy.Publisher(topic, cls, queue_size=100)
            self._out_pubs[topic] = pub
        return pub

    # --------------------------------------------------------------- services
    def _srv_pause(self, _req):
        res = TriggerResponse()
        with self._cond:
            if self._state == 'playing':
                self._state = 'paused'
                self._pause_enter = _now()
            res.success = self._state == 'paused'
            res.message = self._state
            self._cond.notify_all()
        return res

    def _srv_resume(self, _req):
        res = TriggerResponse()
        with self._cond:
            if self._state == 'paused':
                self._base_wall += _now() - self._pause_enter
                self._pause_enter = None
                self._state = 'playing'
            elif self._state == 'finished':
                self._restart_req = True
                self._state = 'playing'
            res.success = self._state == 'playing'
            res.message = self._state
            self._cond.notify_all()
        return res

    def _srv_restart(self, _req):
        res = TriggerResponse()
        with self._cond:
            self._restart_req = True
            if self._state == 'finished':
                self._state = 'playing'
            res.success = True
            res.message = 'restarting'
            self._cond.notify_all()
        return res

    def _srv_toggle(self, _req):
        res = TriggerResponse()
        with self._cond:
            if self._state == 'playing':
                self._state = 'paused'
                self._pause_enter = _now()
            else:
                if self._state == 'paused':
                    self._base_wall += _now() - self._pause_enter
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
            rospy.logwarn('ignoring invalid rate %s', new_rate)
            return
        with self._cond:
            if self._t_last_bag is not None and self._t0_bag is not None:
                elapsed = self._t_last_bag - self._t0_bag
                # keep the playback position continuous across the rate change
                self._base_wall = _now() - elapsed / new_rate
            self._rate = new_rate
            self._cond.notify_all()
        rospy.loginfo('playback rate set to %sx', new_rate)

    # ------------------------------------------------------------ worker loop
    def _run(self):
        with self._cond:
            self._state = 'playing' if self._auto_start else 'paused'
            if self._state == 'paused':
                self._pause_enter = _now()
            self._cond.notify_all()

        while not rospy.is_shutdown() and not self._shutdown:
            # ---- (re)start handling ------------------------------------
            with self._cond:
                restart = self._restart_req
                self._restart_req = False
            if restart or self._bag is None:
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
                topic, msg, bag_ts = self._pushback
                self._pushback = None
            else:
                try:
                    topic, msg, t = next(self._reader)
                    bag_ts = t.to_sec()
                except StopIteration:
                    if self._loop:
                        continue        # reopen at top of loop
                    with self._cond:
                        self._state = 'finished'
                    rospy.loginfo('replay finished')
                    with self._cond:
                        while not self._restart_req and not self._shutdown \
                                and not rospy.is_shutdown():
                            self._cond.wait(0.5)
                        if self._shutdown:
                            break
                    continue

            if self._t0_bag is None:
                self._t0_bag = bag_ts
                self._base_wall = _now()

            self._t_last_bag = bag_ts

            # ---- pace against wall clock --------------------------------
            if not self._wait_paced(bag_ts):
                continue                # restart/shutdown requested

            # ---- publish (original message, original stamps) ------------
            pub = self._publisher_for(topic, self._available[topic])
            pub.publish(msg)

            self._published[topic] = self._published.get(topic, 0) + 1
            self._published_total += 1

            if self._clock_pub is not None:
                clk = Clock()
                clk.clock = rospy.Time.from_sec(bag_ts)
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
            target = base + (bag_ts - self._t0_bag) / rate
            now = _now()
            if now >= target:
                return True
            time.sleep(min(0.02, target - now))

    # --------------------------------------------------------------- status
    def _publish_status(self, _event=None):
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
            elapsed = tlast - t0
            if self._bag_duration > 0.0:
                progress = min(1.0, elapsed / self._bag_duration)
        now = _now()
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
        if self._bag is not None:
            try:
                self._bag.close()
            except Exception:
                pass


def main():
    ReplayNode()
    rospy.spin()


if __name__ == '__main__':
    main()
