#!/usr/bin/env python3
"""Publish a stored pose stream, paced by /clock (for GNSS-aided DEM runs).

Reads a CSV written by ``dem_mapping gnss_warp`` (or any
``t,x,y,z,qx,qy,qz,qw`` file) and republishes it as
``nav_msgs/msg/Odometry``.  Publishing is driven by the ``/clock`` topic of
a bag replay node, so the poses arrive exactly when the corresponding raw
scans do - ``dem_node`` in ``pose`` mode can then insert every scan at its
GNSS-aided position and the resulting grid lives directly in UTM.

If no ``/clock`` is seen within a grace period the node falls back to
wall-clock pacing relative to the first pose stamp.

Usage
-----
    ros2 run dem_replay pose_play -- \
        --csv out/warped_odom.csv --topic /Odometry_warp
"""

import argparse
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


class PosePlayer(Node):

    def __init__(self, csv_path, topic, frame_id, clock_grace):
        super().__init__('pose_play')
        data = np.genfromtxt(csv_path, delimiter=',', names=True)
        self._t = np.asarray(data['t'], dtype=np.float64)
        self._p = np.stack([data['x'], data['y'], data['z']], axis=1)
        self._q = np.stack([data['qx'], data['qy'], data['qz'],
                            data['qw']], axis=1)
        self._i = 0
        self._clock_seen = None
        self._clock_grace = clock_grace
        self._started = time.monotonic()
        self._frame = frame_id
        qos = QoSProfile(depth=50,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self._pub = self.create_publisher(Odometry, topic, qos)
        self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        self.get_logger().info(
            f'pose_play: {csv_path} -> {topic} '
            f'({len(self._t)} poses, {self._t[-1] - self._t[0]:.1f} s), '
            f'waiting for /clock ...')

    def _publish_upto(self, t):
        n = 0
        while self._i < len(self._t) and self._t[self._i] <= t:
            msg = Odometry()
            sec = float(self._t[self._i])
            msg.header.stamp.sec = int(sec)
            msg.header.stamp.nanosec = int(round((sec - int(sec)) * 1e9))
            msg.header.frame_id = self._frame
            msg.child_frame_id = 'base_link'
            msg.pose.pose.position.x = float(self._p[self._i, 0])
            msg.pose.pose.position.y = float(self._p[self._i, 1])
            msg.pose.pose.position.z = float(self._p[self._i, 2])
            msg.pose.pose.orientation.x = float(self._q[self._i, 0])
            msg.pose.pose.orientation.y = float(self._q[self._i, 1])
            msg.pose.pose.orientation.z = float(self._q[self._i, 2])
            msg.pose.pose.orientation.w = float(self._q[self._i, 3])
            self._pub.publish(msg)
            self._i += 1
            n += 1
        return n

    def _clock_cb(self, msg):
        self._clock_seen = time.monotonic()
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self._publish_upto(t)
        if self._i >= len(self._t) and not getattr(self, '_done', False):
            self._done = True
            self.get_logger().info(f'pose_play: all {self._i} poses sent')

    def tick(self):
        """Fallback + watchdog, called from the spin loop."""
        now = time.monotonic()
        if self._clock_seen is None and now - self._started > self._clock_grace:
            # no replay running: pace against wall clock instead
            wall_t = self._t[0] + (now - self._started)
            self._publish_upto(wall_t)
        elif self._clock_seen is not None and now - self._clock_seen > 10.0 \
                and self._i < len(self._t):
            # replay stalled/stopped: flush the rest so the DEM sees them
            self.get_logger().warn('pose_play: /clock stopped, flushing '
                                   'remaining poses')
            self._publish_upto(self._t[-1] + 1.0)


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='pose_play', description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--csv', required=True)
    ap.add_argument('--topic', default='/Odometry_warp')
    ap.add_argument('--frame-id', default='utm')
    ap.add_argument('--clock-grace', type=float, default=15.0)
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = PosePlayer(args.csv, args.topic, args.frame_id, args.clock_grace)

    def _sigint(_sig, _frm):
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _sigint)

    try:
        # bounded spin: an idle node must still process signals
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            node.tick()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
