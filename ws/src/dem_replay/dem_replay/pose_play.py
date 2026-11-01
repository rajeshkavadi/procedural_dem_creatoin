#!/usr/bin/env python
"""Publish a stored pose stream, paced by /clock (for GNSS-aided DEM runs).

Reads a CSV written by ``dem_mapping gnss_warp`` (or any
``t,x,y,z,qx,qy,qz,qw`` file) and republishes it as
``nav_msgs/Odometry``.  Publishing is driven by the ``/clock`` topic of a
bag replay node, so the poses arrive exactly when the corresponding raw
scans do - ``dem_node`` in ``pose`` mode can then insert every scan at its
GNSS-aided position and the resulting grid lives directly in UTM.

If no ``/clock`` is seen within a grace period the node falls back to
wall-clock pacing relative to the first pose stamp.

Usage
-----
    rosrun dem_replay pose_play \
        --csv out/warped_odom.csv --topic /Odometry_warp
"""

from __future__ import division

import argparse
import sys
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


class PosePlayer(object):

    def __init__(self, csv_path, topic, frame_id, clock_grace):
        rospy.init_node('pose_play')
        data = np.genfromtxt(csv_path, delimiter=',', names=True)
        self._t = np.asarray(data['t'], dtype=np.float64)
        self._p = np.stack([data['x'], data['y'], data['z']], axis=1)
        self._q = np.stack([data['qx'], data['qy'], data['qz'],
                            data['qw']], axis=1)
        self._i = 0
        self._done = False
        self._clock_seen = None
        self._clock_grace = clock_grace
        self._started = time.time()
        self._frame = frame_id
        self._pub = rospy.Publisher(topic, Odometry, queue_size=50)
        self._sub = rospy.Subscriber('/clock', Clock, self._clock_cb,
                                     queue_size=10)
        # bounded timer instead of spin_once(): keeps the fallback and the
        # stall watchdog running even when no /clock message arrives
        self._timer = rospy.Timer(rospy.Duration(0.2), self._tick)
        rospy.loginfo(
            'pose_play: %s -> %s (%d poses, %.1f s), waiting for /clock ...',
            csv_path, topic, len(self._t), self._t[-1] - self._t[0])

    def _publish_upto(self, t):
        n = 0
        while self._i < len(self._t) and self._t[self._i] <= t:
            msg = Odometry()
            msg.header.stamp = rospy.Time.from_sec(float(self._t[self._i]))
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
        self._clock_seen = time.time()
        self._publish_upto(msg.clock.to_sec())
        if self._i >= len(self._t) and not self._done:
            self._done = True
            rospy.loginfo('pose_play: all %s poses sent', self._i)

    def _tick(self, _event=None):
        """Fallback pacing + stall watchdog."""
        now = time.time()
        if self._clock_seen is None and now - self._started > self._clock_grace:
            # no replay running: pace against wall clock instead
            wall_t = self._t[0] + (now - self._started)
            self._publish_upto(wall_t)
        elif self._clock_seen is not None and now - self._clock_seen > 10.0 \
                and self._i < len(self._t):
            # replay stalled/stopped: flush the rest so the DEM sees them
            rospy.logwarn('pose_play: /clock stopped, flushing remaining poses')
            self._publish_upto(self._t[-1] + 1.0)


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='pose_play', description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--csv', required=True)
    ap.add_argument('--topic', default='/Odometry_warp')
    ap.add_argument('--frame-id', default='utm')
    ap.add_argument('--clock-grace', type=float, default=15.0)
    args, _ros_args = ap.parse_known_args(argv)

    PosePlayer(args.csv, args.topic, args.frame_id, args.clock_grace)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
