#!/usr/bin/env python
"""Record a pose/odometry topic to CSV while the pipeline runs (ROS 1).

FAST-LIO2's trajectory is computed *live* - it is not in the bag - so the
georeferencing step afterwards needs somebody to write it down.  This node
subscribes to ``/Odometry`` (or any pose-like topic) and appends one CSV row
per message:

    t,x,y,z,qx,qy,qz,qw

``t`` is the **message header stamp** (epoch seconds when replaying a bag),
which is what the GNSS track in the bag uses, so the two can be matched
without any clock games.

Usage
-----
    rosrun dem_replay record_odom --out /path/odom.csv
    rosrun dem_replay record_odom --topic /Odometry --out odom.csv \
        --every 0.1

Stop it with Ctrl-C (or SIGINT from a script) - the file is flushed and
closed properly on shutdown.
"""

from __future__ import division

import argparse
import sys

import rospy


class OdomRecorder(object):

    def __init__(self, topic, msgtype, out_path, every, max_rows):
        rospy.init_node('record_odom')
        self.topic = topic
        self.out_path = out_path
        self.every = every              # min seconds between rows (0 = all)
        self.max_rows = max_rows
        self.rows = 0
        self.last_written = None
        self._closed = False
        self._fh = open(out_path, 'w')  # line buffered below
        self._fh.write('t,x,y,z,qx,qy,qz,qw\n')
        self._fh.flush()

        from roslib.message import get_message_class
        cls = get_message_class(msgtype.replace('/msg/', '/'))
        if cls is None:
            raise SystemExit('unknown message type: {}'.format(msgtype))
        self._sub = rospy.Subscriber(topic, cls, self._cb, queue_size=200)
        rospy.on_shutdown(self.close)
        rospy.loginfo('recording %s (%s) -> %s', topic, cls.__name__,
                      out_path)

    def _cb(self, msg):
        name = type(msg).__name__
        if name in ('Odometry', 'PoseWithCovarianceStamped'):
            t = msg.header.stamp.to_sec()
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
        else:                                  # PoseStamped & friends
            t = msg.header.stamp.to_sec()
            p, q = msg.pose.position, msg.pose.orientation

        if self.every and self.last_written is not None \
                and (t - self.last_written) < self.every:
            return
        self.last_written = t
        self._fh.write(
            '{:.9f},{:.6f},{:.6f},{:.6f},'
            '{:.9f},{:.9f},{:.9f},{:.9f}\n'.format(
                t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        self._fh.flush()
        self.rows += 1
        if self.max_rows and self.rows >= self.max_rows:
            rospy.loginfo('row limit %s reached', self.max_rows)
            rospy.signal_shutdown('row limit reached')

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self._fh.close()
        except Exception:
            pass
        rospy.loginfo('wrote %s rows to %s', self.rows, self.out_path)


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='record_odom', description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/Odometry')
    ap.add_argument('--type', default='nav_msgs/Odometry',
                    help='message type of the pose topic')
    ap.add_argument('--out', required=True, help='output CSV path')
    ap.add_argument('--every', type=float, default=0.0,
                    help='write at most one row per N seconds (0 = all)')
    ap.add_argument('--max-rows', type=int, default=0)
    # rosrun appends remapping arguments (__name:=..., _param:=...)
    args, _ros_args = ap.parse_known_args(argv)

    node = OdomRecorder(args.topic, args.type, args.out, args.every,
                        args.max_rows)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
