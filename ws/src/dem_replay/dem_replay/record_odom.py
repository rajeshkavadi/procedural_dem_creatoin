#!/usr/bin/env python3
"""Record a pose/odometry topic to CSV while the pipeline runs.

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
    ros2 run dem_replay record_odom -- --out /path/odom.csv
    ros2 run dem_replay record_odom -- --topic /Odometry --out odom.csv \
        --every 0.1

Stop it with Ctrl-C (or SIGINT from a script) - the file is flushed and
closed properly on shutdown.
"""

import argparse
import signal
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


class OdomRecorder(Node):

    def __init__(self, topic, msgtype, out_path, every, max_rows):
        super().__init__('record_odom')
        self.topic = topic
        self.out_path = out_path
        self.every = every              # min seconds between rows (0 = all)
        self.max_rows = max_rows
        self.rows = 0
        self.last_written = None
        self.started = time.time()
        self._fh = open(out_path, 'w', buffering=1)   # line buffered
        self._fh.write('t,x,y,z,qx,qy,qz,qw\n')

        cls = get_message(msgtype)
        self.create_subscription(cls, topic, self._cb, 200)
        self.get_logger().info(f'recording {topic} ({cls.__name__}) '
                               f'-> {out_path}')

    def _cb(self, msg):
        name = type(msg).__name__
        if name == 'Odometry':
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
        elif name == 'PoseWithCovarianceStamped':
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
        else:                                  # PoseStamped & friends
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p, q = msg.pose.position, msg.pose.orientation

        if self.every and self.last_written is not None \
                and (t - self.last_written) < self.every:
            return
        self.last_written = t
        self._fh.write(f'{t:.9f},{p.x:.6f},{p.y:.6f},{p.z:.6f},'
                       f'{q.x:.9f},{q.y:.9f},{q.z:.9f},{q.w:.9f}\n')
        self.rows += 1
        if self.max_rows and self.rows >= self.max_rows:
            self.get_logger().info(f'row limit {self.max_rows} reached')
            rclpy.shutdown()

    def close(self):
        try:
            self._fh.close()
        except Exception:
            pass
        self.get_logger().info(f'wrote {self.rows} rows to {self.out_path}')


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='record_odom', description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/Odometry')
    ap.add_argument('--type', default='nav_msgs/msg/Odometry',
                    help='message type of the pose topic')
    ap.add_argument('--out', required=True, help='output CSV path')
    ap.add_argument('--every', type=float, default=0.0,
                    help='write at most one row per N seconds (0 = all)')
    ap.add_argument('--max-rows', type=int, default=0)
    # rclpy consumes its own args (e.g. --ros-args); keep them intact
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = OdomRecorder(args.topic, args.type, args.out, args.every,
                        args.max_rows)

    def _sigint(_sig, _frm):
        node.close()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:       # rclpy already shut down
        pass
    finally:
        node.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
