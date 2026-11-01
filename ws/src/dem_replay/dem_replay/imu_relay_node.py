#!/usr/bin/env python
"""IMU unit relay (ROS 1 / Melodic).

The Livox Mid-360 driver publishes linear acceleration in ``g`` units while
ROS conventions (and FAST-LIO2) expect m/s^2.  This node republishes the
recorded/live IMU stream unchanged except for a configurable scale factor so
that estimators always see SI units.

It never modifies the original topic: raw data stays available for inspection.

Parameters
----------
~input_topic   : source IMU topic   (default /mid360/livox/imu)
~output_topic  : scaled IMU topic   (default /mid360/livox/imu_si)
~accel_scale   : multiply linear_acceleration by this (default 9.80665)
~gyro_scale    : multiply angular_velocity by this    (default 1.0)
"""

from __future__ import division

import rospy
from sensor_msgs.msg import Imu


class ImuRelayNode(object):

    def __init__(self):
        rospy.init_node('imu_relay')
        self._accel_scale = float(rospy.get_param('~accel_scale', 9.80665))
        self._gyro_scale = float(rospy.get_param('~gyro_scale', 1.0))

        in_topic = rospy.get_param('~input_topic', '/mid360/livox/imu')
        out_topic = rospy.get_param('~output_topic',
                                    '/mid360/livox/imu_si')
        self._pub = rospy.Publisher(out_topic, Imu, queue_size=200)
        self._sub = rospy.Subscriber(in_topic, Imu, self._cb, queue_size=200)
        rospy.loginfo('imu_relay: %s -> %s (accel x%s, gyro x%s)',
                      in_topic, out_topic, self._accel_scale, self._gyro_scale)

    def _cb(self, msg):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity.x = msg.angular_velocity.x * self._gyro_scale
        out.angular_velocity.y = msg.angular_velocity.y * self._gyro_scale
        out.angular_velocity.z = msg.angular_velocity.z * self._gyro_scale
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration.x = \
            msg.linear_acceleration.x * self._accel_scale
        out.linear_acceleration.y = \
            msg.linear_acceleration.y * self._accel_scale
        out.linear_acceleration.z = \
            msg.linear_acceleration.z * self._accel_scale
        out.linear_acceleration_covariance = \
            msg.linear_acceleration_covariance
        self._pub.publish(out)


def main():
    ImuRelayNode()
    rospy.spin()


if __name__ == '__main__':
    main()
