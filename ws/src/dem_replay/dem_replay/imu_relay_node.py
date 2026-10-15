#!/usr/bin/env python3
"""IMU unit relay.

The Livox Mid-360 driver publishes linear acceleration in ``g`` units while
ROS conventions (and FAST-LIO2) expect m/s^2.  This node republishes the
recorded/live IMU stream unchanged except for a configurable scale factor so
that estimators always see SI units.

It never modifies the original topic: raw data stays available for inspection.

Parameters
----------
input_topic   : source IMU topic   (default /mid360/livox/imu)
output_topic  : scaled IMU topic   (default /mid360/livox/imu_si)
accel_scale   : multiply linear_acceleration by this (default 9.80665)
gyro_scale    : multiply angular_velocity by this    (default 1.0)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import Imu


class ImuRelayNode(Node):

    def __init__(self):
        super().__init__('imu_relay')
        self.declare_parameter('input_topic', '/mid360/livox/imu')
        self.declare_parameter('output_topic', '/mid360/livox/imu_si')
        self.declare_parameter('accel_scale', 9.80665)
        self.declare_parameter('gyro_scale', 1.0)

        self._accel_scale = float(self.get_parameter('accel_scale').value)
        self._gyro_scale = float(self.get_parameter('gyro_scale').value)

        qos = QoSProfile(depth=200,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self._pub = self.create_publisher(Imu, out_topic, qos)
        self.create_subscription(Imu, in_topic, self._cb, qos)
        self.get_logger().info(
            f'imu_relay: {in_topic} -> {out_topic} '
            f'(accel x{self._accel_scale}, gyro x{self._gyro_scale})')

    def _cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity.x = msg.angular_velocity.x * self._gyro_scale
        out.angular_velocity.y = msg.angular_velocity.y * self._gyro_scale
        out.angular_velocity.z = msg.angular_velocity.z * self._gyro_scale
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration.x = msg.linear_acceleration.x * self._accel_scale
        out.linear_acceleration.y = msg.linear_acceleration.y * self._accel_scale
        out.linear_acceleration.z = msg.linear_acceleration.z * self._accel_scale
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRelayNode()
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
