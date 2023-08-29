#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from wasp_autonomous_systems_interfaces.msg import Encoders
from control_msgs.msg import DynamicJointState

from math import pi


class EncoderPublisher(Node):

    def __init__(self):
        super().__init__('encoders')
        self._pub = self.create_publisher(Encoders, '/motor/encoders', 10)
        self._sub = self.create_subscription(
            DynamicJointState, '/dynamic_joint_states', self.odom_callback, 10)

        self._enc = Encoders()

    def odom_callback(self, msg: DynamicJointState):
        now = self.get_clock().now()
        last = Time().from_msg(self._enc.header.stamp)
        delta = now - last if last else 0

        self._enc.header.stamp = now.to_msg()

        left = int(msg.interface_values[1].values[0] / (2.0 * pi) * 3072.0)
        right = int(msg.interface_values[0].values[0] / (2.0 * pi) * 3072.0)

        self._enc.delta_encoder_left = left - self._enc.encoder_left
        self._enc.delta_encoder_right = right - self._enc.encoder_right

        self._enc.encoder_left = left
        self._enc.encoder_right = right
        self._enc.delta_time_left = delta.nanoseconds / 1000000.0
        self._enc.delta_time_right = delta.nanoseconds / 1000000.0

        self._pub.publish(self._enc)


def main():
    rclpy.init()
    node = EncoderPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
