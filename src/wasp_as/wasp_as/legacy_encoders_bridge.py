#!/usr/bin/env python
"""Bridges the pre-rename wasp_autonomous_systems_interfaces message types
(as recorded in rosbags/real_robot) to the current wasp_as_interfaces types,
so student nodes (e.g. odometry.py) can subscribe to the modern type
without needing to know about the legacy rename. See
src/wasp_autonomous_systems_interfaces/package.xml for the full story.
"""

import rclpy
from rclpy.node import Node

from wasp_as_interfaces.msg import DutyCycles, Encoders
from wasp_autonomous_systems_interfaces.msg import DutyCycles as LegacyDutyCycles
from wasp_autonomous_systems_interfaces.msg import Encoders as LegacyEncoders


class LegacyEncodersBridge(Node):

    def __init__(self):
        super().__init__('legacy_encoders_bridge')

        self._encoders_pub = self.create_publisher(Encoders, '/motor/encoders', 10)
        self.create_subscription(
            LegacyEncoders, '/motor/encoders_legacy', self.encoders_callback, 10)

        self._duty_cycles_pub = self.create_publisher(DutyCycles, '/motor/duty_cyles', 10)
        self.create_subscription(
            LegacyDutyCycles, '/motor/duty_cyles_legacy', self.duty_cycles_callback, 10)

    def encoders_callback(self, msg: LegacyEncoders):
        out = Encoders()
        out.header = msg.header
        out.encoder_left = msg.encoder_left
        out.encoder_right = msg.encoder_right
        out.delta_encoder_left = msg.delta_encoder_left
        out.delta_encoder_right = msg.delta_encoder_right
        self._encoders_pub.publish(out)

    def duty_cycles_callback(self, msg: LegacyDutyCycles):
        out = DutyCycles()
        out.header = msg.header
        out.duty_cycle_left = msg.duty_cycle_left
        out.duty_cycle_right = msg.duty_cycle_right
        self._duty_cycles_pub.publish(out)


def main():
    rclpy.init()
    node = LegacyEncodersBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
