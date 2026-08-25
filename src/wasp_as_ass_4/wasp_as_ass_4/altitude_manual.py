#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry


class AltitudeManual(Node):

    def __init__(self):
        super().__init__('altitude_manual')

        self.declare_parameter('thrust', 0.0)
        self.declare_parameter('m1_offset', 0.0)
        self.declare_parameter('m2_offset', 0.0)
        self.declare_parameter('m3_offset', 0.0)
        self.declare_parameter('m4_offset', 0.0)

        self._thrust_pub = self.create_publisher(Actuators, 'drone/motor_speed', 10)

        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        # The control signal message
        u = Actuators()
        u.header.stamp = msg.header.stamp

        # We get the thrust signal from rqt (or command line or some other place)
        thrust = self.get_parameter(
            'thrust').get_parameter_value().double_value

        # We are treating the drone as a single-input single-output (SISO) system
        # but keep in mind that there are four input, the speed of each motor m1-m4
        u.velocity = [
            thrust +
            self.get_parameter('m1_offset').get_parameter_value().double_value,
            thrust +
            self.get_parameter('m2_offset').get_parameter_value().double_value,
            thrust +
            self.get_parameter('m3_offset').get_parameter_value().double_value,
            thrust +
            self.get_parameter('m4_offset').get_parameter_value().double_value,
        ]

        self._thrust_pub.publish(u)


def main():
    rclpy.init()
    node = AltitudeManual()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
