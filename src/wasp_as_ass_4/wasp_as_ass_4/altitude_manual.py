#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from wasp_autonomous_systems_interfaces.msg import Thrust
from geometry_msgs.msg import PointStamped


class AltitudeManual(Node):

    def __init__(self):
        super().__init__('altitude_manual')

        self.declare_parameter('thrust', 0.0)
        self.declare_parameter('m1_offset', 0.0)
        self.declare_parameter('m2_offset', 0.0)
        self.declare_parameter('m3_offset', 0.0)
        self.declare_parameter('m4_offset', 0.0)

        self._thrust_pub = self.create_publisher(Thrust, 'thrust', 10)

        self.create_subscription(
            PointStamped, '/mavic_2_pro/gps', self.gps_callback, 10)

    def gps_callback(self, msg: PointStamped):
        # The control signal message
        u = Thrust()
        u.header.stamp = msg.header.stamp

        # We get the thrust signal from rqt (or command line or some other place)
        thrust = self.get_parameter(
            'thrust').get_parameter_value().double_value

        # We are treating the drone as a single-input single-output (SISO) system 
        # but keep in mind that there are four input, the speed of each motor m1-m4
        u.m1 = thrust + \
            self.get_parameter('m1_offset').get_parameter_value().double_value
        u.m2 = thrust + \
            self.get_parameter('m2_offset').get_parameter_value().double_value
        u.m3 = thrust + \
            self.get_parameter('m3_offset').get_parameter_value().double_value
        u.m4 = thrust + \
            self.get_parameter('m4_offset').get_parameter_value().double_value

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
