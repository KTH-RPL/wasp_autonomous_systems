#!/usr/bin/env python
import signal

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped


class AltitudeManual(Node):

    def __init__(self):
        super().__init__('altitude_manual')

        self.declare_parameter('thrust', 0.0)
        self.declare_parameter('m1_offset', 0.0)
        self.declare_parameter('m2_offset', 0.0)
        self.declare_parameter('m3_offset', 0.0)
        self.declare_parameter('m4_offset', 0.0)

        self._thrust_pub = self.create_publisher(Float64MultiArray, 'thrust', 10)

        self.create_subscription(
            PointStamped, '/mavic_2_pro/gps', self.gps_callback, 10)

        self.shutting_down = False
        self._previous_time = None
        self._previous_z = 0.0
        self._last_thrust = 0.0
        self._last_velocity = 0.0

    def gps_callback(self, msg: PointStamped):
        if self.shutting_down:
            return

        # Track vertical velocity too, purely so land() below can brake with
        # it - not needed for the manual control itself.
        time = Time.from_msg(msg.header.stamp)
        if self._previous_time is not None:
            dt = (time - self._previous_time).nanoseconds / 10**9
            if dt:
                self._last_velocity = (msg.point.z - self._previous_z) / dt
        self._previous_time = time
        self._previous_z = msg.point.z

        # We get the thrust signal from rqt (or command line or some other place)
        thrust = self.get_parameter(
            'thrust').get_parameter_value().double_value
        self._last_thrust = thrust

        # We are treating the drone as a single-input single-output (SISO) system
        # but keep in mind that there are four input, the speed of each motor m1-m4
        m1 = thrust + \
            self.get_parameter('m1_offset').get_parameter_value().double_value
        m2 = thrust + \
            self.get_parameter('m2_offset').get_parameter_value().double_value
        m3 = thrust + \
            self.get_parameter('m3_offset').get_parameter_value().double_value
        m4 = thrust + \
            self.get_parameter('m4_offset').get_parameter_value().double_value

        u = Float64MultiArray()
        u.data = [thrust, m1 - thrust, m2 - thrust, m3 - thrust, m4 - thrust]
        self._thrust_pub.publish(u)

    def land(self):
        # Called from the SIGINT handler below, while rclpy's context is
        # still valid. A held constant thrust (this file has no height
        # feedback) can build real vertical velocity - cutting straight to
        # zero thrust would then let it free-fall and land hard enough to
        # tip over. Brake first: counter whatever vertical velocity was last
        # measured for a short window (using only the thrust value we were
        # already commanding, not any assumed/hardcoded hover value), then
        # cut power once it's settled.
        self.shutting_down = True
        K_BRAKE = 15.0
        u = Float64MultiArray()
        u.data = [self._last_thrust - K_BRAKE * self._last_velocity]
        for _ in range(20):
            self._thrust_pub.publish(u)
            rclpy.spin_once(self, timeout_sec=0.02)
        u.data = [0.0]
        for _ in range(5):
            self._thrust_pub.publish(u)
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    rclpy.init()
    node = AltitudeManual()

    # NOTE: rclpy.init() installs its own SIGINT handler that calls
    # rclpy.shutdown() as soon as Ctrl+C is pressed - by the time a plain
    # `except KeyboardInterrupt` around rclpy.spin() runs, the ROS context is
    # already invalid and any publish() in that handler silently fails. We
    # install our own handler *before* spinning instead, so we can publish
    # the final zero-thrust command while the context is still alive.
    stop = {'flag': False}

    def handle_sigint(signum, frame):
        stop['flag'] = True

    signal.signal(signal.SIGINT, handle_sigint)

    while rclpy.ok() and not stop['flag']:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.land()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
