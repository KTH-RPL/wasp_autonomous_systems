#!/usr/bin/env python
import signal

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped


class AltitudePID(Node):

    def __init__(self):
        super().__init__('altitude_pid')

        self.declare_parameter('r', 0.0)
        self.declare_parameter('kp', 0.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('u_feedforward', 0.0)

        self._thrust_pub = self.create_publisher(Float64MultiArray, 'thrust', 10)

        self.create_subscription(
            PointStamped, '/mavic_2_pro/gps', self.gps_callback, 10)

        self._previous_time = None
        self._integral = 0
        self._previous_z = 0

        self.shutting_down = False
        self._last_thrust = 0.0
        self._last_velocity = 0.0

    def gps_callback(self, msg: PointStamped):
        if self.shutting_down:
            return

        # Initialize varaible that allow us to calculate deltas in time and z
        if not self._previous_time:
            self._previous_time = Time.from_msg(msg.header.stamp)
            self._previous_z = msg.point.z
            return

        time = Time.from_msg(msg.header.stamp)
        dt = (time - self._previous_time).nanoseconds / 10**9

        r = self.get_parameter(
            'r').get_parameter_value().double_value
        u_feedforward = self.get_parameter(
            'u_feedforward').get_parameter_value().double_value
        kp = self.get_parameter('kp').get_parameter_value().double_value
        ki = self.get_parameter('ki').get_parameter_value().double_value
        kd = self.get_parameter('kd').get_parameter_value().double_value

        # The height error
        error = r - msg.point.z

        # The P-part which is proportional to the error
        proportional = error

        # The integral part which integrates the error if we are using the I-part (ki not 0)
        self._integral = self._integral + error * dt if ki else 0

        # The derivative part which we calculate unless for some reason dt=0.
        # In a proper implementation you would here typically add a low pass filter
        # to avoid too high signals as a result of noise but also changes in the
        # reference signal.
        #
        # A common way to approximate the derivate is by the differiential
        # (error - previous_error)/dt which can be rewritten as
        # (r - previous_r)/dt - (z - previous_z)/dt
        #
        # Since we will work with step changes in the reference signal
        # we remove the reference signal from the derivate by noting that the first term
        # will be zero except when the reference changes in which case the derivate will be
        # huge which is what we want to filter away. We therefore drop that part and end up with
        # the following where we handle the case where dt=0 also
        derivative = -(msg.point.z - self._previous_z) / dt if dt else 0

        # Put the pieces together with u_feedforward + PID
        thrust = u_feedforward + kp * proportional + \
            ki * self._integral + kd * derivative

        # Store previous values for time and z to calculate dt and dz/dt respectively
        self._previous_time = time
        self._previous_z = msg.point.z

        self._last_thrust = thrust
        self._last_velocity = -derivative  # derivative above is -dz/dt

        # We are treatinh the drone as a single-input single output system but
        # keep in mind that there are four input, the speed of each motor m1-m4
        u = Float64MultiArray()
        u.data = [thrust]
        self._thrust_pub.publish(u)

    def land(self):
        # Called from the SIGINT handler below, while rclpy's context is
        # still valid. Brake first: counter whatever vertical velocity was
        # last measured for a short window (using only the thrust value we
        # were already commanding, not any assumed/hardcoded hover value),
        # then cut power once it's settled - cutting straight to zero thrust
        # from a fast climb/descent could otherwise land hard enough to tip
        # the drone over.
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
    node = AltitudePID()

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
