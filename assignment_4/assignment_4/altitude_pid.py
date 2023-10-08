#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from wasp_autonomous_systems_interfaces.msg import Thrust
from geometry_msgs.msg import PointStamped


class AltitudePID(Node):

    def __init__(self):
        super().__init__('altitude_pid')

        self.declare_parameter('r', 0.0)
        self.declare_parameter('kp', 0.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('u_feedforward', 0.0)

        self._thrust_pub = self.create_publisher(Thrust, 'thrust', 10)

        self.create_subscription(
            PointStamped, '/mavic_2_pro/gps', self.gps_callback, 10)

        self._previous_time = None
        self._integral = 0
        self._previous_z = 0

    def gps_callback(self, msg: PointStamped):
        # The control signal message
        u = Thrust()
        u.header.stamp = msg.header.stamp

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

        # We are treatinh the drone as a single-input single output system but
        # keep in mind that there are four input, the speed of each motor m1-m4
        u.m1 = thrust
        u.m2 = thrust
        u.m3 = thrust
        u.m4 = thrust

        self._thrust_pub.publish(u)


def main():
    rclpy.init()
    node = AltitudePID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
