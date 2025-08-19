#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from wasp_autonomous_systems_interfaces.msg import Collision
from geometry_msgs.msg import TwistStamped

from random import getrandbits


class AutonomousController(Node):

    def __init__(self):
        super().__init__('autonomous_controller')
        self._pub = self.create_publisher(TwistStamped, '/diffdrive_controller/cmd_vel', 10)
        self.create_subscription(
            Collision, '/collision_detected', self.collision_callback, 10)
        self._back = 0
        self._left = 0
        self._right = 0
        timer_period = 0.01  # seconds
        self._timer = self.create_timer(
            timer_period, self.timer_callback, clock=self.get_clock())
        # self._timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        '''This Function is called at a fixed interval'''

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._back:
            # Move backwards
            msg.twist.linear.x = -0.3
            self._back -= 1
        elif self._left:
            # Turn left
            msg.twist.angular.z = 1.0
            self._left -= 1
        elif self._right:
            # Turn right
            msg.twist.angular.z = -1.0
            self._right -= 1
        else:
            # Move forward
            msg.twist.linear.x = 0.3
        # Publish command
        self._pub.publish(msg)

    def collision_callback(self, msg: Collision):
        '''This function is called every time the collision detection topic is published to'''

        if self._left or self._right:
            # We are currently already moving away from a previous collision, disregard this message
            return

        if msg.collision:
            # There was a collision
            # Move backwards for 100 time steps
            self._back = 100
            # Flip a coin to see if we should turn left or right
            if getrandbits(1):
                # Turn left for 200 time steps
                self._left = 75
                self._right = 0
            else:
                # Turn right for 200 time steps
                self._left = 0
                self._right = 75


def main():
    rclpy.init()
    node = AutonomousController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
