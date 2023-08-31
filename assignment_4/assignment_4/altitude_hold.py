#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from wasp_autonomous_systems_interfaces.msg import Thrust
from geometry_msgs.msg import PointStamped


class AltitudeHold(Node):

    def __init__(self):
        super().__init__('altitude_hold')

        # Declare a ROS parameter that we can read at runtime
        self.declare_parameter('target_height', 0.0)

        # Initialize the thrust publisher
        self._thrust_pub = self.create_publisher(Thrust, 'thrust', 10)

        # Subscribe to GPS topic (to get position) and call callback function on each recieved message
        self.create_subscription(
            PointStamped, '/Mavic_2_PRO/gps', self.gps_callback, 10)

        # Store the time last message was recieved
        self._previous_time = None

    def gps_callback(self, msg: PointStamped):
        """Takes GPS position and adjusts thrust.

        This function is called every time the GPS position is updated (i.e., when a message is published on the '/Mavic_2_PRO/gps' topic).

        Your task is to update the thrust based on the GPS position in 'msg' and the target height that you can read from the ROS parameter server. 

        You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- An GPS ROS message. To see more information about it run 'ros2 interface show geometry_msgs/msg/PointStamped' in a terminal.
        """

        thrust = Thrust()
        thrust.header.stamp = msg.header.stamp

        if not self._previous_time:
            # Wait until we have at least recieved two messages
            self._previous_time = Time.from_msg(msg.header.stamp)
            return

        # Check delta time in seconds since last message
        time = Time.from_msg(msg.header.stamp)
        delta_time = (time - self._previous_time).nanoseconds / 10**9
        self._previous_time = time

        # Get the target height from the ROS parameter server
        target_height = self.get_parameter(
            'target_height').get_parameter_value().double_value

        # TODO: Fill in

        thrust.m1 = 0.0  # TODO: Fill in
        thrust.m2 = 0.0  # TODO: Fill in
        thrust.m3 = 0.0  # TODO: Fill in
        thrust.m4 = 0.0  # TODO: Fill in

        self._thrust_pub.publish(thrust)


def main():
    rclpy.init()
    node = AltitudeHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
