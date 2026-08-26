#!/usr/bin/env python
import signal

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped


class AltitudeHold(Node):

    def __init__(self):
        super().__init__('altitude_hold')

        # Declare a ROS parameter that we can read at runtime
        self.declare_parameter('target_height', 0.0)

        # Initialize the thrust publisher
        self._thrust_pub = self.create_publisher(Float64MultiArray, 'thrust', 10)

        # Subscribe to GPS topic (to get position) and call callback function on each recieved message
        self.create_subscription(
            PointStamped, '/mavic_2_pro/gps', self.gps_callback, 10)

        # Store the time last message was recieved
        self._previous_time = None
        self._previous_z = 0.0

        self.shutting_down = False
        self._last_thrust = 0.0
        self._last_velocity = 0.0

    def gps_callback(self, msg: PointStamped):
        """Takes GPS position and adjusts thrust.

        This function is called every time the GPS position is updated (i.e., when a message is published on the '/mavic_2_pro/gps' topic).

        Your task is to update the thrust based on the GPS position in 'msg' and the target height that you can read from the ROS parameter server.

        You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- An GPS ROS message. To see more information about it run 'ros2 interface show geometry_msgs/msg/PointStamped' in a terminal.
        """
        if self.shutting_down:
            return

        if not self._previous_time:
            # Wait until we have at least recieved two messages
            self._previous_time = Time.from_msg(msg.header.stamp)
            self._previous_z = msg.point.z
            return

        # Check delta time in seconds since last message
        time = Time.from_msg(msg.header.stamp)
        delta_time = (time - self._previous_time).nanoseconds / 10**9
        if delta_time:
            self._last_velocity = (msg.point.z - self._previous_z) / delta_time
        self._previous_time = time
        self._previous_z = msg.point.z

        # Get the target height from the ROS parameter server
        target_height = self.get_parameter(
            'target_height').get_parameter_value().double_value

        # TODO: Fill in

        thrust = 0.0  # TODO: Fill in

        self._last_thrust = thrust

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
    node = AltitudeHold()

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
