#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from wasp_autonomous_systems_interfaces.msg import Collision


class CollisionDetector(Node):

    def __init__(self):
        super().__init__('collision_detector')
        self._pub = self.create_publisher(Collision, 'collision_detected', 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

    def imu_callback(self, msg: Imu):
        """Uses IMU to determine if a collision occured.

        This function is called every time the imu is updated (i.e., when a message is published on the '/imu' topic).

        Your task is to determine from the IMU 'msg' if a collision occured. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- A IMU ROS message. To see more information about it run 'ros2 interface show sensor_msgs/msg/Imu' in a terminal.
        """

        collision_detection_msg = Collision()
        collision_detection_msg.header.stamp = msg.header.stamp

        # Get the IMU linear acceleration from the message
        linear_acc_x = msg.linear_acceleration.x
        linear_acc_y = msg.linear_acceleration.y
        linear_acc_z = msg.linear_acceleration.z

        # TODO: Fill in

        collision_detection_msg.collision = False  # TODO: Fill in

        self._pub.publish(collision_detection_msg)


def main(args=None):
    rclpy.init()
    node = CollisionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
