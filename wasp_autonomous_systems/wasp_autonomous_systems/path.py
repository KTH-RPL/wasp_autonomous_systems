#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path

from math import pi


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path')
        self._pub = self.create_publisher(Path, '/path', 10)
        self.create_subscription(
            PointStamped, '/TurtleBot3Burger/gps', self.gps_callback, 10)

        self._path = Path()

    def gps_callback(self, msg: PointStamped):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point

        self._path.header = msg.header
        self._path.poses.append(pose)

        self._pub.publish(self._path)


def main():
    rclpy.init()
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
