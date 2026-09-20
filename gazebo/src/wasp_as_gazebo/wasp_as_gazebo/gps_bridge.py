#!/usr/bin/env python
"""Republishes a nav_msgs/Odometry position as a geometry_msgs/PointStamped.

Webots' GPS device publishes a PointStamped on /<robot>/gps, and both the
Assignment 4 exercise files and Assignment 2's RViz config are written
against that. Gazebo has no equivalent device here - position comes out of
the OdometryPublisher system as an Odometry message - so this node does
the last step of the translation, keeping those files simulator-agnostic.

Topics and the frame_id to stamp with are parameters, because the two
assignments want different ones:

  Assignment 4  /model/quadrotor/odometry  -> /mavic_2_pro/gps  (frame: gps)
  Assignment 2  /model/TurtleBot3Burger/odometry
                                           -> /TurtleBot3Burger/gps (frame: gps)

The frame name matters: collision_detection.rviz uses "gps" as its fixed
frame and there is no TF tree behind it, so the point has to be stamped
with the frame it is already expressed in - exactly what Webots does.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


class GpsBridge(Node):

    def __init__(self):
        super().__init__('gps_bridge')

        self.declare_parameter('odometry_topic', '/odom')
        self.declare_parameter('point_topic', '/gps')
        self.declare_parameter('frame_id', 'gps')

        odometry_topic = self.get_parameter(
            'odometry_topic').get_parameter_value().string_value
        point_topic = self.get_parameter(
            'point_topic').get_parameter_value().string_value
        self._frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value

        self._pub = self.create_publisher(PointStamped, point_topic, 10)
        self.create_subscription(
            Odometry, odometry_topic, self.odom_callback, qos_profile_sensor_data)

    def odom_callback(self, msg: Odometry):
        point = PointStamped()
        point.header.stamp = msg.header.stamp
        point.header.frame_id = self._frame_id
        point.point = msg.pose.pose.position
        self._pub.publish(point)


def main(args=None):
    rclpy.init(args=args)
    node = GpsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
