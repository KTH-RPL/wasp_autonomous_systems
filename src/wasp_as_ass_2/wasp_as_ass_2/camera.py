import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import Header
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
from sys import platform


def get_camera(index: int) -> cv2.VideoCapture:
    if platform == "linux" or platform == "linux2":
        return cv2.VideoCapture(index)
    elif platform == "darwin":
        return cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    elif platform == "win32":
        return cv2.VideoCapture(index)
    raise Exception(f"Platform '{platform}' not recognized")


def main(args=None):
    rclpy.init(args=args)

    node = Node("camera")
    node.declare_parameter(
        'index', 0, ParameterDescriptor(description='Camera index.'))
    node.declare_parameter('framerate', 10.0, ParameterDescriptor(
        description='Camera framerate (frames per second).'))
    node.declare_parameter('image_height', 720, ParameterDescriptor(
        description='Image height in pixels.'))
    node.declare_parameter('image_width', 1280, ParameterDescriptor(
        description='Image weidth in pixels.'))
    node.declare_parameter('frame_id', "camera_link",
                           ParameterDescriptor(description='Frame ID.'))

    pub = node.create_publisher(Image, 'image', 10)

    frame_id = node.get_parameter(
        'frame_id').get_parameter_value().string_value

    timer_period = 1 / \
        node.get_parameter(
            'framerate').get_parameter_value().double_value  # seconds
    cap = get_camera(0)
    cap.set(3, node.get_parameter(
        'image_width').get_parameter_value().integer_value)
    cap.set(4, node.get_parameter(
        'image_height').get_parameter_value().integer_value)

    # Bridge to convert between ROS and OpenCV
    cv_bridge = CvBridge()

    while rclpy.ok():
        header = Header()
        header.frame_id = frame_id
        header.stamp = node.get_clock().now().to_msg()
        success, img = cap.read()
        if not success:
            continue
        if 0 == pub.get_subscription_count():
            continue
        ret = cv_bridge.cv2_to_imgmsg(img[..., ::-1], encoding="rgb8", header=header)
        pub.publish(ret)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
