import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

from rcl_interfaces.msg import ParameterDescriptor

from ultralytics import YOLO

import os


class Segmentation(Node):

    def __init__(self):
        super().__init__('segmentation')

        # Device parameter
        self.declare_parameter('device', 'cpu', ParameterDescriptor(
            description='Select device to run YOLO model on, use \'cpu\' for cpu, \'0\', \'1\', ... for GPU X'))

        self._model = YOLO('yolo11n-seg.pt')

        # Bridge to convert between ROS and OpenCV
        self._cv_bridge = CvBridge()

        # Subscribers
        sub = self.create_subscription(Image, '/image', self.image_callback, 1)
        compressed_sub = self.create_subscription(
            CompressedImage, sub.topic_name + '/compressed', self.image_callback, 1)

        # Publisher
        self._seg_pub = self.create_publisher(
            Image, sub.topic_name + '_segmentation', 10)
        self._seg_compressed_pub = self.create_publisher(
            CompressedImage, compressed_sub.topic_name + '/segmentation', 10)

    def image_callback(self, msg):
        if 0 == self._seg_pub.get_subscription_count() and 0 == self._seg_compressed_pub.get_subscription_count():
            return

        # Convert from ROS to OpenCV
        if type(msg) is Image:
            image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        else:
            image = self._cv_bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding='rgb8')

        device = self.get_parameter(
            'device').get_parameter_value().string_value

        result = self._model(image, verbose=False, device=device)

        for r in result:
            # Plot a BGR numpy array of predictions
            im_array = r.plot()
            # Convert from OpenCV to ROS
            if 0 < self._seg_pub.get_subscription_count():
                ret = self._cv_bridge.cv2_to_imgmsg(
                    im_array[..., ::-1], encoding="rgb8")
                ret.header = msg.header
                self._seg_pub.publish(ret)
            if 0 < self._seg_compressed_pub.get_subscription_count():
                ret = self._cv_bridge.cv2_to_compressed_imgmsg(
                    im_array[..., ::-1])
                ret.header = msg.header
                self._seg_compressed_pub.publish(ret)


def main():
    rclpy.init()
    node = Segmentation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
