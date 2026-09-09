import os

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

from rcl_interfaces.msg import ParameterDescriptor

import cv2

from wasp_as_ass_2.gallery import gallery_dir


class SampleKittiGallery(Node):

    def __init__(self):
        super().__init__('sample_kitti_gallery')

        self.declare_parameter('num_images', 16, ParameterDescriptor(
            description='How many images to sample into the gallery.'))
        # rosbags/kitti has 1058 camera messages over ~110s (~9.6Hz) -
        # confirmed via `ros2 bag info`. every_nth=10 (an earlier guess)
        # only ever sampled the first 160 messages, i.e. the first ~17s -
        # one block, one direction, all near-duplicates. 66 spreads 16
        # images across the full ~1058-message sequence instead
        # (16 * 66 = 1056).
        self.declare_parameter('every_nth', 66, ParameterDescriptor(
            description='Save every Nth incoming frame, for temporal spread.'))

        self._cv_bridge = CvBridge()
        self._frame_count = 0
        self._saved_count = 0
        self._done = False

        os.makedirs(gallery_dir(), exist_ok=True)

        sub = self.create_subscription(Image, '/image', self.image_callback, 1)
        self.create_subscription(
            CompressedImage, sub.topic_name + '/compressed', self.image_callback, 1)

    def image_callback(self, msg):
        if self._done:
            return

        every_nth = self.get_parameter('every_nth').get_parameter_value().integer_value
        self._frame_count += 1
        if self._frame_count % every_nth != 0:
            return

        if type(msg) is Image:
            image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        else:
            image = self._cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='rgb8')

        path = os.path.join(gallery_dir(), f'{self._saved_count}.png')
        cv2.imwrite(path, image[..., ::-1])
        self._saved_count += 1
        self.get_logger().info(f'Saved {path}')

        num_images = self.get_parameter('num_images').get_parameter_value().integer_value
        if self._saved_count >= num_images:
            self._done = True
            self.get_logger().info(
                f'Gallery sampling complete: {self._saved_count} images in {gallery_dir()}')


def main():
    rclpy.init()
    node = SampleKittiGallery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
