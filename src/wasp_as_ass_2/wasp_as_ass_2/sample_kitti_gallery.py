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
        # 66 spreads 16 images across the full ~1058-message rosbag sequence 
        # (16 * 66 = 1056), rather than bunching them into the first few seconds.
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

        # This node does nothing at all until something publishes images, and
        # on its own it used to say nothing while waiting - so if you forgot
        # the rosbag, the only symptom was a silent terminal. Say up front
        # what it is waiting for, then keep saying it until frames arrive.
        self.get_logger().info(
            f'Waiting for images on {sub.topic_name}. Nothing will happen until '
            'the KITTI rosbag is playing: start it in a second terminal with '
            "'pixi run ass_2_kitti_rosbag'.")
        self._waiting_timer = self.create_timer(10.0, self.waiting_reminder)

    def waiting_reminder(self):
        # _frame_count, not _saved_count: only every every_nth frame is saved
        # (66 by default, about 6.6 s of bag at full rate), so a slower
        # playback rate would still be reporting "no images" well after they
        # started arriving.
        if self._frame_count or self._done:
            self._waiting_timer.cancel()
            return
        self.get_logger().warn(
            'Still no images received. Is the rosbag playing? In a second '
            "terminal: 'pixi run ass_2_kitti_rosbag'")

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
        num_images = self.get_parameter('num_images').get_parameter_value().integer_value
        self.get_logger().info(f'Saved {path} ({self._saved_count}/{num_images})')

        if self._saved_count >= num_images:
            self._done = True
            self.get_logger().info(
                f'Gallery sampling complete: {self._saved_count} images in '
                f'{gallery_dir()}. You can stop this node and the rosbag now, and '
                'run the CLIP/DINO tasks.')


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
