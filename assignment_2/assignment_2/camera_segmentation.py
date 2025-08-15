import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

from rcl_interfaces.msg import ParameterDescriptor

from ultralytics import YOLO

import os

class CameraSegmentation(Node):

    def __init__(self):
        super().__init__('camera_segmentation')

        # Load a pretrained model (recommended for training)
        if 'YOLO_MODELS_DIR' in os.environ and os.path.isdir(os.environ['YOLO_MODELS_DIR']):
            self._model = YOLO(os.path.join(os.environ['YOLO_MODELS_DIR'], 'yolo11n-seg_int8_openvino_model'))
        else:
            self._model = YOLO('yolo11n-seg.pt')

        # Bridge to convert between ROS and OpenCV
        self._cv_bridge = CvBridge()

        # Publisher
        self._seg_pub = self.create_publisher(
            Image, '/image/segmentation', 10)
        self._seg_compressed_pub = self.create_publisher(
            CompressedImage, '/image/compressed/segmentation', 10)

        # Device parameter
        self.declare_parameter('device', '', ParameterDescriptor(
            description='Select device to run YOLOv8 on, use \'cpu\' for cpu, \'0\', \'1\', ... for GPU X'))

        # Subscribe to image topic
        self.create_subscription(Image, '/image', self.image_callback, 1)
        self.create_subscription(
            CompressedImage, '/image/compressed', self.image_compressed_callback, 1)

    def image_callback(self, image: Image):
        if 0 == self._seg_pub.get_subscription_count() and 0 == self._seg_compressed_pub.get_subscription_count():
            return

        # Convert from ROS to OpenCV
        cv_image = self._cv_bridge.imgmsg_to_cv2(image, desired_encoding='rgb8')

        self.process_image(cv_image, image.header)

    def image_compressed_callback(self, image: CompressedImage):
        if 0 == self._seg_pub.get_subscription_count() and 0 == self._seg_compressed_pub.get_subscription_count():
            return

        # Convert from ROS to OpenCV
        cv_image = self._cv_bridge.compressed_imgmsg_to_cv2(image, desired_encoding='rgb8')

        self.process_image(cv_image, image.header)

    def process_image(self, image, header):
        device = self.get_parameter('device').get_parameter_value().string_value

        result = self._model(image, verbose=False, device=device)

        for r in result:
            # Plot a BGR numpy array of predictions
            im_array = r.plot()
            # Convert from OpenCV to ROS
            if 0 < self._seg_pub.get_subscription_count():
                ret = self._cv_bridge.cv2_to_imgmsg(im_array[..., ::-1], encoding="rgb8")
                ret.header = header
                self._seg_pub.publish(ret)
            if 0 < self._seg_compressed_pub.get_subscription_count():
                ret = self._cv_bridge.cv2_to_compressed_imgmsg(im_array[..., ::-1])
                ret.header = header
                self._seg_compressed_pub.publish(ret)


def main():
    rclpy.init()
    node = CameraSegmentation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
