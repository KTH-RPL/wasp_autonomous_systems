import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rcl_interfaces.msg import ParameterDescriptor

from ultralytics import YOLO


class CameraSegmentation(Node):

    def __init__(self):
        super().__init__('yolo')

        # Load a pretrained model (recommended for training)
        self._model = YOLO("yolov8n-seg.pt")

        # Bridge to convert between ROS and OpenCV
        self._cv_bridge = CvBridge()

        # Publisher
        self._seg_pub = self.create_publisher(
            Image, 'segmentation/image_raw', 10)

        # Device parameter
        self.declare_parameter('device', '', ParameterDescriptor(
            description='Select device to run YOLOv8 on, use \'cpu\' for cpu, \'0\', \'1\', ... for GPU X'))

        # Subscribe to image topic
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)

    def image_callback(self, image: Image):
        # Convert from ROS to OpenCV
        cv_image = self._cv_bridge.imgmsg_to_cv2(
            image, desired_encoding='rgb8')

        device = self.get_parameter(
            'device').get_parameter_value().string_value

        result = self._model(cv_image, verbose=False, device=device)

        for r in result:
            # Plot a BGR numpy array of predictions
            im_array = r.plot()
            # Convert from OpenCV to ROS
            image = self._cv_bridge.cv2_to_imgmsg(
                im_array[..., ::-1], encoding="rgb8")
            image.header = image.header
            self._seg_pub.publish(image)


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
