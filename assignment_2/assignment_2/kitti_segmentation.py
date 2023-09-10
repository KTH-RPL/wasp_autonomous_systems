import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge

from rcl_interfaces.msg import ParameterDescriptor

import message_filters

from ultralytics import YOLO


class KittiSegmentation(Node):

    def __init__(self):
        super().__init__('kitti_segmentation')

        # Load a pretrained model (recommended for training)
        self._model = YOLO("yolov8n-seg.pt")

        # Bridge to convert between ROS and OpenCV
        self._cv_bridge = CvBridge()

        # Device parameter
        self.declare_parameter('device', '', ParameterDescriptor(
            description='Select device to run YOLOv8 on, use \'cpu\' for cpu, \'0\', \'1\', ... for GPU X'))

        # Publisher
        self._seg_pub = self.create_publisher(
            Image, 'segmentation/image_raw', 10)
        self._seg_info_pub = self.create_publisher(
            CameraInfo, 'segmentation/camera_info', 10)

        # Subscribe to image and camera info topics
        image_sub = message_filters.Subscriber(
            self, Image, '/kitti/camera/color/left/image')
        info_sub = message_filters.Subscriber(
            self, CameraInfo, '/kitti/camera/color/left/camera_info')

        # Approximate synchronize the two topics using their respective timestamp.
        # This means that the registered callback will be called only when the two topics
        # receives a messages at approximatily the same time.
        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, info_sub], 10, 0.05)
        ts.registerCallback(self.callback)

        self._frustum = None

    def callback(self, image: Image, info: CameraInfo):
        # Convert from ROS to OpenCV
        cv_image = self._cv_bridge.imgmsg_to_cv2(image)

        seg_image = self.segment_image(cv_image)

        if seg_image:
            self._seg_info_pub.publish(info)
            self.publish_segmentation(seg_image, image.header)

    def segment_image(self, image):
        device = self.get_parameter(
            'device').get_parameter_value().string_value

        # Use the model to do prediction
        result = self._model(image, verbose=False, device=device)

        assert 1 >= len(result)

        return result[0] if result else None

    def publish_segmentation(self, image, header):
        # Plot a BGR numpy array of predictions
        im_array = image.plot()
        # Convert from OpenCV to ROS
        image = self._cv_bridge.cv2_to_imgmsg(
            im_array[..., ::-1], encoding="rgb8")
        image.header = header
        self._seg_pub.publish(image)


def main():
    rclpy.init()
    node = KittiSegmentation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
