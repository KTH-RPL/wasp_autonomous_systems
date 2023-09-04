import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO


class Yolo(Node):

    def __init__(self):
        super().__init__('yolo')

        # Load a pretrained model (recommended for training)
        self._model = YOLO("yolov8n-seg.pt")

        # Bridge to convert between ROS and OpenCV
        self._cv_bridge = CvBridge()

        # Publisher
        self._pub = self.create_publisher(Image, 'yolo', 10)

        # Subscribe to image topic and call callback function on each recieved message
        self.create_subscription(
            Image, '/kitti/camera/color/left/image', self.image_callback, 10)

    def image_callback(self, msg: Image):
        # Convert from ROS to OpenCV
        image = self._cv_bridge.imgmsg_to_cv2(msg)

        # Use the model to do prediction
        results = self._model(image, verbose=False, device='cpu')
        for r in results:
            # Plot a BGR numpy array of predictions
            im_array = r.plot()
            # Convert from OpenCV to ROS
            image = self._cv_bridge.cv2_to_imgmsg(
                im_array[..., ::-1], encoding="rgb8")
            self._pub.publish(image)


def main():
    rclpy.init()
    node = Yolo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
