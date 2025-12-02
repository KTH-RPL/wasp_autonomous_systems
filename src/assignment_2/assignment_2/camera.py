import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Image, CompressedImage

import cv2
from cv_bridge import CvBridge
from sys import platform

#  -p framerate:=30.0 -p image_height:=720 -p image_width:=1280 -p pixel_format:=uyvy

class Camera(Node):

    def __init__(self):
        super().__init__('camera')
        self.pub_ = self.create_publisher(Image, 'image', 10)
        self.pub_comp_ = self.create_publisher(CompressedImage, 'image_compressed', 10)

        # Bridge to convert between ROS and OpenCV
        self._cv_bridge = CvBridge()

        self.declare_parameter('index', 0, ParameterDescriptor(description='Camera index.'))
        self.declare_parameter('framerate', 10.0, ParameterDescriptor(description='Camera framerate (frames per second).'))
        self.declare_parameter('image_height', 720, ParameterDescriptor(description='Image height in pixels.'))
        self.declare_parameter('image_width', 1280, ParameterDescriptor(description='Image weidth in pixels.'))

        timer_period = 1 / self.get_parameter('framerate').get_parameter_value().double_value # seconds
        self.cap = self.get_camera(0)
        self.cap.set(3, self.get_parameter('image_width').get_parameter_value().integer_value)
        self.cap.set(4, self.get_parameter('image_height').get_parameter_value().integer_value)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_camera(self, index: int) -> cv2.VideoCapture:
        if platform == "linux" or platform == "linux2":
            return cv2.VideoCapture(0)
        elif platform == "darwin":
            return cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)
        elif platform == "win32":
            return cv2.VideoCapture(0)
        raise Exception(f"Platform '{platform}' not recognized") 

    def timer_callback(self):
        success, img = self.cap.read()
        if success:
            # if 0 < self.pub_.get_subscription_count():
            ret = self._cv_bridge.cv2_to_imgmsg(img[..., ::-1], encoding="rgb8")
            # ret.header = msg.header
            self.pub_.publish(ret)
            # if 0 < self.pub_comp_.get_subscription_count():
            ret = self._cv_bridge.cv2_to_compressed_imgmsg(img[..., ::-1])
            # ret.header = msg.header
            self.pub_comp_.publish(ret)
            # cv2.imshow("Result", img)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     return
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    camera = Camera()

    rclpy.spin(camera)

    camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()