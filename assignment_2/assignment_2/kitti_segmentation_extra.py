import rclpy
from rclpy.clock import JumpThreshold, TimeJump
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Duration

from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import ColorRGBA
import message_filters
from sensor_msgs_py.point_cloud2 import read_points_numpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_matrix
from assignment_2.tf2_sensor_msgs import transform_points

from ultralytics import YOLO
from cv_bridge import CvBridge

import numpy as np

import cv2


class Frustum:
    """Frustum represented with 5 points.

    Typically you would have a near plane and a far plane in a frustum. However, to simplify we store the camera position and treat that single point as the near plane. This would be equal to having a near plane distance of 0, so it makes sense for us to do this here. What we really have here is a pyramid, but in computer vision and for rendering one often uses 'frustums'. 

    The frustum represents what the camera can see, and is often called a 'viewing frustum' in 3D computer graphics. Using the 5 points, it is possible to do something called 'frustum culling'. When performing frustum culling, everything that is outside of the viewing frustum is removed. We will use this to _exclude_ all points of the point cloud, provided by the LiDAR, that are not visible by the camera. We want to exlcude these points since we are not able to color or segment them with the image information from the camera (since they are outside the field of view of the camera).
    """

    def __init__(self, camera_position, far_top_left, far_bottom_left, far_bottom_right, far_top_right):
        # The position of the camera
        self.camera_position = camera_position
        # The far top left corner of the frustum
        self.far_top_left = far_top_left
        # The far bottom left corner of the frustum
        self.far_bottom_left = far_bottom_left
        # The far bottom right corner of the frustum
        self.far_bottom_right = far_bottom_right
        # The far top right corner of the frustum
        self.far_top_right = far_top_right


class Instance:
    """Object instance."""

    def __init__(self, label: str, confidence: float, mask):
        # Label as a string (e.g., 'car', 'person', etc.).
        self.label = label
        # The networks confidence in the prediction, [0, 1].
        self.confidence = confidence
        # Binary mask as a 2D array/image. An element/pixel of the mask has a corresponding pixel in the original image. Meaning the binary pixel 'mask[i, j]' corresponds to the color pixel 'cv_image[i, j]' in the original color image. If a pixel of the mask is truthy (term often used in Python, meaning that it returns true in a boolean context), then that pixel is part of the instance. Likewise, if a pixel of the mask is falsy, then that pixel is not part of the instance.
        self.mask = mask


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

        self.declare_parameter('downsample_voxel_size', 0.2, ParameterDescriptor(
            description='Select voxel size in meters for downsampling the point clouds'))

        self.declare_parameter('num_clouds_accum', 1, ParameterDescriptor(
            description='How many point clouds to accumulate'))

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=True)

        # Publisher
        self._seg_pub = self.create_publisher(
            Image, 'segmentation/image_raw', 10)
        self._seg_info_pub = self.create_publisher(
            CameraInfo, 'segmentation/camera_info', 10)
        self._map_pub = self.create_publisher(Marker, 'map', 10)
        self._frustum_pub = self.create_publisher(Marker, 'frustum', 10)

        # Subscribe to image, camera info, and point cloud topics
        image_sub = message_filters.Subscriber(
            self, Image, '/kitti/camera/color/left/image')
        info_sub = message_filters.Subscriber(
            self, CameraInfo, '/kitti/camera/color/left/camera_info')
        cloud_sub = message_filters.Subscriber(
            self, PointCloud2, '/kitti/velo')

        self._static_id = 0
        self._dynamic_id = 0

        self._num_clouds_accum = 1

        # Approximate synchronize the three topics using their respective timestamp.
        # This means that the registered callback will be called only when the three topics
        # receives a messages at approximatily the same time.
        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, info_sub, cloud_sub], 10, 0.05)
        ts.registerCallback(self.callback)

        # Detect jumps in time
        threshold = JumpThreshold(min_forward=None, min_backward=Duration(
            seconds=-1), on_clock_change=True)
        self.jump_handle = self.get_clock().create_jump_callback(
            threshold, post_callback=self.time_jump_callback)

    def time_jump_callback(self, _: TimeJump):
        self.get_logger().warning("Detected jump back in time. Clearing maps and tf buffer.")
        self._tf_buffer.clear()
        # Clear the map
        self._static_id = 0
        self._dynamic_id = 0
        self._map_pub.publish(Marker(action=Marker.DELETEALL))

    def callback(self, image: Image, info: CameraInfo, cloud: PointCloud2):
        # Convert from ROS to OpenCV
        cv_image = self._cv_bridge.imgmsg_to_cv2(image)

        # Segment image to get instances
        instances = self.segment_image(cv_image, info)

        # Color the dynamic objects
        dynamic_image = self.color_dynamic(cv_image, instances)

        # Find neccessary transforms
        try:
            cam_to_pc, pc_to_cam, cam_to_world = self.lookup_transforms(
                cloud.header, info.header)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')
            return

        voxel_size = self.get_parameter(
            'downsample_voxel_size').get_parameter_value().double_value

        # Get current frustum
        frustum = self.frustum(info, cam_to_pc.transform, 200.0)
        self.publish_frustum(frustum, cloud.header)

        xyz = read_points_numpy(cloud, ['x', 'y', 'z'], True)

        xyz = self.downsample_cloud(xyz, voxel_size)

        xyz = self.crop_cloud(xyz, frustum)

        xyz = transform_points(xyz, pc_to_cam.transform)

        pixel = self.cloud_to_pixel(xyz, info)

        static_xyz, static_pixel, dynamic_xyz, dynamic_pixel = self.seperate_static_dynamic(
            xyz, pixel, dynamic_image)

        static_rgb = self.color_cloud(static_xyz, cv_image, static_pixel)
        dynamic_rgb = self.color_cloud(
            dynamic_xyz, dynamic_image, dynamic_pixel)

        static_xyz = transform_points(static_xyz, cam_to_world.transform)
        dynamic_xyz = transform_points(dynamic_xyz, cam_to_world.transform)

        header = cloud.header
        header.frame_id = 'world'

        self.publish_update(static_xyz, static_rgb, dynamic_xyz,
                            dynamic_rgb, voxel_size, header)

    def color_dynamic(self, cv_image, instances: list[Instance]):
        """Fill in a color for the pixels you consider dynamic. Look at the instance label and confidence to determine if the instance should be colored. Use the instance mask to know which pixels to color. By using different colors for different classes/labels and/or instances you will be able to tell them apart when viewing the Dynamic Map in RViz."""

        dynamic_color = np.zeros(cv_image.shape, dtype=cv_image.dtype)

        # TODO: Implement

        for instance in instances:
            # Just an example, you should change this and add more
            if instance.label == "label" and instance.confidence > 0.0:
                # The colors are representated as 3 8-bit unsigned integers, so the range is [0..255] for each color component (i.e., red, green, blue). So below we set 100% red and 0% green and blue.
                dynamic_color[0 < instance.mask] = [255, 0, 0]

        return dynamic_color

    def crop_cloud(self, cloud, frustum: Frustum):
        """Use the planes defined by the frustum and return only the points that are inside all planes.

        For this cross and dot product will be useful, to get normals and check on which side, respectively.
        """

        # TODO: Fill in

        return cloud

    def coord_to_pixel(self, info: CameraInfo, x: float, y: float, z: float) -> tuple[int, int]:
        """Project the 3D point (x, y, z) to 2D pixel (u, v) using the camera's projection matrix 'info.p'."""

        u = 0  # TODO: Fill in
        v = 0  # TODO: Fill in

        # TODO: Fill in

        # We add this check in case the frustum culling left some points outside or at the border of the frustum.
        if info.width <= u < 0:
            u = 0
        if info.height <= v < 0:
            v = 0

        return (int(u), int(v))

    def segment_image(self, cv_image, info: CameraInfo) -> list[Instance]:
        device = self.get_parameter(
            'device').get_parameter_value().string_value

        # Use the model to do prediction
        results = self._model(cv_image, verbose=False, device=device, conf=0.3)

        assert 1 >= len(results)

        instances = []
        if results[0].masks is not None:
            print('Segmentation execution time')
            print(f'\tPreprocess: {results[0].speed["preprocess"]:.2f} ms')
            print(f'\tInference: {results[0].speed["inference"]:.2f} ms')
            print(f'\tPostprocess: {results[0].speed["postprocess"]:.2f} ms')

            # Publish segmentation image
            self.publish_segmentation(results[0], info)

            # Fill in detected instances
            h, w, _ = results[0].orig_img.shape
            for i in range(len(results[0].masks.data)):
                c = results[0].names[int(results[0].boxes.cls[i])]
                conf = results[0].boxes.conf[i]
                mask_raw = results[0].masks[i].cpu(
                ).data.numpy().transpose(1, 2, 0)
                mask = cv2.resize(mask_raw, (w, h))
                instances.append(Instance(c, conf, mask.astype(int)))

        return instances

    def seperate_static_dynamic(self, xyz, pixel, dynamic_image):
        static_indices = np.where(
            dynamic_image[pixel[:, 1], pixel[:, 0]] == [0, 0, 0])
        dynamic_indices = np.where(
            dynamic_image[pixel[:, 1], pixel[:, 0]] != [0, 0, 0])
        static_xyz = np.copy(xyz[static_indices, :]).reshape((-1, 3))
        static_pixel = np.copy(pixel[static_indices, :]).reshape((-1, 2))
        dynamic_xyz = np.copy(xyz[dynamic_indices, :]).reshape((-1, 3))
        dynamic_pixel = np.copy(pixel[dynamic_indices, :]).reshape((-1, 2))
        return static_xyz, static_pixel, dynamic_xyz, dynamic_pixel

    def cloud_to_pixel(self, cloud, info):
        return np.array([self.coord_to_pixel(info, *p) for p in cloud], dtype=int)

    def lookup_transforms(self, cloud_header, camera_header):
        cam_to_pc = self._tf_buffer.lookup_transform(
            cloud_header.frame_id, camera_header.frame_id, cloud_header.stamp)
        pc_to_cam = self._tf_buffer.lookup_transform(
            camera_header.frame_id, cloud_header.frame_id, cloud_header.stamp)
        cam_to_world = self._tf_buffer.lookup_transform(
            'world', camera_header.frame_id, camera_header.stamp, Duration(seconds=0.1))
        return cam_to_pc, pc_to_cam, cam_to_world

    def frustum(self, info: CameraInfo, transform, far_dist=100.0) -> Frustum:
        translation = np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z])
        rotation = quaternion_matrix(
            [transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z])[:3, :3]

        height = info.height
        width = info.width
        cx = info.p[2]
        cy = info.p[6]
        fx = info.p[0]
        fy = info.p[5]
        far_plane = np.array(
            [(np.array([0, 0, 0, width, width]) - cx) *
             np.array([0, far_dist, far_dist, far_dist, far_dist]) / fx,
             (np.array([0, 0, height, height, 0]) - cy) *
             np.array([0, far_dist, far_dist, far_dist, far_dist]) / fy,
             np.array([0, far_dist, far_dist, far_dist, far_dist])])
        far_plane = np.dot(rotation, far_plane) + np.tile(
            translation.reshape(3, 1), (1, far_plane.shape[1]))

        return Frustum(*(far_plane.T))

    def downsample_cloud(self, cloud, voxel_size, return_indices=False):
        cloud /= voxel_size
        cloud = np.floor(cloud)
        tmp = [tuple(row) for row in cloud]
        if return_indices:
            cloud, indices = np.unique(tmp, True, axis=0)
            cloud *= voxel_size
            return cloud, indices
        else:
            cloud = np.unique(tmp, False, axis=0)
            cloud *= voxel_size
            return cloud

    def color_cloud(self, cloud, cv_image, pixel):
        rgb = np.zeros((cloud.shape[0], 3), dtype=float)
        for i in range(cloud.shape[0]):
            rgb[i] = self.color(cv_image, *pixel[i])
        return rgb

    def color(self, cv_image, u, v):
        if 0 <= v < cv_image.shape[0] and 0 <= u < cv_image.shape[1]:
            return 255 * cv_image[v, u]
        return np.zeros((1, 1, 3), dtype=float)

    def publish_segmentation(self, image, info: CameraInfo):
        # Plot a BGR numpy array of predictions
        im_array = image.plot()
        # Convert from OpenCV to ROS
        image = self._cv_bridge.cv2_to_imgmsg(
            im_array[..., ::-1], encoding="rgb8")
        image.header = info.header
        self._seg_info_pub.publish(info)
        self._seg_pub.publish(image)

    def publish_frustum(self, frustum: Frustum, header):
        m = Marker(header=header)
        m.ns = 'frustum'
        m.id = 0
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.ADD

        m.scale.x = m.scale.y = m.scale.z = 0.1

        m.color.a = 0.5
        m.color.r = 1.0
        m.color.g = 0.5

        cam = Point(
            x=frustum.camera_position[0], y=frustum.camera_position[1], z=frustum.camera_position[2])
        tl = Point(
            x=frustum.far_top_left[0], y=frustum.far_top_left[1], z=frustum.far_top_left[2])
        bl = Point(
            x=frustum.far_bottom_left[0], y=frustum.far_bottom_left[1], z=frustum.far_bottom_left[2])
        br = Point(
            x=frustum.far_bottom_right[0], y=frustum.far_bottom_right[1], z=frustum.far_bottom_right[2])
        tr = Point(
            x=frustum.far_top_right[0], y=frustum.far_top_right[1], z=frustum.far_top_right[2])

        m.points.extend([cam, tl, bl])
        m.points.extend([cam, bl, br])
        m.points.extend([cam, br, tr])
        m.points.extend([cam, tr, tl])

        self._frustum_pub.publish(m)

    def publish_update(self, static_xyz, static_rgb, dynamic_xyz, dynamic_rgb, voxel_size, header):
        marker = Marker(header=header, action=Marker.ADD)

        num_clouds_accum = self.get_parameter(
            'num_clouds_accum').get_parameter_value().integer_value

        if self._num_clouds_accum > num_clouds_accum:
            # Remove everything that is seen up until now
            self._static_id = 0
            self._dynamic_id = 0
            self._map_pub.publish(Marker(action=Marker.DELETEALL))

        self._num_clouds_accum = num_clouds_accum

        # Static map
        marker.ns = 'Static Map'
        marker.id = self._static_id % num_clouds_accum
        self._static_id += 1
        marker.type = Marker.CUBE_LIST
        marker.scale.x = marker.scale.y = marker.scale.z = voxel_size
        marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in static_xyz]
        marker.colors = [ColorRGBA(r=c[2], g=c[1], b=c[0], a=1.0)
                         for c in static_rgb]
        self._map_pub.publish(marker)

        # Dynamic map
        marker.ns = 'Dynamic Map'
        marker.id = self._dynamic_id % num_clouds_accum
        self._dynamic_id += 1
        marker.type = Marker.CUBE_LIST
        marker.scale.x = marker.scale.y = marker.scale.z = voxel_size + 0.05
        marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in dynamic_xyz]
        marker.colors = [ColorRGBA(r=c[0], g=c[1], b=c[2], a=1.0)
                         for c in dynamic_rgb]
        self._map_pub.publish(marker)


def main():
    rclpy.init()
    node = KittiSegmentation()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        # rclpy.spin(node)
        executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
