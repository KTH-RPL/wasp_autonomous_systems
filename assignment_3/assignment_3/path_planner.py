#!/usr/bin/env python

from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker

from assignment_3.grid_map import GridMap
from assignment_3.inflate_map import inflate_map
from assignment_3.create_map import create_map
from assignment_3.utility import to_ros_path
from assignment_3.breadth_first import bf
from assignment_3.dijkstra import dijkstra
from assignment_3.astar import astar
from assignment_3.rrt import rrt
from assignment_3.rrtstar import rrtstar

from math import ceil, hypot
import time


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self._map_pub = self.create_publisher(OccupancyGrid, 'map', qos)
        self._walls_pub = self.create_publisher(Marker, 'walls', qos)
        self._path_pub = self.create_publisher(Path, 'path', qos)
        self._start_pub = self.create_publisher(Marker, 'start', qos)
        self._goal_pub = self.create_publisher(Marker, 'goal', qos)
        self._search_pub = self.create_publisher(Marker, 'search', qos)

        self.add_on_set_parameters_callback(self.config_callback)

        self._robot_radius = 0.6

        self._planner_algorithm = None
        self._planner_eight_connectivity = None
        self._planner_sampling_iterations = None
        self._planner_sampling_max_edge_length = None
        self._planner_sampling_stop_when_goal_found = None

        self._map_resolution = None
        self._map_width = None
        self._map_height = None
        self._map_id = None
        self._map_num_walls = None
        self._map_inflate = None

        self._robot_position = None
        self._goal_position = None

        self._grid_map = None

        # self.declare_parameter('robot_radius', 0.25, ParameterDescriptor(
        #     description='Robot radius in meter'))

        self.declare_parameters(
            namespace='planner',
            parameters=[
                ('algorithm', 'bf', ParameterDescriptor(
                    description='Planner to use, supported: bf, dijkstra, astar, rrt, rrtstar')),
                ('8_connectivity', False, ParameterDescriptor(
                    description='Connectivity for cell planning (i.e., bf, dijkstra, astar)')),
                ('sampling_iterations', 10000, ParameterDescriptor(
                    description='How many samples for sampling based planning (i.e., rrt, rrtstar)')),
                ('sampling_max_edge_length', 0.5, ParameterDescriptor(
                    description='Max edge length, in meter, for sampling based planning (i.e., rrt, rrtstar)')),
                ('sampling_stop_when_goal_found', False, ParameterDescriptor(
                    description='Stop sampling when goal has been found, for sampling based planning (i.e., rrt, rrtstar)'))
            ])

        self.declare_parameters(
            namespace='map',
            parameters=[
                ('resolution', 1.0, ParameterDescriptor(
                    description='Cell size of the grid map in meters')),
                ('width', 40.0, ParameterDescriptor(
                    description='Width of the grid map in meters')),
                ('height', 40.0, ParameterDescriptor(
                    description='Height of the grid map in meters')),
                ('id', 0,
                 ParameterDescriptor(
                     description='Map id [0..4] are manually created maps, other are randomly generate maps')),
                ('num_walls', 5, ParameterDescriptor(
                    description='Number of walls to create when randomly generating map (i.e., map id is not one of [0..4])')),
                ('inflate', False, ParameterDescriptor(
                    description='Whether to inflate the map'))
            ])

        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.robot_callback, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

    def config_callback(self, config: list[Parameter]):
        update_map = False
        update_plan = False

        for p in config:
            # if 'robot_radius' == p.name:
            #     self._robot_radius = p.get_parameter_value().double_value
            #     update_map = True
            #     self.publish_start_and_goal()
            if 'map.resolution' == p.name:
                self._map_resolution = p.get_parameter_value().double_value
            elif 'map.width' == p.name:
                self._map_width = p.get_parameter_value().double_value
            elif 'map.height' == p.name:
                self._map_height = p.get_parameter_value().double_value
            elif 'map.id' == p.name:
                self._map_id = p.get_parameter_value().integer_value
            elif 'map.num_walls' == p.name:
                self._map_num_walls = p.get_parameter_value().integer_value
            elif 'map.inflate' == p.name:
                self._map_inflate = p.get_parameter_value().bool_value
            elif 'planner.algorithm' == p.name:
                self._planner_algorithm = p.get_parameter_value().string_value
            elif 'planner.8_connectivity' == p.name:
                self._planner_eight_connectivity = p.get_parameter_value().bool_value
            elif 'planner.sampling_iterations' == p.name:
                self._planner_sampling_iterations = p.get_parameter_value().integer_value
            elif 'planner.sampling_max_edge_length' == p.name:
                self._planner_sampling_max_edge_length = p.get_parameter_value().double_value
            elif 'planner.sampling_stop_when_goal_found' == p.name:
                self._planner_sampling_stop_when_goal_found = p.get_parameter_value().bool_value

            update_map = update_map or p.name.startswith('map.')
            update_plan = update_plan or update_map or p.name.startswith(
                'planner.')

        if self._map_resolution and self._map_width and self._map_height and None != self._map_id and None != self._map_num_walls and None != self._map_inflate and self._robot_radius:
            if update_map:
                self.update_map()

            if update_plan:
                self.plan()

        return SetParametersResult(successful=True)

    def robot_callback(self, msg: PoseWithCovarianceStamped):
        # TODO: Transform to map frame
        self._robot_position = (msg.pose.pose.position.x,
                                msg.pose.pose.position.y)

        self.plan(msg.header)

    def goal_callback(self, msg: PoseStamped):
        # TODO: Transform to map frame
        self._goal_position = (msg.pose.position.x, msg.pose.position.y)

        self.plan(msg.header)

    def plan(self, header=None):
        self.publish_start_and_goal()

        if not self._robot_position:
            print('Robot position not set, cannot plan')
            return
        if not self._goal_position:
            print('Goal position not set, cannot plan')
            return

        planner = None
        if 'bf' == self._planner_algorithm or 'breadth_first' == self._planner_algorithm:
            print(f'Using breadth first planner')
            planner = bf
        elif 'dijkstra' == self._planner_algorithm:
            print(f'Using Dijkstra planner')
            planner = dijkstra
        elif 'a*' == self._planner_algorithm or 'astar' == self._planner_algorithm:
            print(f'Using A* planner')
            planner = astar
        elif 'rrt' == self._planner_algorithm:
            print(f'Using RRT planner')
            planner = rrt
        elif 'rrt*' == self._planner_algorithm or 'rrtstar' == self._planner_algorithm:
            print(f'Using RRT* planner')
            planner = rrtstar
        else:
            print(
                f'Planner {self._planner_algorithm} not supported, please select one of: bf, dijkstra, astar, rrt')
            return

        st = time.time()
        if self._planner_algorithm.startswith('rrt'):
            path, search = planner(self._grid_map, *self._robot_position,
                                   *self._goal_position, iterations=self._planner_sampling_iterations, max_edge_length=self._planner_sampling_max_edge_length, early_stop=self._planner_sampling_stop_when_goal_found)
        else:
            path, search = planner(
                self._grid_map, *self._robot_position, *self._goal_position, eight_connectivity=self._planner_eight_connectivity)
        et = time.time()
        elapsed_time = et - st

        if not path:
            print(f'No path found')
        else:
            path_length = 0.0
            prev = path[0]
            for e in path:
                path_length += hypot(e[0] - prev[0], e[1] - prev[1])
                prev = e
            print(f'Path length: {path_length:.2f}')

        print(f'Execution time: {elapsed_time:.2f} seconds')

        stamp = header.stamp if header else self.get_clock().now().to_msg()
        self._path_pub.publish(to_ros_path(path, 'map', stamp))

        if self._planner_algorithm.startswith('rrt'):
            self.publish_search_tree(search)
        else:
            self.publish_search_cells(search)

    def update_map(self):
        self._grid_map, walls = create_map(
            self._map_resolution, self._map_width, self._map_height, self._map_id, self._map_num_walls, self._robot_radius)

        if self._map_inflate:
            inflate_map(self._grid_map, ceil(
                2 * self._robot_radius / self._grid_map.resolution))

        self._map_pub.publish(self._grid_map.to_ros('map'))
        self._walls_pub.publish(self.walls_to_ros(walls))

    def walls_to_ros(self, walls: list[tuple[tuple[float, float], tuple[float, float]]]) -> Marker:
        m = Marker()
        m.header.frame_id = 'map'

        m.ns = 'walls'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.scale.x = 0.1
        m.color.a = 1.0
        m.color.g = 1.0

        for wall in walls:
            m.points.append(Point(x=wall[0][0], y=wall[0][1]))
            m.points.append(Point(x=wall[1][0], y=wall[1][1]))

        return m

    def publish_start_and_goal(self):
        if not self._robot_radius:
            return

        m = Marker()
        m.header.frame_id = 'map'
        m.id = 0
        m.type = Marker.SPHERE
        m.scale.x = m.scale.y = m.scale.z = 2 * self._robot_radius
        m.color.a = 1.0

        if self._robot_position:
            m.ns = 'start'
            m.pose.position.x = self._robot_position[0]
            m.pose.position.y = self._robot_position[1]
            m.color.r = 26 / 255
            m.color.g = 95 / 255
            m.color.b = 180 / 255
            self._start_pub.publish(m)

        if self._goal_position:
            m.ns = 'goal'
            m.pose.position.x = self._goal_position[0]
            m.pose.position.y = self._goal_position[1]
            m.color.r = 229 / 255
            m.color.g = 165 / 255
            m.color.b = 10 / 255
            self._goal_pub.publish(m)

    def publish_search_cells(self, cells):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns = 'search'
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.scale.x = m.scale.y = m.scale.z = self._grid_map.resolution - 0.001
        m.color.a = 0.5
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 0.7

        for cell in cells:
            p = self._grid_map.coord(*cell)
            m.points.append(Point(x=p[0], y=p[1], z=-m.scale.z / 2 + 0.01))

        self._search_pub.publish(m)

    def publish_search_tree(self, edges):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns = 'search'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.scale.x = m.scale.y = m.scale.z = 0.1
        m.color.a = 0.5
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 0.7

        for edge in edges:
            m.points.append(
                Point(x=edge[0][0], y=edge[0][1], z=-m.scale.z / 2 + 0.01))
            m.points.append(
                Point(x=edge[1][0], y=edge[1][1], z=-m.scale.z / 2 + 0.01))

        self._search_pub.publish(m)


def main():
    rclpy.init()
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
