#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    robot = 'waffle_rgbd'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_file_name = 'turtlebot3_' + robot + '.urdf'
    frame_prefix = LaunchConfiguration('frame_prefix', default='')

    print('urdf_file_name : {}'.format(urdf_file_name))

    urdf_path = os.path.join(
        get_package_share_directory('wasp_autonomous_systems'),
        'urdf',
        urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix': PythonExpression(["'", frame_prefix, "/'"])
            }],
        ),
    ])