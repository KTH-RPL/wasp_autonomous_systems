#!/usr/bin/env python

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    resources_dir = get_package_share_directory('wasp_autonomous_systems')

    urdf_file_name = 'urdf/car.urdf'
    urdf = os.path.join(resources_dir, urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'robot_description': robot_desc}],
            arguments=[urdf])
    ]


def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
