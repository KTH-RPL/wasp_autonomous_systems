#!/usr/bin/env python
"""Gazebo simulation for Assignment 4, included by ass_4_manual_launch.py
and ass_4_pid_launch.py. The Gazebo counterpart of
wasp_as_webots/launch/course_world_launch.py.

The Gazebo 3D window is on by default on every platform, as a separate
`gz sim -g` process - see launch_utils.gz_args/gz_gui for why it is split
that way. Pass false to run headless; pixi task arguments are positional,
so from the task that reads:

    pixi run ass_4_manual false

On the ROS side there is nothing beyond rqt_plot's height trace, which is
the same as the Webots path offers for this assignment.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wasp_as_gazebo.launch_utils import gz_args, gz_gui


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('wasp_as_gazebo')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    world = os.path.join(package_dir, 'worlds', 'course_quadrotor_world.sdf')
    gui = LaunchConfiguration('gui').perform(context)

    # From the side and low: this assignment is about watching altitude,
    # so a view along the ground beats looking down from above.
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': gz_args(world),
            'on_exit_shutdown': 'true',
        }.items(),
    )] + gz_gui(gui, camera_pose='-6 0 1.5 0 0.1 0')


def generate_launch_description():
    package_dir = get_package_share_directory('wasp_as_gazebo')

    bridge_config = os.path.join(package_dir, 'config', 'quadrotor_bridge.yaml')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{
            'use_sim_time': True,
            'config_file': bridge_config,
        }],
        output='screen',
        respawn=True,
    )

    # The Webots side does this inside webots_ros2_driver's C++ plugin; here
    # it is a plain ROS node, so that Assignment 4's exercise files see the
    # identical /thrust and /mavic_2_pro/gps interface in both simulators.
    mavic_controller = Node(
        package='wasp_as_gazebo',
        executable='mavic_controller',
        name='mavic_controller',
        parameters=[{
            'use_sim_time': True,
            'motor_speed_topic': '/quadrotor/command/motor_speed',
        }],
        output='screen',
        respawn=True,
    )

    gps_bridge = Node(
        package='wasp_as_gazebo',
        executable='gps_bridge',
        name='gps_bridge',
        parameters=[{
            'use_sim_time': True,
            'odometry_topic': '/model/quadrotor/odometry',
            'point_topic': '/mavic_2_pro/gps',
            'frame_id': 'gps',
        }],
        output='screen',
        respawn=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='auto',
            description='Gazebo 3D window. On by default on every platform; '
                        'pass false to run headless (useful under WSL2, '
                        'where it falls back to software rendering).'),
        OpaqueFunction(function=launch_setup),
        bridge,
        mavic_controller,
        gps_bridge,
    ])
