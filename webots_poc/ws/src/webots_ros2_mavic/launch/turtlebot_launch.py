#!/usr/bin/env python
"""Adapted from ht25's wasp_autonomous_systems/launch/webots_turtlebot_launch.py
to test Assignment 2 (collision detection) against native macOS Webots.
No Ros2Supervisor (crash-loops on macOS, and the robot is statically placed
in the world, so it isn't needed) and no `path`/`encoders` course-specific
nodes (not needed to validate connectivity/IMU - assignment_2's actual
exercise, collision_detection.py, only needs /imu; driving is done manually
via /cmd_vel for this test)."""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_mavic')
    gui = LaunchConfiguration('gui', default='true')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', 'turtlebot_collision_detection.wbt']),
        ros2_supervisor=False,
        gui=gui,
        mode='realtime',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yaml')
    mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
                ('/diffdrive_controller/odom', '/odom')]
    turtlebot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': os.path.join(package_dir, 'resource', 'turtlebot_webots.urdf'),
             'use_sim_time': False,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=ros_control_spawners
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        webots,
        robot_state_publisher,
        footprint_publisher,
        turtlebot_driver,
        waiting_nodes,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
