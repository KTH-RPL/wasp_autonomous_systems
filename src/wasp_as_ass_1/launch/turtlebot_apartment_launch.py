#!/usr/bin/env python
"""Webots launch for Assignment 1 (RGB-D camera). Replaces the old
Gazebo-based turtlebot_simulation_launch.xml (left in place but
unreferenced) - turtlebot3_waffle_rgbd's camera sensor is exactly what
crashes Gazebo on macOS (Ogre2 SIGSEGV), and this assignment's whole point
is comparing simulated RGB-D data to real recorded data, so there's no
camera-less workaround. The Webots Turtlebot here uses a Kinect-equivalent
RGB-D device (webots_ros2_driver's Ros2RGBD plugin) instead, streaming
/camera/color/image_raw, /camera/depth/image_raw, and /camera/color/points.

ros2_supervisor=True + use_sim_time=True give accurate simulation-time
timestamps, matching the Gazebo version's use_sim_time=true convention.

NOTE for student-facing instructions: the apartment world's doors (and
possibly other thin geometry) have no collision boundary - driving the
robot manually confirmed it can pass straight through a closed door.
Worth a heads-up to students so they don't mistake it for a bug in their
own code if they drive through one while exploring the apartment."""

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
    webots_pkg_dir = get_package_share_directory('wasp_as_webots')
    ass_1_dir = get_package_share_directory('wasp_as_ass_1')
    gui = LaunchConfiguration('gui', default='true')
    # Must be a bare LaunchConfiguration, not a literal path - see
    # wasp_as_webots's course_world_launch.py for why (Ros2Supervisor's
    # world-file injection silently no-ops otherwise).
    world = LaunchConfiguration('world', default='turtlebot_apartment.wbt')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([webots_pkg_dir, 'worlds', world]),
        ros2_supervisor=True,
        gui=gui,
        mode='realtime',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
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

    ros2_control_params = os.path.join(webots_pkg_dir, 'resource', 'ros2control.yaml')
    mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
                ('/diffdrive_controller/odom', '/odom')]
    turtlebot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': os.path.join(webots_pkg_dir, 'resource', 'turtlebot_webots_rgbd.urdf'),
             'use_sim_time': True,
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

    rviz_config_file = os.path.join(ass_1_dir, 'rviz', 'turtlebot_simulation.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    encoders = Node(
        package='wasp_as',
        executable='encoders',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='turtlebot_apartment.wbt'),
        DeclareLaunchArgument('gui', default_value='true'),
        webots,
        webots._supervisor,
        robot_state_publisher,
        footprint_publisher,
        turtlebot_driver,
        waiting_nodes,
        rviz,
        encoders,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
