#!/usr/bin/env python
"""Soak-test variant of robot_launch.py: no Ros2Supervisor (crash-loops on macOS,
not needed for the manual/PID/hold flight controllers under test)."""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_mavic')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui', default='true')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=False,
        gui=gui,
        mode='realtime',
    )

    robot_description_path = os.path.join(package_dir, 'resource', 'mavic_webots.urdf')
    mavic_driver = WebotsController(
        robot_name='Mavic_2_PRO',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        respawn=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='mavic_world.wbt',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
        ),
        webots,
        mavic_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
