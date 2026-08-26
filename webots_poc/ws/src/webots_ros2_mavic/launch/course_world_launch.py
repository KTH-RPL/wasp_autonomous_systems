#!/usr/bin/env python
"""Same as soak_test_launch.py, but against the actual course world/proto/urdf
(wasp_autonomous_systems/{worlds,resource}/... from ht24/ht25), pulled in to
check whether the higher maxTorque (1000.0 vs upstream's 30.0) changes the
liftoff-instability-at-high-thrust behavior found against the upstream
webots_ros2_mavic example world."""

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
    gui = LaunchConfiguration('gui', default='true')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', 'course_mavic_world.wbt']),
        ros2_supervisor=False,
        gui=gui,
        mode='realtime',
    )

    robot_description_path = os.path.join(package_dir, 'resource', 'course_mavic_webots.urdf')
    mavic_driver = WebotsController(
        robot_name='mavic_2_pro',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        respawn=True
    )

    return LaunchDescription([
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
