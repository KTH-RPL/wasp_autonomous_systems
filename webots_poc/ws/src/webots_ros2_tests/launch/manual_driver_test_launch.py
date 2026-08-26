#!/usr/bin/env python3
"""Standalone (non-pytest) launch of the driver_test.wbt world, for manual verification."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_tests')
    robot_description_path = os.path.join(package_dir, 'resource', 'driver_test.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'driver_test.wbt'),
        gui=False,
        mode='fast',
        ros2_supervisor=True
    )

    webots_driver = WebotsController(
        robot_name='Pioneer_3_AT',
        parameters=[{'robot_description': robot_description_path, 'use_sim_time': True}]
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        webots_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
