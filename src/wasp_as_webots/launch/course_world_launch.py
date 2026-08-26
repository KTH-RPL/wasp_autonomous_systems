#!/usr/bin/env python
"""Webots launch for Assignment 4 (Mavic altitude control). Included by
wasp_as_ass_4's manual/hold/pid_launch.xml in place of the old
quadrotor_launch.xml (which wraps wasp_as's Gazebo launch, left in place
but unreferenced). ros2_supervisor=True + use_sim_time=True give accurate
simulation-time timestamps; Ros2MavicController (the C++ plugin loaded via
the URDF below) drives the four rotors from a single atomic
Float64MultiArray on /thrust, avoiding the liftoff-flip race that four
separate per-motor topics would have."""

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
    package_dir = get_package_share_directory('wasp_as_webots')
    gui = LaunchConfiguration('gui', default='true')
    # Must be a bare LaunchConfiguration (not a literal string/PathJoinSubstitution
    # baked at construction time) - WebotsLauncher's Ros2Supervisor-injection
    # mechanism works by overwriting the 'world' launch configuration's value
    # (to point at its temp copy with the injected supervisor Robot node)
    # right before the actual `webots` process command line gets resolved.
    # A literal path bypasses that indirection entirely and silently loads
    # the original world file with no supervisor robot in it - confirmed by
    # testing (Ros2Supervisor retries forever, /clock never publishes).
    world = LaunchConfiguration('world', default='course_mavic_world.wbt')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True,
        gui=gui,
        mode='realtime',
    )

    robot_description_path = os.path.join(package_dir, 'resource', 'course_mavic_webots.urdf')
    mavic_driver = WebotsController(
        robot_name='mavic_2_pro',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': True},
        ],
        respawn=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='course_mavic_world.wbt',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
        ),
        webots,
        webots._supervisor,
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
