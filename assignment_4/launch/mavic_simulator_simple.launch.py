#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots Mavic 2 Pro driver."""

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
    wasp_dir = get_package_share_directory('wasp_autonomous_systems')
    world = LaunchConfiguration('world', default='mavic_world_simple.wbt')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gui = LaunchConfiguration('gui', default='true')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([wasp_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True,
        gui=gui
    )

    robot_description_path = os.path.join(wasp_dir, 'urdf', 'mavic_webots.urdf')
    mavic_driver = WebotsController(
        robot_name='mavic_2_pro',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time},
        ],
        respawn=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='mavic_world_simple.wbt',
            description='Choose one of the world files from `src/wasp_autonomous_systems/wasp_autonomous_systems/world` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Enable or disable Webots GUI (if you have a slow computer it can be useful to turn this off)'
        ),
        webots,
        webots._supervisor,
        mavic_driver,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
