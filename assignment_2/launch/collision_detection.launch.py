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

"""Launch Webots TurtleBot3 Burger driver."""

import os
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    pkg_dir = get_package_share_directory('assignment_2')
    world = LaunchConfiguration(
        'world', default='turtlebot_collision_detection.wbt')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gui = LaunchConfiguration('gui', default='true')

    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution([
        FindPackageShare('wasp_autonomous_systems'), 'launch', 'webots_turtlebot.launch.py'])]),
        launch_arguments={
            'world': world,
            'mode': mode,
            'gui': gui
    }.items())

    path = Node(package='wasp_autonomous_systems',
                namespace='', executable='path')

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-d', [os.path.join(pkg_dir, 'rviz', 'collision_detection.rviz')]]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='turtlebot_collision_detection.wbt',
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
        sim,
        path,
        rviz
    ])
