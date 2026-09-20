#!/usr/bin/env python
"""Assignment 4, PID altitude control - Gazebo alternative to
wasp_as_ass_4/launch/pid_launch.xml.

Same node set as the Webots launch, with quadrotor_world_launch.py
(Gazebo) swapped in for course_world_launch.py (Webots). altitude_pid
itself is the unmodified wasp_as_ass_4 node, log_file argument included.
Pixi task arguments are positional, log_file first:

    pixi run ass_4_pid step_response.csv
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('wasp_as_gazebo')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'quadrotor_world_launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items(),
    )

    altitude_pid = Node(
        package='wasp_as_ass_4',
        executable='altitude_pid',
        name='altitude_pid',
        arguments=['--log-file', LaunchConfiguration('log_file')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        respawn=True,
    )

    rqt_reconfigure = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        output='screen',
    )

    rqt_plot = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='rqt_plot',
        arguments=['/mavic_2_pro/gps/point/z'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        # Default "none" (a real value, not an empty string - `ros2 launch`
        # rejects an empty argument), same convention as pid_launch.xml.
        DeclareLaunchArgument('log_file', default_value='none'),
        simulation,
        altitude_pid,
        rqt_reconfigure,
        rqt_plot,
    ])
