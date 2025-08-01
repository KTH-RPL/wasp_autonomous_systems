#!/usr/bin/env python

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    loop = LaunchConfiguration('loop', default='false')
    bag_dir = LaunchConfiguration('bag_dir')
    rate = LaunchConfiguration('rate', default='1')
    queue_size = LaunchConfiguration('queue_size', default='100')

    pkg_dir = get_package_share_directory('assignment_2')

    # RViz
    use_rviz = LaunchConfiguration("rviz", default=False)
    rviz_config = os.path.join(pkg_dir, "rviz", "collision_detection.rviz")
    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': loop}],
        arguments=['-d', rviz_config],
        condition=launch.conditions.IfCondition(use_rviz),
    )

    # Foxglove
    use_foxglove = LaunchConfiguration("foxglove", default=False)
    foxglove = IncludeLaunchDescription(XMLLaunchDescriptionSource([PathJoinSubstitution([
        FindPackageShare('wasp_autonomous_systems'), 'launch', 'foxglove_bridge_launch.xml'])]),
        launch_arguments={'use_sim_time': loop}.items(),
        condition=launch.conditions.IfCondition(use_foxglove))

    return [
        ExecuteProcess(
            condition=IfCondition(loop),
            cmd=[
                "ros2",
                "bag",
                "play",
                "--read-ahead-queue-size",
                queue_size,
                "--loop",
                "--clock",
                "100",
                "-r",
                rate,
                bag_dir,
                "--qos-profile-overrides-path",
                os.path.join(bag_dir.perform(context),
                             "reliability_override.yaml"),
            ],
            output="screen",
        ),

        ExecuteProcess(
            condition=UnlessCondition(loop),
            cmd=[
                "ros2",
                "bag",
                "play",
                "--read-ahead-queue-size",
                queue_size,
                "-r",
                rate,
                bag_dir,
                "--qos-profile-overrides-path",
                os.path.join(bag_dir.perform(context),
                             "reliability_override.yaml"),
            ],
            output="screen",
        ),

        rviz,
        foxglove
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'loop',
            default_value='false',
            description='Loop the ROS bag'),
        DeclareLaunchArgument(
            'bag_dir', description='Directory to ROS bag to play'),
        DeclareLaunchArgument('rate', default_value='1',
                              description='Rate at which to play back messages. Valid range > 0.0.'),
        DeclareLaunchArgument('queue_size', default_value='100',
                              description='How many messages to read from the bag ahead of time. Bigger value uses more RAM but makes execution faster.'),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Enable or disable RViz'
        ),
        DeclareLaunchArgument(
            'foxglove',
            default_value='false',
            description='Enable or disable Foxglove'
        ),

        OpaqueFunction(function=launch_setup),
    ])
