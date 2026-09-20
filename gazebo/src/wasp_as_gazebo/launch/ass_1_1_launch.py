#!/usr/bin/env python
"""Assignment 1.1, RGB-D camera in an apartment - Gazebo alternative to
wasp_as_ass_1/launch/turtlebot_apartment_launch.py.

This is the one task here that is not guaranteed to work everywhere. It
needs an RGB-D camera, so unlike the other two worlds it loads gz-sim's
rendering-backed sensors system - the part with a history of failing on
macOS (gazebosim/gz-sim#960, #2877). It has been seen rendering correctly
there on the Gazebo build Pixi installs, but not for long enough to call
it dependable.

It is offered on every platform regardless, with a warning on macOS. This
whole setup exists for people who cannot run Webots at all, and for them a
task that might work beats one that refuses to start.

The robot publishes on the same topics the Webots RGB-D driver does -
/camera/color/image_raw, /camera/depth/image_raw, /camera/color/points -
so wasp_as_ass_1's own turtlebot_simulation.rviz config is used unchanged
(the renaming happens in config/turtlebot_rgbd_bridge.yaml). Drive it with
the existing teleop task in a second terminal:

    pixi run ass_1_1_teleop
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wasp_as_gazebo.launch_utils import gz_args, gz_gui


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('wasp_as_gazebo')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    world = os.path.join(package_dir, 'worlds', 'turtlebot_apartment.sdf')
    gui = LaunchConfiguration('gui').perform(context)

    # Top-down over the house (which sits at -3 1 in this world), so the
    # rooms and doorways are laid out in front of you while driving.
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': gz_args(world),
            'on_exit_shutdown': 'true',
        }.items(),
    )] + gz_gui(gui, camera_pose='-3 1 12 0 1.57 0')


def generate_launch_description():
    package_dir = get_package_share_directory('wasp_as_gazebo')
    ass_1_dir = get_package_share_directory('wasp_as_ass_1')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    robot = 'turtlebot3_waffle_rgbd'
    model_path = os.path.join(package_dir, 'models', robot, 'model.sdf')
    urdf_path = os.path.join(package_dir, 'models', robot + '.urdf')
    bridge_config = os.path.join(
        package_dir, 'config', 'turtlebot_rgbd_bridge.yaml')

    # Both the apartment (model://turtlebot3_house) and the robot's own
    # meshes (model://turtlebot3_common/...) resolve out of the
    # ros-jazzy-turtlebot3-gazebo conda package, so neither is vendored here.
    resource_paths = [
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(turtlebot3_gazebo_dir, 'models')),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(package_dir, 'models')),
    ]

    # RViz's RobotModel display needs /robot_description, which Gazebo does
    # not provide - this URDF is the twin of the SDF spawned above. Read
    # here rather than run through xacro: the file has no xacro directives
    # left, and xacro is not installed in the gazebo environment.
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
        output='screen',
        respawn=True,
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='robot_spawner',
        arguments=['-name', robot, '-file', model_path,
                   '-x', '0.0', '-y', '0.0', '-z', '0.01'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{
            'use_sim_time': True,
            'config_file': bridge_config,
        }],
        output='screen',
        respawn=True,
    )

    # Gazebo's DiffDrive has no command timeout, unlike the ros2_control
    # diff_drive_controller the Webots path uses - see cmd_vel_watchdog.py.
    cmd_vel_watchdog = Node(
        package='wasp_as_gazebo',
        executable='cmd_vel_watchdog',
        name='cmd_vel_watchdog',
        parameters=[{'use_sim_time': True}],
        output='screen',
        respawn=True,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(ass_1_dir, 'rviz', 'turtlebot_simulation.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # encoders reads control_msgs/DynamicJointState, which ros2_control
    # publishes on the Webots path. Gazebo emits a plain JointState instead,
    # so this converts between them - see joint_state_bridge.py.
    joint_state_bridge = Node(
        package='wasp_as_gazebo',
        executable='joint_state_bridge',
        name='joint_state_bridge',
        parameters=[{'use_sim_time': True}],
        output='screen',
        respawn=True,
    )

    encoders = Node(
        package='wasp_as',
        executable='encoders',
        name='encoders',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # This task has run correctly on macOS, camera image, depth and point
    # cloud included. It is still the only task here that depends on
    # Gazebo's rendering-backed sensors, which have a poor history on that
    # platform, so the note stays - but as "tell us if it misbehaves"
    # rather than "expect this to fail".
    warning = []
    if sys.platform == 'darwin':
        warning = [LogInfo(msg=(
            '\n'
            '  NOTE: this is the only task here that uses a simulated camera.\n'
            '  It is expected to work on macOS, but it is the one task where\n'
            '  Gazebo has given trouble on this platform in the past. If the\n'
            '  camera views in RViz stay empty, or Gazebo closes by itself,\n'
            '  that is not something you did wrong - please tell the course\n'
            '  staff. Everything else in this setup is unaffected.\n'))]

    return LaunchDescription(warning + resource_paths + [
        DeclareLaunchArgument(
            'gui', default_value='auto',
            description='Gazebo 3D window. On by default on every platform; '
                        'pass false to run headless (useful under WSL2, '
                        'where it falls back to software rendering).'),
        OpaqueFunction(function=launch_setup),
        spawn,
        bridge,
        cmd_vel_watchdog,
        joint_state_bridge,
        robot_state_publisher,
        rviz,
        encoders,
    ])
