#!/usr/bin/env python
"""Assignment 2, collision detection - Gazebo alternative to
wasp_as_ass_2/launch/collision_detection_webots_launch.py.

collision_detection and autonomous_controller are the unmodified
wasp_as_ass_2/wasp_as nodes, and they stay bundled into this one launch
file for the same reason the Webots version does: the room is tiny and
autonomous_controller drives forward from its first tick, so anything that
starts late misses the first collision and finds the robot already wedged
into a wall.

The world loads paused and is released once the Gazebo window is actually
showing it, so the first collision is not already over by the time you can
see the arena. Nothing has to be clicked; see launch_utils.

No ros2_control here, unlike the Webots path - Gazebo's DiffDrive system
takes a velocity command straight off a topic, so the whole
controller_manager/spawner layer has nothing to do. /cmd_vel is still
bridged as TwistStamped so autonomous_controller.py needs no change.

The Gazebo 3D window is on by default, as it is everywhere in this
package; see quadrotor_world_launch.py. RViz runs either way, on
wasp_as_ass_2's own collision_detection.rviz config, unchanged.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wasp_as_gazebo.launch_utils import (gui_enabled, gz_args, gz_gui,
                                         gz_unpause_when_gui_ready)


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('wasp_as_gazebo')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    world = os.path.join(package_dir, 'worlds', 'turtlebot_collision_detection.sdf')
    gui = LaunchConfiguration('gui').perform(context)

    # Straight down onto the 1 x 1 m arena from 2 m - Gazebo's default
    # view (6 m back, 6 m up) makes a room this small unreadable.
    # Load paused when there is a window coming, and let
    # gz_unpause_when_gui_ready release it once that window is actually
    # showing the scene. autonomous_controller drives forward from its first
    # tick and the robot is at a wall ~1.6 s later, so an unpaused load means
    # the first collision - the whole point of the exercise - has already
    # happened by the time anyone can see the arena. Headless, there is
    # nothing to wait for, so it runs straight away.
    run_immediately = not gui_enabled(gui)

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': gz_args(world, run=run_immediately),
            'on_exit_shutdown': 'true',
        }.items(),
    )] + gz_gui(gui, camera_pose='0 0 2.0 0 1.57 0') \
       + gz_unpause_when_gui_ready('turtlebot_collision_detection', gui)


def generate_launch_description():
    package_dir = get_package_share_directory('wasp_as_gazebo')
    ass_2_dir = get_package_share_directory('wasp_as_ass_2')

    bridge_config = os.path.join(
        package_dir, 'config', 'turtlebot_bridge.yaml')

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

    # Feeds collision_detection.rviz's "Current Position" display, which is
    # configured against the Webots GPS device's /TurtleBot3Burger/gps.
    gps_bridge = Node(
        package='wasp_as_gazebo',
        executable='gps_bridge',
        name='gps_bridge',
        parameters=[{
            'use_sim_time': True,
            'odometry_topic': '/model/TurtleBot3Burger/ground_truth_odometry',
            'point_topic': '/TurtleBot3Burger/gps',
            'frame_id': 'gps',
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

    rqt_plot = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='rqt_plot',
        # No topics on the command line - same as the Webots launch, where
        # rqt_plot's CLI topic arguments were confirmed never to subscribe
        # in this environment. Add /imu/linear_acceleration/x once it is up.
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(ass_2_dir, 'rviz', 'collision_detection.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    collision_detection = Node(
        package='wasp_as_ass_2',
        executable='collision_detection',
        name='collision_detection',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    autonomous_controller = Node(
        package='wasp_as',
        executable='autonomous_controller',
        name='autonomous_controller',
        parameters=[{'use_sim_time': True}],
        output='screen',
        respawn=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='auto',
            description='Gazebo 3D window. On by default on every platform; '
                        'pass false to run headless (useful under WSL2, '
                        'where it falls back to software rendering).'),
        OpaqueFunction(function=launch_setup),
        bridge,
        gps_bridge,
        cmd_vel_watchdog,
        rqt_plot,
        rviz,
        collision_detection,
        autonomous_controller,
    ])
