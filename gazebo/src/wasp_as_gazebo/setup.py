from setuptools import setup

package_name = 'wasp_as_gazebo'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/ass_1_1_launch.py',
    'launch/ass_2_collision_launch.py',
    'launch/ass_4_manual_launch.py',
    'launch/ass_4_pid_launch.py',
    'launch/quadrotor_world_launch.py',
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/course_quadrotor_world.sdf',
    'worlds/turtlebot_apartment.sdf',
    'worlds/turtlebot_collision_detection.sdf',
]))
data_files.append(('share/' + package_name + '/models', [
    'models/turtlebot3_waffle_rgbd.urdf',
]))
data_files.append(('share/' + package_name + '/models/turtlebot3_waffle_rgbd', [
    'models/turtlebot3_waffle_rgbd/model.sdf',
    'models/turtlebot3_waffle_rgbd/model.config',
]))
data_files.append(('share/' + package_name + '/config', [
    'config/quadrotor_bridge.yaml',
    'config/turtlebot_bridge.yaml',
    'config/turtlebot_rgbd_bridge.yaml',
]))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='Patric Jensfelt',
    maintainer_email='patric@kth.se',
    description='Gazebo alternative to the Webots simulator for the camera-less assignments',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mavic_controller = wasp_as_gazebo.mavic_controller:main',
            'gps_bridge = wasp_as_gazebo.gps_bridge:main',
            'cmd_vel_watchdog = wasp_as_gazebo.cmd_vel_watchdog:main',
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],
    }
)
