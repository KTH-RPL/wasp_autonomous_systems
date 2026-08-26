"""webots_ros2 package setup file."""

from setuptools import setup


package_name = 'webots_ros2_mavic'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py', 'launch/soak_test_launch.py', 'launch/test_supervisor_launch.py', 'launch/course_world_launch.py', 'launch/turtlebot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/mavic_world.wbt', 'worlds/.mavic_world.wbproj', 'worlds/course_mavic_world.wbt', 'worlds/turtlebot_collision_detection.wbt',
    'worlds/turtlebot_apartment.wbt',
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/mavic_webots.urdf', 'resource/course_mavic_webots.urdf', 'resource/Mavic2Pro.proto',
    'resource/turtlebot_webots.urdf', 'resource/TurtleBot3Burger.proto', 'resource/ros2control.yaml',
    'resource/Kinect.proto', 'resource/turtlebot_webots_rgbd.urdf',
]))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='2025.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Mavic 2 Pro robot ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
