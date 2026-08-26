from setuptools import setup

package_name = 'wasp_as_webots'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/course_world_launch.py',
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/course_mavic_world.wbt', 'worlds/turtlebot_collision_detection.wbt', 'worlds/turtlebot_apartment.wbt',
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/course_mavic_webots.urdf', 'resource/Mavic2Pro.proto',
    'resource/turtlebot_webots.urdf', 'resource/TurtleBot3Burger.proto', 'resource/ros2control.yaml',
    'resource/Kinect.proto', 'resource/turtlebot_webots_rgbd.urdf',
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
    description='Course Webots worlds, robot resources, and launch files for assignments 1, 2, and 4',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
