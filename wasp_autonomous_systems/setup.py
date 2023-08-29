from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wasp_autonomous_systems'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))
data_files.append((os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.[wbt]*'))))
data_files.append(('share/' + package_name + '/resource', ['resource/ros2control.yaml']))
data_files.append(('share/' + package_name + '/resource', ['resource/TurtleBot3Burger.proto']))
data_files.append(('share/' + package_name + '/resource', ['resource/Kinect.proto']))
data_files.append((os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '**/*'))))
data_files.append(('share/' + package_name + '/urdf', ['urdf/turtlebot_webots.urdf']))
data_files.append(('share/' + package_name + '/urdf', ['urdf/car.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Duberg',
    maintainer_email='dduberg@kth.se',
    description='Resources for the WASP Autonomous Systems course',
    license='TODO: License declaration',
    tests_require=[],
    entry_points={
        'console_scripts': [
            'autonomous_controller = wasp_autonomous_systems.autonomous_controller:main',
            'encoders = wasp_autonomous_systems.encoders:main',
            'path = wasp_autonomous_systems.path:main',
        ],
    },
)
