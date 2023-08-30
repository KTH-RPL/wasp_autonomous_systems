from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'assignment_2'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))
data_files.append(('share/' + package_name + '/rviz', ['rviz/odometry.rviz']))
data_files.append(('share/' + package_name + '/rviz', ['rviz/collision_detection.rviz']))
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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry = assignment_2.odometry:main',
            'odometry_solution = assignment_2.odometry_solution:main',
            'collision_detection = assignment_2.collision_detection:main',
            'collision_detection_solution = assignment_2.collision_detection_solution:main'
        ],
    },
)
