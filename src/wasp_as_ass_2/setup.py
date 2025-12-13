from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wasp_as_ass_2'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    (os.path.join('share', package_name, 'rviz'), glob('config/*.rviz')),
    ('share/' + package_name, ['package.xml'])
]

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
            f'camera = {package_name}.camera:main',
            f'odometry = {package_name}.odometry:main',
            f'collision_detection = {package_name}.collision_detection:main',
            f'kitti_segmentation_extra = {package_name}.kitti_segmentation_extra:main',
            f'segmentation = {package_name}.segmentation:main'
        ],
    },
)
