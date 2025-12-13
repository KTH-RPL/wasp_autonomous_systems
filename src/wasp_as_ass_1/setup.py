import os
from glob import glob
from setuptools import setup

package_name = 'wasp_as_ass_1'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    (os.path.join('share', package_name, 'rviz'), glob('config/*.rviz')),
    ('share/' + package_name, ['package.xml'])
]

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Duberg',
    maintainer_email='dduberg@kth.se',
    description='WASP Autonomous Systems Course Assignment 1',
    license='TODO: License declaration',
    tests_require=[],
    entry_points={
        'console_scripts': [
        ],
    },
)
