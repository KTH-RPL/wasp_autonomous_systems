from setuptools import find_packages, setup
import os
from glob import glob


def all_files_in_folder(folder: str):
    files = glob(os.path.join(folder, "**/*"), recursive=True)
    files = [f for f in files if os.path.isfile(f)]
    return [(os.path.join('share', package_name, os.path.split(path)[0]), [path]) for path in files]


package_name = 'wasp_as'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml'])
]
# car.xacro/meshes - used by the RViz "Car" model in the KITTI-based launches
# (ass_1_3, ass_2_3, elective task A), not Gazebo-specific despite living
# alongside the Gazebo assets removed in 259e068 - that commit deleted these
# too by mistake, only noticed once elective task A's launch failed on the
# missing car.xacro.
data_files.extend(all_files_in_folder('models'))
data_files.extend(all_files_in_folder('meshes'))

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
            f'autonomous_controller = {package_name}.autonomous_controller:main',
            f'encoders = {package_name}.encoders:main',
            f'legacy_encoders_bridge = {package_name}.legacy_encoders_bridge:main',
            f'path = {package_name}.path:main',
        ],
    },
)
