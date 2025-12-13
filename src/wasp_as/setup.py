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
    # Launch files
    (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    # Config files
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    # Worlds
    (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
    ('share/' + package_name, ['package.xml'])
]
# Meshes
data_files.extend(all_files_in_folder('meshes'))
# Models
data_files.extend(all_files_in_folder('models'))
# Media
data_files.extend(all_files_in_folder('media'))

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
            f'path = {package_name}.path:main',
        ],
    },
)
