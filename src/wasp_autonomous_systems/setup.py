from setuptools import find_packages, setup
import os
from glob import glob

def all_files_in_folder(folder: str):
    files = glob(os.path.join(folder, "**/*"), recursive=True)
    files = [f for f in files if os.path.isfile(f)]
    return [(os.path.join('share', package_name, os.path.split(path)[0]), [path]) for path in files]

package_name = 'wasp_autonomous_systems'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    # Launch files
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    # Params
    (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
    # Urdf
    (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    # Worlds
    (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
    ('share/' + package_name, ['package.xml'])
]
# Meshes
data_files.extend(all_files_in_folder('meshes'))
# Models
data_files.extend(all_files_in_folder('models'))
# Photos
data_files.extend(all_files_in_folder('photos'))

# data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
# data_files.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))
# data_files.append((os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.[world]*'))))
# # data_files.append((os.path.join('share', package_name, 'models'), glob(os.path.join('models', '**/*.*'), recursive=True)))
# # data_files.extend([(os.path.join('share', package_name, os.path.split(path)[0]), [path]) for path in glob.glob('models/**', recursive=True)])
# data_files.append(('share/' + package_name + '/resource', ['resource/ros2control.yaml']))
# data_files.append(('share/' + package_name + '/resource', ['resource/TurtleBot3Burger.proto']))
# data_files.append(('share/' + package_name + '/resource', ['resource/Kinect.proto']))
# data_files.append(('share/' + package_name + '/resource', ['resource/Mavic2Pro.proto']))
# data_files.append((os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '**/*'))))
# data_files.append(('share/' + package_name + '/urdf', ['urdf/turtlebot_webots.urdf']))
# data_files.append(('share/' + package_name + '/urdf', ['urdf/turtlebot_webots_rgbd.urdf']))
# data_files.append(('share/' + package_name + '/urdf', ['urdf/car.urdf']))
# data_files.append(('share/' + package_name + '/urdf', ['urdf/mavic_webots.urdf']))
# data_files.append(('share/' + package_name, ['package.xml']))

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
