import os
from glob import glob
from setuptools import setup

package_name = 'wasp_as_ass_4'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Duberg',
    maintainer_email='dduberg@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=[],
    entry_points={
        'console_scripts': [
            f'altitude_hold = {package_name}.altitude_hold:main',
            f'altitude_manual = {package_name}.altitude_manual:main',
            f'altitude_pid = {package_name}.altitude_pid:main'
        ],
    },
)
