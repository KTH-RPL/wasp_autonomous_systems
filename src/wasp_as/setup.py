from setuptools import find_packages, setup

package_name = 'wasp_as'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
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
