from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'lidar_ws_plus_bringup'


def one_file_per_destination(destination, pattern):
    """Avoid colcon's symlink-install collision on multi-file data entries."""
    return [(destination, [filename]) for filename in glob(pattern)]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=([
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ] + one_file_per_destination(
        os.path.join('share', package_name, 'launch'), 'launch/*.launch.py'
    ) + one_file_per_destination(
        os.path.join('share', package_name, 'config'), 'config/*.yaml'
    ) + one_file_per_destination(
        os.path.join('share', package_name, 'scripts'), 'scripts/*.sh'
    )),
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='wan',
    maintainer_email='wan@example.com',
    description='Integrated real-vehicle LiDAR bringup and command mux.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'command_mux = lidar_ws_plus_bringup.command_mux:main',
        ],
    },
)
