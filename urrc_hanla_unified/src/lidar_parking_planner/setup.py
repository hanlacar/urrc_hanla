from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'lidar_parking_planner'


def one_file_per_destination(destination, pattern):
    """Avoid the Jazzy colcon symlink_data multi-file destination bug."""
    return [(destination, [filename]) for filename in glob(pattern)]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=([
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ] + one_file_per_destination(
        os.path.join('share', package_name, 'config'), 'config/*.yaml'
    ) + one_file_per_destination(
        os.path.join('share', package_name, 'launch'), 'launch/*.launch.py'
    ) + one_file_per_destination(
        os.path.join('share', package_name, 'rviz'), 'rviz/*.rviz'
    )),
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='ww',
    maintainer_email='ww@example.com',
    description='Low-speed scan-only parking planner.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'parking_planner_node = lidar_parking_planner.parking_planner_node:main',
            'parking_space_memory_node = lidar_parking_planner.parking_space_memory_node:main',
            'constant_drive_test_node = lidar_parking_planner.constant_drive_test_node:main',
            'encoder_monitor_node = lidar_parking_planner.encoder_monitor_node:main',
        ],
    },
)
