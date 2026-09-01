from glob import glob
import os

from setuptools import find_packages, setup


package_name = "race_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="urrc_hanla",
    maintainer_email="maintainer@example.com",
    description="Camera-based YOLO11 object detection for the race vehicle.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "yolo_camera = race_perception.yolo_camera_node:main",
            "lane_center = race_perception.lane_center_node:main",
            "traffic_light_color = race_perception.traffic_light_color_node:main",
            "lidar_camera_guard = race_perception.lidar_camera_guard_node:main",
        ],
    },
)
