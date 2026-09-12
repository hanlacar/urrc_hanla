from glob import glob
import os

from setuptools import find_packages, setup


package_name = "race_control"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "routes"), glob("routes/*.csv") + glob("routes/*.yaml")),
    ],
    install_requires=["setuptools"], tests_require=["pytest"], zip_safe=True,
    maintainer="urrc_hanla", maintainer_email="maintainer@example.com",
    description="Pure Pursuit path tracking for the race vehicle.", license="Apache-2.0",
    entry_points={"console_scripts": [
        "pure_pursuit = race_control.pure_pursuit_node:main",
        "autonomy_output = race_control.autonomy_output_node:main",
        "course_mission = race_control.course_mission_node:main",
        "fused_waypoint = race_control.fused_waypoint_node:main",
        "section_transition = race_control.section_transition_node:main",
        "force_section = race_control.force_section_node:main",
        "curvature_speed_planner = race_control.curvature_speed_planner_node:main",
        "visual_slam_route = race_control.visual_slam_route_node:main",
        "sensor_sync_monitor = race_control.sensor_sync_monitor_node:main",
    ]},
)
