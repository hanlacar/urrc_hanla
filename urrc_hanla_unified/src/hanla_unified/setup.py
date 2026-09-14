from glob import glob
import os
from setuptools import find_packages, setup

package_name = "hanla_unified"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml") + glob("config/*.xml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "routes"), glob("routes/*.csv")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="urrc_hanla",
    maintainer_email="maintainer@example.com",
    description="Integrated mission decision and final command authority",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "mission_decision = hanla_unified.mission_decision_node:main",
        "csv_only_command = hanla_unified.csv_only_command_node:main",
        "mcu_odom_adapter = hanla_unified.mcu_odom_adapter:main",
    ]},
)
