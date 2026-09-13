from glob import glob
import os

from setuptools import find_packages, setup

package_name = "t870_mcu"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="t870",
    maintainer_email="user@example.com",
    description="T870 Arduino serial bridge + command arbiter",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bridge = t870_mcu.bridge_node:main",
            "manager = t870_mcu.manager_node:main",
        ]
    },
)
