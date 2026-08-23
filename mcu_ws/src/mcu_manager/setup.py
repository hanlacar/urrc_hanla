from glob import glob
import os
from setuptools import find_packages, setup

package_name = "mcu_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="URRC",
    maintainer_email="urrc@example.com",
    description="Independent drive/wheel command arbitration and safety manager.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mcu_manager_node = mcu_manager.mcu_manager_node:main",
        ],
    },
)
