from glob import glob
import os

from setuptools import find_packages, setup


package_name = "race_vehicle_interface"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["ARDUINO_PROTOCOL.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "firmware", "encoder_direction_uno"),
            glob("firmware/encoder_direction_uno/*"),
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="urrc_hanla",
    maintainer_email="maintainer@example.com",
    description="Fail-safe race vehicle command adapter.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vehicle_interface_node = "
            "race_vehicle_interface.vehicle_interface_node:main",
            "arduino_serial_bridge_node = "
            "race_vehicle_interface.arduino_serial_bridge_node:main",
            "encoder_serial_bridge_node = "
            "race_vehicle_interface.encoder_serial_bridge_node:main",
            "measured_motion_state_node = "
            "race_vehicle_interface.measured_motion_state_node:main",
            "forward_stop_controller_node = "
            "race_vehicle_interface.forward_stop_controller_node:main",
            "manual_keyboard_drive_node = "
            "race_vehicle_interface.manual_keyboard_drive_node:main",
            "cmd_mux_node = race_vehicle_interface.cmd_mux_node:main",
        ],
    },
)
