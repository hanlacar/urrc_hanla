#!/usr/bin/env python3

import math
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8, String

from race_interfaces.msg import AutonomyObservation, PathState


def traffic_light_code(value):
    return {
        "RED": AutonomyObservation.TRAFFIC_RED,
        "YELLOW": AutonomyObservation.TRAFFIC_YELLOW,
        "GREEN": AutonomyObservation.TRAFFIC_GREEN,
    }.get(str(value).strip().upper(), AutonomyObservation.TRAFFIC_UNKNOWN)


class AutonomyOutputNode(Node):
    """Aggregate internal ROS topics into two integration topics."""

    def __init__(self):
        super().__init__("autonomy_output_node")
        defaults = {
            "observation_topic": "/autonomy/observation",
            "path_output_topic": "/autonomy/path",
            "publish_rate_hz": 20.0,
            "perception_timeout_sec": 0.5,
            "imu_timeout_sec": 0.2,
            "speed_timeout_sec": 0.5,
            "path_timeout_sec": 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.values = {
            "perception_valid": False,
            "imu_valid": False,
            "speed_valid": False,
            "stop_detected": False,
            "traffic_light": AutonomyObservation.TRAFFIC_UNKNOWN,
            "pitch_deg": 0.0,
            "roll_deg": 0.0,
            "yaw_deg": 0.0,
            "speed_kph": 0.0,
            "path_valid": False,
            "path_confidence": 0.0,
            "path_mode": 0,
            "path": Path(),
        }
        self.times = {}

        self.observation_pub = self.create_publisher(
            AutonomyObservation, self.param("observation_topic"), 10
        )
        self.path_pub = self.create_publisher(
            PathState, self.param("path_output_topic"), 10
        )

        self.create_subscription(Bool, "/camera/perception_valid", lambda m: self.set_value("perception_valid", bool(m.data)), 10)
        self.create_subscription(Bool, "/perception/stop_detected", lambda m: self.set_value("stop_detected", bool(m.data)), 10)
        self.create_subscription(String, "/perception/traffic_light_state", lambda m: self.set_value("traffic_light", traffic_light_code(m.data)), 10)
        self.create_subscription(Bool, "/imu/valid", lambda m: self.set_value("imu_valid", bool(m.data)), 10)
        self.create_subscription(Float32, "/imu/pitch_deg", lambda m: self.set_number("pitch_deg", m.data), 10)
        self.create_subscription(Float32, "/imu/roll_deg", lambda m: self.set_number("roll_deg", m.data), 10)
        self.create_subscription(Float32, "/imu/relative_yaw_deg", lambda m: self.set_number("yaw_deg", m.data), 10)
        self.create_subscription(Bool, "/vehicle/speed_valid", lambda m: self.set_value("speed_valid", bool(m.data)), 10)
        self.create_subscription(Float32, "/vehicle/speed_kph", lambda m: self.set_number("speed_kph", m.data), 10)
        self.create_subscription(Bool, "/camera/path_valid", lambda m: self.set_value("path_valid", bool(m.data)), 10)
        self.create_subscription(Float32, "/camera/path_confidence", lambda m: self.set_number("path_confidence", m.data), 10)
        self.create_subscription(Int8, "/camera/path_mode", lambda m: self.set_value("path_mode", int(m.data)), 10)
        self.create_subscription(Path, "/camera/path", lambda m: self.set_value("path", m), 10)

        rate = max(1.0, float(self.param("publish_rate_hz")))
        self.create_timer(1.0 / rate, self.publish_outputs)

    def param(self, name):
        return self.get_parameter(name).value

    def set_value(self, name, value):
        self.values[name] = value
        self.times[name] = time.monotonic()

    def set_number(self, name, value):
        value = float(value)
        if math.isfinite(value):
            self.set_value(name, value)

    def fresh(self, names, timeout_name):
        now = time.monotonic()
        timeout = float(self.param(timeout_name))
        return all(name in self.times and now - self.times[name] <= timeout for name in names)

    def publish_outputs(self):
        now = self.get_clock().now().to_msg()
        perception_fresh = self.fresh(
            ("perception_valid", "stop_detected"), "perception_timeout_sec"
        )
        imu_fresh = self.fresh(
            ("imu_valid", "pitch_deg", "roll_deg", "yaw_deg"), "imu_timeout_sec"
        )
        speed_fresh = self.fresh(("speed_valid", "speed_kph"), "speed_timeout_sec")
        path_fresh = self.fresh(
            ("path_valid", "path_confidence", "path_mode", "path"), "path_timeout_sec"
        )

        observation = AutonomyObservation()
        observation.header.stamp = now
        observation.header.frame_id = "base_link"
        observation.perception_valid = perception_fresh and self.values["perception_valid"]
        observation.imu_valid = imu_fresh and self.values["imu_valid"]
        observation.speed_valid = speed_fresh and self.values["speed_valid"]
        observation.stop_detected = observation.perception_valid and self.values["stop_detected"]
        observation.traffic_light = self.values["traffic_light"] if perception_fresh else AutonomyObservation.TRAFFIC_UNKNOWN
        observation.pitch_deg = float(self.values["pitch_deg"])
        observation.roll_deg = float(self.values["roll_deg"])
        observation.yaw_deg = float(self.values["yaw_deg"])
        observation.speed_kph = float(self.values["speed_kph"])
        self.observation_pub.publish(observation)

        path_state = PathState()
        path_state.header.stamp = now
        path_state.header.frame_id = "base_link"
        path_state.valid = path_fresh and self.values["path_valid"]
        path_state.confidence = float(self.values["path_confidence"]) if path_fresh else 0.0
        path_state.mode = int(self.values["path_mode"]) if path_fresh else 0
        path_state.path = self.values["path"] if path_fresh else Path()
        self.path_pub.publish(path_state)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyOutputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
