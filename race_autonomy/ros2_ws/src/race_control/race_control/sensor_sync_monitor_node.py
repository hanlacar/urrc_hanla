#!/usr/bin/env python3
"""Report D456 RGB/depth/RTAB odom/BEV timestamp alignment."""

import json
import math
from collections import deque

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from .visual_slam_route import nearest_stamp_skew


class SensorSyncMonitor(Node):
    def __init__(self):
        super().__init__("sensor_sync_monitor")
        defaults = {
            "rgb_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "odom_topic": "/odom", "bev_topic": "/camera/bev/path",
            "rgb_depth_max_skew_sec": 0.015,
            "rgb_info_max_skew_sec": 0.015,
            "rgb_odom_max_skew_sec": 0.10,
            "rgb_bev_max_skew_sec": 0.001,
            "maximum_age_sec": 0.30, "publish_rate_hz": 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.stamps = {key: deque(maxlen=120)
                       for key in ("rgb", "depth", "info", "odom", "bev")}
        self.create_subscription(Image, self.p("rgb_topic"),
                                 lambda m: self.add("rgb", m), qos_profile_sensor_data)
        self.create_subscription(Image, self.p("depth_topic"),
                                 lambda m: self.add("depth", m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.p("camera_info_topic"),
                                 lambda m: self.add("info", m), qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p("odom_topic"),
                                 lambda m: self.add("odom", m),
                                 qos_profile_sensor_data)
        self.create_subscription(Path, self.p("bev_topic"),
                                 lambda m: self.add("bev", m), 10)
        self.status_pub = self.create_publisher(String, "/diagnostics/sensor_sync", 10)
        self.valid_pub = self.create_publisher(Bool, "/diagnostics/sensor_sync_valid", 10)
        self.create_timer(1.0/max(0.1, float(self.p("publish_rate_hz"))), self.tick)

    def p(self, name):
        return self.get_parameter(name).value

    def add(self, key, message):
        stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec)*1e-9
        if math.isfinite(stamp) and stamp > 0.0:
            self.stamps[key].append(stamp)

    def tick(self):
        if not self.stamps["rgb"]:
            payload = {"valid": False, "reason": "missing_rgb"}
            self.valid_pub.publish(Bool(data=False))
            self.status_pub.publish(String(data=json.dumps(payload)))
            return
        rgb = self.stamps["rgb"][-1]
        now = self.get_clock().now().nanoseconds*1e-9
        skews = {key: nearest_stamp_skew(self.stamps[key], rgb)
                 for key in ("depth", "info", "odom", "bev")}
        limits = {
            "depth": float(self.p("rgb_depth_max_skew_sec")),
            "info": float(self.p("rgb_info_max_skew_sec")),
            "odom": float(self.p("rgb_odom_max_skew_sec")),
            "bev": float(self.p("rgb_bev_max_skew_sec")),
        }
        required = ("depth", "info", "odom")
        aligned = all(skews[key] <= limits[key] for key in required)
        fresh = now-rgb <= float(self.p("maximum_age_sec"))
        valid = aligned and fresh
        payload = {
            "valid": valid, "rgb_age_ms": (now-rgb)*1000.0,
            "rgb_depth_skew_ms": skews["depth"]*1000.0,
            "rgb_info_skew_ms": skews["info"]*1000.0,
            "rgb_odom_skew_ms": skews["odom"]*1000.0,
            "rgb_bev_skew_ms": (skews["bev"]*1000.0
                                if math.isfinite(skews["bev"]) else None),
            "bev_present": bool(self.stamps["bev"]),
            "sample_counts": {key: len(value) for key, value in self.stamps.items()},
        }
        self.valid_pub.publish(Bool(data=valid))
        self.status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))


def main(args=None):
    rclpy.init(args=args); node=SensorSyncMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
