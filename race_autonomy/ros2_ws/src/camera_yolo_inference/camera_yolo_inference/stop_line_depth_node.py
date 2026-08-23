#!/usr/bin/env python3
"""Estimate stop-line distance from aligned D456 depth and YOLO detections."""
import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String


class StopLineDepthNode(Node):
    def __init__(self):
        super().__init__("stop_line_depth_node")
        for name, value in {"depth_topic":"/camera/depth/image_raw", "max_pair_age_sec":0.35,
                            "minimum_valid_pixels":20, "roi_shrink_ratio":0.25}.items():
            self.declare_parameter(name, value)
        self.box = None
        self.box_time = 0.0
        self.depth_times = deque(maxlen=120)
        self.distance_pub = self.create_publisher(Float32, "/perception/stop_line_distance_m", 10)
        self.valid_pub = self.create_publisher(Bool, "/perception/stop_line_distance_valid", 10)
        self.depth_fps_pub = self.create_publisher(Float32, "/perception/depth_fps", 10)
        self.status_pub = self.create_publisher(String, "/perception/stop_line_depth_status", 10)
        self.debug_pub = self.create_publisher(String, "/perception/stop_line_depth_debug_json", 10)
        self.create_subscription(String, "/perception/detections_json", self.on_detections, 10)
        self.create_subscription(Image, self.get_parameter("depth_topic").value, self.on_depth, qos_profile_sensor_data)
        self.create_timer(1.0, self.publish_depth_fps)

    def publish_depth_fps(self):
        fps = 0.0
        if len(self.depth_times) >= 2:
            duration = self.depth_times[-1] - self.depth_times[0]
            if duration > 0.0:
                fps = (len(self.depth_times) - 1) / duration
        self.depth_fps_pub.publish(Float32(data=float(fps)))

    def on_detections(self, msg):
        try:
            detections = json.loads(msg.data).get("detections", [])
            stops = [d for d in detections if str(d.get("class_name", "")).lower().replace("_", "") in {"stop", "stopline"} and len(d.get("xyxy", [])) == 4]
            self.box = max(stops, key=lambda d: float(d.get("confidence", 0.0)))["xyxy"] if stops else None
            self.box_time = time.monotonic()
        except (ValueError, TypeError, KeyError, json.JSONDecodeError):
            self.box = None

    @staticmethod
    def depth_array(msg):
        if msg.encoding in ("16UC1", "mono16"):
            row = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)
            return row[:, :msg.width].astype(np.float32) * 0.001
        if msg.encoding == "32FC1":
            row = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)
            return row[:, :msg.width]
        raise ValueError("unsupported depth encoding")

    def on_depth(self, msg):
        self.depth_times.append(time.monotonic())
        valid = False
        distance = math.nan
        valid_pixels = 0
        status = "unknown"
        roi = None
        try:
            if self.box is None or time.monotonic() - self.box_time > float(self.get_parameter("max_pair_age_sec").value):
                raise ValueError("no_fresh_stop_detection")
            depth = self.depth_array(msg)
            x1, y1, x2, y2 = self.box
            shrink = float(self.get_parameter("roi_shrink_ratio").value)
            dx, dy = (x2-x1)*shrink, (y2-y1)*shrink
            x1, x2 = max(0, int(x1+dx)), min(msg.width, int(x2-dx))
            y1, y2 = max(0, int(y1+dy)), min(msg.height, int(y2-dy))
            roi = [x1, y1, x2, y2]
            values = depth[y1:y2, x1:x2]
            values = values[np.isfinite(values) & (values > 0.05)]
            valid_pixels = int(values.size)
            if valid_pixels < int(self.get_parameter("minimum_valid_pixels").value):
                raise ValueError("insufficient_depth_pixels")
            distance = float(np.median(values)); valid = math.isfinite(distance)
            status = "ok" if valid else "non_finite_distance"
        except (ValueError, TypeError) as error:
            status = str(error)
        self.valid_pub.publish(Bool(data=valid))
        self.status_pub.publish(String(data=status))
        self.debug_pub.publish(String(data=json.dumps({
            "valid": valid,
            "status": status,
            "distance_m": distance if valid else None,
            "valid_pixels": valid_pixels,
            "minimum_valid_pixels": int(self.get_parameter("minimum_valid_pixels").value),
            "depth_encoding": msg.encoding,
            "depth_width": int(msg.width),
            "depth_height": int(msg.height),
            "roi_xyxy": roi,
        })))
        if valid:
            self.distance_pub.publish(Float32(data=distance))


def main(args=None):
    rclpy.init(args=args); node = StopLineDepthNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
