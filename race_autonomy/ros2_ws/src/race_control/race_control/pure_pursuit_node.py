#!/usr/bin/env python3

import json
import math
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8, String
from camera_navigation.path_generator import path_mode_name

from .pure_pursuit import (
    dynamic_lookahead,
    rpm_to_speed_mps,
    select_lookahead_point,
    steering_angle_deg,
)


class PurePursuitNode(Node):
    """Convert a vehicle-relative camera path into speed and steering targets."""

    def __init__(self):
        super().__init__("pure_pursuit")
        defaults = {
            "path_topic": "/perception/local_path_json",
            "path_input_type": "json",
            "nav_path_valid_topic": "/camera/path_valid",
            "nav_path_confidence_topic": "/camera/path_confidence",
            "nav_path_lateral_to_right_sign": -1.0,
            "speed_feedback_topic": "/vehicle/speed_mps",
            "speed_feedback_is_rpm": False,
            "wheel_radius_m": 0.15,
            "rpm_per_wheel_rpm": 1.0,
            "steering_topic": "/camera/target_steering_deg",
            "target_speed_topic": "/camera/target_speed_mps",
            "status_topic": "/control/pure_pursuit_status_json",
            "lookahead_topic": "/control/lookahead_m",
            "lookahead_point_topic": "/control/lookahead_point_json",
            "wheelbase_m": 0.73,
            "minimum_lookahead_m": 0.8,
            "lookahead_gain_s": 1.0,
            "maximum_lookahead_m": 2.0,
            "maximum_steering_deg": 27.0,
            "steering_sign": 1.0,
            "minimum_confidence": 0.45,
            "path_timeout_sec": 0.3,
            "control_rate_hz": 20.0,
            "commanded_speed_mps": 0.0,
            "allow_reverse": False,
            "steering_rate_limit_deg_s": 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.path = []
        self.confidence = 0.0
        self.path_time = None
        self.speed_mps = 0.0
        self.observed_mode = 0
        self.active_mode = 0
        self.path_shift_m = 0.0
        self.curvature = 0.0
        self.limited_steering = 0.0
        self.last_control_time = time.monotonic()
        self.steering_pub = self.create_publisher(Float32, self.param("steering_topic"), 10)
        self.speed_pub = self.create_publisher(Float32, self.param("target_speed_topic"), 10)
        self.status_pub = self.create_publisher(String, self.param("status_topic"), 10)
        self.path_status_pub = self.create_publisher(String, "/control/path_status", 10)
        self.lookahead_pub = self.create_publisher(
            Float32, self.param("lookahead_topic"), 10
        )
        self.lookahead_point_pub = self.create_publisher(
            String, self.param("lookahead_point_topic"), 10
        )
        if str(self.param("path_input_type")) == "nav_path":
            self.nav_path_valid = False
            self.create_subscription(Path, self.param("path_topic"), self.on_nav_path, 10)
            self.create_subscription(
                Bool, self.param("nav_path_valid_topic"),
                lambda msg: setattr(self, "nav_path_valid", bool(msg.data)), 10,
            )
            self.create_subscription(
                Float32, self.param("nav_path_confidence_topic"),
                lambda msg: setattr(self, "confidence", float(msg.data)), 10,
            )
        else:
            self.nav_path_valid = True
            self.create_subscription(String, self.param("path_topic"), self.on_path, 10)
        self.create_subscription(Float32, self.param("speed_feedback_topic"), self.on_speed, 10)
        self.create_subscription(Int8, "/camera/path_observed_mode", lambda m:setattr(self,"observed_mode",int(m.data)), 10)
        self.create_subscription(Int8, "/camera/path_mode", lambda m:setattr(self,"active_mode",int(m.data)), 10)
        self.create_subscription(Float32, "/camera/path_jump_m", lambda m:setattr(self,"path_shift_m",float(m.data)), 10)
        self.create_subscription(Float32, "/control/path_curvature", lambda m:setattr(self,"curvature",float(m.data)), 10)
        self.create_timer(1.0 / max(float(self.param("control_rate_hz")), 1.0), self.control)
        self.get_logger().warning("Controller ready with propulsion target locked at 0 m/s")

    def param(self, name):
        return self.get_parameter(name).value

    def on_path(self, msg):
        try:
            payload = json.loads(msg.data)
            points = payload.get("points", [])
            if not isinstance(points, list):
                raise ValueError("points must be a list")
            self.path = points if bool(payload.get("detected", False)) else []
            self.confidence = float(payload.get("confidence", 0.0))
            self.path_time = time.monotonic()
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.path, self.confidence, self.path_time = [], 0.0, None
            self.get_logger().warning(f"Invalid local path: {exc}")

    def on_speed(self, msg):
        if not math.isfinite(msg.data):
            return
        if bool(self.param("speed_feedback_is_rpm")):
            self.speed_mps = rpm_to_speed_mps(
                msg.data, float(self.param("wheel_radius_m")),
                float(self.param("rpm_per_wheel_rpm")),
            )
        else:
            self.speed_mps = float(msg.data)

    def on_nav_path(self, msg):
        sign = float(self.param("nav_path_lateral_to_right_sign"))
        self.path = [
            [float(pose.pose.position.x), sign * float(pose.pose.position.y)]
            for pose in msg.poses
            if math.isfinite(pose.pose.position.x)
            and math.isfinite(pose.pose.position.y)
        ]
        self.path_time = time.monotonic()

    def control(self):
        fresh = self.path_time is not None and time.monotonic() - self.path_time <= float(self.param("path_timeout_sec"))
        valid = (
            fresh
            and self.nav_path_valid
            and self.confidence >= float(self.param("minimum_confidence"))
            and bool(self.path)
        )
        lookahead = dynamic_lookahead(
            self.speed_mps, float(self.param("minimum_lookahead_m")),
            float(self.param("lookahead_gain_s")), float(self.param("maximum_lookahead_m")),
        )
        target = select_lookahead_point(self.path, lookahead) if valid else None
        raw_steering = steering_angle_deg(target, float(self.param("wheelbase_m")), float(self.param("maximum_steering_deg")))
        raw_steering *= float(self.param("steering_sign"))
        now = time.monotonic()
        dt = max(0.0, now-self.last_control_time)
        self.last_control_time = now
        maximum_change = float(self.param("steering_rate_limit_deg_s"))*dt
        steering = max(self.limited_steering-maximum_change,
                       min(raw_steering,self.limited_steering+maximum_change)) if valid else 0.0
        self.limited_steering = steering
        configured_speed = float(self.param("commanded_speed_mps"))
        reverse_refused = configured_speed < 0.0 and not bool(self.param("allow_reverse"))
        target_speed = configured_speed if valid and not reverse_refused else 0.0
        self.steering_pub.publish(Float32(data=float(steering)))
        self.speed_pub.publish(Float32(data=float(target_speed)))
        self.lookahead_pub.publish(Float32(data=float(lookahead)))
        self.lookahead_point_pub.publish(String(data=json.dumps({
            "path_valid": valid,
            "lookahead_m": lookahead,
            "point": target,
            "coordinate_frame": "base_link",
            "axis_convention": "forward_m,lateral_right_m",
        })))
        self.status_pub.publish(String(data=json.dumps({
            "path_valid": valid, "confidence": self.confidence,
            "measured_speed_mps": self.speed_mps,
            "lookahead_m": lookahead, "target_point": target,
            "steering_deg": steering, "target_speed_mps": target_speed,
            "reverse_refused": reverse_refused,
        })))
        self.path_status_pub.publish(String(data=json.dumps({
            "observed_mode": path_mode_name(self.observed_mode),
            "active_mode": path_mode_name(self.active_mode),
            "path_valid": bool(valid),
            "path_confidence": float(self.confidence),
            "path_shift_m": float(self.path_shift_m),
            "curvature": float(self.curvature),
            "lookahead_m": float(lookahead),
            "raw_steering_deg": float(raw_steering),
            "limited_steering_deg": float(steering),
        }, separators=(",", ":"))))


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
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
