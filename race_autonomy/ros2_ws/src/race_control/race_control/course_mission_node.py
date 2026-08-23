#!/usr/bin/env python3

import math
import time
from dataclasses import replace

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8, Int32, String

from .course_mission import CourseMission, MissionInput, section_after_ramp_detection


class CourseMissionNode(Node):
    """Final command arbiter for camera, GPS, Depth and Nav2 missions."""

    def __init__(self):
        super().__init__("course_mission_node")
        defaults = {
            "ramp_pitch_deg": 5.0, "ramp_delay_sec": 0.5,
            "ramp_slow_pitch_deg": 10.0, "ramp_slow_hold_sec": 3.0,
            "ramp_level_pitch_deg": 3.0,
            "camera_to_front_bumper_m": 0.755,
            "desired_front_bumper_clearance_m": 0.5,
            "minimum_stop_sec": 0.5,
            "command_rate_hz": 20.0,
            "sensor_timeout_sec": 0.5,
        }
        for name, value in defaults.items(): self.declare_parameter(name, value)
        self.stop_depth_threshold_m = (
            float(self.p("camera_to_front_bumper_m")) +
            float(self.p("desired_front_bumper_clearance_m")))
        self.logic = CourseMission(
            self.p("ramp_pitch_deg"), self.p("ramp_delay_sec"),
            self.stop_depth_threshold_m, self.p("minimum_stop_sec"),
            self.p("ramp_level_pitch_deg"), self.p("ramp_slow_pitch_deg"),
            self.p("ramp_slow_hold_sec"))
        self.data = MissionInput()
        self.updated = {}
        self.ramp_section_override = False
        self.camera_drive_pub = self.create_publisher(
            Float32, "/camera_drive", 10)
        self.camera_wheel_pub = self.create_publisher(
            Int32, "/camera_wheel", 10)
        self.turn_pub = self.create_publisher(Int8, "/mission/turn_direction", 10)
        self.mode_pub = self.create_publisher(Int8, "/mission/control_mode", 10)
        self.status_pub = self.create_publisher(String, "/mission/status", 10)
        self.stop_threshold_pub = self.create_publisher(
            Float32, "/mission/stop_depth_threshold_m", 10)

        self.create_subscription(Int8, "/mission/section", self.on_section, 10)
        self.sub(Int8, "/mission/gps_direction", "gps_direction", int)
        self.sub(Float32, "/imu_pitch", "pitch_deg", float)
        self.sub(Bool, "/imu_valid", "imu_valid", bool)
        self.sub(Bool, "/perception/stop_detected", "stop_detected", bool)
        self.sub(Float32, "/perception/stop_line_distance_m", "stop_distance_m", float)
        self.sub(Bool, "/perception/stop_line_distance_valid", "stop_distance_valid", bool)
        self.sub(String, "/perception/traffic_light_state", "traffic_green", lambda x: str(x).strip().upper()=="GREEN")
        self.sub(Bool, "/perception/traffic20_detected", "traffic20_detected", bool)
        self.sub(Bool, "/camera/path_valid", "camera_path_valid", bool)
        self.sub(Float32, "/camera/target_steering_deg", "camera_steering_deg", float)
        self.sub(Int32, "/control/curvature_drive_stage", "planned_drive_stage", int)
        self.sub(Bool, "/control/curvature_plan_valid", "speed_plan_valid", bool)
        self.sub(Float32, "/camera/yellow_line_ahead_m", "yellow_ahead_m", float)
        self.sub(Bool, "/camera/yellow_line_ahead_valid", "yellow_ahead_valid", bool)
        self.create_timer(1.0/max(1.0,float(self.p("command_rate_hz"))), self.control)

    def p(self,name): return self.get_parameter(name).value
    def sub(self, msg_type, topic, field, convert):
        def callback(msg):
            value = convert(msg.data)
            if not isinstance(value, float) or math.isfinite(value):
                setattr(self.data, field, value)
                self.updated[field] = time.monotonic()
        self.create_subscription(msg_type, topic, callback, 10)

    def on_section(self, msg):
        section = int(msg.data)
        if self.ramp_section_override and section == 2:
            return
        if section != 2:
            self.ramp_section_override = False
        self.data.section = section
        self.updated["section"] = time.monotonic()

    def control(self):
        now = time.monotonic(); self.data.now = now
        timeout = float(self.p("sensor_timeout_sec"))
        fresh = lambda *fields: all(now-self.updated.get(field, -1e9) <= timeout for field in fields)
        # Section 1 ends at the physical ramp rather than waiting for an
        # external section publisher.  Only fresh, valid IMU data may latch
        # this one-way 1 -> 2 transition.
        detected_section = section_after_ramp_detection(
            self.data.section,
            self.data.imu_valid and fresh("imu_valid", "pitch_deg"),
            self.data.pitch_deg,
            self.p("ramp_pitch_deg"),
        )
        if detected_section != self.data.section:
            self.data.section = detected_section
            self.updated["section"] = now
            self.get_logger().info(
                f"Ramp pitch {self.data.pitch_deg:.2f} deg detected; "
                "mission section latched 1 -> 2")
        safe = replace(
            self.data,
            camera_path_valid=self.data.camera_path_valid and fresh("camera_path_valid", "camera_steering_deg"),
            speed_plan_valid=(self.data.speed_plan_valid and
                              fresh("speed_plan_valid", "planned_drive_stage")),
            imu_valid=self.data.imu_valid and fresh("imu_valid", "pitch_deg"),
            stop_detected=self.data.stop_detected and fresh("stop_detected"),
            stop_distance_valid=self.data.stop_distance_valid and fresh("stop_distance_valid", "stop_distance_m"),
            traffic_green=self.data.traffic_green and fresh("traffic_green"),
            traffic20_detected=self.data.traffic20_detected and fresh("traffic20_detected"),
            yellow_ahead_valid=(self.data.yellow_ahead_valid and
                                fresh("yellow_ahead_valid", "yellow_ahead_m")),
        )
        output = self.logic.update(safe)
        if self.logic.section_request is not None:
            self.data.section = int(self.logic.section_request)
            self.ramp_section_override = self.data.section == 3
            self.updated["section"] = now
            self.logic.section_request = None
            self.get_logger().info(
                "Ramp level held for 1.0 s; active section 2 -> 3")
        self.camera_drive_pub.publish(Float32(data=float(output.stage)))
        self.camera_wheel_pub.publish(
            Int32(data=int(round(output.steering_deg))))
        self.turn_pub.publish(Int8(data=output.turn_direction))
        self.mode_pub.publish(Int8(data=output.control_mode))
        self.status_pub.publish(String(data=output.status))
        self.stop_threshold_pub.publish(
            Float32(data=float(self.stop_depth_threshold_m)))

    def destroy_node(self):
        if rclpy.ok():
            self.camera_drive_pub.publish(Float32(data=0.0))
            self.camera_wheel_pub.publish(Int32(data=0))
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node=CourseMissionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__": main()
