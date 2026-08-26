#!/usr/bin/env python3

import math
import time
from dataclasses import replace

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8, Int32, String

from .course_mission import CourseMission, MissionInput


class CourseMissionNode(Node):
    """Final command arbiter for camera, GPS, Depth and Nav2 missions."""

    def __init__(self):
        super().__init__("course_mission_node")
        defaults = {
            "ramp_pitch_deg": 5.0, "ramp_delay_sec": 0.5,
            "ramp_slow_pitch_deg": 5.0, "ramp_slow_hold_sec": 3.0,
            "ramp_level_pitch_deg": 3.0,
            "ramp_pitch_confirm_sec": 0.3,
            "stop_line_rearm_sec": 0.5,
            "ramp_post_stop_drive_sec": 3.0,
            "ramp_second_line_stop_sec": 1.0,
            "traffic20_rearm_sec": 0.5,
            "stop_line_trigger_distance_m": 2.0,
            "minimum_stop_sec": 2.0,
            "green_confirm_sec": 2.0,
            "actual_stop_speed_mps": 0.05,
            "command_rate_hz": 20.0,
            "sensor_timeout_sec": 0.5,
        }
        for name, value in defaults.items(): self.declare_parameter(name, value)
        self.stop_depth_threshold_m = float(
            self.p("stop_line_trigger_distance_m"))
        self.logic = CourseMission(
            self.p("ramp_pitch_deg"), self.p("ramp_delay_sec"),
            self.stop_depth_threshold_m, self.p("minimum_stop_sec"),
            self.p("ramp_level_pitch_deg"), self.p("ramp_slow_pitch_deg"),
            self.p("ramp_slow_hold_sec"), self.p("green_confirm_sec"),
            self.p("actual_stop_speed_mps"),
            self.p("ramp_pitch_confirm_sec"),
            self.p("stop_line_rearm_sec"),
            self.p("ramp_post_stop_drive_sec"),
            self.p("ramp_second_line_stop_sec"),
            self.p("traffic20_rearm_sec"))
        self.data = MissionInput()
        self.updated = {}
        self.active_section_pub = self.create_publisher(
            Int8, "/mission/active_section", 10)
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
        self.create_subscription(
            String, "/perception/traffic_light_state",
            self.on_traffic_light, 10)
        self.sub(Bool, "/perception/traffic20_detected", "traffic20_detected", bool)
        self.create_subscription(
            String,"/perception/final_signal_state",self.on_final_signal,10)
        self.sub(Bool, "/camera/path_valid", "camera_path_valid", bool)
        self.sub(Float32, "/camera/target_steering_deg", "camera_steering_deg", float)
        self.sub(Int32, "/control/curvature_drive_stage", "planned_drive_stage", int)
        self.sub(Bool, "/control/curvature_plan_valid", "speed_plan_valid", bool)
        self.sub(Float32, "/camera/yellow_line_ahead_m", "yellow_ahead_m", float)
        self.sub(Bool, "/camera/yellow_line_ahead_valid", "yellow_ahead_valid", bool)
        self.sub(Float32, "/vehicle/speed_mps", "speed_mps", float)
        self.sub(Bool, "/vehicle/speed_valid", "speed_valid", bool)
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
        if not 1 <= section <= 13:
            self.get_logger().warning(f"Ignoring invalid GPS section: {section}")
            return
        if section != self.data.section:
            # Never carry a signal decision from one intersection/mission
            # into the next one. Within a section, however, UNKNOWN means
            # "no newer decision" and the latest confirmed color is retained.
            self.data.traffic_green=False
            self.data.traffic_red=False
            self.data.traffic_yellow=False
            self.data.final_signal_green=False
            self.data.final_signal_red=False
        self.data.section = section
        self.updated["section"] = time.monotonic()

    def on_traffic_light(self, msg):
        state = str(msg.data).strip().upper()
        now = time.monotonic()
        if state in {"GREEN","RED","YELLOW"}:
            self.data.traffic_green = state == "GREEN"
            self.data.traffic_red = state == "RED"
            self.data.traffic_yellow = state == "YELLOW"
        for field in ("traffic_green", "traffic_red", "traffic_yellow"):
            self.updated[field] = now

    def on_final_signal(self,msg):
        state=str(msg.data).strip().upper();now=time.monotonic()
        if state in {"GREEN","RED"}:
            self.data.final_signal_green=state=="GREEN"
            self.data.final_signal_red=state=="RED"
        self.updated["final_signal_green"]=now
        self.updated["final_signal_red"]=now

    def control(self):
        now = time.monotonic(); self.data.now = now
        timeout = float(self.p("sensor_timeout_sec"))
        fresh = lambda *fields: all(now-self.updated.get(field, -1e9) <= timeout for field in fields)
        safe = replace(
            self.data,
            camera_path_valid=self.data.camera_path_valid and fresh("camera_path_valid", "camera_steering_deg"),
            speed_plan_valid=(self.data.speed_plan_valid and
                              fresh("speed_plan_valid", "planned_drive_stage")),
            imu_valid=self.data.imu_valid and fresh("imu_valid", "pitch_deg"),
            stop_detected=self.data.stop_detected and fresh("stop_detected"),
            stop_distance_valid=self.data.stop_distance_valid and fresh("stop_distance_valid", "stop_distance_m"),
            traffic_green=self.data.traffic_green and fresh("traffic_green"),
            traffic_red=self.data.traffic_red and fresh("traffic_red"),
            traffic_yellow=(self.data.traffic_yellow and
                            fresh("traffic_yellow")),
            traffic20_detected=self.data.traffic20_detected and fresh("traffic20_detected"),
            final_signal_green=(self.data.final_signal_green and
                                fresh("final_signal_green")),
            final_signal_red=(self.data.final_signal_red and
                              fresh("final_signal_red")),
            yellow_ahead_valid=(self.data.yellow_ahead_valid and
                                fresh("yellow_ahead_valid", "yellow_ahead_m")),
            speed_valid=(self.data.speed_valid and
                         fresh("speed_valid", "speed_mps")),
        )
        output = self.logic.update(safe)
        self.active_section_pub.publish(Int8(data=int(self.data.section)))
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
