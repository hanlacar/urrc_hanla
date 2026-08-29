#!/usr/bin/env python3

import math
import time
from dataclasses import replace

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8, Int32, String

from .course_mission import (
    CourseMission,
    MissionInput,
    camera_emergency_stop,
)
from .path_stability import path_jump_metrics, path_spatial_quality


class CourseMissionNode(Node):
    """Final command arbiter for camera, GPS, Depth and Nav2 missions."""

    SECTION_TO_VEHICLE_MODE = {
        1: "START",
        2: "SLOPE",
        3: "CRANK",
        4: "INTERSECTION_1",
        5: "S_COURSE",
        6: "INTERSECTION_2",
        7: "T_PARK",
        8: "INTERSECTION_3",
        9: "ACCELERATION",
        10: "PARALLEL_PARK",
        11: "FINISH",
    }

    def __init__(self):
        super().__init__("course_mission_node")
        defaults = {
            "initial_section": 1,
            "ramp_pitch_deg": 15.0, "ramp_delay_sec": 0.5,
            "ramp_slow_pitch_deg": 5.0, "ramp_slow_hold_sec": 3.0,
            "ramp_level_pitch_deg": 3.0,
            "ramp_pitch_confirm_sec": 0.5,
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
            "path_jump_threshold_m": 0.25,
            "path_jump_forward_min_m": 0.5,
            "path_jump_forward_max_m": 3.0,
            "path_required_forward_span_m": 1.5,
            "path_required_near_point_m": 1.0,
            "path_maximum_lateral_step_m": 0.15,
            "vehicle_speed_topic": "/vehicle/speed_mps",
            "vehicle_speed_valid_topic": "/vehicle/speed_valid",
            "input_guard_topic": "",
            "input_guard_timeout_sec": 0.3,
            "output_maximum_stage": 3,
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
        initial_section = int(self.p("initial_section"))
        if not 1 <= initial_section <= 11:
            raise ValueError("initial_section must be between 1 and 11")
        self.data = MissionInput(section=initial_section)
        self.previous_path = None
        self.path_accuracy = 0.0
        self.path_confidence = 0.0
        self.path_temporal_stability = 0.0
        self.path_spatial_quality = 0.0
        self.path_jump_m = 0.0
        self.path_jump_detected = False
        self.control_was_active = False
        self.updated = {}
        self.active_section_pub = self.create_publisher(
            Int8, "/mission/active_section", 10)
        self.vehicle_mode_pub = self.create_publisher(
            String, "/vehicle_mode", 10)
        self.camera_drive_pub = self.create_publisher(
            Float32, "/camera_drive", 10)
        self.camera_wheel_pub = self.create_publisher(
            Int32, "/camera_wheel", 10)
        self.camera_stop_pub = self.create_publisher(
            Bool, "/camera_stop", 10)
        self.turn_pub = self.create_publisher(Int8, "/mission/turn_direction", 10)
        self.mode_pub = self.create_publisher(Int8, "/mission/control_mode", 10)
        self.status_pub = self.create_publisher(String, "/mission/status", 10)
        self.stop_threshold_pub = self.create_publisher(
            Float32, "/mission/stop_depth_threshold_m", 10)
        self.path_accuracy_pub = self.create_publisher(
            Float32, "/camera/path_accuracy", 10)
        self.path_temporal_pub = self.create_publisher(
            Float32, "/camera/path_temporal_stability", 10)
        self.path_spatial_pub = self.create_publisher(
            Float32, "/camera/path_spatial_quality", 10)
        self.path_jump_pub = self.create_publisher(
            Float32, "/camera/path_jump_m", 10)
        self.path_jump_detected_pub = self.create_publisher(
            Bool, "/camera/path_jump_detected", 10)

        self.create_subscription(Int8, "/mission/section", self.on_section, 10)
        self.create_subscription(Path, "/camera/path", self.on_path, 10)
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
        self.create_subscription(
            Float32, "/camera/path_confidence", self.on_path_confidence, 10)
        self.sub(Float32, "/camera/target_steering_deg", "camera_steering_deg", float)
        self.sub(Int32, "/control/curvature_drive_stage", "planned_drive_stage", int)
        self.sub(Bool, "/control/curvature_plan_valid", "speed_plan_valid", bool)
        self.sub(Float32, "/camera/yellow_line_ahead_m", "yellow_ahead_m", float)
        self.sub(Bool, "/camera/yellow_line_ahead_valid", "yellow_ahead_valid", bool)
        self.sub(Float32, str(self.p("vehicle_speed_topic")),
                 "speed_mps", float)
        self.sub(Bool, str(self.p("vehicle_speed_valid_topic")),
                 "speed_valid", bool)
        self.input_guard_topic = str(self.p("input_guard_topic")).strip()
        if self.input_guard_topic:
            self.sub(Bool, self.input_guard_topic, "input_guard_alive", bool)
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
        if not 1 <= section <= 11:
            self.get_logger().warning(f"Ignoring invalid GPS section: {section}")
            return
        if section != self.data.section:
            self.previous_path = None
            self.path_accuracy = 0.0
            self.path_confidence = 0.0
            self.path_temporal_stability = 0.0
            self.path_spatial_quality = 0.0
            self.path_jump_m = 0.0
            self.path_jump_detected = False
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

    def on_path(self, msg):
        current = [(pose.pose.position.x, pose.pose.position.y)
                   for pose in msg.poses]
        if len(current) < 2:
            self.path_accuracy = 0.0
            self.path_temporal_stability = 0.0
            self.path_spatial_quality = 0.0
            self.path_jump_m = 0.0
            self.path_jump_detected = False
            self.previous_path = None
            return
        spatial, _spatial_valid, _diagnostics = path_spatial_quality(
            current, self.p("path_required_forward_span_m"),
            self.p("path_required_near_point_m"),
            self.p("path_maximum_lateral_step_m"))
        self.path_spatial_quality = spatial
        if self.previous_path is None:
            # No comparison exists yet. Never advertise unverified 100%.
            self.path_temporal_stability = 0.0
            self.path_accuracy = 0.0
            self.path_jump_m = 0.0
            self.path_jump_detected = False
        else:
            accuracy, median, _maximum, jumped, valid = path_jump_metrics(
                self.previous_path, current,
                self.p("path_jump_threshold_m"),
                self.p("path_jump_forward_min_m"),
                self.p("path_jump_forward_max_m"))
            self.path_temporal_stability = accuracy if valid else 0.0
            # MCU-facing "accuracy" is the evidence-based confidence produced
            # by the path planner. Temporal/spatial stability remain available
            # on their dedicated diagnostic topics and must not redefine it.
            self.path_accuracy = self.path_confidence
            self.path_jump_m = median if valid else 0.0
            self.path_jump_detected = jumped if valid else False
        self.previous_path = current

    def on_path_confidence(self, msg):
        value = float(msg.data)
        if math.isfinite(value):
            self.path_confidence = max(0.0, min(1.0, value))
            self.path_accuracy = self.path_confidence
            self.updated["path_confidence"] = time.monotonic()

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
        if self.input_guard_topic:
            guard_alive = (self.data.input_guard_alive and
                           fresh("input_guard_alive") and
                           now-self.updated.get("input_guard_alive", -1e9) <=
                           float(self.p("input_guard_timeout_sec")))
            if not guard_alive:
                output = self.logic.stopped("SAFE_STOP:INPUT_STREAM_LOST")
        maximum_stage = max(0, min(3, int(self.p("output_maximum_stage"))))
        if output.stage > maximum_stage:
            output = replace(output, stage=maximum_stage,
                             status=f"{output.status}:OUTPUT_STAGE_LIMIT_{maximum_stage}")
        self.active_section_pub.publish(Int8(data=int(self.data.section)))
        self.vehicle_mode_pub.publish(String(data=self.SECTION_TO_VEHICLE_MODE.get(
            int(self.data.section), "IDLE")))
        emergency_stop = camera_emergency_stop(
            output.status, self.control_was_active)
        if output.stage > 0:
            self.control_was_active = True

        self.camera_drive_pub.publish(Float32(data=float(output.stage)))
        self.camera_wheel_pub.publish(
            Int32(data=int(round(output.steering_deg))))
        # /camera_stop is the MCU's independent hard-stop channel. Ordinary
        # traffic-light, stop-line and curvature stops use drive stage zero;
        # only an explicit guarded input-stream loss after motion asserts it.
        self.camera_stop_pub.publish(Bool(data=emergency_stop))
        self.turn_pub.publish(Int8(data=output.turn_direction))
        self.mode_pub.publish(Int8(data=output.control_mode))
        self.status_pub.publish(String(data=output.status))
        self.stop_threshold_pub.publish(
            Float32(data=float(self.stop_depth_threshold_m)))
        self.path_accuracy_pub.publish(Float32(data=float(self.path_accuracy)))
        self.path_temporal_pub.publish(
            Float32(data=float(self.path_temporal_stability)))
        self.path_spatial_pub.publish(
            Float32(data=float(self.path_spatial_quality)))
        self.path_jump_pub.publish(Float32(data=float(self.path_jump_m)))
        self.path_jump_detected_pub.publish(
            Bool(data=bool(self.path_jump_detected)))

    def destroy_node(self):
        if rclpy.ok():
            self.camera_drive_pub.publish(Float32(data=0.0))
            self.camera_wheel_pub.publish(Int32(data=0))
            self.camera_stop_pub.publish(Bool(data=True))
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node=CourseMissionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__": main()
