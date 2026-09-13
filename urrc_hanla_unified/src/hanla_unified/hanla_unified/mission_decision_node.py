"""Single authority that publishes the final speed and steering for the MCU."""

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8, Int32, String

from .decision import (
    Candidate, DecisionInput, decide, limit_rate, requires_emergency_brake,
)


class MissionDecisionNode(Node):
    def __init__(self):
        super().__init__("mission_decision")
        defaults = {
            "initial_section": 1, "publish_rate_hz": 20.0,
            "source_timeout_sec": 0.5, "camera_confidence_min": 0.8,
            "maximum_speed_mps": 0.70, "maximum_steering_deg": 22.0,
            "stage_1_speed_mps": 0.229, "stage_2_speed_mps": 0.455,
            "stage_3_speed_mps": 0.70, "lidar_steering_sign": -1.0,
            "maximum_steering_rate_deg_s": 45.0,
            "slope_hold_sec": 3.0, "exit_hold_sec": 5.0,
        }
        for key, value in defaults.items(): self.declare_parameter(key, value)
        self.section = int(self.p("initial_section"))
        self.values = {}
        self.times = {}
        self.slope_hold_until = 0.0
        self.slope_hold_consumed = False
        self.exit_hold_until = 0.0
        self.exit_hold_consumed = False
        self.exit_request_sent = False
        self.last_steering_command = 0.0
        self.last_command_time = time.monotonic()

        # This node is the sole command arbiter.  The MCU bridge receives only
        # the final drive, wheel and dynamic-stop commands.
        self.speed_pub = self.create_publisher(Float32, "/cmd_drive", 10)
        self.steer_pub = self.create_publisher(Float32, "/cmd_wheel", 10)
        self.stop_pub = self.create_publisher(Bool, "/cmd_stop", 10)
        # Applied integrated mode for LiDAR safety/planning consumers.  This
        # mirrors the selected mission section; it is never fabricated by the
        # production SIMPLE compatibility bridge.
        self.mode_pub = self.create_publisher(String, "/mcu/current_mode", 10)
        self.hold_pub = self.create_publisher(Bool, "/mcu/slope_hold", 10)
        self.source_pub = self.create_publisher(String, "/mission/active_source", 10)
        self.status_pub = self.create_publisher(String, "/mission/decision_status", 10)
        self.exit_route_pub = self.create_publisher(
            String, "/mission/exit_route_request", 10)
        self.parking_slot_pub = self.create_publisher(
            String, "/mission/parking_slot", 10)

        self.sub(Int8, "/mission/section", "section", lambda m: self.set_section(m.data))
        self.sub(Float32, "/camera/target_speed_mps", "camera_speed", lambda m: m.data)
        self.sub(Float32, "/camera/target_steering_deg", "camera_steer", lambda m: m.data)
        self.sub(Float32, "/camera/path_confidence", "camera_conf", lambda m: m.data)
        self.sub(Bool, "/camera/path_valid", "camera_valid", lambda m: m.data)
        self.sub(Float32, "/gps_drive", "dr_drive", lambda m: m.data)
        self.sub(Int32, "/gps_wheel", "dr_steer", lambda m: m.data)
        self.sub(Float32, "/lidar_drive", "lidar_drive", lambda m: m.data)
        self.sub(Int32, "/lidar_wheel", "lidar_steer", lambda m: m.data)
        self.sub(Bool, "/lidar_stop", "lidar_stop", lambda m: m.data)
        self.sub(Bool, "/lidar/stop_required", "lidar_safety_stop", lambda m: m.data)
        self.sub(Float32, "/parking/drive_cmd", "parking_drive", lambda m: m.data)
        self.sub(Int32, "/parking/wheel_cmd", "parking_steer", lambda m: m.data)
        self.sub(Bool, "/parking/stop_cmd", "parking_stop", lambda m: m.data)
        self.sub(String, "/perception/traffic_light_state", "traffic", lambda m: m.data)
        self.sub(Bool, "/dr/intersection_timeout_release",
                 "intersection_timeout_release", lambda m: m.data)
        self.sub(Float32, "/imu/pitch_deg", "pitch", lambda m: m.data)
        self.sub(Bool, "/mission/ramp_waypoint_reached", "ramp_waypoint", lambda m: m.data)
        self.sub(Bool, "/avoidance/active", "avoidance_active", lambda m: m.data)
        self.sub(Bool, "/mission/exit_waypoint_reached", "exit_waypoint", lambda m: m.data)
        self.sub(String, "/t_parking/selected_slot", "t_slot", lambda m: m.data)
        self.sub(String, "/parallel_parking/selected_slot", "parallel_slot", lambda m: m.data)
        self.create_timer(1.0 / float(self.p("publish_rate_hz")), self.tick)

    def p(self, name): return self.get_parameter(name).value

    def sub(self, msg_type, topic, key, convert):
        self.create_subscription(msg_type, topic,
            lambda msg: self.update(key, convert(msg)), 10)

    def update(self, key, value):
        self.values[key] = value
        self.times[key] = time.monotonic()

    def set_section(self, value):
        value = int(value)
        if value != self.section:
            self.slope_hold_consumed = False
            self.exit_hold_consumed = False
            self.exit_request_sent = False
        self.section = value
        return value

    def fresh(self, *keys):
        now = time.monotonic(); timeout = float(self.p("source_timeout_sec"))
        return all(k in self.times and now - self.times[k] <= timeout for k in keys)

    def stage_speed(self, value):
        value = float(value); sign = -1.0 if value < 0 else 1.0
        stage = int(round(abs(value)))
        if stage == 0: return 0.0
        if stage not in (1, 2, 3) or abs(abs(value) - stage) > 1e-3: return math.nan
        return sign * float(self.p(f"stage_{stage}_speed_mps"))

    def candidate(self, source):
        if source == "camera":
            valid = self.fresh("camera_speed", "camera_steer", "camera_conf", "camera_valid")
            return Candidate(float(self.values.get("camera_speed", 0.0)),
                float(self.values.get("camera_steer", 0.0)),
                valid and bool(self.values.get("camera_valid", False)),
                float(self.values.get("camera_conf", 0.0)))
        drive_key, steer_key = f"{source}_drive", f"{source}_steer"
        if source == "lidar" and self.section in (7, 10):
            # Prefer a dedicated parking executor when it is active. The
            # segmented DR follower also owns modes 7/10 and publishes on
            # /lidar_*; use that input when /parking/* is absent or stale.
            if self.fresh("parking_drive", "parking_steer"):
                drive_key, steer_key = "parking_drive", "parking_steer"
        speed = self.stage_speed(self.values.get(drive_key, 0.0))
        steer = float(self.values.get(steer_key, 0.0))
        if source == "lidar": steer *= float(self.p("lidar_steering_sign"))
        valid = self.fresh(drive_key, steer_key)
        if source == "lidar" and self.section == 5:
            valid = (valid and self.fresh("avoidance_active")
                     and bool(self.values.get("avoidance_active", False)))
        return Candidate(speed, steer, valid, 1.0)

    def tick(self):
        now = time.monotonic()
        pitch = float(self.values.get("pitch", 0.0)) if self.fresh("pitch") else 0.0
        ramp = bool(self.values.get("ramp_waypoint", False)) and self.fresh("ramp_waypoint")
        if (self.section == 2 and abs(pitch) >= 5.0 and ramp
                and not self.slope_hold_consumed):
            self.slope_hold_until = now + float(self.p("slope_hold_sec"))
            self.slope_hold_consumed = True
        exit_wp = bool(self.values.get("exit_waypoint", False)) and self.fresh("exit_waypoint")
        if self.section == 11 and exit_wp and not self.exit_hold_consumed:
            self.exit_hold_until = now + float(self.p("exit_hold_sec"))
            self.exit_hold_consumed = True

        traffic = str(self.values.get("traffic", "UNKNOWN")).strip().upper()
        signal_go = (
            traffic == "LEFT" if self.section == 8
            else traffic in ("GO", "GREEN")
        )
        timeout_release = (
            self.fresh("intersection_timeout_release")
            and bool(self.values.get("intersection_timeout_release", False))
        )
        data = DecisionInput(self.section, self.candidate("camera"), self.candidate("dr"),
            self.candidate("lidar"),
            traffic_go=(
                (self.fresh("traffic") and signal_go) or timeout_release
            ),
            lidar_stop=((self.fresh("lidar_stop") and bool(self.values.get("lidar_stop", False)))
                        or (self.fresh("lidar_safety_stop")
                            and bool(self.values.get("lidar_safety_stop", False)))
                        or (self.section in (7, 10) and self.fresh("parking_stop")
                            and bool(self.values.get("parking_stop", False)))),
            pitch_deg=pitch, ramp_stop_waypoint=False)
        result = decide(data, float(self.p("camera_confidence_min")))
        slope_hold = now < self.slope_hold_until
        exit_hold = now < self.exit_hold_until
        if slope_hold or exit_hold:
            result = type(result)(0.0, 0.0, "slope_hold" if slope_hold else "exit_hold",
                                  True, slope_hold, "timed_hold")
        if (self.section == 11 and self.exit_hold_consumed and not exit_hold
                and not self.exit_request_sent):
            choice = "A-A" if traffic in ("GO", "GREEN") else (
                "A-B" if traffic == "RED" else "")
            if choice:
                self.exit_route_pub.publish(String(data=choice))
                self.exit_request_sent = True
        slot_key = "t_slot" if self.section == 7 else (
            "parallel_slot" if self.section == 10 else "")
        if slot_key and self.fresh(slot_key):
            slot = str(self.values.get(slot_key, "")).strip()
            if slot:
                self.parking_slot_pub.publish(String(data=slot))
        speed = max(-float(self.p("maximum_speed_mps")),
                    min(float(self.p("maximum_speed_mps")), result.speed_mps))
        steer = max(-float(self.p("maximum_steering_deg")),
                    min(float(self.p("maximum_steering_deg")), result.steering_deg))
        elapsed = max(0.0, min(0.25, now - self.last_command_time))
        steer = limit_rate(
            self.last_steering_command,
            steer,
            float(self.p("maximum_steering_rate_deg_s")),
            elapsed,
        )
        self.last_steering_command = steer
        self.last_command_time = now
        self.speed_pub.publish(Float32(data=float(speed)))
        self.steer_pub.publish(Float32(data=float(steer)))
        # A missing/stale source, traffic light, and timed hold are commanded
        # stops.  They must keep target speed at zero without firing the MCU's
        # dynamic-brake command (B).  Reserve /mcu/emergency_stop for an
        # explicit obstacle/safety-stop input.
        emergency_stop = requires_emergency_brake(result)
        self.stop_pub.publish(Bool(data=emergency_stop))
        self.mode_pub.publish(String(data=str(self.section)))
        self.hold_pub.publish(Bool(data=slope_hold))
        self.source_pub.publish(String(data=result.source))
        self.status_pub.publish(String(data=json.dumps({"section": self.section,
            "source": result.source, "speed_mps": speed, "steering_deg": steer,
            "stop": result.stop, "emergency_stop": emergency_stop,
            "slope_hold": slope_hold, "reason": result.reason})))


def main(args=None):
    rclpy.init(args=args); node = MissionDecisionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
