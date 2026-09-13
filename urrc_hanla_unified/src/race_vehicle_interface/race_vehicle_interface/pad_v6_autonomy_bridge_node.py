#!/usr/bin/env python3
"""Low-speed saved-map/BEV output bridge for MCU_PAD_SINGLE_v6 firmware."""

import glob
import json
import math
import os
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import SetBool

from .pad_v6_autonomy import PadV6Autonomy


class PadV6AutonomyBridge(Node):
    def __init__(self):
        super().__init__("pad_v6_autonomy_bridge_node")
        defaults = {
            "port": "", "baudrate": 115200, "command_timeout_sec": 0.3,
            "center_adc": 484, "adc_per_degree": 18.0,
            "minimum_adc": 158, "maximum_adc": 798,
            "maximum_target_speed_mps": 0.20,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.timeout = float(self.p("command_timeout_sec"))
        self.max_speed = float(self.p("maximum_target_speed_mps"))
        if not 0.05 <= self.timeout <= 1.0 or not 0 < self.max_speed <= 0.3:
            raise ValueError("invalid timeout or maximum target speed")
        self.logic = PadV6Autonomy(
            self.p("center_adc"), self.p("adc_per_degree"),
            self.p("minimum_adc"), self.p("maximum_adc"))
        self.values = {"speed": 0.0, "steer": 0.0, "pursuit": False,
                       "path": False, "plan": False, "stage": 0}
        self.updated = {}
        self.serial = serial.Serial(
            self.find_port(str(self.p("port"))), int(self.p("baudrate")),
            timeout=0, write_timeout=0.5)
        self.receive = bytearray()
        self.repeated_stop()
        self.status_pub = self.create_publisher(String, "/pad_v6_autonomy/status", 10)
        self.armed_pub = self.create_publisher(Bool, "/pad_v6_autonomy/armed", 10)
        self.sub(Float32, "/camera/target_speed_mps", "speed", float)
        self.sub(Float32, "/camera/target_steering_deg", "steer", float)
        self.sub(Bool, "/control/pure_pursuit_valid", "pursuit", bool)
        self.sub(Bool, "/camera/path_valid", "path", bool)
        self.sub(Bool, "/control/curvature_plan_valid", "plan", bool)
        self.sub(Int32, "/control/curvature_drive_stage", "stage", int)
        self.create_service(SetBool, "/pad_v6_autonomy/set_enabled", self.set_enabled)
        self.create_timer(0.05, self.tick)
        self.get_logger().warning(
            "MCU_PAD v6 autonomy bridge DISABLED; first forward command is PWM 40")

    def p(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def find_port(requested):
        if requested:
            return requested
        for path in dict.fromkeys(["/dev/t870_mcu", "/dev/ttyACM0", "/dev/ttyUSB0"]
                                  + sorted(glob.glob("/dev/ttyACM*"))
                                  + sorted(glob.glob("/dev/ttyUSB*"))):
            if os.path.exists(path):
                return path
        raise RuntimeError("MCU serial port not found")

    def sub(self, msg_type, topic, key, convert):
        def callback(msg):
            value = convert(msg.data)
            if not isinstance(value, float) or math.isfinite(value):
                self.values[key] = value
                self.updated[key] = time.monotonic()
        self.create_subscription(msg_type, topic, callback, 10)

    def write(self, packet):
        self.serial.write(packet)
        self.serial.flush()

    def repeated_stop(self):
        for _ in range(5):
            self.write(b"x")
            time.sleep(0.025)

    def set_enabled(self, request, response):
        for packet in self.logic.arm(request.data):
            self.write(packet)
        response.success = True
        response.message = ("enabled; waiting for all fresh valid inputs" if request.data
                            else "disabled and stopped")
        return response

    def tick(self):
        now = time.monotonic()
        fresh = all(now-self.updated.get(key, -1e9) <= self.timeout
                    for key in self.values)
        speed = float(self.values["speed"])
        valid = (fresh and self.values["path"] and self.values["pursuit"] and
                 self.values["plan"] and 0 < speed <= self.max_speed)
        was_moving = self.logic.moving
        packets = self.logic.update(valid, speed, self.values["steer"],
                                    self.values["stage"])
        for packet in packets:
            self.write(packet)
        if was_moving and not self.logic.moving:
            self.repeated_stop()
            self.logic.armed = False  # input loss requires an explicit re-arm
        waiting = self.serial.in_waiting
        if waiting:
            self.receive.extend(self.serial.read(waiting))
        while b"\n" in self.receive:
            raw, _, rest = self.receive.partition(b"\n")
            self.receive = bytearray(rest)
            text = raw.rstrip(b"\r").decode("utf-8", errors="replace").strip()
            if text:
                self.get_logger().info(f"MCU: {text}")
        stale = [key for key in self.values
                 if now-self.updated.get(key, -1e9) > self.timeout]
        blockers = []
        if not self.values["path"]:
            blockers.append("path_invalid")
        if not self.values["pursuit"]:
            blockers.append("pursuit_invalid")
        if not self.values["plan"]:
            blockers.append("curvature_plan_invalid")
        if speed <= 0.0:
            blockers.append("target_speed_zero")
        elif speed > self.max_speed:
            blockers.append("target_speed_over_limit")
        if int(self.values["stage"]) <= 0:
            blockers.append("drive_stage_stop")
        reason = ("RUNNING_PWM40" if self.logic.moving else
                  "DISABLED" if not self.logic.armed else
                  "WAITING_VALID_INPUT")
        self.armed_pub.publish(Bool(data=self.logic.armed))
        self.status_pub.publish(String(data=json.dumps({
            "state": reason,
            "stale_inputs": stale,
            "blockers": blockers,
            "target_speed_mps": speed,
            "target_steering_deg": float(self.values["steer"]),
            "drive_stage": int(self.values["stage"]),
        }, separators=(",", ":"))))

    def close(self):
        if hasattr(self, "serial") and self.serial.is_open:
            self.logic.arm(False)
            self.repeated_stop()
            self.serial.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PadV6AutonomyBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
