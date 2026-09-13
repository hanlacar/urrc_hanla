#!/usr/bin/env python3
"""Keyboard controller that directly owns an MCU_PAD_SINGLE_v6 serial port."""

import glob
import select
import sys
import termios
import time
import tty

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

from .pad_v6_protocol import PadV6CommandState


HELP = ("E enable/disable | W forward step | S reverse step | X/Space stop | "
        "A/D steer | C center | I status | Q stop and quit")


class PadV6KeyboardNode(Node):
    def __init__(self):
        super().__init__("pad_v6_keyboard_node")
        defaults = {
            "port": "", "baudrate": 115200, "center_adc": 484,
            "minimum_adc": 158, "maximum_adc": 798,
            "steering_step_adc": 64, "startup_stop_sec": 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if not sys.stdin.isatty():
            raise RuntimeError("pad v6 keyboard control requires an interactive terminal")
        self.state = PadV6CommandState(
            self.p("center_adc"), self.p("minimum_adc"), self.p("maximum_adc"),
            self.p("steering_step_adc"))
        self.port = self.find_port(str(self.p("port")))
        self.serial = serial.Serial(
            self.port, int(self.p("baudrate")), timeout=0, write_timeout=0.5)
        self.original_terminal = termios.tcgetattr(sys.stdin)
        self.receive_buffer = bytearray()
        self.status_pub = self.create_publisher(String, "/manual_pad_v6/status", 10)
        self.armed_pub = self.create_publisher(Bool, "/manual_pad_v6/armed", 10)
        self.pwm_pub = self.create_publisher(Int32, "/manual_pad_v6/estimated_pwm", 10)
        self.safe_start()
        tty.setcbreak(sys.stdin.fileno())
        self.create_timer(0.02, self.tick)
        self.get_logger().info(f"MCU_PAD v6 keyboard: {self.port}; {HELP}")

    def p(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def find_port(requested):
        if requested:
            return requested
        candidates = ["/dev/t870_mcu", "/dev/ttyACM0", "/dev/ttyUSB0"]
        candidates += sorted(glob.glob("/dev/ttyACM*"))
        candidates += sorted(glob.glob("/dev/ttyUSB*"))
        for path in dict.fromkeys(candidates):
            try:
                with open(path, "rb", buffering=0):
                    pass
            except (FileNotFoundError, PermissionError, OSError):
                continue
            return path
        raise RuntimeError("MCU serial port not found or not readable")

    def write(self, payload):
        self.serial.write(payload)
        self.serial.flush()

    def repeated_stop(self, count=5):
        for _ in range(count):
            self.write(b"x")
            time.sleep(0.025)
        self.state.estimated_pwm = 0

    def safe_start(self):
        duration = float(self.p("startup_stop_sec"))
        if not 0.5 <= duration <= 10.0:
            raise ValueError("startup_stop_sec must be between 0.5 and 10 seconds")
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.write(b"x")
            self.read_serial()
            time.sleep(0.08)
        self.repeated_stop()
        self.write(b"S\n")

    def read_serial(self):
        waiting = self.serial.in_waiting
        if waiting:
            self.receive_buffer.extend(self.serial.read(waiting))
        while b"\n" in self.receive_buffer:
            raw, _, remainder = self.receive_buffer.partition(b"\n")
            self.receive_buffer = bytearray(remainder)
            text = raw.rstrip(b"\r").decode("utf-8", errors="replace").strip()
            if text:
                self.get_logger().info(f"MCU: {text}")
                self.status_pub.publish(String(data=text))

    def tick(self):
        self.read_serial()
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1).lower()
            if key == "q":
                self.state.armed = False
                self.repeated_stop()
                rclpy.shutdown()
                return
            was_armed = self.state.armed
            payload = self.state.key(key)
            if payload is None:
                if key in ("w", "s") and not self.state.armed:
                    self.get_logger().warning("drive ignored: press E to enable")
                continue
            if key in ("e", "x", " "):
                self.repeated_stop()
            else:
                self.write(payload)
            if key == "e":
                label = "ENABLED; press W/S again to move" if self.state.armed else "DISABLED"
                self.get_logger().warning(label)
            elif key in ("w", "s"):
                self.get_logger().info(f"estimated PWM={self.state.estimated_pwm:+d}")
            elif key in ("a", "d", "c"):
                self.get_logger().info(f"steering target ADC={self.state.target_adc}")
            if was_armed and key in ("x", " "):
                self.get_logger().info("drive stopped; control remains enabled")
        self.armed_pub.publish(Bool(data=self.state.armed))
        self.pwm_pub.publish(Int32(data=self.state.estimated_pwm))

    def close(self):
        try:
            self.state.armed = False
            if hasattr(self, "serial") and self.serial.is_open:
                self.repeated_stop()
                self.serial.close()
        finally:
            if hasattr(self, "original_terminal"):
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN,
                                  self.original_terminal)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PadV6KeyboardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
