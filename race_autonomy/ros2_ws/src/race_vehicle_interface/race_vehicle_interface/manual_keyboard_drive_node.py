#!/usr/bin/env python3
"""Interactive low-speed keyboard commands for physical vehicle testing."""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


HELP = ("1/2/3 speed | W forward | S reverse | X/Space stop | "
        "A left | D right | C center | Q quit")


class ManualKeyboardDrive(Node):
    def __init__(self):
        super().__init__("manual_keyboard_drive_node")
        self.declare_parameter("maximum_abs_stage", 3)
        self.declare_parameter("maximum_steering_deg", 27.0)
        self.declare_parameter("steering_step_deg", 3.0)
        self.declare_parameter("command_rate_hz", 20.0)
        if not sys.stdin.isatty():
            raise RuntimeError("manual keyboard drive requires an interactive terminal")
        self.stage = 0
        self.selected_stage = 1
        self.steering = 0.0
        self.original_terminal = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.drive_pub = self.create_publisher(Float32, "/mcu_drive", 10)
        self.steer_pub = self.create_publisher(Int32, "/mcu_wheel", 10)
        rate = max(1.0, float(self.get_parameter("command_rate_hz").value))
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(HELP)

    def publish(self):
        self.drive_pub.publish(Float32(data=float(self.stage)))
        self.steer_pub.publish(Int32(data=int(round(self.steering))))

    def tick(self):
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1).lower()
            limit = int(self.get_parameter("maximum_abs_stage").value)
            steer_limit = float(self.get_parameter("maximum_steering_deg").value)
            step = float(self.get_parameter("steering_step_deg").value)
            if key == "w":
                self.stage = min(self.selected_stage, limit)
            elif key == "s":
                self.stage = -min(self.selected_stage, limit)
            elif key in ("1", "2", "3"):
                selected = min(int(key), limit)
                self.selected_stage = selected
                if self.stage > 0:
                    self.stage = selected
                elif self.stage < 0:
                    self.stage = -selected
            elif key in ("x", " "):
                self.stage = 0
            elif key == "a":
                self.steering = min(steer_limit, self.steering + step)
            elif key == "d":
                self.steering = max(-steer_limit, self.steering - step)
            elif key == "c":
                self.steering = 0.0
            elif key == "q":
                self.stage = 0
                self.steering = 0.0
                self.publish()
                rclpy.shutdown()
                return
            else:
                continue
            self.get_logger().info(
                f"manual command: selected_speed={self.selected_stage}, "
                f"stage={self.stage}, steering={self.steering:.1f} deg")
        self.publish()

    def restore_terminal(self):
        if hasattr(self, "original_terminal"):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_terminal)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ManualKeyboardDrive()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stage = 0
                node.steering = 0.0
                node.publish()
            node.restore_terminal()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
