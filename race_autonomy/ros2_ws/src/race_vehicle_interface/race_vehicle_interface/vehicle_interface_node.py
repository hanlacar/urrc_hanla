#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, Int64, String
from std_srvs.srv import SetBool

from .command_mapping import speed_to_stage, steering_command


class VehicleInterfaceNode(Node):
    """Fail-safe adapter. It starts disarmed and publishes zero while unsafe."""

    def __init__(self):
        super().__init__("vehicle_interface_node")
        defaults = {
            "allow_actuation": False,
            "steering_only": False,
            "encoder_topic": "/inpulse",
            "rpm_topic": "/rpm",
            "steering_feedback_topic": "/steer_angle",
            "target_speed_topic": "/camera/target_speed_mps",
            "target_steering_topic": "/camera/target_steering_deg",
            "cmd_driving_topic": "/mcu_drive",
            "cmd_steer_topic": "/mcu_wheel",
            "feedback_timeout_sec": 0.5,
            "target_timeout_sec": 0.3,
            "command_rate_hz": 20.0,
            "maximum_steering_deg": 27.0,
            # T870 v29: ROS stage 2 -> firmware 3.00, PWM 100 = 0.526 m/s.
            "maximum_abs_stage": 3,
            "stage_per_mps": 3.8022813688,
            "stage_sign": 1.0,
            "steering_sign": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.armed = False
        self.values = {}
        self.times = {}
        self.stage_pub = self.create_publisher(
            Float32, self.param("cmd_driving_topic"), 10
        )
        self.steer_pub = self.create_publisher(
            Int32, self.param("cmd_steer_topic"), 10
        )
        self.ready_pub = self.create_publisher(
            Bool, "/vehicle_interface/ready", 10
        )
        self.status_pub = self.create_publisher(
            String, "/vehicle_interface/status", 10
        )

        self.create_subscription(
            Int64,
            self.param("encoder_topic"),
            lambda msg: self.update("encoder", float(msg.data)),
            10,
        )
        self.create_subscription(
            Float32,
            self.param("rpm_topic"),
            lambda msg: self.update("rpm", float(msg.data)),
            10,
        )
        self.create_subscription(
            Float32,
            self.param("steering_feedback_topic"),
            lambda msg: self.update("steering_feedback", float(msg.data)),
            10,
        )
        self.create_subscription(
            Float32,
            self.param("target_speed_topic"),
            lambda msg: self.update("target_speed", float(msg.data)),
            10,
        )
        self.create_subscription(
            Float32,
            self.param("target_steering_topic"),
            lambda msg: self.update("target_steering", float(msg.data)),
            10,
        )
        self.create_service(
            SetBool, "/vehicle_interface/set_enabled", self.set_enabled
        )
        rate = max(1.0, float(self.param("command_rate_hz")))
        self.create_timer(1.0 / rate, self.control)
        self.get_logger().warning(
            "Vehicle interface started DISARMED; zero commands are enforced."
        )

    def param(self, name):
        return self.get_parameter(name).value

    def update(self, name, value):
        if math.isfinite(value):
            self.values[name] = value
            self.times[name] = time.monotonic()

    def safety_state(self):
        if not bool(self.param("allow_actuation")):
            return False, "allow_actuation_false"
        if int(self.param("maximum_abs_stage")) <= 0:
            return False, "maximum_abs_stage_not_calibrated"
        if float(self.param("stage_per_mps")) <= 0.0:
            return False, "stage_per_mps_not_calibrated"

        now = time.monotonic()
        feedback_timeout = float(self.param("feedback_timeout_sec"))
        target_timeout = float(self.param("target_timeout_sec"))
        for key in ("encoder", "rpm", "steering_feedback"):
            if key not in self.times or now - self.times[key] > feedback_timeout:
                return False, f"stale_{key}"
        for key in ("target_speed", "target_steering"):
            if key not in self.times or now - self.times[key] > target_timeout:
                return False, f"stale_{key}"
        return True, "ready"

    def set_enabled(self, request, response):
        if not request.data:
            self.armed = False
            self.publish_commands(0, 0.0)
            response.success = True
            response.message = "disarmed; zero command published"
            return response

        ready, reason = self.safety_state()
        self.armed = bool(ready)
        response.success = self.armed
        response.message = "armed" if self.armed else f"refused: {reason}"
        return response

    def publish_commands(self, stage, steer):
        self.stage_pub.publish(Float32(data=float(stage)))
        self.steer_pub.publish(Int32(data=int(round(steer))))

    def control(self):
        ready, reason = self.safety_state()
        active = self.armed and ready
        if not active:
            self.armed = False
            stage, steer = 0, 0.0
        else:
            stage = 0 if bool(self.param("steering_only")) else speed_to_stage(
                self.values["target_speed"], float(self.param("stage_per_mps")),
                int(self.param("maximum_abs_stage")), float(self.param("stage_sign")),
            )
            steer = steering_command(
                self.values["target_steering"],
                float(self.param("maximum_steering_deg")),
                float(self.param("steering_sign")),
            )
        self.publish_commands(stage, steer)
        self.ready_pub.publish(Bool(data=bool(ready)))
        status = ("active_steering_only" if bool(self.param("steering_only")) else "active") if active else f"safe_stop:{reason}"
        self.status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = VehicleInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Launch may shut down the rclpy context before this finally block.
        if rclpy.ok():
            try:
                node.publish_commands(0, 0.0)
            except rclpy._rclpy_pybind11.RCLError:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
