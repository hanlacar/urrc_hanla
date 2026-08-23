#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8, Int32, String
from std_srvs.srv import SetBool


class ForwardStopControllerNode(Node):
    """Forward-stage-1/stop test controller guarded by measured motion."""

    def __init__(self):
        super().__init__("forward_stop_controller_node")
        defaults = {
            "allow_forward_command": False,
            "command_rate_hz": 20.0,
            "sensor_timeout_sec": 0.25,
            "forward_stage": 1,
            "steering_deg": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.requested_forward = False
        self.fault_latched = False
        self.motion_valid = False
        self.motion_direction = 0
        self.last_motion_time = None

        self.stage_pub = self.create_publisher(Float32, "/mcu_drive", 10)
        self.steer_pub = self.create_publisher(Int32, "/mcu_wheel", 10)
        self.status_pub = self.create_publisher(
            String, "/forward_stop/status", 10
        )
        self.active_pub = self.create_publisher(
            Bool, "/forward_stop/active", 10
        )
        self.create_subscription(
            Int8,
            "/vehicle/motion_direction",
            self.on_direction,
            10,
        )
        self.create_subscription(
            Bool, "/vehicle/motion_valid", self.on_valid, 10
        )
        self.create_service(
            SetBool, "/forward_stop/set_forward", self.set_forward
        )
        rate = max(1.0, float(self.get_parameter("command_rate_hz").value))
        self.create_timer(1.0 / rate, self.control)

    def on_direction(self, msg):
        self.motion_direction = int(msg.data)
        self.last_motion_time = time.monotonic()
        if self.motion_direction < 0:
            self.fault_latched = True
            self.requested_forward = False

    def on_valid(self, msg):
        self.motion_valid = bool(msg.data)

    def sensor_ready(self):
        timeout = float(self.get_parameter("sensor_timeout_sec").value)
        return (
            self.motion_valid
            and self.last_motion_time is not None
            and time.monotonic() - self.last_motion_time <= timeout
        )

    def set_forward(self, request, response):
        if not request.data:
            self.requested_forward = False
            self.fault_latched = False
            self.publish_command(0)
            response.success = True
            response.message = "stop requested; fault latch cleared"
            return response
        if not bool(self.get_parameter("allow_forward_command").value):
            response.success = False
            response.message = "refused: allow_forward_command is false"
            return response
        if not self.sensor_ready():
            response.success = False
            response.message = "refused: measured encoder state is not valid"
            return response
        if self.fault_latched or self.motion_direction < 0:
            response.success = False
            response.message = "refused: reverse motion or latched fault"
            return response
        self.requested_forward = True
        response.success = True
        response.message = "forward stage 1 requested"
        return response

    def publish_command(self, stage):
        self.stage_pub.publish(Float32(data=float(stage)))
        steering = float(self.get_parameter("steering_deg").value)
        self.steer_pub.publish(Int32(data=int(round(steering))))

    def control(self):
        ready = self.sensor_ready()
        active = self.requested_forward and ready and not self.fault_latched
        stage = int(self.get_parameter("forward_stage").value) if active else 0
        self.publish_command(stage)
        self.active_pub.publish(Bool(data=active))
        if self.fault_latched:
            status = "SAFE_STOP:REVERSE_DETECTED"
        elif not ready:
            status = "SAFE_STOP:ENCODER_INVALID"
        elif active:
            status = "FORWARD_STAGE_1"
        else:
            status = "STOPPED"
        self.status_pub.publish(String(data=status))

    def destroy_node(self):
        self.publish_command(0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ForwardStopControllerNode()
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
