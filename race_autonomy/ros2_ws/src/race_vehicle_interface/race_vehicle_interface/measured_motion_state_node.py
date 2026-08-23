#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, String


class MeasuredMotionStateNode(Node):
    """Expose physical motion state from quadrature direction only."""

    def __init__(self):
        super().__init__("measured_motion_state_node")
        self.declare_parameter("sensor_timeout_sec", 0.25)
        self.direction = 0
        self.direction_time = None
        self.sensor_valid = False

        self.create_subscription(
            Int8,
            "/wheel_encoder/direction",
            self.on_direction,
            10,
        )
        self.create_subscription(
            Bool, "/wheel_encoder/valid", self.on_valid, 10
        )
        self.direction_pub = self.create_publisher(
            Int8, "/vehicle/motion_direction", 10
        )
        self.state_pub = self.create_publisher(
            String, "/vehicle/motion_state", 10
        )
        self.valid_pub = self.create_publisher(
            Bool, "/vehicle/motion_valid", 10
        )
        self.create_timer(0.05, self.publish_state)

    def on_direction(self, msg):
        if msg.data in (-1, 0, 1):
            self.direction = int(msg.data)
            self.direction_time = time.monotonic()

    def on_valid(self, msg):
        self.sensor_valid = bool(msg.data)

    def publish_state(self):
        timeout = float(self.get_parameter("sensor_timeout_sec").value)
        fresh = (
            self.direction_time is not None
            and time.monotonic() - self.direction_time <= timeout
        )
        valid = self.sensor_valid and fresh
        direction = self.direction if valid else 0
        if not valid:
            state = "SENSOR_FAULT"
        elif direction > 0:
            state = "FORWARD"
        elif direction < 0:
            state = "REVERSE"
        else:
            state = "STOPPED"
        self.direction_pub.publish(Int8(data=direction))
        self.state_pub.publish(String(data=state))
        self.valid_pub.publish(Bool(data=valid))


def main(args=None):
    rclpy.init(args=args)
    node = MeasuredMotionStateNode()
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
