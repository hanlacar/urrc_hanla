"""Convert DR CSV follower commands directly into final MCU commands."""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String


class CsvOnlyCommandNode(Node):
    def __init__(self):
        super().__init__("csv_only_command")
        self.declare_parameter("source_timeout_sec", 0.5)
        self.declare_parameter("stage_1_speed_mps", 0.229)
        self.declare_parameter("stage_2_speed_mps", 0.455)
        self.declare_parameter("stage_3_speed_mps", 0.70)
        self.declare_parameter("maximum_steering_deg", 27.0)
        self.declare_parameter("maximum_steering_rate_deg_s", 20.0)

        self.drive = None
        self.wheel = None
        self.drive_time = 0.0
        self.wheel_time = 0.0
        self.last_steer = 0.0
        self.last_tick = time.monotonic()

        self.speed_pub = self.create_publisher(Float32, "/mcu/target_speed_mps", 10)
        self.steer_pub = self.create_publisher(Float32, "/mcu/target_steering_deg", 10)
        self.stop_pub = self.create_publisher(Bool, "/mcu/emergency_stop", 10)
        self.status_pub = self.create_publisher(String, "/csv_only/status", 10)
        self.create_subscription(Float32, "/gps_drive", self.on_drive, 10)
        self.create_subscription(Int32, "/gps_wheel", self.on_wheel, 10)
        self.create_timer(0.05, self.tick)

    def p(self, name):
        return self.get_parameter(name).value

    def on_drive(self, msg):
        self.drive = float(msg.data)
        self.drive_time = time.monotonic()

    def on_wheel(self, msg):
        self.wheel = float(msg.data)
        self.wheel_time = time.monotonic()

    def stage_speed(self, value):
        sign = -1.0 if value < 0.0 else 1.0
        stage = int(round(abs(value)))
        if stage == 0:
            return 0.0
        if stage not in (1, 2, 3) or abs(abs(value) - stage) > 1e-3:
            return 0.0
        return sign * float(self.p(f"stage_{stage}_speed_mps"))

    def tick(self):
        now = time.monotonic()
        timeout = float(self.p("source_timeout_sec"))
        fresh = (
            self.drive is not None and self.wheel is not None
            and now - self.drive_time <= timeout
            and now - self.wheel_time <= timeout
        )
        speed = self.stage_speed(self.drive) if fresh else 0.0
        target = self.wheel if fresh else 0.0
        target = max(-float(self.p("maximum_steering_deg")),
                     min(float(self.p("maximum_steering_deg")), target))
        dt = max(0.0, min(0.25, now - self.last_tick))
        delta = float(self.p("maximum_steering_rate_deg_s")) * dt
        steer = max(self.last_steer - delta, min(self.last_steer + delta, target))
        if not math.isfinite(speed) or not math.isfinite(steer):
            speed, steer, fresh = 0.0, 0.0, False
        self.last_steer = steer
        self.last_tick = now
        self.speed_pub.publish(Float32(data=float(speed)))
        self.steer_pub.publish(Float32(data=float(steer)))
        self.stop_pub.publish(Bool(data=False))
        self.status_pub.publish(String(data="TRACKING" if fresh else "WAITING_FOR_DR"))


def main(args=None):
    rclpy.init(args=args)
    node = CsvOnlyCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
