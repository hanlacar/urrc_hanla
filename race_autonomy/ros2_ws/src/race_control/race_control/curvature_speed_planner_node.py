#!/usr/bin/env python3
"""Plan a discrete drive stage from the camera path's maximum curvature."""

import time

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String

from .curvature_speed_planner import CurvatureStagePlanner, maximum_path_curvature


class CurvatureSpeedPlannerNode(Node):
    def __init__(self):
        super().__init__("curvature_speed_planner_node")
        defaults = {
            "path_topic": "/camera/path",
            "path_valid_topic": "/camera/path_valid",
            "path_confidence_topic": "/camera/path_confidence",
            "output_stage_topic": "/control/curvature_drive_stage",
            "output_curvature_topic": "/control/path_curvature",
            "output_valid_topic": "/control/curvature_plan_valid",
            "output_status_topic": "/control/curvature_speed_status",
            "minimum_path_confidence": 0.45,
            "path_timeout_sec": 0.5,
            "sample_spacing_m": 0.4,
            "slow_curvature_per_m": 0.25,
            "stop_curvature_per_m": 0.60,
            "high_curvature_stop_hold_sec": 1.0,
            "cruise_stage": 2,
            "slow_stage": 1,
            "publish_rate_hz": 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.path = []
        self.path_valid = False
        self.confidence = 0.0
        self.path_time = None
        self.stage_planner = CurvatureStagePlanner()
        self.stage_pub = self.create_publisher(Int32, self.p("output_stage_topic"), 10)
        self.curvature_pub = self.create_publisher(Float32, self.p("output_curvature_topic"), 10)
        self.valid_pub = self.create_publisher(Bool, self.p("output_valid_topic"), 10)
        self.status_pub = self.create_publisher(String, self.p("output_status_topic"), 10)
        self.create_subscription(Path, self.p("path_topic"), self.on_path, 10)
        self.create_subscription(Bool, self.p("path_valid_topic"),
                                 lambda msg: setattr(self, "path_valid", bool(msg.data)), 10)
        self.create_subscription(Float32, self.p("path_confidence_topic"),
                                 lambda msg: setattr(self, "confidence", float(msg.data)), 10)
        self.create_timer(1.0/max(1.0, float(self.p("publish_rate_hz"))), self.tick)

    def p(self, name):
        return self.get_parameter(name).value

    def on_path(self, msg):
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.path_time = time.monotonic()

    def tick(self):
        fresh = (self.path_time is not None and
                 time.monotonic()-self.path_time <= float(self.p("path_timeout_sec")))
        input_valid = (fresh and self.path_valid and
                       self.confidence >= float(self.p("minimum_path_confidence")))
        curvature = maximum_path_curvature(
            self.path, float(self.p("sample_spacing_m"))) if input_valid else None
        valid = curvature is not None
        stage, status = self.stage_planner.update(
            curvature, time.monotonic(), self.p("slow_curvature_per_m"),
            self.p("stop_curvature_per_m"), self.p("high_curvature_stop_hold_sec"),
            self.p("cruise_stage"), self.p("slow_stage"))
        value = float(curvature) if curvature is not None else 0.0
        self.stage_pub.publish(Int32(data=stage))
        self.curvature_pub.publish(Float32(data=value))
        self.valid_pub.publish(Bool(data=valid))
        self.status_pub.publish(String(data=f"{status};curvature={value:.4f};stage={stage}"))


def main(args=None):
    rclpy.init(args=args)
    node = CurvatureSpeedPlannerNode()
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
