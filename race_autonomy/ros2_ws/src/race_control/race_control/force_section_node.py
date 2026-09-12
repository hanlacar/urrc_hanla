"""Publish a selected mission section for stationary integration tests."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, String


class ForceSection(Node):
    def __init__(self):
        super().__init__("force_section")
        self.declare_parameter("section", 2)
        self.declare_parameter("publish_rate_hz", 2.0)
        section = int(self.get_parameter("section").value)
        if not 1 <= section <= 11:
            raise ValueError("section must be between 1 and 11")
        self.section = section
        self.section_pub = self.create_publisher(Int8, "/mission/section", 10)
        self.valid_pub = self.create_publisher(Bool, "/mission/route_valid", 10)
        self.status_pub = self.create_publisher(String, "/mission/route_waypoint_status", 10)
        period = 1.0 / max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self.publish)
        self.get_logger().warning(
            f"FORCE SECTION ACTIVE: publishing section {self.section}; no GPS/MCU required")

    def publish(self):
        self.section_pub.publish(Int8(data=self.section))
        self.valid_pub.publish(Bool(data=True))
        self.status_pub.publish(String(data=json.dumps({
            "valid": True, "forced": True, "section": self.section,
        }, separators=(",", ":"))))


def main(args=None):
    rclpy.init(args=args)
    node = ForceSection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
