"""Bridge mission_manager GPS route status to camera mission section/stop inputs."""

import json
import time
from dataclasses import asdict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, Int32, String

from .route_waypoints import RouteEvents, load_waypoints


class SectionTransitionNode(Node):
    def __init__(self):
        super().__init__("section_transition_node")
        for name, default in {
            "route_csv": "", "segments_yaml": "",
            "status_topic": "/gps_navigation/status",
            "status_timeout_sec": 0.5,
        }.items():
            self.declare_parameter(name, default)
        self.events = RouteEvents(load_waypoints(
            self.get_parameter("route_csv").value,
            self.get_parameter("segments_yaml").value))
        self.timeout = float(self.get_parameter("status_timeout_sec").value)
        if not 0 < self.timeout < 60:
            raise ValueError("status_timeout_sec must be between 0 and 60")
        self.last_update = -float("inf")
        self.point = None
        self.arrival = None
        self.active_section = None
        self.error = "waiting for GPS status"
        self.section_pub = self.create_publisher(Int8, "/mission/section", 10)
        self.valid_pub = self.create_publisher(Bool, "/mission/route_valid", 10)
        self.index_pub = self.create_publisher(Int32, "/mission/stop_line_waypoint", 10)
        self.reached_pub = self.create_publisher(Bool, "/mission/stop_line_reached", 10)
        self.ramp_pub = self.create_publisher(Bool, "/mission/ramp_waypoint_reached", 10)
        self.status_pub = self.create_publisher(String, "/mission/route_waypoint_status", 10)
        self.create_subscription(String, self.get_parameter("status_topic").value,
                                 self.on_status, 10)
        self.create_subscription(Int8, "/mission/active_section", self.on_active_section, 10)
        self.create_timer(0.05, self.publish)

    def on_active_section(self, msg):
        self.active_section = (int(msg.data), time.monotonic())

    def on_status(self, msg):
        try:
            self.point, self.arrival = self.events.update(json.loads(msg.data))
        except (ValueError, TypeError, KeyError) as exc:
            self.point = None
            self.arrival = None
            self.events.arrival_index = None
            self.error = str(exc)
            return
        self.last_update = time.monotonic()
        self.error = ""

    def publish(self):
        now = time.monotonic()
        valid = self.point is not None and now - self.last_update <= self.timeout
        if not valid:
            self.events.arrival_index = None
            self.arrival = None
        if valid:
            self.section_pub.publish(Int8(data=self.point.section))
        reached = valid and self.arrival is not None
        # Two different DDS topics have no cross-topic delivery ordering.
        # Wait for the consumer's section acknowledgement, then repeat the
        # level while stopped so arrival cannot be lost during section entry.
        ramp_reached = bool(reached and self.point.section == 2 and
                            self.active_section is not None and
                            self.active_section[0] == 2 and
                            now - self.active_section[1] <= self.timeout)
        self.valid_pub.publish(Bool(data=valid))
        self.index_pub.publish(Int32(data=self.arrival if reached else -1))
        self.reached_pub.publish(Bool(data=reached))
        self.ramp_pub.publish(Bool(data=ramp_reached))
        self.status_pub.publish(String(data=json.dumps({
            "valid": valid,
            "reason": self.error if self.error else ("" if valid else "status timeout"),
            "section": self.point.section if valid else None,
            "route_index": self.point.index if valid else None,
            "stop_line": asdict(self.events.points[self.arrival]) if reached else None,
            "ramp_arrival_delivered": ramp_reached,
        }, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SectionTransitionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
