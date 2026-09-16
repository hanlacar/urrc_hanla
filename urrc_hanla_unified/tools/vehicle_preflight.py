#!/usr/bin/env python3
"""Read-only ROS graph gate before a lifted-wheel or low-speed vehicle test."""

from __future__ import annotations

from collections import Counter, defaultdict
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


EXPECTED_PUBLISHERS = {
    "/odom": 1,
    "/front/scan": 1,
    "/mcu/encoder": 1,
    "/mcu/steer_a0": 1,
    "/mcu/steering_feedback_valid": 1,
    "/mcu/steering_feedback_status": 1,
    "/mcu/drive_block_reason": 1,
    "/mcu/cmd_drive": 1,
    "/mcu/cmd_wheel": 1,
    "/mcu/cmd_stop": 1,
    "/cmd_drive": 1,
    "/cmd_wheel": 1,
    "/cmd_stop": 1,
    "/avoidance/drive_cmd": 1,
    "/avoidance/wheel_cmd": 1,
    "/avoidance/stop_cmd": 1,
    "/avoidance/active": 1,
}

REQUIRED_SERVICES = {
    "/dr_route/start",
    "/dr_route/stop",
    "/dr_route/reset",
    "/avoidance/route/start",
    "/avoidance/route/stop",
    "/vehicle/emergency_stop",
    "/vehicle/emergency_stop/reset",
}


def full_name(name: str, namespace: str) -> str:
    namespace = namespace.rstrip("/")
    return f"{namespace}/{name}" if namespace else f"/{name}"


def gid(value) -> tuple[int, ...]:
    try:
        return tuple(int(item) for item in value)
    except TypeError:
        return ()


class GraphAudit(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_preflight_audit")
        self.tf_sources = defaultdict(set)
        dynamic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            TFMessage, "/tf",
            lambda msg, info: self.record_tf("/tf", msg, info), dynamic_qos)
        self.create_subscription(
            TFMessage, "/tf_static",
            lambda msg, info: self.record_tf("/tf_static", msg, info), static_qos)

    def record_tf(self, topic, msg, info) -> None:
        publisher_gid = (info.get("publisher_gid") if isinstance(info, dict)
                         else getattr(info, "publisher_gid", ()))
        publisher = gid(publisher_gid)
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if parent == "odom" and child == "base_link":
                self.tf_sources[(parent, child)].add((topic, publisher))

    def endpoint_name(self, endpoint) -> str:
        return full_name(endpoint.node_name, endpoint.node_namespace)

    def publisher_gid_names(self, topic: str) -> dict[tuple[int, ...], str]:
        return {
            gid(endpoint.endpoint_gid): self.endpoint_name(endpoint)
            for endpoint in self.get_publishers_info_by_topic(topic)
        }


def main() -> None:
    rclpy.init()
    node = GraphAudit()
    failures = []
    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        print("=== node names ===")
        names = [full_name(name, namespace)
                 for name, namespace in node.get_node_names_and_namespaces()
                 if name != node.get_name()]
        duplicates = sorted(name for name, count in Counter(names).items()
                            if count > 1)
        print("duplicates:", duplicates or "none")
        if duplicates:
            failures.append(f"duplicate node names: {duplicates}")
        for required_name in ("/lidar_safety", "/mcu_odom_adapter",
                              "/integrated_mcu_simple_compat"):
            count = names.count(required_name)
            print(f"{required_name}: {count}")
            if count != 1:
                failures.append(f"{required_name} count={count}, expected=1")

        serial_bridges = [name for name in names if name in {
            "/t870_cmd_bridge", "/t870_mcu_simple_bridge"}]
        print("serial MCU bridge:", serial_bridges or "none")
        if len(serial_bridges) != 1:
            failures.append(
                "exactly one serial MCU bridge is required; found "
                f"{serial_bridges}")

        print("\n=== critical topic endpoints ===")
        for topic, expected in EXPECTED_PUBLISHERS.items():
            publishers = [node.endpoint_name(endpoint) for endpoint in
                          node.get_publishers_info_by_topic(topic)]
            subscribers = [node.endpoint_name(endpoint) for endpoint in
                           node.get_subscriptions_info_by_topic(topic)]
            print(f"{topic}: publishers={len(publishers)} {publishers}; "
                  f"subscribers={len(subscribers)} {subscribers}")
            if len(publishers) != expected:
                failures.append(
                    f"{topic} publisher count={len(publishers)}, "
                    f"expected={expected}: {publishers}")

        topic_types = dict(node.get_topic_names_and_types())
        print("\n=== /cmd_* and /avoidance/* inventory ===")
        for topic in sorted(topic_types):
            if topic.startswith("/cmd_") or topic.startswith("/avoidance/"):
                publishers = [node.endpoint_name(endpoint) for endpoint in
                              node.get_publishers_info_by_topic(topic)]
                subscribers = [node.endpoint_name(endpoint) for endpoint in
                               node.get_subscriptions_info_by_topic(topic)]
                print(f"{topic} {topic_types[topic]}: pub={publishers}, "
                      f"sub={subscribers}")

        services = {name for name, _ in node.get_service_names_and_types()}
        print("\n=== required start/stop services ===")
        for service in sorted(REQUIRED_SERVICES):
            present = service in services
            print(f"{service}: {'OK' if present else 'MISSING'}")
            if not present:
                failures.append(f"missing service: {service}")

        print("\n=== odom -> base_link TF ownership ===")
        sources = node.tf_sources[("odom", "base_link")]
        resolved = []
        gid_names = {}
        for topic in ("/tf", "/tf_static"):
            gid_names.update(node.publisher_gid_names(topic))
        for topic, publisher_gid in sorted(sources):
            owner = gid_names.get(publisher_gid)
            if owner is None and not publisher_gid:
                # Some Jazzy RMW implementations omit publisher_gid from the
                # Python callback info. A sole endpoint is still unambiguous.
                endpoints = [node.endpoint_name(endpoint) for endpoint in
                             node.get_publishers_info_by_topic(topic)]
                if len(endpoints) == 1:
                    owner = endpoints[0]
            resolved.append((topic, owner or f"unknown_gid:{publisher_gid}"))
        print("sources:", resolved or "none observed in 3 seconds")
        if len(sources) != 1:
            failures.append(
                "odom -> base_link TF must have exactly one publisher; "
                f"observed {resolved}")
        elif resolved[0][0] != "/tf":
            failures.append("odom -> base_link must be dynamic /tf, not static")
        elif resolved[0][1] != "/mcu_odom_adapter":
            failures.append(
                "odom -> base_link owner must be /mcu_odom_adapter; "
                f"observed {resolved[0][1]}")

        print("\n=== RESULT ===")
        if failures:
            for failure in failures:
                print("FAIL:", failure)
            raise SystemExit(1)
        print("PASS: real-vehicle graph has one owner for each critical path")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
