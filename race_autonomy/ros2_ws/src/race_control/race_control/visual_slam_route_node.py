#!/usr/bin/env python3
"""Record a map-frame SLAM route, then follow it with bounded BEV correction."""

import json
import math
import os
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .visual_slam_route import (align_route_to_pose, blend_bev_correction, load_route,
                                interpolate_y,
                                nearest_index, nearest_stamp_skew,
                                quaternion_yaw, route_to_base,
                                save_route, should_sample, wrap_angle)


class VisualSlamRouteNode(Node):
    def __init__(self):
        super().__init__("visual_slam_route")
        defaults = {
            "mode": "follow",
            "route_file": "",
            "map_frame": "map", "base_frame": "base_link",
            "record_on_start": True, "record_rate_hz": 10.0,
            "minimum_sample_distance_m": 0.10,
            "minimum_sample_yaw_deg": 5.0,
            "minimum_route_points": 20,
            "publish_rate_hz": 20.0,
            "forward_min_m": 0.30, "forward_max_m": 6.0,
            "minimum_local_points": 5,
            "maximum_route_error_m": 1.5,
            "maximum_heading_error_deg": 75.0,
            "tf_timeout_sec": 0.10, "pose_timeout_sec": 0.50,
            "bev_path_topic": "/camera/bev/path",
            "bev_valid_topic": "/camera/bev/path_valid",
            "bev_confidence_topic": "/camera/bev/path_confidence",
            "bev_timeout_sec": 0.30, "bev_minimum_confidence": 0.45,
            "require_bev": False,
            "align_route_to_start": False,
            "odom_topic": "/rtabmap/odom",
            "slam_bev_max_skew_sec": 0.10,
            "bev_correction_gain": 0.55,
            "bev_maximum_correction_m": 0.35,
            "output_path_topic": "/camera/path",
            "output_valid_topic": "/camera/path_valid",
            "output_confidence_topic": "/camera/path_confidence",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.mode = str(self.p("mode")).lower()
        if self.mode not in ("record", "follow"):
            raise ValueError("mode must be 'record' or 'follow'")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.points, self.source_points = [], []
        self.route_frame, self.progress = self.p("map_frame"), 0
        self.origin_aligned, self.origin_pose = False, None
        self.recording = self.mode == "record" and bool(self.p("record_on_start"))
        self.last_pose_time = None
        self.bev_path, self.bev_valid, self.bev_confidence = [], False, 0.0
        self.bev_time = None
        self.bev_stamp = None
        self.odom_stamps = deque(maxlen=100)

        self.path_pub = self.create_publisher(Path, self.p("output_path_topic"), 10)
        self.valid_pub = self.create_publisher(Bool, self.p("output_valid_topic"), 10)
        self.confidence_pub = self.create_publisher(
            Float32, self.p("output_confidence_topic"), 10)
        route_qos = QoSProfile(depth=1)
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.recorded_pub = self.create_publisher(
            Path, "/visual_slam/recorded_path", route_qos)
        self.raw_pub = self.create_publisher(Path, "/visual_slam/local_route", 10)
        self.status_pub = self.create_publisher(String, "/visual_slam/route_status", 10)
        self.sync_status_pub = self.create_publisher(
            String, "/visual_slam/sync_status", 10)
        self.correction_pub = self.create_publisher(
            Float32, "/visual_slam/bev_lateral_correction_m", 10)
        self.create_subscription(Path, self.p("bev_path_topic"), self.on_bev_path, 10)
        # RTAB-Map publishes odometry with sensor-data QoS (BEST_EFFORT).
        # A default RELIABLE subscription is incompatible and silently receives
        # no samples, which invalidates route/BEV timestamp synchronization.
        self.create_subscription(
            Odometry, self.p("odom_topic"), self.on_odom,
            qos_profile_sensor_data)
        self.create_subscription(Bool, self.p("bev_valid_topic"),
                                 lambda m: setattr(self, "bev_valid", bool(m.data)), 10)
        self.create_subscription(Float32, self.p("bev_confidence_topic"),
                                 lambda m: setattr(self, "bev_confidence", float(m.data)), 10)
        self.create_service(SetBool, "/visual_slam/set_recording", self.set_recording)
        self.create_service(Trigger, "/visual_slam/save_route", self.save_service)
        self.create_service(Trigger, "/visual_slam/reload_route", self.reload_service)
        self.create_service(Trigger, "/visual_slam/clear_route", self.clear_service)
        if self.mode == "follow":
            try:
                self.reload()
            except (OSError, ValueError) as error:
                self.get_logger().error(f"Route is not ready: {error}")
        rate = self.p("record_rate_hz") if self.mode == "record" else self.p("publish_rate_hz")
        self.create_timer(1.0 / max(1.0, float(rate)), self.tick)
        self.get_logger().warning(
            f"Visual SLAM route mode={self.mode}, propulsion remains controlled elsewhere")

    def p(self, name):
        return self.get_parameter(name).value

    def pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.p("map_frame")), str(self.p("base_frame")),
                rclpy.time.Time(),
                timeout=Duration(seconds=float(self.p("tf_timeout_sec"))))
        except TransformException as error:
            self.publish_invalid(f"tf_unavailable:{error}")
            return None
        t, q = transform.transform.translation, transform.transform.rotation
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1.0e-9
        age = self.get_clock().now().nanoseconds * 1.0e-9 - stamp
        if stamp > 0.0 and age > float(self.p("pose_timeout_sec")):
            self.publish_invalid(f"stale_tf:{age:.3f}s")
            return None
        self.last_pose_time = time.monotonic()
        return float(t.x), float(t.y), quaternion_yaw(q.x, q.y, q.z, q.w)

    def header(self, frame):
        from std_msgs.msg import Header
        return Header(stamp=self.get_clock().now().to_msg(), frame_id=str(frame))

    def make_path(self, points, frame):
        message = Path(header=self.header(frame))
        for x, y in points:
            pose = PoseStamped(header=message.header)
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    def on_bev_path(self, message):
        if message.header.frame_id != str(self.p("base_frame")) or any(
                not math.isfinite(value) for pose in message.poses
                for value in (pose.pose.position.x, pose.pose.position.y)):
            self.bev_path, self.bev_time, self.bev_stamp = [], None, None
            return
        self.bev_path = [(float(p.pose.position.x), float(p.pose.position.y))
                         for p in message.poses]
        self.bev_time = time.monotonic()
        self.bev_stamp = (float(message.header.stamp.sec)
                          + float(message.header.stamp.nanosec) * 1.0e-9)

    def on_odom(self, message):
        stamp = (float(message.header.stamp.sec)
                 + float(message.header.stamp.nanosec) * 1.0e-9)
        if math.isfinite(stamp) and stamp > 0.0:
            self.odom_stamps.append(stamp)

    def publish_invalid(self, reason):
        self.path_pub.publish(Path(header=self.header(self.p("base_frame"))))
        self.valid_pub.publish(Bool(data=False))
        self.confidence_pub.publish(Float32(data=0.0))
        self.status_pub.publish(String(data=json.dumps({
            "mode": self.mode, "valid": False, "reason": reason,
            "route_points": len(self.points), "progress_index": self.progress,
        }, separators=(",", ":"))))

    def tick(self):
        pose = self.pose()
        if pose is None:
            return
        if self.mode == "record":
            self.record_tick(*pose)
        else:
            self.follow_tick(*pose)

    def record_tick(self, x, y, yaw):
        if self.recording and should_sample(
                self.points, x, y, yaw, float(self.p("minimum_sample_distance_m")),
                math.radians(float(self.p("minimum_sample_yaw_deg")))):
            self.points.append((x, y, yaw))
        self.recorded_pub.publish(self.make_path([(p[0], p[1]) for p in self.points],
                                                 self.p("map_frame")))
        self.status_pub.publish(String(data=json.dumps({
            "mode": "record", "recording": self.recording,
            "route_points": len(self.points), "route_file": self.p("route_file"),
        }, separators=(",", ":"))))

    def follow_tick(self, x, y, yaw):
        if bool(self.p("align_route_to_start")) and not self.origin_aligned:
            self.points = align_route_to_pose(self.source_points, x, y, yaw)
            self.progress = 0
            self.origin_aligned = True
            self.origin_pose = (x, y, yaw)
            self.get_logger().warning(
                "Latched current vehicle pose as route origin: "
                f"map=({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f} deg), "
                "local=(0, 0, 0 deg)")
        if len(self.points) < int(self.p("minimum_route_points")):
            return self.publish_invalid("route_too_short")
        self.recorded_pub.publish(self.make_path(
            [(p[0], p[1]) for p in self.points], self.p("map_frame")))
        closest = nearest_index(self.points, x, y, self.progress)
        route_x, route_y, route_yaw = self.points[closest]
        distance = math.hypot(route_x - x, route_y - y)
        heading_error = abs(wrap_angle(route_yaw - yaw))
        if distance > float(self.p("maximum_route_error_m")):
            return self.publish_invalid(f"route_lost:{distance:.3f}m")
        if heading_error > math.radians(float(self.p("maximum_heading_error_deg"))):
            return self.publish_invalid(f"heading_mismatch:{math.degrees(heading_error):.1f}deg")
        self.progress = max(self.progress, closest)
        raw = route_to_base(self.points, self.progress, x, y, yaw,
                            float(self.p("forward_min_m")), float(self.p("forward_max_m")))
        if len(raw) < int(self.p("minimum_local_points")):
            return self.publish_invalid("route_finished_or_too_few_forward_points")
        bev_fresh = (self.bev_time is not None and
                     time.monotonic() - self.bev_time <= float(self.p("bev_timeout_sec")))
        stamp_skew = nearest_stamp_skew(self.odom_stamps, self.bev_stamp)
        time_aligned = stamp_skew <= float(self.p("slam_bev_max_skew_sec"))
        bev_used = (bev_fresh and self.bev_valid and
                    time_aligned and
                    self.bev_confidence >= float(self.p("bev_minimum_confidence")) and
                    len(self.bev_path) >= 2)
        if bool(self.p("require_bev")) and not bev_used:
            return self.publish_invalid("bev_unavailable_or_unsynchronized")
        if bool(self.p("require_bev")) and not any(
                interpolate_y(self.bev_path, x) is not None for x, _ in raw):
            return self.publish_invalid("bev_has_no_route_overlap")
        gain = 0.0
        if bev_used:
            confidence_scale = min(1.0, max(0.0, self.bev_confidence))
            gain = float(self.p("bev_correction_gain")) * confidence_scale
        fused, correction = blend_bev_correction(
            raw, self.bev_path if bev_used else [], gain,
            float(self.p("bev_maximum_correction_m")))
        confidence = max(0.0, min(1.0, 1.0 - distance /
                         max(float(self.p("maximum_route_error_m")), 1.0e-6)))
        self.raw_pub.publish(self.make_path(raw, self.p("base_frame")))
        self.path_pub.publish(self.make_path(fused, self.p("base_frame")))
        self.valid_pub.publish(Bool(data=True))
        self.confidence_pub.publish(Float32(data=float(confidence)))
        self.correction_pub.publish(Float32(data=float(correction)))
        self.sync_status_pub.publish(String(data=json.dumps({
            "valid": bool(time_aligned), "bev_fresh": bool(bev_fresh),
            "bev_stamp": self.bev_stamp, "odom_samples": len(self.odom_stamps),
            "slam_bev_skew_ms": (stamp_skew*1000.0 if math.isfinite(stamp_skew)
                                 else None),
            "maximum_skew_ms": float(self.p("slam_bev_max_skew_sec"))*1000.0,
            "bev_used": bool(bev_used),
        }, separators=(",", ":"))))
        self.status_pub.publish(String(data=json.dumps({
            "mode": "follow", "valid": True, "route_file": self.p("route_file"),
            "route_points": len(self.points), "progress_index": self.progress,
            "route_error_m": distance,
            "signed_route_lateral_error_m": (
                -math.sin(route_yaw) * (x - route_x)
                + math.cos(route_yaw) * (y - route_y)),
            "signed_heading_error_deg": math.degrees(wrap_angle(yaw - route_yaw)),
            "heading_error_deg": math.degrees(heading_error),
            "local_points": len(fused), "bev_used": bev_used,
            "bev_confidence": self.bev_confidence,
            "slam_bev_skew_ms": (stamp_skew*1000.0 if math.isfinite(stamp_skew)
                                 else None),
            "bev_lateral_correction_m": correction,
            "origin_aligned": self.origin_aligned,
            "origin_map_pose": self.origin_pose,
        }, separators=(",", ":"))))

    def set_recording(self, request, response):
        if self.mode != "record":
            response.success, response.message = False, "node is not in record mode"
            return response
        self.recording = bool(request.data)
        response.success = True
        response.message = "recording" if self.recording else "recording paused"
        return response

    def save(self):
        if len(self.points) < int(self.p("minimum_route_points")):
            raise ValueError(f"only {len(self.points)} route points")
        save_route(str(self.p("route_file")), self.points, str(self.p("map_frame")))

    def save_service(self, request, response):
        del request
        try:
            self.save()
            response.success, response.message = True, str(self.p("route_file"))
        except (OSError, ValueError) as error:
            response.success, response.message = False, str(error)
        return response

    def reload(self):
        frame, points = load_route(str(self.p("route_file")))
        if frame != str(self.p("map_frame")):
            raise ValueError(f"route frame {frame} != {self.p('map_frame')}")
        self.route_frame = frame
        self.source_points = list(points)
        self.points = list(points)
        self.progress = 0
        self.origin_aligned = not bool(self.p("align_route_to_start"))
        self.origin_pose = None
        self.get_logger().info(f"Loaded {len(points)} route points")

    def reload_service(self, request, response):
        del request
        try:
            self.reload()
            response.success, response.message = True, f"loaded {len(self.points)} points"
        except (OSError, ValueError) as error:
            response.success, response.message = False, str(error)
        return response

    def clear_service(self, request, response):
        del request
        if self.mode != "record" or self.recording:
            response.success = False
            response.message = "pause recording before clearing the in-memory route"
        else:
            self.points = []
            response.success, response.message = True, "in-memory route cleared; file untouched"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisualSlamRouteNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
