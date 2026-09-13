"""Pass camera steering through, overriding it when a LaserScan sees an obstacle."""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32, String


def sector_min(scan, low_deg, high_deg):
    values = []
    for index, distance in enumerate(scan.ranges):
        angle = math.degrees(scan.angle_min + index * scan.angle_increment)
        if low_deg <= angle <= high_deg and math.isfinite(distance):
            if scan.range_min <= distance <= scan.range_max:
                values.append(distance)
    return min(values) if values else math.inf


class LidarCameraGuard(Node):
    def __init__(self):
        super().__init__("lidar_camera_guard")
        defaults = {
            "scan_topic": "/scan",
            "camera_wheel_topic": "/camera_wheel",
            "output_wheel_topic": "/lidar_wheel",
            "output_stop_topic": "/lidar_stop",
            "drive_mode_topic": "/drive_mode",
            "avoidance_active_topic": "/avoidance/active",
            "avoidance_section": "5",
            "avoid_distance_m": 1.50,
            "clear_distance_m": 1.80,
            "clear_confirm_sec": 0.80,
            "hard_stop_distance_m": 0.45,
            "side_clearance_m": 0.65,
            "front_half_angle_deg": 18.0,
            "side_sector_angle_deg": 65.0,
            "avoid_steer_deg": 20,
            "maximum_steer_deg": 27,
            "scan_timeout_sec": 0.30,
            "camera_wheel_timeout_sec": 0.50,
            "publish_hz": 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.avoid_distance = float(self.get_parameter("avoid_distance_m").value)
        self.clear_distance = float(self.get_parameter("clear_distance_m").value)
        self.clear_confirm = float(self.get_parameter("clear_confirm_sec").value)
        self.hard_stop_distance = float(self.get_parameter("hard_stop_distance_m").value)
        self.side_clearance = float(self.get_parameter("side_clearance_m").value)
        self.front_angle = float(self.get_parameter("front_half_angle_deg").value)
        self.side_angle = float(self.get_parameter("side_sector_angle_deg").value)
        self.avoid_steer = int(self.get_parameter("avoid_steer_deg").value)
        self.max_steer = int(self.get_parameter("maximum_steer_deg").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        self.camera_timeout = float(self.get_parameter("camera_wheel_timeout_sec").value)

        self.scan = None
        self.scan_time = 0.0
        self.camera_wheel = 0
        self.camera_time = 0.0
        self.drive_mode = ""
        self.avoiding = False
        self.avoid_direction = 0
        self.clear_since = None
        self.pub_wheel = self.create_publisher(
            Int32, str(self.get_parameter("output_wheel_topic").value), 10)
        self.pub_stop = self.create_publisher(
            Bool, str(self.get_parameter("output_stop_topic").value), 10)
        self.pub_active = self.create_publisher(
            Bool, str(self.get_parameter("avoidance_active_topic").value), 10)
        self.pub_state = self.create_publisher(String, "/avoidance/state", 10)
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self._on_scan, 10)
        self.create_subscription(
            Int32, str(self.get_parameter("camera_wheel_topic").value),
            self._on_camera_wheel, 10)
        self.create_subscription(
            String, str(self.get_parameter("drive_mode_topic").value),
            lambda msg: setattr(self, "drive_mode", str(msg.data).strip()), 10)
        hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            "camera steering guarded by live lidar; missing input causes STOP")

    def _on_scan(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

    def _on_camera_wheel(self, msg):
        self.camera_wheel = max(-self.max_steer, min(self.max_steer, int(msg.data)))
        self.camera_time = time.monotonic()

    def _publish(self, wheel, stop):
        wheel_msg = Int32()
        wheel_msg.data = int(max(-self.max_steer, min(self.max_steer, wheel)))
        stop_msg = Bool()
        stop_msg.data = bool(stop)
        self.pub_wheel.publish(wheel_msg)
        self.pub_stop.publish(stop_msg)

    def _state(self, value):
        self.pub_state.publish(String(data=value))

    def _tick(self):
        now = time.monotonic()
        active = self.drive_mode == str(
            self.get_parameter("avoidance_section").value).strip()
        self.pub_active.publish(Bool(data=active))
        if not active:
            self.avoiding = False
            self.avoid_direction = 0
            self.clear_since = None
            self._publish(self.camera_wheel, False)
            self._state("INACTIVE")
            return
        if (self.scan is None or now - self.scan_time > self.scan_timeout or
                now - self.camera_time > self.camera_timeout):
            self._publish(0, True)
            self._state("STOP:INPUT_STALE")
            return

        front = sector_min(self.scan, -self.front_angle, self.front_angle)
        left = sector_min(self.scan, self.front_angle, self.side_angle)
        right = sector_min(self.scan, -self.side_angle, -self.front_angle)

        if front <= self.hard_stop_distance:
            self._publish(0, True)
            self._state("STOP:OBSTACLE_TOO_CLOSE")
            self.clear_since = None
        elif self.avoiding:
            if max(left, right) < self.side_clearance:
                self._publish(0, True)
                self._state("STOP:NO_SIDE_CLEARANCE")
                self.clear_since = None
                return
            if front >= self.clear_distance:
                if self.clear_since is None:
                    self.clear_since = now
                elif now - self.clear_since >= self.clear_confirm:
                    self.avoiding = False
                    self.avoid_direction = 0
                    self.clear_since = None
            else:
                self.clear_since = None
            if self.avoiding:
                self._publish(self.avoid_direction * self.avoid_steer, False)
                self._state(
                    "AVOID_LEFT" if self.avoid_direction > 0 else "AVOID_RIGHT")
            else:
                self._publish(self.camera_wheel, False)
                self._state("CAMERA_REACQUIRE")
        elif front < self.avoid_distance:
            if max(left, right) < self.side_clearance:
                self._publish(0, True)
                self._state("STOP:NO_SIDE_CLEARANCE")
            else:
                self.avoiding = True
                self.avoid_direction = 1 if left >= right else -1
                self.clear_since = None
                self._publish(self.avoid_direction * self.avoid_steer, False)
                self._state(
                    "AVOID_LEFT" if self.avoid_direction > 0 else "AVOID_RIGHT")
        else:
            self._publish(self.camera_wheel, False)
            self._state("CAMERA_FOLLOW")


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
