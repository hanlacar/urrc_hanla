#!/usr/bin/env python3

import json
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String


class LaneCenterNode(Node):
    """Detect two white lane lines and publish their center."""

    def __init__(self) -> None:
        super().__init__("lane_center")
        self.declare_parameter("input_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("roi_top_ratio", 0.52)
        self.declare_parameter("roi_top_width_ratio", 0.45)
        self.declare_parameter("white_saturation_max", 80)
        self.declare_parameter("white_value_min", 180)
        self.declare_parameter("minimum_band_pixels", 30)
        self.declare_parameter("minimum_points_per_line", 4)
        self.declare_parameter("lookahead_ratio", 0.68)
        self.declare_parameter("path_near_distance_m", 0.5)
        self.declare_parameter("path_far_distance_m", 6.0)
        self.declare_parameter("path_lateral_span_near_m", 3.0)
        self.declare_parameter("path_lateral_span_far_m", 1.2)

        self.debug_pub = self.create_publisher(
            Image, "/perception/lane_center_image", 10
        )
        self.detected_pub = self.create_publisher(
            Bool, "/perception/lane_detected", 10
        )
        self.error_pub = self.create_publisher(
            Float32, "/perception/lane_error", 10
        )
        self.heading_pub = self.create_publisher(
            Float32, "/perception/lane_heading_error", 10
        )
        self.state_pub = self.create_publisher(
            String, "/perception/lane_state_json", 10
        )
        self.path_pub = self.create_publisher(
            String, "/perception/local_path_json", 10
        )
        topic = str(self.get_parameter("input_topic").value)
        self.sub = self.create_subscription(
            Image, topic, self.image_callback, qos_profile_sensor_data
        )
        self.get_logger().info(f"Lane detector subscribed to {topic}")

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.from_image_msg(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return

        height, width = frame.shape[:2]
        mask, roi_polygon = self.make_white_mask(frame)
        left_points, right_points = self.find_lane_points(mask)
        debug = frame.copy()
        cv2.polylines(debug, [roi_polygon], True, (255, 180, 0), 2)

        detected = len(left_points) >= int(
            self.get_parameter("minimum_points_per_line").value
        ) and len(right_points) >= int(
            self.get_parameter("minimum_points_per_line").value
        )

        lateral_error = 0.0
        heading_error = 0.0
        confidence = 0.0
        center_points = []
        path_points = []
        if detected:
            left_fit = np.polyfit(
                [p[1] for p in left_points], [p[0] for p in left_points], 2
            )
            right_fit = np.polyfit(
                [p[1] for p in right_points], [p[0] for p in right_points], 2
            )
            ys = np.linspace(height - 1, int(height * 0.55), 20)
            for y_value in ys:
                left_x = float(np.polyval(left_fit, y_value))
                right_x = float(np.polyval(right_fit, y_value))
                if left_x < right_x:
                    center_points.append(
                        (int((left_x + right_x) * 0.5), int(y_value))
                    )

            if len(center_points) < 4:
                detected = False
            else:
                bottom_x = float(center_points[0][0])
                lookahead_y = int(
                    height * float(self.get_parameter("lookahead_ratio").value)
                )
                target = min(
                    center_points, key=lambda point: abs(point[1] - lookahead_y)
                )
                lateral_error = (bottom_x - width * 0.5) / (width * 0.5)
                heading_error = math.atan2(
                    target[0] - bottom_x,
                    max(center_points[0][1] - target[1], 1),
                )
                confidence = min(len(left_points), len(right_points)) / 12.0
                confidence = float(min(confidence, 1.0))
                path_points = self.to_vehicle_path(center_points, width, height)

                self.draw_curve(debug, left_fit, height, (0, 255, 0))
                self.draw_curve(debug, right_fit, height, (0, 255, 0))
                cv2.polylines(
                    debug,
                    [np.asarray(center_points, dtype=np.int32)],
                    False,
                    (255, 0, 255),
                    4,
                )
                cv2.circle(debug, target, 9, (0, 0, 255), -1)

        for point in left_points + right_points:
            cv2.circle(debug, point, 3, (0, 255, 255), -1)
        cv2.line(
            debug,
            (width // 2, height),
            (width // 2, int(height * 0.5)),
            (255, 255, 0),
            2,
        )
        status = (
            f"lane={detected} error={lateral_error:+.3f} "
            f"heading={heading_error:+.3f}"
        )
        cv2.putText(
            debug, status, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (0, 255, 0) if detected else (0, 0, 255), 2
        )

        detected_msg = Bool()
        detected_msg.data = detected
        error_msg = Float32()
        error_msg.data = float(lateral_error)
        heading_msg = Float32()
        heading_msg.data = float(heading_error)
        state_msg = String()
        state_msg.data = json.dumps(
            {
                "detected": detected,
                "lateral_error": lateral_error,
                "heading_error": heading_error,
                "confidence": confidence,
                "center_points_px": center_points,
            }
        )
        self.detected_pub.publish(detected_msg)
        self.error_pub.publish(error_msg)
        self.heading_pub.publish(heading_msg)
        self.state_pub.publish(state_msg)
        path_msg = String()
        path_msg.data = json.dumps(
            {
                "detected": detected,
                "confidence": confidence,
                "coordinate_frame": "base_link",
                "axis_convention": "forward_m,lateral_right_m",
                "points": path_points,
            }
        )
        self.path_pub.publish(path_msg)
        debug_msg = self.to_image_msg(debug)
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

    def to_vehicle_path(self, center_points, width, height):
        """Approximate image centerline points in the vehicle ground plane.

        This is intentionally parameterised so measured camera calibration can
        replace the initial trapezoidal approximation without controller edits.
        """
        near = float(self.get_parameter("path_near_distance_m").value)
        far = float(self.get_parameter("path_far_distance_m").value)
        near_span = float(self.get_parameter("path_lateral_span_near_m").value)
        far_span = float(self.get_parameter("path_lateral_span_far_m").value)
        roi_top = height * float(self.get_parameter("roi_top_ratio").value)
        usable_height = max((height - 1) - roi_top, 1.0)
        path = []
        for pixel_x, pixel_y in center_points:
            distance_ratio = ((height - 1) - pixel_y) / usable_height
            distance_ratio = min(max(distance_ratio, 0.0), 1.0)
            forward = near + distance_ratio * (far - near)
            lateral_span = near_span + distance_ratio * (far_span - near_span)
            lateral = ((pixel_x - width * 0.5) / max(width * 0.5, 1.0)) * lateral_span
            path.append([round(forward, 4), round(lateral, 4)])
        return path

    def make_white_mask(self, frame):
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        saturation_max = int(
            self.get_parameter("white_saturation_max").value
        )
        value_min = int(self.get_parameter("white_value_min").value)
        white = cv2.inRange(
            hsv,
            np.array([0, 0, value_min], dtype=np.uint8),
            np.array([179, saturation_max, 255], dtype=np.uint8),
        )

        top_y = int(height * float(self.get_parameter("roi_top_ratio").value))
        top_half = int(
            width
            * float(self.get_parameter("roi_top_width_ratio").value)
            * 0.5
        )
        polygon = np.array(
            [
                [0, height - 1],
                [width // 2 - top_half, top_y],
                [width // 2 + top_half, top_y],
                [width - 1, height - 1],
            ],
            dtype=np.int32,
        )
        roi = np.zeros_like(white)
        cv2.fillPoly(roi, [polygon], 255)
        mask = cv2.bitwise_and(white, roi)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask, polygon

    def find_lane_points(self, mask):
        height, width = mask.shape
        top_y = int(height * float(self.get_parameter("roi_top_ratio").value))
        band_count = 12
        band_height = max((height - top_y) // band_count, 1)
        minimum_pixels = int(
            self.get_parameter("minimum_band_pixels").value
        )
        left_points = []
        right_points = []
        for index in range(band_count):
            y_bottom = height - index * band_height
            y_top = max(y_bottom - band_height, top_y)
            if y_top >= y_bottom:
                continue
            band = mask[y_top:y_bottom]
            histogram = np.count_nonzero(band, axis=0)
            midpoint = width // 2
            left_x = int(np.argmax(histogram[:midpoint]))
            right_x = int(np.argmax(histogram[midpoint:]) + midpoint)
            y_center = (y_top + y_bottom) // 2
            if histogram[left_x] >= minimum_pixels:
                left_points.append((left_x, y_center))
            if histogram[right_x] >= minimum_pixels:
                right_points.append((right_x, y_center))
        return left_points, right_points

    @staticmethod
    def draw_curve(frame, coefficients, height, color):
        ys = np.linspace(height - 1, int(height * 0.55), 30)
        points = np.array(
            [[int(np.polyval(coefficients, y)), int(y)] for y in ys],
            dtype=np.int32,
        )
        cv2.polylines(frame, [points], False, color, 3)

    @staticmethod
    def from_image_msg(msg: Image):
        if msg.encoding not in ("rgb8", "bgr8"):
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        frame = row[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(frame)

    @staticmethod
    def to_image_msg(frame) -> Image:
        msg = Image()
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(frame).tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneCenterNode()
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
