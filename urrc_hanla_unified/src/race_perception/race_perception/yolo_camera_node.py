#!/usr/bin/env python3

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Bool, String
from ultralytics import YOLO

from .segmentation_path import (
    centerline_from_mask,
    pixels_to_ground_path,
    quaternion_to_pitch_deg,
)


class YoloCameraNode(Node):
    """Run a trained YOLO11n segmentation model on the RealSense color stream."""

    def __init__(self) -> None:
        super().__init__("yolo_camera")

        self.declare_parameter(
            "weights",
            "/home/parkjinwoo/camera_ws/models/hanla_yolo11n_seg_best.pt",
        )
        self.declare_parameter(
            "input_topic", "/camera/image_raw"
        )
        self.declare_parameter("output_topic", "/perception/detections_image")
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("inference_fps", 15.0)
        self.declare_parameter("device", "")
        self.declare_parameter("road_class_names", ["road", "drivable_area"])
        self.declare_parameter("path_band_count", 16)
        self.declare_parameter("path_minimum_pixels", 20)
        self.declare_parameter("path_top_ratio", 0.35)
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_height_m", 0.0)
        self.declare_parameter("camera_pitch_down_deg", 10.0)
        self.declare_parameter("camera_forward_offset_m", 0.0)
        self.declare_parameter("camera_lateral_offset_m", 0.0)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("imu_valid_topic", "/imu/valid")
        self.declare_parameter("use_imu_pitch_compensation", True)
        self.declare_parameter("require_fresh_imu", True)
        self.declare_parameter("imu_timeout_sec", 0.25)
        self.declare_parameter("imu_pitch_sign", 1.0)
        self.declare_parameter("imu_pitch_offset_deg", 0.0)
        self.declare_parameter("path_minimum_forward_m", 0.2)
        self.declare_parameter("path_maximum_forward_m", 10.0)

        weights = str(self.get_parameter("weights").value)
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.get_logger().info(f"Loading trained YOLO11n-seg weights: {weights}")
        self.model = YOLO(weights)
        self.get_logger().info(f"Model classes: {self.model.names}")

        self.debug_pub = self.create_publisher(Image, output_topic, 10)
        self.detections_pub = self.create_publisher(
            String, "/perception/detections_json", 10
        )
        self.path_pub = self.create_publisher(
            String, "/perception/local_path_json", 10
        )
        self.pixel_path_pub = self.create_publisher(
            String, "/perception/local_path_pixels_json", 10
        )
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.camera_intrinsics = None
        self.imu_pitch_deg = None
        self.imu_pitch_time = None
        self.imu_valid = False
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self.imu_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("imu_valid_topic").value),
            self.imu_valid_callback,
            10,
        )

        inference_fps = max(
            float(self.get_parameter("inference_fps").value), 1.0
        )
        self.inference_period = 1.0 / inference_fps
        self.last_inference_time = 0.0
        self.get_logger().info(f"Subscribing to RealSense image: {input_topic}")
        self.get_logger().info(f"Publishing YOLO image: {output_topic}")
        if float(self.get_parameter("camera_height_m").value) <= 0.0:
            self.get_logger().warning(
                "Metric projection locked until measured camera height and pitch are set"
            )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if len(msg.k) == 9 and msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.camera_intrinsics = (
                float(msg.k[0]), float(msg.k[4]),
                float(msg.k[2]), float(msg.k[5]),
            )

    def imu_callback(self, msg: Imu) -> None:
        try:
            pitch = quaternion_to_pitch_deg(
                msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w,
            )
        except ValueError:
            return
        self.imu_pitch_deg = (
            float(self.get_parameter("imu_pitch_sign").value) * pitch
            - float(self.get_parameter("imu_pitch_offset_deg").value)
        )
        self.imu_pitch_time = time.monotonic()

    def imu_valid_callback(self, msg: Bool) -> None:
        self.imu_valid = bool(msg.data)

    def image_callback(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_inference_time < self.inference_period:
            return
        self.last_inference_time = now

        try:
            frame = self.from_image_msg(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return

        result = self.model.predict(
            source=frame,
            conf=float(self.get_parameter("confidence").value),
            iou=float(self.get_parameter("iou").value),
            imgsz=int(self.get_parameter("image_size").value),
            device=self.get_parameter("device").value or None,
            verbose=False,
        )[0]

        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls.item())
                detections.append(
                    {
                        "class_id": class_id,
                        "class_name": result.names[class_id],
                        "confidence": round(float(box.conf.item()), 4),
                        "xyxy": [round(float(v), 1) for v in box.xyxy[0].tolist()],
                    }
                )

        road_mask, road_confidence = self.road_mask(result, frame.shape[:2])
        center_pixels = centerline_from_mask(
            road_mask,
            int(self.get_parameter("path_band_count").value),
            int(self.get_parameter("path_minimum_pixels").value),
            float(self.get_parameter("path_top_ratio").value),
        )
        self.pixel_path_pub.publish(String(data=json.dumps({
            "detected": len(center_pixels) >= 4,
            "coordinate_frame": "image",
            "axis_convention": "pixel_x_right,pixel_y_down",
            "image_width": frame.shape[1],
            "image_height": frame.shape[0],
            "points": [[x, y] for x, y in center_pixels],
        })))
        path = []
        height_m = float(self.get_parameter("camera_height_m").value)
        mount_pitch_deg = float(self.get_parameter("camera_pitch_down_deg").value)
        use_imu = bool(self.get_parameter("use_imu_pitch_compensation").value)
        imu_fresh = (
            self.imu_pitch_time is not None
            and time.monotonic() - self.imu_pitch_time
            <= float(self.get_parameter("imu_timeout_sec").value)
        )
        imu_required_ok = (
            not use_imu
            or not bool(self.get_parameter("require_fresh_imu").value)
            or (imu_fresh and self.imu_valid)
        )
        vehicle_pitch_deg = self.imu_pitch_deg if use_imu and imu_fresh else 0.0
        # REP-103 positive R_y pitches the vehicle nose downward.
        effective_pitch_deg = mount_pitch_deg + vehicle_pitch_deg
        projection_calibrated = (
            self.camera_intrinsics is not None
            and height_m > 0.0
            and 0.0 < effective_pitch_deg < 90.0
            and imu_required_ok
        )
        if projection_calibrated:
            path = pixels_to_ground_path(
                center_pixels, *self.camera_intrinsics,
                height_m, effective_pitch_deg,
                float(self.get_parameter("camera_forward_offset_m").value),
                float(self.get_parameter("camera_lateral_offset_m").value),
                float(self.get_parameter("path_minimum_forward_m").value),
                float(self.get_parameter("path_maximum_forward_m").value),
            )
        path_detected = len(path) >= 4
        self.path_pub.publish(String(data=json.dumps({
            "detected": path_detected,
            "confidence": road_confidence if path_detected else 0.0,
            "source": "yolo_segmentation_ground_projection",
            "projection_calibrated": projection_calibrated,
            "camera_mount_pitch_down_deg": mount_pitch_deg,
            "vehicle_pitch_deg": vehicle_pitch_deg,
            "effective_camera_pitch_down_deg": effective_pitch_deg,
            "imu_fresh": imu_fresh,
            "imu_valid": self.imu_valid,
            "coordinate_frame": "base_link",
            "axis_convention": "forward_m,lateral_right_m",
            "points": path if path_detected else [],
        })))

        detections_msg = String()
        detections_msg.data = json.dumps(
            {
                "stamp_sec": msg.header.stamp.sec,
                "stamp_nanosec": msg.header.stamp.nanosec,
                "detections": detections,
            },
            ensure_ascii=False,
        )
        self.detections_pub.publish(detections_msg)

        debug_frame = result.plot()
        for index in range(1, len(center_pixels)):
            cv2.line(debug_frame, center_pixels[index - 1], center_pixels[index], (255, 0, 255), 4)
        debug_msg = self.to_image_msg(debug_frame)
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

    def road_mask(self, result, image_shape):
        if result.masks is None or result.boxes is None:
            return None, 0.0
        aliases = {str(name).lower() for name in self.get_parameter("road_class_names").value}
        classes = result.boxes.cls.detach().cpu().numpy().astype(int)
        confidences = result.boxes.conf.detach().cpu().numpy()
        masks = result.masks.data.detach().cpu().numpy()
        selected = [index for index, class_id in enumerate(classes)
                    if str(result.names[class_id]).lower() in aliases]
        if not selected:
            return None, 0.0
        merged = np.max(masks[selected], axis=0)
        merged = cv2.resize(merged, (image_shape[1], image_shape[0]), interpolation=cv2.INTER_NEAREST)
        return (merged >= 0.5).astype(np.uint8), float(np.max(confidences[selected]))

    @staticmethod
    def from_image_msg(msg: Image):
        """Convert rgb8/bgr8 sensor_msgs/Image to an OpenCV BGR frame."""
        if msg.encoding not in ("rgb8", "bgr8"):
            raise ValueError(
                f"Unsupported camera encoding '{msg.encoding}'; "
                "expected rgb8 or bgr8"
            )
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        frame = row[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(frame)

    @staticmethod
    def to_image_msg(frame) -> Image:
        """Convert a contiguous BGR8 OpenCV frame to sensor_msgs/Image."""
        if not frame.flags["C_CONTIGUOUS"]:
            frame = frame.copy()
        msg = Image()
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = frame.shape[1] * frame.shape[2]
        msg.data = frame.tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = YoloCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"[yolo_camera] {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
