#!/usr/bin/env python3

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .traffic_light_color import classify_traffic_light_bgr, clipped_box, fuse_traffic_light_state


class TrafficLightColorNode(Node):
    """Use OpenCV HSV inside YOLO traffic-light boxes to classify color."""

    def __init__(self):
        super().__init__("traffic_light_color")
        defaults = {
            "input_image_topic": "/camera/image_raw",
            "detections_topic": "/perception/detections_json",
            "state_topic": "/perception/traffic_light_state",
            "result_topic": "/perception/traffic_light_state_json",
            "debug_image_topic": "/perception/traffic_light_image",
            "candidate_class_names": ["R_liht", "Y_light", "G_light", "etc_light"],
            "detection_timeout_sec": 0.3,
            "minimum_yolo_confidence": 0.25,
            "saturation_min": 90,
            "value_min": 100,
            "minimum_color_pixels": 20,
            "dominance_ratio": 1.35,
            "box_padding_ratio": 0.05,
            "publish_debug_image": True,
            "allow_yolo_class_fallback": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.detections = []
        self.detection_time = None
        self.state_pub = self.create_publisher(String, self.param("state_topic"), 10)
        self.result_pub = self.create_publisher(String, self.param("result_topic"), 10)
        self.debug_pub = self.create_publisher(Image, self.param("debug_image_topic"), 10)
        self.create_subscription(String, self.param("detections_topic"), self.on_detections, 10)
        self.create_subscription(Image, self.param("input_image_topic"), self.on_image, qos_profile_sensor_data)

    def param(self, name):
        return self.get_parameter(name).value

    def on_detections(self, msg):
        try:
            data = json.loads(msg.data)
            detections = data.get("detections", [])
            if not isinstance(detections, list):
                raise ValueError("detections must be a list")
            self.detections = detections
            self.detection_time = time.monotonic()
        except (ValueError, TypeError, json.JSONDecodeError):
            self.detections = []
            self.detection_time = None

    def on_image(self, msg):
        frame = self.from_image_msg(msg)
        fresh = self.detection_time is not None and (
            time.monotonic() - self.detection_time <= float(self.param("detection_timeout_sec"))
        )
        candidates = []
        accepted = set(str(name) for name in self.param("candidate_class_names"))
        if fresh:
            candidates = [
                item for item in self.detections
                if item.get("class_name") in accepted
                and float(item.get("confidence", 0.0)) >= float(self.param("minimum_yolo_confidence"))
            ]

        best = {"state": "UNKNOWN", "confidence": 0.0, "box": None, "counts": {}, "source": "unknown"}
        debug = frame.copy()
        for detection in candidates:
            box = clipped_box(
                detection.get("xyxy"), frame.shape[1], frame.shape[0],
                float(self.param("box_padding_ratio")),
            )
            if box is None:
                continue
            x1, y1, x2, y2 = box
            hsv_state, color_confidence, counts = classify_traffic_light_bgr(
                frame[y1:y2, x1:x2],
                int(self.param("saturation_min")), int(self.param("value_min")),
                int(self.param("minimum_color_pixels")), float(self.param("dominance_ratio")),
            )
            state,score,source=fuse_traffic_light_state(
                detection.get("class_name"),float(detection.get("confidence",0.0)),
                hsv_state,color_confidence,bool(self.param("allow_yolo_class_fallback")))
            if score > best["confidence"]:
                best = {"state": state, "confidence": score, "box": list(box), "counts": counts, "source": source}
            color = {"RED": (0, 0, 255), "YELLOW": (0, 255, 255), "GREEN": (0, 255, 0)}.get(state, (128, 128, 128))
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
            cv2.putText(debug, f"{state} [{source}]", (x1, max(20, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        self.state_pub.publish(String(data=best["state"]))
        self.result_pub.publish(String(data=json.dumps({
            **best, "detection_fresh": fresh, "candidate_count": len(candidates),
        })))
        if bool(self.param("publish_debug_image")):
            output = self.to_image_msg(debug)
            output.header = msg.header
            self.debug_pub.publish(output)

    @staticmethod
    def from_image_msg(msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")
        row = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        frame = row[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else frame.copy()

    @staticmethod
    def to_image_msg(frame):
        output = Image()
        output.height, output.width = frame.shape[:2]
        output.encoding = "bgr8"
        output.is_bigendian = False
        output.step = output.width * 3
        output.data = np.ascontiguousarray(frame).tobytes()
        return output


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightColorNode()
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
