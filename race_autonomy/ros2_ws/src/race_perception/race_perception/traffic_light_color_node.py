#!/usr/bin/env python3
"""Confirm traffic-light state directly from labeled YOLO detections."""

import json
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int8, String

from .traffic_light_color import (
    best_labeled_light,
    finish_signal_from_bgr,
    update_light_confirmation,
)


class TrafficLightColorNode(Node):
    """Convert R/Y/G YOLO labels into a temporally confirmed signal state."""

    def __init__(self):
        super().__init__("traffic_light_color")
        defaults = {
            "detections_topic": "/perception/detections_json",
            "state_topic": "/perception/traffic_light_state",
            "result_topic": "/perception/traffic_light_state_json",
            "candidate_class_names": [
                "R_light", "Y_light", "G_light", "etc_light"],
            "minimum_yolo_confidence": 0.25,
            "stop_line_distance_topic": "/perception/stop_line_distance_m",
            "stop_line_distance_valid_topic": (
                "/perception/stop_line_distance_valid"),
            "activation_distance_m": 3.0,
            "distance_timeout_sec": 0.35,
            "confirmation_sec": 3.0,
            "final_image_topic":"/camera/image_raw",
            "final_state_topic":"/perception/final_signal_state",
            "final_process_hz":10.0,
            "final_confirmation_sec":0.3,
            "final_roi":[0.2,0.05,0.8,0.70],
            "final_minimum_blob_area":40,
            "final_dominance_ratio":1.35,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.stop_line_distance_m = None
        self.stop_line_distance_valid = False
        self.stop_line_distance_time = None
        self.signal_tracker = None
        self.active_section=1;self.final_last_process=0.0
        self.final_candidate="UNKNOWN";self.final_candidate_start=None
        self.state_pub = self.create_publisher(
            String, self.param("state_topic"), 10)
        self.result_pub = self.create_publisher(
            String, self.param("result_topic"), 10)
        self.final_state_pub=self.create_publisher(
            String,self.param("final_state_topic"),10)
        self.create_subscription(
            String, self.param("detections_topic"), self.on_detections, 10)
        self.create_subscription(
            Float32, self.param("stop_line_distance_topic"),
            self.on_stop_line_distance, 10)
        self.create_subscription(
            Bool, self.param("stop_line_distance_valid_topic"),
            self.on_stop_line_distance_valid, 10)
        self.create_subscription(
            Int8,"/mission/active_section",
            lambda msg:setattr(self,"active_section",int(msg.data)),10)
        self.create_subscription(
            Image,self.param("final_image_topic"),self.on_final_image,
            qos_profile_sensor_data)

    def param(self, name):
        return self.get_parameter(name).value

    def on_stop_line_distance(self, msg):
        value = float(msg.data)
        if math.isfinite(value) and value >= 0.0:
            self.stop_line_distance_m = value
            self.stop_line_distance_time = time.monotonic()

    def on_stop_line_distance_valid(self, msg):
        self.stop_line_distance_valid = bool(msg.data)
        if not self.stop_line_distance_valid:
            self.signal_tracker = None

    def confirmation_zone_active(self):
        distance_fresh = (
            self.stop_line_distance_valid and
            self.stop_line_distance_m is not None and
            self.stop_line_distance_time is not None and
            time.monotonic() - self.stop_line_distance_time <=
            float(self.param("distance_timeout_sec")))
        in_zone = (
            distance_fresh and self.stop_line_distance_m <=
            float(self.param("activation_distance_m")))
        return distance_fresh, in_zone

    def on_detections(self, msg):
        try:
            payload = json.loads(msg.data)
            detections = payload.get("detections", [])
            if not isinstance(detections, list):
                raise ValueError("detections must be a list")
        except (ValueError, TypeError, json.JSONDecodeError):
            detections = []

        best, candidate_count = best_labeled_light(
            detections, self.param("candidate_class_names"),
            self.param("minimum_yolo_confidence"))
        distance_fresh, in_zone = self.confirmation_zone_active()
        confirmed_state, self.signal_tracker = update_light_confirmation(
            best["state"], in_zone, self.signal_tracker, time.monotonic(),
            self.param("confirmation_sec"))
        candidate = (self.signal_tracker or {}).get("candidate")
        accumulated_sec = float(
            (self.signal_tracker or {}).get("accumulated_sec", 0.0))
        self.state_pub.publish(String(data=confirmed_state))
        self.result_pub.publish(String(data=json.dumps({
            **best,
            "raw_state": best["state"],
            "state": confirmed_state,
            "candidate_count": candidate_count,
            "stop_line_distance_m": self.stop_line_distance_m,
            "stop_line_distance_fresh": distance_fresh,
            "in_confirmation_zone": in_zone,
            "confirmation_candidate": candidate,
            "confirmation_elapsed_sec": accumulated_sec,
            "confirmation_required_sec": float(self.param("confirmation_sec")),
        })))

    def on_final_image(self,msg):
        if self.active_section != 11:
            self.final_candidate="UNKNOWN";self.final_candidate_start=None
            return
        now=time.monotonic();period=1.0/max(1.0,float(self.param("final_process_hz")))
        if now-self.final_last_process < period:return
        self.final_last_process=now
        if msg.encoding not in ("bgr8","rgb8") or msg.step < msg.width*3:
            self.final_state_pub.publish(String(data="UNKNOWN"));return
        rows=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)
        frame=rows[:,:msg.width*3].reshape(msg.height,msg.width,3)
        if msg.encoding=="rgb8":frame=frame[:,:,::-1]
        state,_=finish_signal_from_bgr(
            frame,tuple(self.param("final_roi")),
            self.param("final_minimum_blob_area"),
            self.param("final_dominance_ratio"))
        if state=="UNKNOWN":
            self.final_candidate="UNKNOWN";self.final_candidate_start=None
            confirmed="UNKNOWN"
        elif state != self.final_candidate:
            self.final_candidate=state;self.final_candidate_start=now
            confirmed="UNKNOWN"
        elif (self.final_candidate_start is not None and
              now-self.final_candidate_start >=
              float(self.param("final_confirmation_sec"))):
            confirmed=state
        else:confirmed="UNKNOWN"
        self.final_state_pub.publish(String(data=confirmed))


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
