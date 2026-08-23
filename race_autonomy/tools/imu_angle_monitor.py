#!/usr/bin/env python3
"""Continuously display calibrated IMU pitch against a target angle."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class ImuAngleMonitor(Node):
    def __init__(self, target_deg, tolerance_deg, timeout_sec):
        super().__init__("imu_angle_monitor")
        self.target = abs(float(target_deg))
        self.tolerance = abs(float(tolerance_deg))
        self.timeout = abs(float(timeout_sec))
        self.pitch = None
        self.valid = False
        self.pitch_time = None
        self.valid_time = None
        self.create_subscription(Float32, "/imu/pitch_deg", self.on_pitch, 10)
        self.create_subscription(Bool, "/imu/valid", self.on_valid, 10)
        self.create_timer(0.05, self.display)

    def on_pitch(self, msg):
        if math.isfinite(msg.data):
            self.pitch = float(msg.data)
            self.pitch_time = time.monotonic()

    def on_valid(self, msg):
        self.valid = bool(msg.data)
        self.valid_time = time.monotonic()

    def display(self):
        now = time.monotonic()
        fresh = (
            self.pitch is not None
            and self.pitch_time is not None
            and self.valid_time is not None
            
            and now - self.pitch_time <= self.timeout
            and now - self.valid_time <= self.timeout
        )
        if not fresh:
            text = "IMU WAITING: /imu/pitch_deg 또는 /imu/valid 수신 없음"
        elif not self.valid:
        
        
            text = "IMU INVALID: calibration 및 센서 연결 확인"
        else:
            magnitude = abs(self.pitch)
            error = magnitude - self.target
            if abs(error) <= self.tolerance:
                state = "TARGET OK"
            elif error < 0.0:
                state = f"{abs(error):.2f}° 더 올리기"
            else:
                state = f"{abs(error):.2f}° 내리기"
            direction = "UP(+Pitch)" if self.pitch >= 0.0 else "DOWN(-Pitch)"
            text = (
                f"Pitch={self.pitch:+7.3f}° | abs={magnitude:6.3f}° | "
                f"target={self.target:.2f}° | error={error:+6.3f}° | "
                f"{direction} | {state}"
            )
        print(f"\r{text:<120}", end="", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-deg", type=float, default=15.0)
    parser.add_argument("--tolerance-deg", type=float, default=0.2)
    parser.add_argument("--timeout-sec", type=float, default=0.5)
    args = parser.parse_args()
    rclpy.init()
    node = ImuAngleMonitor(args.target_deg, args.tolerance_deg, args.timeout_sec)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
