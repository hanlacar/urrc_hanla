#!/usr/bin/env python3
"""Select one fresh vehicle-command source and publish the MCU command pair."""

import math
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String


class SourceState:
    def __init__(self):
        self.drive = 0.0
        self.wheel = 0
        self.drive_time = None
        self.wheel_time = None


class CmdMuxNode(Node):
    def __init__(self):
        super().__init__("cmd_mux_node")
        defaults = {
            "sources": ["camera", "gps", "lidar"],
            "mode_map": (
                "NORMAL:camera,SLOPE:camera,T_PARK:lidar,"
                "PARALLEL_PARK:lidar,GPS_TRACK:gps,IDLE:none"
            ),
            "mode_topic": "/vehicle_mode",
            "estop_topic": "/estop_lock",
            "camera_drive_topic": "/camera_drive",
            "camera_wheel_topic": "/camera_wheel",
            "output_drive_topic": "/mcu_drive",
            "output_wheel_topic": "/mcu_wheel",
            "publish_hz": 20.0,
            "source_timeout_sec": 0.3,
            "manual_enable": True,
            "manual_timeout_sec": 0.3,
            "maximum_abs_stage": 3,
            "maximum_steering_deg": 27.0,
            "stop_steering_deg": 0.0,
            "initial_mode": "IDLE",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.source_names = list(self.param("sources"))
        if bool(self.param("manual_enable")):
            self.source_names.append("manual")
        self.sources = {name: SourceState() for name in self.source_names}
        self.mode_map = self.parse_mode_map(str(self.param("mode_map")))
        self.mode = str(self.param("initial_mode")).strip().upper()
        self.estop = False
        self.active = None

        for name in self.source_names:
            drive_topic = (str(self.param("camera_drive_topic")) if name == "camera"
                           else f"/{name}/mcu_drive")
            wheel_topic = (str(self.param("camera_wheel_topic")) if name == "camera"
                           else f"/{name}/mcu_wheel")
            self.create_subscription(
                Float32, drive_topic,
                lambda msg, source=name: self.on_drive(source, msg), 10)
            self.create_subscription(
                Int32, wheel_topic,
                lambda msg, source=name: self.on_wheel(source, msg), 10)

        self.create_subscription(
            String, str(self.param("mode_topic")), self.on_mode, 10)
        self.create_subscription(
            Bool, str(self.param("estop_topic")), self.on_estop, 10)
        self.drive_pub = self.create_publisher(
            Float32, str(self.param("output_drive_topic")), 10)
        self.wheel_pub = self.create_publisher(
            Int32, str(self.param("output_wheel_topic")), 10)
        self.active_pub = self.create_publisher(String, "/cmd_mux/active", 10)
        self.status_pub = self.create_publisher(String, "/cmd_mux/status", 10)
        rate = max(1.0, float(self.param("publish_hz")))
        self.publish_timer = self.create_timer(1.0 / rate, self.tick)

    def param(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def parse_mode_map(value):
        result = {}
        for pair in value.split(","):
            if ":" not in pair:
                continue
            mode, source = pair.split(":", 1)
            result[mode.strip().upper()] = source.strip()
        return result

    def on_drive(self, source, msg):
        state = self.sources[source]
        value = float(msg.data)
        if not math.isfinite(value):
            return
        state.drive = value
        state.drive_time = time.monotonic()

    def on_wheel(self, source, msg):
        state = self.sources[source]
        state.wheel = int(msg.data)
        state.wheel_time = time.monotonic()

    def on_mode(self, msg):
        self.mode = msg.data.strip().upper()

    def on_estop(self, msg):
        self.estop = bool(msg.data)

    def fresh(self, source, timeout):
        state = self.sources.get(source)
        if state is None or state.drive_time is None or state.wheel_time is None:
            return False
        now = time.monotonic()
        return (
            now - state.drive_time <= timeout
            and now - state.wheel_time <= timeout
        )

    def select(self):
        if self.estop:
            return None, "ESTOP"
        if bool(self.param("manual_enable")) and self.fresh(
                "manual", float(self.param("manual_timeout_sec"))):
            return "manual", "MANUAL"
        source = self.mode_map.get(self.mode, "none")
        if source in ("", "none") or source not in self.sources:
            return None, f"MODE_STOP:{self.mode}"
        if not self.fresh(source, float(self.param("source_timeout_sec"))):
            return None, f"STALE:{source}"
        return source, "ACTIVE"

    def tick(self):
        if not rclpy.ok():
            return
        source, reason = self.select()
        if source is None:
            drive = 0.0
            wheel = int(round(float(self.param("stop_steering_deg"))))
        else:
            state = self.sources[source]
            limit = max(0, int(self.param("maximum_abs_stage")))
            steer_limit = max(0.0, float(self.param("maximum_steering_deg")))
            drive = max(-float(limit), min(float(limit), state.drive))
            wheel = int(round(max(-steer_limit, min(steer_limit, state.wheel))))
        try:
            self.drive_pub.publish(Float32(data=float(drive)))
            self.wheel_pub.publish(Int32(data=int(wheel)))
            self.active_pub.publish(String(data=source or "stop"))
            self.status_pub.publish(String(data=f"{reason};mode={self.mode}"))
        except RCLError:
            # SIGINT may invalidate the context while this timer callback is
            # already executing.  Shutdown must remain clean and idempotent.
            return
        if source != self.active:
            self.get_logger().info(
                f"command source: {self.active or 'stop'} -> {source or 'stop'} "
                f"({reason}, mode={self.mode})")
            self.active = source

    def destroy_node(self):
        if hasattr(self, "publish_timer"):
            self.publish_timer.cancel()
        if rclpy.ok():
            try:
                self.drive_pub.publish(Float32(data=0.0))
                self.wheel_pub.publish(Int32(data=int(round(float(self.param("stop_steering_deg"))))))
            except RCLError:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdMuxNode()
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
