#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, Int32, Int64, String, UInt32

try:
    import serial
except ImportError:
    serial = None

from .encoder_protocol import parse_encoder_frame


class EncoderSerialBridgeNode(Node):
    """Read signed quadrature data from a dedicated Arduino Uno."""

    def __init__(self):
        super().__init__("encoder_serial_bridge_node")
        defaults = {
            "port": "/dev/ttyACM1",
            "baudrate": 115200,
            "reconnect_interval_sec": 1.0,
            "feedback_timeout_sec": 0.2,
            "direction_sign": 1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.serial_port = None
        self.receive_buffer = bytearray()
        self.last_open_attempt = 0.0
        self.last_frame_time = None
        self.last_sensor_valid = False
        self.last_error = ""

        self.count_pub = self.create_publisher(
            Int64, "/wheel_encoder/count", 10
        )
        self.delta_pub = self.create_publisher(
            Int32, "/wheel_encoder/delta", 10
        )
        self.direction_pub = self.create_publisher(
            Int8, "/wheel_encoder/direction", 10
        )
        self.edge_age_pub = self.create_publisher(
            UInt32, "/wheel_encoder/edge_age_ms", 10
        )
        self.invalid_pub = self.create_publisher(
            UInt32, "/wheel_encoder/invalid_transitions", 10
        )
        self.connected_pub = self.create_publisher(
            Bool, "/wheel_encoder/connected", 10
        )
        self.valid_pub = self.create_publisher(
            Bool, "/wheel_encoder/valid", 10
        )
        self.status_pub = self.create_publisher(
            String, "/wheel_encoder/status", 10
        )

        self.create_timer(0.005, self.poll_serial)
        self.create_timer(0.05, self.publish_health)

    def param(self, name):
        return self.get_parameter(name).value

    def report_error(self, text):
        if text != self.last_error:
            self.get_logger().error(text)
            self.last_error = text
        self.status_pub.publish(String(data=text))

    def open_serial(self):
        if serial is None:
            self.report_error("pyserial_not_installed")
            return
        now = time.monotonic()
        if now - self.last_open_attempt < float(
            self.param("reconnect_interval_sec")
        ):
            return
        self.last_open_attempt = now
        try:
            self.serial_port = serial.Serial(
                port=str(self.param("port")),
                baudrate=int(self.param("baudrate")),
                timeout=0,
                write_timeout=0,
            )
            self.receive_buffer.clear()
            self.last_error = ""
            self.get_logger().info(
                f"Opened quadrature encoder port {self.serial_port.port}"
            )
        except (OSError, serial.SerialException) as exc:
            self.serial_port = None
            self.report_error(f"serial_open_failed:{exc}")

    def close_serial(self):
        port, self.serial_port = self.serial_port, None
        if port is not None:
            try:
                port.close()
            except (OSError, serial.SerialException):
                pass

    def poll_serial(self):
        if self.serial_port is None:
            self.open_serial()
            return
        try:
            waiting = self.serial_port.in_waiting
            if waiting:
                self.receive_buffer.extend(self.serial_port.read(waiting))
            while b"\n" in self.receive_buffer:
                raw, _, remainder = self.receive_buffer.partition(b"\n")
                self.receive_buffer = bytearray(remainder)
                self.handle_line(raw.rstrip(b"\r"))
            if len(self.receive_buffer) > 1024:
                self.receive_buffer.clear()
                self.report_error("serial_line_too_long")
        except (OSError, serial.SerialException) as exc:
            self.report_error(f"serial_read_failed:{exc}")
            self.close_serial()

    def handle_line(self, raw):
        frame = parse_encoder_frame(raw)
        if frame is None:
            return
        count, delta, direction, edge_age_ms, invalid, signal_valid = frame
        sign = -1 if int(self.param("direction_sign")) < 0 else 1
        count *= sign
        delta *= sign
        direction *= sign
        self.count_pub.publish(Int64(data=count))
        self.delta_pub.publish(Int32(data=delta))
        self.direction_pub.publish(Int8(data=direction))
        self.edge_age_pub.publish(UInt32(data=edge_age_ms))
        self.invalid_pub.publish(UInt32(data=invalid))
        self.last_frame_time = time.monotonic()
        self.last_sensor_valid = signal_valid

    def publish_health(self):
        connected = self.serial_port is not None and self.serial_port.is_open
        fresh = (
            connected
            and self.last_frame_time is not None
            and time.monotonic() - self.last_frame_time
            <= float(self.param("feedback_timeout_sec"))
        )
        self.connected_pub.publish(Bool(data=connected))
        valid = fresh and self.last_sensor_valid
        self.valid_pub.publish(Bool(data=valid))
        self.status_pub.publish(
            String(data="valid" if valid else "waiting_for_encoder_motion_test")
        )

    def destroy_node(self):
        self.close_serial()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderSerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
