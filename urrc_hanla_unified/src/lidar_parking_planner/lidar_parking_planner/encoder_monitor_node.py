#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


DRIVE_TOPIC = '/drive'
ENCODER_COUNT_TOPIC = '/parking/encoder/count'
ENCODER_DELTA_COUNT_TOPIC = '/parking/encoder/delta_count'
ENCODER_COUNT_RATE_TOPIC = '/parking/encoder/count_rate'
DEFAULT_MIN_DT_SEC = 0.001


@dataclass(frozen=True)
class EncoderUpdate:
    """One raw encoder observation and any values derived from it."""

    encoder_count: int
    delta_count: Optional[int]
    dt_sec: Optional[float]
    count_rate: Optional[float]


class EncoderDeltaCalculator:
    """Track raw cumulative counts without assigning distance or direction."""

    def __init__(self, min_dt_sec: float = DEFAULT_MIN_DT_SEC):
        if not math.isfinite(min_dt_sec) or min_dt_sec <= 0.0:
            raise ValueError('min_dt_sec must be finite and greater than zero')
        self.min_dt_sec = min_dt_sec
        self.prev_count: Optional[int] = None
        self.prev_time_sec: Optional[float] = None

    def update(self, current_count: int, current_time_sec: float) -> EncoderUpdate:
        if self.prev_count is None or self.prev_time_sec is None:
            self.prev_count = current_count
            self.prev_time_sec = current_time_sec
            return EncoderUpdate(current_count, None, None, None)

        delta_count = current_count - self.prev_count
        dt_sec = current_time_sec - self.prev_time_sec
        self.prev_count = current_count
        self.prev_time_sec = current_time_sec

        count_rate = None
        if math.isfinite(dt_sec) and dt_sec >= self.min_dt_sec:
            count_rate = delta_count / dt_sec

        return EncoderUpdate(current_count, delta_count, dt_sec, count_rate)


class EncoderMonitor(Node):
    """Expose diagnostics for the cumulative Int32 count received on /drive."""

    def __init__(self):
        super().__init__('encoder_monitor')
        self.declare_parameter('min_dt_sec', DEFAULT_MIN_DT_SEC)
        min_dt_sec = float(self.get_parameter('min_dt_sec').value)
        self.calculator = EncoderDeltaCalculator(min_dt_sec)

        self.count_pub = self.create_publisher(
            Int32, ENCODER_COUNT_TOPIC, 10)
        self.delta_count_pub = self.create_publisher(
            Int32, ENCODER_DELTA_COUNT_TOPIC, 10)
        self.count_rate_pub = self.create_publisher(
            Float32, ENCODER_COUNT_RATE_TOPIC, 10)
        self.drive_sub = self.create_subscription(
            Int32,
            '/drive',
            self.encoder_callback,
            10
        )

        self.get_logger().info(
            f'Encoder monitor started: input={DRIVE_TOPIC} '
            f'(std_msgs/msg/Int32), min_dt_sec={min_dt_sec:.6f}')

    def encoder_callback(self, msg: Int32) -> None:
        current_time_sec = self.get_clock().now().nanoseconds * 1e-9
        update = self.calculator.update(msg.data, current_time_sec)

        self.count_pub.publish(Int32(data=update.encoder_count))

        if update.delta_count is None:
            self.get_logger().info(
                f'[ENCODER] count={update.encoder_count} '
                'delta=N/A dt=N/A rate=N/A count/s')
            return

        self.delta_count_pub.publish(Int32(data=update.delta_count))

        if update.count_rate is None:
            self.get_logger().warning(
                f'[ENCODER] count={update.encoder_count} '
                f'delta={update.delta_count} dt={update.dt_sec:.6f}s '
                'rate=N/A count/s (dt below min_dt_sec or invalid)')
            return

        self.count_rate_pub.publish(Float32(data=update.count_rate))
        self.get_logger().info(
            f'[ENCODER] count={update.encoder_count} '
            f'delta={update.delta_count} dt={update.dt_sec:.3f}s '
            f'rate={update.count_rate:.1f} count/s')


def main(args=None):
    rclpy.init(args=args)
    node = EncoderMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
