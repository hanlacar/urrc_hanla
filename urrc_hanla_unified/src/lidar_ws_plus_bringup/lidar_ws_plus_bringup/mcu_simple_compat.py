#!/usr/bin/env python3
"""Explicit, fail-closed bridge from an arbitrated command to SIMPLE MCU.

Legacy LiDAR topics remain the defaults for standalone lifted-bench use.  An
integrated launch can select the final arbiter topics explicitly.  Fixed
odometry and fake mode 5 remain independent bench-only options.
"""

import math
import signal
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32, Int8, Int32, String
from tf2_ros import StaticTransformBroadcaster


VALID_DRIVE_LEVELS = frozenset({-1, 0, 1, 2, 3})


def make_bench_static_transform(stamp):
    """Return the single fixed odom-to-base transform for lifted bench use."""
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'base_link'
    transform.transform.rotation.w = 1.0
    return transform


def make_bench_odometry(stamp):
    """Return zero-pose, zero-twist odometry with a current message stamp."""
    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.orientation.w = 1.0
    return odom


def translate_drive(value, max_forward_level=1):
    value = float(value)
    if not math.isfinite(value):
        return None
    rounded = int(round(value))
    if rounded not in VALID_DRIVE_LEVELS:
        return None
    if rounded > int(max_forward_level):
        return None
    return float(rounded)


def translate_speed_mps(value, stage_per_mps, max_forward_level=1):
    """Convert a finite physical-speed command to a supported MCU level."""
    value = float(value)
    stage_per_mps = float(stage_per_mps)
    if not math.isfinite(value) or not math.isfinite(stage_per_mps):
        return None
    if stage_per_mps <= 0.0:
        return None
    return translate_drive(
        round(value*stage_per_mps), max_forward_level=max_forward_level)


def translate_wheel(value, sign_multiplier=-1, limit_deg=22):
    multiplier = int(sign_multiplier)
    if multiplier not in (-1, 1):
        raise ValueError('wheel_sign_multiplier must be -1 or 1')
    value = float(value)
    if not math.isfinite(value):
        return None
    wheel = multiplier*int(value)
    return wheel if abs(wheel) <= int(limit_deg) else None


class McuSimpleCompat(Node):

    def __init__(self):
        super().__init__('mcu_simple_compat')
        defaults = {
            'input_drive_topic': '/lidar_drive',
            'input_wheel_topic': '/lidar_wheel',
            'input_stop_topic': '/lidar_stop',
            'drive_input_unit': 'level',
            'wheel_input_type': 'int32',
            'stage_per_mps': 4.3956043956,
            'wheel_sign_multiplier': -1,
            'wheel_limit_deg': 22,
            'max_forward_drive_level': 1,
            'command_timeout_sec': 0.50,
            'publish_mode_5': True,
            'mode_source_topic': '',
            'bench_fake_odom': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if int(self.p['wheel_sign_multiplier']) not in (-1, 1):
            raise ValueError('wheel_sign_multiplier must be -1 or 1')
        if not 0 < int(self.p['wheel_limit_deg']) <= 22:
            raise ValueError('wheel_limit_deg must be in (0, 22]')
        if not 0 <= int(self.p['max_forward_drive_level']) <= 3:
            raise ValueError('max_forward_drive_level must be in [0, 3]')
        if (bool(self.p['bench_fake_odom']) and
                int(self.p['max_forward_drive_level']) > 1):
            raise ValueError(
                'lifted bench max_forward_drive_level must be 0 or 1')
        if float(self.p['command_timeout_sec']) <= 0.0:
            raise ValueError('command_timeout_sec must be positive')
        if str(self.p['drive_input_unit']) not in ('level', 'mps'):
            raise ValueError('drive_input_unit must be level or mps')
        if str(self.p['wheel_input_type']) not in ('int32', 'float32'):
            raise ValueError('wheel_input_type must be int32 or float32')
        if (str(self.p['drive_input_unit']) == 'mps' and
                float(self.p['stage_per_mps']) <= 0.0):
            raise ValueError('stage_per_mps must be positive for mps input')

        self.last_lidar_msg = time.monotonic()
        self.pub_drive = self.create_publisher(Float32, '/mcu/cmd_drive', 10)
        self.pub_wheel = self.create_publisher(Int32, '/mcu/cmd_wheel', 10)
        self.pub_stop = self.create_publisher(Bool, '/mcu/cmd_stop', 10)
        self.pub_mode = None
        if (bool(self.p['publish_mode_5']) or
                str(self.p['mode_source_topic']).strip()):
            self.pub_mode = self.create_publisher(
                String, '/mcu/current_mode', 10)
        self._configure_bench_fake_odom()

        self.create_subscription(
            Float32, str(self.p['input_drive_topic']), self.drive_cb, 10)
        wheel_type = (Float32 if str(self.p['wheel_input_type']) == 'float32'
                      else Int32)
        self.create_subscription(
            wheel_type, str(self.p['input_wheel_topic']), self.wheel_cb, 10)
        self.create_subscription(
            Bool, str(self.p['input_stop_topic']), self.stop_cb, 10)
        if str(self.p['mode_source_topic']).strip():
            self.create_subscription(
                Int8, str(self.p['mode_source_topic']), self.mode_cb, 10)
        self.create_timer(0.05, self.timer_cb)
        self.pub_wheel.publish(Int32(data=0))
        self.pub_drive.publish(Float32(data=0.0))
        self.pub_stop.publish(Bool(data=True))
        self.get_logger().warning(
            'SIMPLE MCU compatibility bridge active; '
            f'inputs={self.p["input_drive_topic"]},'
            f'{self.p["input_wheel_topic"]},{self.p["input_stop_topic"]}; '
            f'wheel_sign_multiplier={self.p["wheel_sign_multiplier"]}, '
            f'bench_fake_odom={bool(self.p["bench_fake_odom"])}')

    def _configure_bench_fake_odom(self):
        self.pub_odom = None
        self.static_tf_br = None
        if not bool(self.p['bench_fake_odom']):
            return
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.static_tf_br = StaticTransformBroadcaster(self)
        transform = make_bench_static_transform(
            self.get_clock().now().to_msg())
        self.static_tf_br.sendTransform(transform)

    def _publish_bench_odometry(self):
        self.pub_odom.publish(make_bench_odometry(
            self.get_clock().now().to_msg()))

    def _fail_closed(self, reason):
        self.pub_wheel.publish(Int32(data=0))
        self.pub_drive.publish(Float32(data=0.0))
        self.pub_stop.publish(Bool(data=True))
        self.get_logger().error(reason)

    def drive_cb(self, msg):
        self.last_lidar_msg = time.monotonic()
        if str(self.p['drive_input_unit']) == 'mps':
            value = translate_speed_mps(
                msg.data, float(self.p['stage_per_mps']),
                int(self.p['max_forward_drive_level']))
        else:
            value = translate_drive(
                msg.data, int(self.p['max_forward_drive_level']))
        if value is None:
            self._fail_closed(f'invalid/unsafe drive level {msg.data!r}')
            return
        self.pub_drive.publish(Float32(data=value))

    def wheel_cb(self, msg):
        self.last_lidar_msg = time.monotonic()
        value = translate_wheel(
            msg.data, int(self.p['wheel_sign_multiplier']),
            int(self.p['wheel_limit_deg']))
        if value is None:
            self._fail_closed(f'wheel exceeds SIMPLE limit: {msg.data!r}')
            return
        self.pub_wheel.publish(Int32(data=value))

    def stop_cb(self, msg):
        self.last_lidar_msg = time.monotonic()
        self.pub_stop.publish(Bool(data=bool(msg.data)))

    def mode_cb(self, msg):
        if self.pub_mode is not None and not bool(self.p['publish_mode_5']):
            self.pub_mode.publish(String(data=str(int(msg.data))))

    def timer_cb(self):
        if bool(self.p['publish_mode_5']) and self.pub_mode is not None:
            self.pub_mode.publish(String(data='5'))
        if (time.monotonic()-self.last_lidar_msg >
                float(self.p['command_timeout_sec'])):
            self.pub_stop.publish(Bool(data=True))
            self.pub_drive.publish(Float32(data=0.0))
        if self.pub_odom is not None:
            self._publish_bench_odometry()

    def publish_shutdown_stop(self):
        for _ in range(3):
            self.pub_wheel.publish(Int32(data=0))
            self.pub_drive.publish(Float32(data=0.0))
            self.pub_stop.publish(Bool(data=True))
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = McuSimpleCompat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    finally:
        if rclpy.ok():
            node.publish_shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
