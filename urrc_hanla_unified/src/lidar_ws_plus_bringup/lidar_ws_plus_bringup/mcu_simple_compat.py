#!/usr/bin/env python3
"""Explicit, fail-closed bridge from an arbitrated command to SIMPLE MCU.

Legacy LiDAR topics remain the defaults for standalone lifted-bench use.  An
integrated launch can select the final arbiter topics explicitly.  Fixed
odometry and fake mode 5 remain independent bench-only options.
"""

import json
import math
import signal
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32, Int8, Int32, String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


VALID_DRIVE_LEVELS = frozenset({-1, 0, 1, 2, 3})


class SteeringFeedbackGate:
    """Validate raw steering ADC without inventing an angle estimate."""

    def __init__(self, center_adc=496, counts_per_deg=18.0,
                 max_steer_deg=22.0, margin_adc=10,
                 valid_samples=3, timeout_sec=0.5):
        if not 0 <= int(center_adc) <= 1023:
            raise ValueError('steering_feedback_center_adc must be in [0, 1023]')
        if float(counts_per_deg) <= 0.0 or float(max_steer_deg) <= 0.0:
            raise ValueError('steering feedback scale and limit must be positive')
        if int(margin_adc) < 0 or int(valid_samples) <= 0:
            raise ValueError('steering feedback margin/samples are invalid')
        if float(timeout_sec) <= 0.0:
            raise ValueError('steering_feedback_timeout_sec must be positive')
        span = float(counts_per_deg) * float(max_steer_deg)
        self.minimum_adc = max(0, int(math.floor(center_adc-span-margin_adc)))
        self.maximum_adc = min(1023, int(math.ceil(center_adc+span+margin_adc)))
        self.valid_samples = int(valid_samples)
        self.timeout_sec = float(timeout_sec)
        self.streak = 0
        self.last_adc = None
        self.last_sample_time = None
        self.last_valid_time = None
        self.sample_valid = False

    def update(self, adc, now):
        self.last_adc = int(adc)
        self.last_sample_time = float(now)
        self.sample_valid = self.minimum_adc <= self.last_adc <= self.maximum_adc
        self.streak = self.streak + 1 if self.sample_valid else 0
        valid = self.valid(now)
        if valid:
            self.last_valid_time = float(now)
        return valid

    def valid(self, now):
        return bool(
            self.sample_valid and self.streak >= self.valid_samples and
            self.last_sample_time is not None and
            0.0 <= float(now)-self.last_sample_time <= self.timeout_sec)

    def reason(self, now):
        if self.last_sample_time is None:
            return 'NO_STEERING_FEEDBACK'
        if float(now)-self.last_sample_time > self.timeout_sec:
            return 'STEERING_FEEDBACK_TIMEOUT'
        if not self.sample_valid:
            return 'STEERING_FEEDBACK_INVALID'
        if self.streak < self.valid_samples:
            return 'STEERING_FEEDBACK_WARMUP'
        return 'OK'

    def last_valid_age(self, now):
        if self.last_valid_time is None:
            return None
        return max(0.0, float(now)-self.last_valid_time)


def command_block_reason(feedback_valid, feedback_reason, input_stop,
                         command_times, now, timeout_sec):
    """Return the fail-closed reason for the three-part command contract."""
    if not feedback_valid:
        return str(feedback_reason)
    if bool(input_stop):
        return 'STOP_COMMAND'
    for name in ('drive', 'wheel', 'stop'):
        stamp = command_times.get(name)
        if stamp is None:
            return f'NO_{name.upper()}_COMMAND'
        if float(now)-float(stamp) > float(timeout_sec):
            return f'{name.upper()}_COMMAND_TIMEOUT'
    return 'NONE'


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
            'steering_feedback_required': True,
            'steering_feedback_topic': '/mcu/steer_a0',
            'steering_feedback_center_adc': 496,
            'steering_feedback_counts_per_deg': 18.0,
            'steering_feedback_max_deg': 22.0,
            'steering_feedback_margin_adc': 10,
            'steering_feedback_valid_samples': 3,
            'steering_feedback_timeout_sec': 0.50,
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

        self.feedback_gate = SteeringFeedbackGate(
            center_adc=int(self.p['steering_feedback_center_adc']),
            counts_per_deg=float(self.p['steering_feedback_counts_per_deg']),
            max_steer_deg=float(self.p['steering_feedback_max_deg']),
            margin_adc=int(self.p['steering_feedback_margin_adc']),
            valid_samples=int(self.p['steering_feedback_valid_samples']),
            timeout_sec=float(self.p['steering_feedback_timeout_sec']))
        self.last_feedback_state = False

        self.command_times = {'drive': None, 'wheel': None, 'stop': None}
        self.input_stop = True
        self.emergency_stop_latched = False
        self.drive_block_reason = ''
        self.pub_drive = self.create_publisher(Float32, '/mcu/cmd_drive', 10)
        self.pub_wheel = self.create_publisher(Int32, '/mcu/cmd_wheel', 10)
        self.pub_stop = self.create_publisher(Bool, '/mcu/cmd_stop', 10)
        self.pub_feedback_valid = self.create_publisher(
            Bool, '/mcu/steering_feedback_valid', 10)
        self.pub_feedback_status = self.create_publisher(
            String, '/mcu/steering_feedback_status', 10)
        self.pub_drive_block_reason = self.create_publisher(
            String, '/mcu/drive_block_reason', 10)
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
        self.create_subscription(
            Int32, str(self.p['steering_feedback_topic']),
            self.steering_feedback_cb, 10)
        if str(self.p['mode_source_topic']).strip():
            self.create_subscription(
                Int8, str(self.p['mode_source_topic']), self.mode_cb, 10)
        self.create_service(
            Trigger, '/vehicle/emergency_stop', self.emergency_stop_cb)
        self.create_service(
            Trigger, '/vehicle/emergency_stop/reset',
            self.emergency_stop_reset_cb)
        self.create_timer(0.05, self.timer_cb)
        if not bool(self.p['steering_feedback_required']):
            self.pub_wheel.publish(Int32(data=0))
        self.pub_drive.publish(Float32(data=0.0))
        self.pub_stop.publish(Bool(data=True))
        self._publish_feedback_state()
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

    def _fail_closed(self, reason, publish_wheel=True):
        # Safety output must reach the MCU before any logging call; a
        # logger exception must never delay drive=0/stop=true.
        if publish_wheel:
            self.pub_wheel.publish(Int32(data=0))
        self.pub_drive.publish(Float32(data=0.0))
        self.pub_stop.publish(Bool(data=True))
        self._set_block_reason(reason)

    def _feedback_valid(self):
        return (not bool(self.p['steering_feedback_required']) or
                self.feedback_gate.valid(time.monotonic()))

    def _block_reason(self, now):
        if self.emergency_stop_latched:
            return 'EMERGENCY_STOP'
        feedback_valid = (not bool(self.p['steering_feedback_required']) or
                          self.feedback_gate.valid(now))
        feedback_reason = (
            'DISABLED' if not bool(self.p['steering_feedback_required'])
            else self.feedback_gate.reason(now))
        return command_block_reason(
            feedback_valid, feedback_reason, self.input_stop,
            self.command_times, now, float(self.p['command_timeout_sec']))

    def _set_block_reason(self, reason):
        reason = str(reason)
        if reason != self.drive_block_reason:
            # rclpy/Jazzy rejects changing severity at one call site.  Keep a
            # fixed warning-level transition record so recovery cannot crash
            # the safety bridge.
            self.get_logger().warning(
                'drive block reason: '
                f'{reason}; raw_steering_adc={self.feedback_gate.last_adc}')
            self.drive_block_reason = reason
        self.pub_drive_block_reason.publish(String(data=reason))

    def _publish_feedback_state(self):
        now = time.monotonic()
        valid = self._feedback_valid()
        reason = ('DISABLED' if not bool(self.p['steering_feedback_required'])
                  else self.feedback_gate.reason(now))
        block_reason = self._block_reason(now)
        self._set_block_reason(block_reason)
        self.pub_feedback_valid.publish(Bool(data=valid))
        self.pub_feedback_status.publish(String(data=json.dumps({
            'valid': valid,
            'reason': reason,
            'raw_adc': self.feedback_gate.last_adc,
            'minimum_adc': self.feedback_gate.minimum_adc,
            'maximum_adc': self.feedback_gate.maximum_adc,
            'last_valid_feedback_age_sec': self.feedback_gate.last_valid_age(now),
            'drive_block_reason': block_reason,
        }, sort_keys=True)))
        if valid != self.last_feedback_state:
            # Keep severity fixed for the same reason as _set_block_reason().
            self.get_logger().warning(
                f'steering feedback gate: {reason}; '
                f'adc={self.feedback_gate.last_adc}')
            self.last_feedback_state = valid
        return valid

    def steering_feedback_cb(self, msg):
        self.feedback_gate.update(msg.data, time.monotonic())
        if not self._feedback_valid():
            # Do not publish a wheel target: W,0 would command the invalid
            # closed loop toward ADC 496 and could move the steering motor.
            # Publish before the diagnostics call below so a logger
            # exception cannot delay this safety output.
            self.pub_drive.publish(Float32(data=0.0))
            self.pub_stop.publish(Bool(data=True))
        self._publish_feedback_state()

    def drive_cb(self, msg):
        now = time.monotonic()
        self.command_times['drive'] = now
        if str(self.p['drive_input_unit']) == 'mps':
            value = translate_speed_mps(
                msg.data, float(self.p['stage_per_mps']),
                int(self.p['max_forward_drive_level']))
        else:
            value = translate_drive(
                msg.data, int(self.p['max_forward_drive_level']))
        if value is None:
            self._fail_closed('INVALID_DRIVE_COMMAND')
            return
        block_reason = self._block_reason(now)
        if value != 0.0 and block_reason != 'NONE':
            self._fail_closed(block_reason, publish_wheel=False)
            return
        self.pub_drive.publish(Float32(data=value))
        self._set_block_reason(block_reason)

    def wheel_cb(self, msg):
        self.command_times['wheel'] = time.monotonic()
        if not self._feedback_valid():
            self._fail_closed(
                self.feedback_gate.reason(time.monotonic()),
                publish_wheel=False)
            return
        value = translate_wheel(
            msg.data, int(self.p['wheel_sign_multiplier']),
            int(self.p['wheel_limit_deg']))
        if value is None:
            self._fail_closed('INVALID_WHEEL_COMMAND')
            return
        self.pub_wheel.publish(Int32(data=value))

    def stop_cb(self, msg):
        now = time.monotonic()
        self.command_times['stop'] = now
        self.input_stop = bool(msg.data)
        block_reason = self._block_reason(now)
        self.pub_stop.publish(Bool(data=block_reason != 'NONE'))
        self._set_block_reason(block_reason)

    def emergency_stop_cb(self, request, response):
        del request
        self.emergency_stop_latched = True
        self.command_times = {'drive': None, 'wheel': None, 'stop': None}
        self.input_stop = True
        self._fail_closed('EMERGENCY_STOP', publish_wheel=False)
        response.success = True
        response.message = 'Emergency stop latched; propulsion command forced to zero'
        return response

    def emergency_stop_reset_cb(self, request, response):
        del request
        self.emergency_stop_latched = False
        # Reset never authorizes motion.  Every command stream must become
        # fresh again, and steering feedback must independently pass its gate.
        self.command_times = {'drive': None, 'wheel': None, 'stop': None}
        self.input_stop = True
        self._fail_closed('EMERGENCY_STOP_RESET_SAFE', publish_wheel=False)
        response.success = True
        response.message = (
            'Emergency stop reset; stopped until fresh valid commands arrive')
        return response

    def mode_cb(self, msg):
        if self.pub_mode is not None and not bool(self.p['publish_mode_5']):
            self.pub_mode.publish(String(data=str(int(msg.data))))

    def timer_cb(self):
        # Compute and act on the safety state before any diagnostics call
        # below can log, so a logger exception cannot delay drive=0/stop=true.
        block_reason = self._block_reason(time.monotonic())
        if block_reason != 'NONE':
            self.pub_stop.publish(Bool(data=True))
            self.pub_drive.publish(Float32(data=0.0))
        self._publish_feedback_state()
        if bool(self.p['publish_mode_5']) and self.pub_mode is not None:
            self.pub_mode.publish(String(data='5'))
        if self.pub_odom is not None:
            self._publish_bench_odometry()

    def publish_shutdown_stop(self):
        for _ in range(3):
            # A W,0 target is unsafe when A0 feedback is invalid: the MCU
            # closed loop would try to reach ADC 496. Stop propulsion without
            # issuing any new steering target.
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
