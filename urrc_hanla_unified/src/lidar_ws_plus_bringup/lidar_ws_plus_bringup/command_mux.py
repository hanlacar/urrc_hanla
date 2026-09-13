#!/usr/bin/env python3
"""Mode-gated, fail-closed owner of the three final LiDAR MCU topics."""

from dataclasses import dataclass
import json
import math
import signal
import time
from typing import Dict, Iterable, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32, Int32, String


VALID_DRIVE_STAGES = frozenset({-1.0, 0.0, 1.0, 2.0, 3.0})


@dataclass
class SourceState:
    drive: float = 0.0
    wheel: int = 0
    stop: bool = True
    drive_time: Optional[float] = None
    wheel_time: Optional[float] = None
    stop_time: Optional[float] = None
    drive_valid: bool = False
    wheel_valid: bool = False

    def command_fresh(self, now: float, timeout: float) -> bool:
        stamps = (self.drive_time, self.wheel_time, self.stop_time)
        return (
            self.drive_valid
            and self.wheel_valid
            and all(stamp is not None for stamp in stamps)
            and all(0.0 <= now - stamp <= timeout for stamp in stamps)
        )


def normalize_modes(values: Iterable[object]) -> frozenset[str]:
    return frozenset(
        str(value).strip().upper() for value in values if str(value).strip())


def select_source(
        mode: str, parking_modes: Iterable[object],
        avoidance_modes: Iterable[object]) -> Optional[str]:
    normalized = str(mode).strip().upper()
    if normalized in normalize_modes(parking_modes):
        return 'parking'
    if normalized in normalize_modes(avoidance_modes):
        return 'avoidance'
    return None


def valid_drive(value: float) -> bool:
    return math.isfinite(float(value)) and float(value) in VALID_DRIVE_STAGES


def choose_output(
        source: Optional[SourceState], now: float, timeout: float,
        forced_stop: bool = False) -> Tuple[float, int, bool, str]:
    if forced_stop:
        return 0.0, 0, True, 'GLOBAL_STOP'
    if source is None:
        return 0.0, 0, False, 'NON_LIDAR_MODE'
    if not source.command_fresh(now, timeout):
        return 0.0, 0, True, 'ACTIVE_SOURCE_TIMEOUT_OR_INVALID'
    if source.stop:
        return 0.0, source.wheel, True, 'SOURCE_STOP'
    return source.drive, source.wheel, False, 'ACTIVE'


class CommandMux(Node):
    """The sole publisher of /lidar_drive, /lidar_wheel, and /lidar_stop."""

    def __init__(self) -> None:
        super().__init__('command_mux')
        defaults = {
            'mode_topic': '/mcu/current_mode',
            'parking_modes': ['T_PARK', 'PARALLEL_PARK'],
            # Preserved from the avoidance workspace. The inspected MCU mode
            # map does not define 5; deployment must reconcile that contract.
            'avoidance_modes': ['5'],
            'parking_drive_topic': '/parking/drive_cmd',
            'parking_wheel_topic': '/parking/wheel_cmd',
            'parking_stop_topic': '/parking/stop_cmd',
            'avoidance_drive_topic': '/avoidance/drive_cmd',
            'avoidance_wheel_topic': '/avoidance/wheel_cmd',
            'avoidance_stop_topic': '/avoidance/stop_cmd',
            'lidar_safety_stop_topic': '/lidar/stop_required',
            'estop_topic': '/estop_lock',
            'command_timeout_sec': 0.50,
            'mode_timeout_sec': 0.50,
            'stop_timeout_sec': 0.50,
            'publish_rate_hz': 20.0,
            'wheel_limit_deg': 22,
            'check_publisher_conflicts': True,
            'conflict_check_period_sec': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.parking_modes = normalize_modes(self.p['parking_modes'])
        self.avoidance_modes = normalize_modes(self.p['avoidance_modes'])
        self.command_timeout = float(self.p['command_timeout_sec'])
        self.mode_timeout = float(self.p['mode_timeout_sec'])
        self.stop_timeout = float(self.p['stop_timeout_sec'])
        self.wheel_limit = int(self.p['wheel_limit_deg'])
        publish_rate = float(self.p['publish_rate_hz'])
        if not self.parking_modes or not self.avoidance_modes:
            raise ValueError('parking_modes and avoidance_modes must not be empty')
        if min(self.command_timeout, self.mode_timeout, self.stop_timeout) <= 0.0:
            raise ValueError('all timeouts must be > 0')
        if publish_rate < 4.0:
            raise ValueError('publish_rate_hz must be >= 4 Hz')
        if not 0 < self.wheel_limit <= 22:
            raise ValueError('wheel_limit_deg must be in (0, 22]')

        self.sources: Dict[str, SourceState] = {
            'parking': SourceState(),
            'avoidance': SourceState(),
        }
        self.mode = ''
        self.mode_time: Optional[float] = None
        self.lidar_stop = True
        self.lidar_stop_time: Optional[float] = None
        self.estop = False
        self.conflicts = []

        self.drive_pub = self.create_publisher(Float32, '/lidar_drive', 10)
        self.wheel_pub = self.create_publisher(Int32, '/lidar_wheel', 10)
        self.stop_pub = self.create_publisher(Bool, '/lidar_stop', 10)
        self.status_pub = self.create_publisher(
            String, '/lidar_ws_plus/command_mux/status', 10)

        self.create_subscription(
            String, str(self.p['mode_topic']), self._mode_callback, 10)
        self._subscribe_source('parking')
        self._subscribe_source('avoidance')
        self.create_subscription(
            Bool, str(self.p['lidar_safety_stop_topic']),
            self._lidar_stop_callback, 10)
        self.create_subscription(
            Bool, str(self.p['estop_topic']), self._estop_callback, 10)
        self.create_timer(1.0 / publish_rate, self._tick)
        if bool(self.p['check_publisher_conflicts']):
            period = max(0.2, float(self.p['conflict_check_period_sec']))
            self.create_timer(period, self._check_conflicts)
        self.get_logger().info(
            'command_mux owns /lidar_drive, /lidar_wheel, /lidar_stop; '
            f'mode source={self.p["mode_topic"]}')

    def _subscribe_source(self, source: str) -> None:
        self.create_subscription(
            Float32, str(self.p[f'{source}_drive_topic']),
            lambda msg, source=source: self._drive_callback(source, msg), 10)
        self.create_subscription(
            Int32, str(self.p[f'{source}_wheel_topic']),
            lambda msg, source=source: self._wheel_callback(source, msg), 10)
        self.create_subscription(
            Bool, str(self.p[f'{source}_stop_topic']),
            lambda msg, source=source: self._stop_callback(source, msg), 10)

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _mode_callback(self, msg: String) -> None:
        self.mode = str(msg.data).strip().upper()
        self.mode_time = self._now()

    def _drive_callback(self, source: str, msg: Float32) -> None:
        state = self.sources[source]
        state.drive = float(msg.data)
        state.drive_valid = valid_drive(state.drive)
        state.drive_time = self._now()
        if not state.drive_valid:
            self.get_logger().error(
                f'invalid {source} drive stage {state.drive!r}; fail-closed')

    def _wheel_callback(self, source: str, msg: Int32) -> None:
        state = self.sources[source]
        state.wheel = int(msg.data)
        state.wheel_valid = abs(state.wheel) <= self.wheel_limit
        state.wheel_time = self._now()
        if not state.wheel_valid:
            self.get_logger().error(
                f'invalid {source} wheel {state.wheel}; fail-closed')

    def _stop_callback(self, source: str, msg: Bool) -> None:
        state = self.sources[source]
        state.stop = bool(msg.data)
        state.stop_time = self._now()

    def _lidar_stop_callback(self, msg: Bool) -> None:
        self.lidar_stop = bool(msg.data)
        self.lidar_stop_time = self._now()

    def _estop_callback(self, msg: Bool) -> None:
        self.estop = bool(msg.data)

    def _check_conflicts(self) -> None:
        conflicts = []
        for topic in ('/lidar_drive', '/lidar_wheel', '/lidar_stop'):
            for endpoint in self.get_publishers_info_by_topic(topic):
                if endpoint.node_name != self.get_name():
                    conflicts.append(
                        f'{topic}:{endpoint.node_namespace}/{endpoint.node_name}')
        updated = sorted(set(conflicts))
        if updated and updated != self.conflicts:
            self.get_logger().error(
                f'final-topic publisher conflict: {updated}; fail-closed')
        self.conflicts = updated

    def _global_stop_active(self, now: float) -> Tuple[bool, str]:
        if self.estop:
            return True, 'ESTOP'
        if self.conflicts:
            return True, 'PUBLISHER_CONFLICT'
        if (self.lidar_stop_time is not None and self.lidar_stop
                and 0.0 <= now - self.lidar_stop_time <= self.stop_timeout):
            return True, 'LIDAR_SAFETY_STOP'
        for name, state in self.sources.items():
            if (state.stop_time is not None and state.stop
                    and 0.0 <= now - state.stop_time <= self.stop_timeout):
                return True, f'{name.upper()}_STOP'
        return False, ''

    def _tick(self) -> None:
        now = self._now()
        mode_fresh = (
            self.mode_time is not None
            and 0.0 <= now - self.mode_time <= self.mode_timeout)
        source_name = select_source(
            self.mode, self.parking_modes, self.avoidance_modes)
        forced_stop, forced_reason = self._global_stop_active(now)
        if not mode_fresh:
            source_name = None
            forced_stop = True
            forced_reason = 'MODE_TIMEOUT'
        source = self.sources[source_name] if source_name else None
        drive, wheel, stop, state = choose_output(
            source, now, self.command_timeout, forced_stop)
        if forced_reason:
            state = forced_reason
        self.wheel_pub.publish(Int32(data=int(wheel)))
        self.drive_pub.publish(Float32(data=float(drive)))
        self.stop_pub.publish(Bool(data=bool(stop)))
        self.status_pub.publish(String(data=json.dumps({
            'state': state,
            'mode': self.mode,
            'mode_fresh': mode_fresh,
            'active_source': source_name or '',
            'drive': drive,
            'wheel': wheel,
            'stop': stop,
            'publisher_conflicts': self.conflicts,
        }, sort_keys=True)))

    def publish_shutdown_stop(self, repeats: int = 5) -> None:
        for _ in range(max(1, repeats)):
            self.wheel_pub.publish(Int32(data=0))
            self.drive_pub.publish(Float32(data=0.0))
            self.stop_pub.publish(Bool(data=True))
            time.sleep(0.05)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = CommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
