#!/usr/bin/env python3
import math
import time
from functools import partial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8, Int32, String

from .command_selector import CommandSelector, STOP_SOURCE
from .command_state import ArbitrationStatus
from .input_manager import InputManager
from .gps_camera_transition import GpsCameraTransition, WAIT_FIRST_GPS
from .safety_manager import SafetyManager
from .section_entry_stop import SectionEntryStop


class McuManagerNode(Node):
    """Central drive/wheel arbiter with independent source selection."""

    def __init__(self):
        super().__init__("mcu_manager")

        # Core behavior
        self.declare_parameter("source_names", ["camera", "lidar", "gps", "manual"])
        self.declare_parameter(
            "mode_map",
            [
                "IDLE:none:none",
                "NORMAL:camera:camera",
                "INTERSECTION:gps:gps",
                "T_PARK:lidar:lidar",
                "PARALLEL_PARK:lidar:lidar",
                "SLOPE:camera:camera",
                "ACCELERATION:camera:camera",
                "MANUAL:manual:manual",
            ],
        )
        self.declare_parameter("initial_mode", "IDLE")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("drive_timeout_s", 0.5)
        self.declare_parameter("wheel_timeout_s", 0.5)
        self.declare_parameter("stop_drive_on_wheel_fault", True)
        self.declare_parameter("wheel_failsafe_mode", "hold_last")
        self.declare_parameter("gps_camera_wait_s", 2.0)
        self.declare_parameter("section_topic", "/mission/active_section")
        self.declare_parameter("speed_topic", "/vehicle/speed_mps")
        self.declare_parameter("speed_valid_topic", "/vehicle/speed_valid")
        self.declare_parameter("speed_timeout_s", 0.5)
        self.declare_parameter("stop_speed_threshold_mps", 0.05)
        self.declare_parameter("stop_confirm_s", 0.2)
        self.declare_parameter("section_entry_hold_s", 2.0)

        # Safety limits
        self.declare_parameter("drive_validation_mode", "allowed_values")
        self.declare_parameter("drive_allowed_values", [-1.0, 0.0, 1.0, 2.0, 3.0])
        self.declare_parameter("drive_min", -1.0)
        self.declare_parameter("drive_max", 3.0)
        self.declare_parameter("wheel_min", -27)
        self.declare_parameter("wheel_max", 27)

        # Control/status topics
        self.declare_parameter("mode_topic", "/vehicle_mode")
        self.declare_parameter("estop_topic", "/estop_lock")
        self.declare_parameter("output_drive_topic", "/mcu_drive")
        self.declare_parameter("output_wheel_topic", "/mcu_wheel")
        self.declare_parameter("status_mode_topic", "/mcu/current_mode")
        self.declare_parameter("status_drive_source_topic", "/mcu/active_drive_source")
        self.declare_parameter("status_wheel_source_topic", "/mcu/active_wheel_source")
        self.declare_parameter("status_safety_topic", "/mcu/safety_state")
        self.declare_parameter("status_ready_topic", "/mcu/ready")

        gp = lambda name: self.get_parameter(name).value
        self.source_names = [str(v).strip() for v in gp("source_names")]
        missing_sources = {"camera", "gps"}.difference(self.source_names)
        if missing_sources:
            raise ValueError(
                "required command sources missing: %s"
                % ", ".join(sorted(missing_sources)))
        self.drive_timeout_s = float(gp("drive_timeout_s"))
        self.wheel_timeout_s = float(gp("wheel_timeout_s"))
        self.stop_drive_on_wheel_fault = bool(gp("stop_drive_on_wheel_fault"))
        self.wheel_failsafe_mode = str(gp("wheel_failsafe_mode")).strip().lower()
        if self.wheel_failsafe_mode not in ("hold_last", "center"):
            raise ValueError("wheel_failsafe_mode must be hold_last or center")

        publish_hz = float(gp("publish_hz"))
        if publish_hz <= 0.0:
            raise ValueError("publish_hz must be > 0")
        if self.drive_timeout_s <= 0.0 or self.wheel_timeout_s <= 0.0:
            raise ValueError("timeouts must be > 0")

        self.inputs = InputManager(self.source_names)
        self.selector = CommandSelector(self.source_names, gp("mode_map"))
        self.safety = SafetyManager(
            drive_validation_mode=gp("drive_validation_mode"),
            drive_allowed_values=gp("drive_allowed_values"),
            drive_min=gp("drive_min"),
            drive_max=gp("drive_max"),
            wheel_min=gp("wheel_min"),
            wheel_max=gp("wheel_max"),
        )

        self.mode = str(gp("initial_mode")).strip().upper()
        self.estop = False
        self.last_wheel_output = 0
        self.last_status = ArbitrationStatus(mode=self.mode)
        self.gps_camera_transition = GpsCameraTransition(
            wait_sec=float(gp("gps_camera_wait_s")))
        self.section_entry_stop = SectionEntryStop(
            speed_threshold_mps=float(gp("stop_speed_threshold_mps")),
            stop_confirm_sec=float(gp("stop_confirm_s")),
            hold_sec=float(gp("section_entry_hold_s")),
        )
        self.speed_mps = 0.0
        self.speed_valid = False
        self.speed_time = None
        self.speed_valid_time = None
        self.speed_timeout_s = float(gp("speed_timeout_s"))

        # Source input topics are parameters so integration later is config-only.
        self.input_topics = {}
        for source in self.source_names:
            drive_param = "%s_drive_topic" % source
            wheel_param = "%s_wheel_topic" % source
            self.declare_parameter(drive_param, "/%s_drive" % source)
            self.declare_parameter(wheel_param, "/%s_wheel" % source)
            drive_topic = str(self.get_parameter(drive_param).value)
            wheel_topic = str(self.get_parameter(wheel_param).value)
            self.input_topics[source] = (drive_topic, wheel_topic)
            self.create_subscription(
                Float32,
                drive_topic,
                partial(self._drive_cb, source),
                10,
            )
            self.create_subscription(
                Int32,
                wheel_topic,
                partial(self._wheel_cb, source),
                10,
            )

        self.create_subscription(String, str(gp("mode_topic")), self._mode_cb, 10)
        self.create_subscription(Bool, str(gp("estop_topic")), self._estop_cb, 10)
        self.create_subscription(Int8, str(gp("section_topic")), self._section_cb, 10)
        self.create_subscription(Float32, str(gp("speed_topic")), self._speed_cb, 10)
        self.create_subscription(Bool, str(gp("speed_valid_topic")), self._speed_valid_cb, 10)

        self.pub_drive = self.create_publisher(Float32, str(gp("output_drive_topic")), 10)
        self.pub_wheel = self.create_publisher(Int32, str(gp("output_wheel_topic")), 10)
        self.pub_mode = self.create_publisher(String, str(gp("status_mode_topic")), 10)
        self.pub_drive_source = self.create_publisher(
            String, str(gp("status_drive_source_topic")), 10
        )
        self.pub_wheel_source = self.create_publisher(
            String, str(gp("status_wheel_source_topic")), 10
        )
        self.pub_safety = self.create_publisher(String, str(gp("status_safety_topic")), 10)
        self.pub_ready = self.create_publisher(Bool, str(gp("status_ready_topic")), 10)

        self.timer = self.create_timer(1.0 / publish_hz, self._tick)
        self.get_logger().info(
            "MCU manager started: sources=%s initial_mode=%s publish_hz=%.1f"
            % (self.source_names, self.mode, publish_hz)
        )

    def _drive_cb(self, source: str, msg: Float32) -> None:
        now = time.monotonic()
        value = float(msg.data)
        valid, reason = self.safety.validate_drive(value)
        self.inputs.update_drive(source, value, now, valid, reason)
        if not valid:
            self.get_logger().warn(
                "invalid drive from %s: %s (%s)" % (source, value, reason)
            )

    def _wheel_cb(self, source: str, msg: Int32) -> None:
        now = time.monotonic()
        value = int(msg.data)
        valid, reason = self.safety.validate_wheel(value)
        self.inputs.update_wheel(source, value, now, valid, reason)
        if not valid:
            self.get_logger().warn(
                "invalid wheel from %s: %s (%s)" % (source, value, reason)
            )

    def _mode_cb(self, msg: String) -> None:
        new_mode = str(msg.data).strip().upper()
        if new_mode != self.mode:
            self.get_logger().info("mode: %s -> %s" % (self.mode, new_mode))
            self.mode = new_mode

    def _estop_cb(self, msg: Bool) -> None:
        new_state = bool(msg.data)
        if new_state != self.estop:
            if new_state:
                self.get_logger().warn("E-STOP=true")
            else:
                self.get_logger().info("E-STOP released")
        self.estop = new_state

    def _section_cb(self, msg: Int8) -> None:
        if self.section_entry_stop.set_section(int(msg.data)):
            self.get_logger().info(
                "section %d entry: stopping until encoder confirms zero speed"
                % int(msg.data))

    def _speed_cb(self, msg: Float32) -> None:
        value = float(msg.data)
        if not math.isfinite(value):
            self.speed_valid = False
            self.speed_time = None
            self.get_logger().error("invalid non-finite vehicle speed")
            return
        self.speed_mps = value
        self.speed_time = time.monotonic()

    def _speed_valid_cb(self, msg: Bool) -> None:
        self.speed_valid = bool(msg.data)
        self.speed_valid_time = time.monotonic()

    def _failsafe_wheel(self) -> int:
        if self.wheel_failsafe_mode == "center":
            return 0
        return self.last_wheel_output

    def _tick(self) -> None:
        now = time.monotonic()
        drive_source, wheel_source, known_mode = self.selector.select(self.mode)

        transition_state = None
        if self.mode == "INTERSECTION":
            gps_drive_ok, _, _ = self.inputs.drive_status(
                "gps", now, self.drive_timeout_s)
            gps_wheel_ok, _, _ = self.inputs.wheel_status(
                "gps", now, self.wheel_timeout_s)
            camera_drive_ok, _, _ = self.inputs.drive_status(
                "camera", now, self.drive_timeout_s)
            camera_wheel_ok, _, _ = self.inputs.wheel_status(
                "camera", now, self.wheel_timeout_s)
            override, transition_state = self.gps_camera_transition.update(
                self.mode, now,
                gps_drive_ok and gps_wheel_ok,
                camera_drive_ok and camera_wheel_ok,
            )
            if override is not None:
                drive_source = override
                wheel_source = override
        else:
            self.gps_camera_transition.update(
                self.mode, now, gps_ok=False, camera_ok=False)

        drive_ok = False
        wheel_ok = False
        drive_reason = "stopped"
        wheel_reason = "stopped"
        drive_value = 0.0
        wheel_value = self._failsafe_wheel()

        if drive_source != STOP_SOURCE:
            drive_ok, drive_reason, drive_value = self.inputs.drive_status(
                drive_source, now, self.drive_timeout_s
            )
        if wheel_source != STOP_SOURCE:
            wheel_ok, wheel_reason, wheel_value = self.inputs.wheel_status(
                wheel_source, now, self.wheel_timeout_s
            )

        if self.estop:
            drive_value = 0.0
            wheel_value = self._failsafe_wheel()
            drive_ok = False
            wheel_ok = False
            safety_state = "ESTOP"
        elif not known_mode:
            drive_value = 0.0
            wheel_value = self._failsafe_wheel()
            drive_ok = False
            wheel_ok = False
            safety_state = "UNKNOWN_MODE"
        elif drive_source == STOP_SOURCE and wheel_source == STOP_SOURCE:
            drive_value = 0.0
            wheel_value = self._failsafe_wheel()
            safety_state = "IDLE"
        else:
            faults = []
            if drive_source != STOP_SOURCE and not drive_ok:
                drive_value = 0.0
                faults.append("DRIVE_%s_%s" % (drive_source.upper(), drive_reason.upper()))
            if wheel_source != STOP_SOURCE and not wheel_ok:
                wheel_value = self._failsafe_wheel()
                faults.append("WHEEL_%s_%s" % (wheel_source.upper(), wheel_reason.upper()))
                if self.stop_drive_on_wheel_fault:
                    drive_value = 0.0
            safety_state = "OK" if not faults else ";".join(faults)

        if transition_state in (WAIT_FIRST_GPS,
                                "GPS_TO_CAMERA_WAIT_2SEC",
                                "GPS_TO_CAMERA_WAIT_CAMERA"):
            drive_value = 0.0
            wheel_value = 0
            drive_ok = False
            wheel_ok = False
            safety_state = transition_state

        speed_fresh = (
            self.speed_time is not None and
            self.speed_valid_time is not None and
            now-self.speed_time <= self.speed_timeout_s and
            now-self.speed_valid_time <= self.speed_timeout_s
        )
        section_stop, section_state = self.section_entry_stop.update(
            now, self.speed_valid and speed_fresh, self.speed_mps)
        if section_stop:
            drive_value = 0.0
            wheel_value = 0
            drive_ok = False
            wheel_ok = False
            safety_state = section_state

        # E-stop and unknown-mode reporting must never be hidden by a lower
        # priority transition or section-entry state.
        if self.estop:
            drive_value = 0.0
            wheel_value = self._failsafe_wheel()
            drive_ok = False
            wheel_ok = False
            safety_state = "ESTOP"
        elif not known_mode:
            drive_value = 0.0
            wheel_value = self._failsafe_wheel()
            drive_ok = False
            wheel_ok = False
            safety_state = "UNKNOWN_MODE"

        if wheel_ok and not self.estop and known_mode:
            self.last_wheel_output = int(wheel_value)

        drive_msg = Float32()
        drive_msg.data = float(drive_value)
        wheel_msg = Int32()
        wheel_msg.data = int(wheel_value)
        self.pub_drive.publish(drive_msg)
        self.pub_wheel.publish(wheel_msg)

        effective_drive_source = drive_source if drive_ok and not self.estop and known_mode else "stop"
        effective_wheel_source = wheel_source if wheel_ok and not self.estop and known_mode else "stop"
        ready = safety_state == "OK"

        self._publish_status(
            ArbitrationStatus(
                mode=self.mode,
                drive_source=effective_drive_source,
                wheel_source=effective_wheel_source,
                safety_state=safety_state,
                ready=ready,
            )
        )

    def _publish_status(self, status: ArbitrationStatus) -> None:
        msg = String(); msg.data = status.mode; self.pub_mode.publish(msg)
        msg = String(); msg.data = status.drive_source; self.pub_drive_source.publish(msg)
        msg = String(); msg.data = status.wheel_source; self.pub_wheel_source.publish(msg)
        msg = String(); msg.data = status.safety_state; self.pub_safety.publish(msg)
        ready = Bool(); ready.data = status.ready; self.pub_ready.publish(ready)

        if status != self.last_status:
            self.get_logger().info(
                "mode=%s drive=%s wheel=%s safety=%s"
                % (
                    status.mode,
                    status.drive_source,
                    status.wheel_source,
                    status.safety_state,
                )
            )
            self.last_status = status


def main(args=None):
    rclpy.init(args=args)
    node = McuManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
