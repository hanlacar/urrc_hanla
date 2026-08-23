#!/usr/bin/env python3
"""T870 ROS 2 <-> Arduino serial bridge for firmware v28.

Input:
  /mcu_drive  std_msgs/Float32  {-1,0,1,2,3}
  /mcu_wheel  std_msgs/Int32    [-27,+27] deg, -left/+right
  /estop_lock std_msgs/Bool     True latches E-stop; False does not clear it

E-stop reset:
  /mcu/reset_estop std_srvs/Trigger

Drive and wheel freshness are intentionally independent. A stale wheel command does
not stop a valid drive command. A stale drive command stops propulsion only.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

try:
    from tf2_ros import TransformBroadcaster
    TF_OK = True
except ImportError:
    TransformBroadcaster = None
    TF_OK = False

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial missing: sudo apt install python3-serial") from exc

from .protocol import (
    drive_serial_command,
    parse_drive_stage,
    parse_status,
    valid_wheel_deg,
    wheel_serial_command,
)


class McuBridge(Node):
    def __init__(self):
        super().__init__("mcu_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("send_hz", 10.0)
        self.declare_parameter("drive_timeout_s", 0.5)
        self.declare_parameter("wheel_timeout_s", 0.5)
        self.declare_parameter("status_poll_hz", 5.0)
        self.declare_parameter("max_steer_deg", 27)
        self.declare_parameter("steer_limit_ms", 440)
        self.declare_parameter("steer_cmd_mode", "W")
        self.declare_parameter("wheel_timeout_policy", "hold_last")
        self.declare_parameter("latch_on_firmware_fault", True)
        self.declare_parameter("reconnect_delay_s", 1.0)
        self.declare_parameter("counts_per_meter", 0.0)
        self.declare_parameter("wheelbase_m", 0.73)
        self.declare_parameter("encoder_signed", False)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        gp = lambda name: self.get_parameter(name).value
        self.port = str(gp("port"))
        self.baud = int(gp("baud"))
        self.send_period = 1.0 / float(gp("send_hz"))
        self.drive_timeout = float(gp("drive_timeout_s"))
        self.wheel_timeout = float(gp("wheel_timeout_s"))
        status_hz = float(gp("status_poll_hz"))
        self.status_period = 1.0 / status_hz if status_hz > 0 else 0.0
        self.max_deg = int(gp("max_steer_deg"))
        self.steer_limit_ms = int(gp("steer_limit_ms"))
        self.steer_mode = str(gp("steer_cmd_mode")).upper()
        self.wheel_timeout_policy = str(gp("wheel_timeout_policy")).lower()
        self.latch_on_firmware_fault = bool(gp("latch_on_firmware_fault"))
        self.reconnect_delay = float(gp("reconnect_delay_s"))
        self.cpm = float(gp("counts_per_meter"))
        self.wheelbase = float(gp("wheelbase_m"))
        self.encoder_signed = bool(gp("encoder_signed"))
        self.odom_frame = str(gp("odom_frame"))
        self.base_frame = str(gp("base_frame"))

        if self.wheel_timeout_policy not in ("hold_last", "center"):
            raise ValueError("wheel_timeout_policy must be hold_last or center")

        self.ser = None
        self.ser_lock = threading.Lock()
        self.connected = False
        self._last_connect_attempt = 0.0

        self.cmd_stage = 0
        self.cmd_deg = 0
        self.last_drive_rx = 0.0
        self.last_wheel_rx = 0.0
        self.have_drive = False
        self.have_wheel = False
        self.wheel_dirty = False
        self.last_status_req = 0.0
        self.drive_timeout_warned = False
        self.wheel_timeout_warned = False

        # Latched safety state. False messages never clear the latch.
        self.ros_estop_asserted = False
        self.estop_latched = False
        self.firmware_fault = 0

        # Existing encoder-based odometry support is preserved. Disabled while cpm=0.
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.distance_m = 0.0
        self.prev_count = None
        self.prev_count_t = None

        self.sub_drive = self.create_subscription(Float32, "/mcu_drive", self.cb_drive, 10)
        self.sub_wheel = self.create_subscription(Int32, "/mcu_wheel", self.cb_wheel, 10)
        self.sub_estop = self.create_subscription(Bool, "/estop_lock", self.cb_estop, 10)

        self.pub_conn = self.create_publisher(Bool, "/arduino/connected", 10)
        self.pub_status = self.create_publisher(String, "/arduino/status", 10)
        self.pub_raw = self.create_publisher(String, "/arduino/raw_status", 10)
        self.pub_fault = self.create_publisher(Int32, "/arduino/fault", 10)
        self.pub_drive = self.create_publisher(Int32, "/drive", 10)
        self.pub_wheel = self.create_publisher(Float32, "/wheel", 10)
        self.pub_rpm = self.create_publisher(Float32, "/rpm", 10)
        self.pub_a0 = self.create_publisher(Int32, "/steer_a0", 10)
        self.pub_steer_ms = self.create_publisher(Int32, "/steer_position_ms", 10)
        self.pub_estop = self.create_publisher(Bool, "/mcu/estop_latched", 10)
        self.pub_speed = self.create_publisher(Float32, "/vehicle/speed_mps", 10)
        self.pub_distance = self.create_publisher(Float32, "/vehicle/distance_m", 10)
        self.pub_speed_valid = self.create_publisher(Bool, "/vehicle/speed_valid", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.tf_bc = TransformBroadcaster(self) if TF_OK else None

        self.reset_srv = self.create_service(Trigger, "/mcu/reset_estop", self.cb_reset_estop)

        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()
        self.tx_timer = self.create_timer(self.send_period, self.tx_tick)
        self.conn_timer = self.create_timer(0.2, self.conn_tick)

        self.get_logger().info(
            f"mcu_bridge ready: {self.port} @ {self.baud}, send={1/self.send_period:.1f}Hz, "
            f"drive_timeout={self.drive_timeout:.2f}s, wheel_timeout={self.wheel_timeout:.2f}s"
        )

    def cb_drive(self, msg: Float32):
        stage = parse_drive_stage(float(msg.data))
        if stage is None:
            self.get_logger().error(
                f"invalid /mcu_drive={msg.data!r}; accepted values are -1,0,1,2,3. Command rejected."
            )
            # Invalid propulsion is fail-safe: stop propulsion immediately.
            self.cmd_stage = 0
            self.have_drive = True
            self.last_drive_rx = time.monotonic()
            return
        self.cmd_stage = stage
        self.have_drive = True
        self.last_drive_rx = time.monotonic()
        self.drive_timeout_warned = False

    def cb_wheel(self, msg: Int32):
        deg = int(msg.data)
        if not valid_wheel_deg(deg, self.max_deg):
            self.get_logger().error(
                f"invalid /mcu_wheel={deg}; allowed range is -{self.max_deg}..+{self.max_deg}. Command rejected."
            )
            return
        self.cmd_deg = deg
        self.have_wheel = True
        self.last_wheel_rx = time.monotonic()
        self.wheel_dirty = True
        self.wheel_timeout_warned = False

    def cb_estop(self, msg: Bool):
        self.ros_estop_asserted = bool(msg.data)
        if msg.data:
            self.estop_latched = True
            self.get_logger().error("ROS E-STOP asserted: latch set")
        # False deliberately does not clear estop_latched.

    def cb_reset_estop(self, _request, response):
        if self.ros_estop_asserted:
            response.success = False
            response.message = "reset denied: /estop_lock is still true"
            return response
        if self.firmware_fault != 0:
            response.success = False
            response.message = f"reset denied: Arduino fault={self.firmware_fault}"
            return response
        self.estop_latched = False
        response.success = True
        response.message = "E-stop latch cleared; fresh ROS commands are still required"
        return response

    def open_serial(self):
        now = time.monotonic()
        if now - self._last_connect_attempt < self.reconnect_delay:
            return False
        self._last_connect_attempt = now
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2.0)  # Arduino USB reset
            with self.ser_lock:
                self.ser = ser
            self.connected = True

            # Reconnect policy B: discard all pre-disconnect commands.
            self.cmd_stage = 0
            self.cmd_deg = 0
            self.have_drive = False
            self.have_wheel = False
            self.wheel_dirty = False
            self.last_drive_rx = 0.0
            self.last_wheel_rx = 0.0

            # Startup/reconnect always begins with propulsion stop.
            self.send_line("1.00")
            self.get_logger().info(f"serial connected: {self.port}; STOP sent, waiting for fresh ROS commands")
            return True
        except serial.SerialException as exc:
            self.connected = False
            with self.ser_lock:
                self.ser = None
            self.get_logger().warn(f"serial unavailable: {exc}")
            return False

    def close_serial(self):
        with self.ser_lock:
            ser = self.ser
            self.ser = None
        self.connected = False
        if ser:
            try:
                ser.close()
            except Exception:
                pass

    def send_line(self, line: str) -> bool:
        with self.ser_lock:
            ser = self.ser
            if ser is None:
                return False
            try:
                ser.write((line + "\n").encode("ascii"))
                return True
            except serial.SerialException:
                pass
        self.get_logger().error("serial TX failed; connection dropped")
        self.close_serial()
        return False

    def conn_tick(self):
        if not self.connected:
            self.open_serial()
        msg = Bool(); msg.data = self.connected; self.pub_conn.publish(msg)
        e = Bool(); e.data = self.estop_latched; self.pub_estop.publish(e)

    def tx_tick(self):
        if not self.connected:
            return
        now = time.monotonic()

        drive_fresh = self.have_drive and (now - self.last_drive_rx <= self.drive_timeout)
        wheel_fresh = self.have_wheel and (now - self.last_wheel_rx <= self.wheel_timeout)

        if self.estop_latched:
            stage_to_send = 0
        elif drive_fresh:
            stage_to_send = self.cmd_stage
        else:
            stage_to_send = 0
            if self.have_drive and not self.drive_timeout_warned:
                self.get_logger().warn("/mcu_drive timeout -> propulsion STOP; wheel is unaffected")
                self.drive_timeout_warned = True

        # Drive watchdog feeding at fixed 10 Hz.
        self.send_line(drive_serial_command(stage_to_send))

        # Wheel timeout never stops drive.
        if self.have_wheel and not wheel_fresh and not self.wheel_timeout_warned:
            self.get_logger().warn(f"/mcu_wheel timeout -> policy={self.wheel_timeout_policy}; drive continues independently")
            self.wheel_timeout_warned = True
            if self.wheel_timeout_policy == "center":
                self.cmd_deg = 0
                self.wheel_dirty = True

        # E-stop freezes steering at its current command; no automatic recenter.
        if not self.estop_latched and self.wheel_dirty:
            cmd = wheel_serial_command(self.cmd_deg, self.max_deg, self.steer_limit_ms, self.steer_mode)
            if self.send_line(cmd):
                self.wheel_dirty = False

        if self.status_period > 0 and now - self.last_status_req >= self.status_period:
            if self.send_line("S"):
                self.last_status_req = now

    def rx_loop(self):
        buf = b""
        while rclpy.ok():
            with self.ser_lock:
                ser = self.ser
            if ser is None:
                time.sleep(0.1)
                continue
            try:
                chunk = ser.read(256)
            except serial.SerialException:
                self.close_serial()
                time.sleep(0.1)
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self.handle_line(raw.decode("utf-8", "replace").strip())

    def handle_line(self, line: str):
        status = parse_status(line)
        if status is None:
            return

        m = String(); m.data = status["raw"]; self.pub_raw.publish(m)
        m = String(); m.data = status["state"]; self.pub_status.publish(m)
        m = Int32(); m.data = status["fault"]; self.pub_fault.publish(m)
        m = Int32(); m.data = status["adc"]; self.pub_a0.publish(m)
        m = Float32(); m.data = status["rpm"]; self.pub_rpm.publish(m)
        m = Int32(); m.data = status["encoder_count"]; self.pub_drive.publish(m)
        m = Int32(); m.data = status["steer_ms"]; self.pub_steer_ms.publish(m)

        steer_deg = status["steer_ms"] / (self.steer_limit_ms / float(self.max_deg))
        m = Float32(); m.data = float(round(steer_deg, 1)); self.pub_wheel.publish(m)

        self.update_odom(status["encoder_count"], status["steer_ms"])

        self.firmware_fault = status["fault"]
        if self.latch_on_firmware_fault and self.firmware_fault != 0:
            if not self.estop_latched:
                self.get_logger().error(f"Arduino fault={self.firmware_fault}: E-stop latch set")
            self.estop_latched = True


    def update_odom(self, count: int, steer_ms: int):
        valid = Bool(); valid.data = self.cpm > 0; self.pub_speed_valid.publish(valid)
        if self.cpm <= 0:
            return

        now = time.monotonic()
        if self.prev_count is None:
            self.prev_count = count
            self.prev_count_t = now
            return

        delta = count - self.prev_count
        dt = now - self.prev_count_t
        self.prev_count = count
        self.prev_count_t = now
        if dt <= 0.0:
            return

        if not self.encoder_signed:
            if self.cmd_stage < 0:
                delta = -abs(delta)
            elif self.cmd_stage > 0:
                delta = abs(delta)
            else:
                delta = 0

        steer_deg = steer_ms / (self.steer_limit_ms / float(self.max_deg))
        steer_rad = math.radians(steer_deg)
        d_front = delta / self.cpm
        d = d_front * math.cos(steer_rad)
        speed = d / dt
        self.distance_m += abs(d)

        dtheta = d * math.tan(steer_rad) / self.wheelbase
        if abs(dtheta) > 1e-6:
            radius = d / dtheta
            self.x += radius * (math.sin(self.th + dtheta) - math.sin(self.th))
            self.y -= radius * (math.cos(self.th + dtheta) - math.cos(self.th))
        else:
            self.x += d * math.cos(self.th)
            self.y += d * math.sin(self.th)
        self.th = math.atan2(math.sin(self.th + dtheta), math.cos(self.th + dtheta))

        m = Float32(); m.data = float(speed); self.pub_speed.publish(m)
        m = Float32(); m.data = float(self.distance_m); self.pub_distance.publish(m)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th * 0.5)
        odom.pose.pose.orientation.w = math.cos(self.th * 0.5)
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = speed * math.tan(steer_rad) / self.wheelbase
        self.pub_odom.publish(odom)

        if self.tf_bc:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = odom.pose.pose.orientation.z
            tf.transform.rotation.w = odom.pose.pose.orientation.w
            self.tf_bc.send_transform(tf)

    def shutdown(self):
        # Best effort stop before port close.
        try:
            self.send_line("1.00")
            time.sleep(0.05)
        finally:
            self.close_serial()


def main(args=None):
    rclpy.init(args=args)
    node = McuBridge()
    node.open_serial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
