#!/usr/bin/env python3
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String
import serial


def find_serial_port(preferred: str = '/dev/t870_mcu'):
    if preferred and os.path.exists(preferred):
        return os.path.realpath(preferred)

    by_id = Path('/dev/serial/by-id')
    if by_id.is_dir():
        candidates = []
        for p in by_id.iterdir():
            name = p.name.lower()
            if any(x in name for x in ('gps', 'gnss', 'u-blox', 'ublox')):
                continue
            score = 0
            if 'arduino' in name or 'mega' in name:
                score = 100
            elif any(x in name for x in ('ch340', 'ch341', 'wch')):
                score = 50
            if score:
                candidates.append((score, str(p.resolve())))
        if candidates:
            candidates.sort(reverse=True)
            return candidates[0][1]

    acms = sorted(Path('/dev').glob('ttyACM*'))
    if len(acms) == 1:
        return str(acms[0])
    return None


class T870CmdBridge(Node):
    def __init__(self):
        super().__init__('t870_cmd_bridge')

        # Topics: exactly two command subscriptions.
        self.declare_parameter('drive_topic', '/cmd_drive')
        self.declare_parameter('wheel_topic', '/cmd_wheel')

        self.declare_parameter('port', 'auto')
        self.declare_parameter('preferred_symlink', '/dev/t870_mcu')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('reset_wait_s', 2.0)
        self.declare_parameter('reconnect_s', 1.0)
        self.declare_parameter('tx_hz', 10.0)
        self.declare_parameter('drive_timeout_s', 0.5)

        self.declare_parameter('reverse_pwm', 50)
        self.declare_parameter('stage1_pwm', 50)
        self.declare_parameter('stage2_pwm', 75)
        self.declare_parameter('stage3_pwm', 100)

        self.declare_parameter('steer_center_adc', 484)
        self.declare_parameter('steer_counts_per_deg', 18.0)
        self.declare_parameter('max_steer_deg', 22)
        self.declare_parameter('steer_pwm', 130)
        self.declare_parameter('steer_tolerance_adc', 4)
        self.declare_parameter('steer_settle_ms', 250)
        self.declare_parameter('steer_fine_band_adc', 50)
        self.declare_parameter('steer_fine_on_ms', 30)
        self.declare_parameter('steer_fine_cycle_ms', 90)
        self.declare_parameter('steer_timeout_ms', 5000)
        self.declare_parameter('firmware_drive_timeout_ms', 700)
        self.declare_parameter('firmware_status_ms', 200)
        self.declare_parameter('front_forward_level', 0)
        self.declare_parameter('rear_forward_level', 0)
        self.declare_parameter('steer_left_level', 1)
        self.declare_parameter('encoder_cpr', 163.0)

        p = lambda k: self.get_parameter(k).value
        self.drive_topic = str(p('drive_topic'))
        self.wheel_topic = str(p('wheel_topic'))
        self.port_param = str(p('port'))
        self.preferred_symlink = str(p('preferred_symlink'))
        self.baud = int(p('baud'))
        self.reset_wait_s = float(p('reset_wait_s'))
        self.reconnect_s = float(p('reconnect_s'))
        self.tx_period = 1.0 / max(1.0, float(p('tx_hz')))
        self.drive_timeout_s = float(p('drive_timeout_s'))

        self.reverse_pwm = int(p('reverse_pwm'))
        self.stage_pwm = {0: 0, 1: int(p('stage1_pwm')), 2: int(p('stage2_pwm')), 3: int(p('stage3_pwm'))}
        self.center_adc = int(p('steer_center_adc'))
        self.counts_per_deg = float(p('steer_counts_per_deg'))
        self.max_deg = int(p('max_steer_deg'))
        self.steer_pwm = int(p('steer_pwm'))
        self.steer_tol = int(p('steer_tolerance_adc'))
        self.steer_settle_ms = int(p('steer_settle_ms'))
        self.steer_fine_band = int(p('steer_fine_band_adc'))
        self.steer_fine_on_ms = int(p('steer_fine_on_ms'))
        self.steer_fine_cycle_ms = int(p('steer_fine_cycle_ms'))
        self.steer_timeout_ms = int(p('steer_timeout_ms'))
        self.fw_drive_timeout_ms = int(p('firmware_drive_timeout_ms'))
        self.fw_status_ms = int(p('firmware_status_ms'))
        self.front_fwd = 1 if int(p('front_forward_level')) else 0
        self.rear_fwd = 1 if int(p('rear_forward_level')) else 0
        self.steer_left = 1 if int(p('steer_left_level')) else 0
        self.encoder_cpr = float(p('encoder_cpr'))

        self.ser = None
        self.port = None
        self.opened_at = 0.0
        self.last_connect_attempt = 0.0
        self.ready = False
        self.rx_buf = bytearray()

        self.cmd_drive = 0
        self.last_drive_rx = None
        self.last_wheel_cmd = 0
        self.last_enc = None
        self.last_enc_t = None

        # ONLY TWO command subscriptions.
        self.create_subscription(Float32, self.drive_topic, self.cb_drive, 10)
        self.create_subscription(Int32, self.wheel_topic, self.cb_wheel, 10)

        # Feedback only.
        self.pub_connected = self.create_publisher(Bool, '/mcu/connected', 10)
        self.pub_ready = self.create_publisher(Bool, '/mcu/ready', 10)
        self.pub_applied_drive = self.create_publisher(Float32, '/mcu/applied_drive', 10)
        self.pub_applied_wheel = self.create_publisher(Int32, '/mcu/applied_wheel', 10)
        self.pub_a0 = self.create_publisher(Int32, '/mcu/steer_a0', 10)
        self.pub_steer_deg = self.create_publisher(Float32, '/mcu/steer_deg', 10)
        self.pub_encoder = self.create_publisher(Int32, '/mcu/encoder', 10)
        self.pub_encA = self.create_publisher(Int32, '/mcu/encA', 10)
        self.pub_encoder_rate = self.create_publisher(Float32, '/mcu/encoder_rate', 10)
        self.pub_rpm = self.create_publisher(Float32, '/mcu/rpm', 10)
        self.pub_raw = self.create_publisher(String, '/mcu/raw_status', 10)
        self.pub_fw = self.create_publisher(String, '/mcu/fw_message', 10)

        self.create_timer(0.02, self.io_tick)
        self.create_timer(self.tx_period, self.tx_tick)

        self.get_logger().info(
            f'INPUT ONLY: {self.drive_topic} (Float32), {self.wheel_topic} (Int32)')
        self.get_logger().info(
            'No manager / no mode / no arbitration / no automatic drive / no cmd_stop subscription')
        self.get_logger().info(
            f'steer: center={self.center_adc}, +LEFT/-RIGHT, clamp ±{self.max_deg}deg')
        self.get_logger().info('drive encoder: ENC_A Arduino Mega D2, RISING only, debounce 200us')

    def resolve_port(self):
        if self.port_param.lower() not in ('auto', '', 'none'):
            return self.port_param
        return find_serial_port(self.preferred_symlink)

    def open_serial(self):
        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_s:
            return
        self.last_connect_attempt = now
        port = self.resolve_port()
        if not port:
            return
        try:
            self.ser = serial.Serial(port, self.baud, timeout=0, write_timeout=0.2)
            self.port = port
            self.opened_at = now
            self.ready = False
            self.rx_buf.clear()
            self.publish_bool(self.pub_connected, True)
            self.publish_bool(self.pub_ready, False)
            self.get_logger().info(f'serial opened: {port} @ {self.baud}; reset wait {self.reset_wait_s:.1f}s')
        except Exception as e:
            self.ser = None
            self.publish_bool(self.pub_connected, False)
            self.get_logger().warn(f'serial open failed: {type(e).__name__}: {e}')

    def close_serial(self, reason=''):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.ready = False
        self.port = None
        if rclpy.ok():
            self.publish_bool(self.pub_connected, False)
            self.publish_bool(self.pub_ready, False)
        if reason and rclpy.ok():
            self.get_logger().warn(reason)

    def send(self, text):
        if self.ser is None:
            return False
        try:
            self.ser.write((text + '\n').encode('ascii'))
            return True
        except Exception as e:
            self.close_serial(f'serial write failed: {type(e).__name__}: {e}')
            return False

    def configure_and_arm(self):
        # No steering movement. Configure, force drive=0, then arm.
        cmds = [
            f'CFG,CENTER,{self.center_adc}',
            f'CFG,COUNTS_PER_DEG,{self.counts_per_deg:.6f}',
            f'CFG,MAX_DEG,{self.max_deg}',
            f'CFG,STEER_PWM,{self.steer_pwm}',
            f'CFG,STEER_TOL,{self.steer_tol}',
            f'CFG,STEER_SETTLE_MS,{self.steer_settle_ms}',
            f'CFG,STEER_FINE_BAND,{self.steer_fine_band}',
            f'CFG,STEER_FINE_ON_MS,{self.steer_fine_on_ms}',
            f'CFG,STEER_FINE_CYCLE_MS,{self.steer_fine_cycle_ms}',
            f'CFG,STEER_TIMEOUT_MS,{self.steer_timeout_ms}',
            f'CFG,DRIVE_TIMEOUT_MS,{self.fw_drive_timeout_ms}',
            f'CFG,STATUS_MS,{self.fw_status_ms}',
            f'CFG,FRONT_FWD,{self.front_fwd}',
            f'CFG,REAR_FWD,{self.rear_fwd}',
            f'CFG,STEER_LEFT,{self.steer_left}',
            'X', 'ARM', 'D,0',
        ]
        for c in cmds:
            if not self.send(c):
                return False
        self.ready = True
        self.publish_bool(self.pub_ready, True)
        self.get_logger().info('Arduino READY; drive=0; waiting /cmd_drive and /cmd_wheel')
        return True

    def io_tick(self):
        if self.ser is None:
            self.open_serial()
            return
        if not self.ready and time.monotonic() - self.opened_at >= self.reset_wait_s:
            self.configure_and_arm()

        try:
            waiting = self.ser.in_waiting
            if waiting:
                self.rx_buf.extend(self.ser.read(waiting))
                if len(self.rx_buf) > 8192:
                    self.rx_buf.clear()
                while b'\n' in self.rx_buf:
                    line, _, rest = self.rx_buf.partition(b'\n')
                    self.rx_buf = bytearray(rest)
                    text = line.decode('utf-8', errors='replace').strip()
                    if text:
                        self.handle_line(text)
        except Exception as e:
            self.close_serial(f'serial read failed: {type(e).__name__}: {e}')

    def tx_tick(self):
        if not self.ready:
            self.publish_applied_drive(0)
            return

        now = time.monotonic()
        fresh = self.last_drive_rx is not None and now - self.last_drive_rx <= self.drive_timeout_s
        stage = self.cmd_drive if fresh else 0
        self.send(f'D,{self.stage_to_pwm(stage)}')
        self.publish_applied_drive(stage)

    def stage_to_pwm(self, stage):
        if stage == -1:
            return -abs(self.reverse_pwm)
        return abs(self.stage_pwm.get(stage, 0))

    def cb_drive(self, msg):
        stage = int(round(float(msg.data)))
        if stage not in (-1, 0, 1, 2, 3):
            self.get_logger().warn(f'cmd_drive ignored: {msg.data}; allowed -1,0,1,2,3')
            return
        self.cmd_drive = stage
        self.last_drive_rx = time.monotonic()

    def cb_wheel(self, msg):
        deg = max(-self.max_deg, min(self.max_deg, int(msg.data)))
        self.last_wheel_cmd = deg
        if self.ready:
            self.send(f'W,{deg}')
            m = Int32(); m.data = deg
            self.pub_applied_wheel.publish(m)

    def handle_line(self, text):
        if text.startswith('STAT,'):
            self.handle_status(text)
            return
        m = String(); m.data = text
        self.pub_fw.publish(m)
        if text.startswith(('ERR,', 'EVT,')):
            self.get_logger().warn(f'MCU: {text}')

    def handle_status(self, text):
        # STAT,adc,target_adc,target_deg,drive_signed_pwm,steer_active,armed,encA
        parts = text.split(',')
        if len(parts) != 8:
            return
        try:
            adc = int(parts[1])
            enc_a = int(parts[7])
        except ValueError:
            return

        m = String(); m.data = text; self.pub_raw.publish(m)
        m = Int32(); m.data = adc; self.pub_a0.publish(m)
        m = Float32(); m.data = float(adc - self.center_adc) / self.counts_per_deg; self.pub_steer_deg.publish(m)
        m = Int32(); m.data = enc_a; self.pub_encoder.publish(m)
        m = Int32(); m.data = enc_a; self.pub_encA.publish(m)

        now = time.monotonic()
        rate = 0.0
        rpm = 0.0
        if self.last_enc is not None and self.last_enc_t is not None:
            dt = now - self.last_enc_t
            if dt > 1e-3:
                rate = (enc_a - self.last_enc) / dt
                if self.encoder_cpr > 0:
                    rpm = rate * 60.0 / self.encoder_cpr
        self.last_enc = enc_a
        self.last_enc_t = now
        m = Float32(); m.data = float(rate); self.pub_encoder_rate.publish(m)
        m = Float32(); m.data = float(rpm); self.pub_rpm.publish(m)

    def publish_applied_drive(self, stage):
        m = Float32(); m.data = float(stage)
        self.pub_applied_drive.publish(m)

    @staticmethod
    def publish_bool(pub, value):
        m = Bool(); m.data = bool(value)
        pub.publish(m)

    def shutdown(self):
        if self.ser is not None:
            try:
                self.send('D,0')
                self.send('X')
                self.send('DISARM')
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None


def main(args=None):
    rclpy.init(args=args)
    node = T870CmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
