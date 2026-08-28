#!/usr/bin/env python3
"""
manager_node.py  —  T870 명령 중재기 (v3)

중재 규칙
---------
[1] /estop_lock          물리 비상. 래치. 서비스로만 해제
[2] 급정거 (/<src>_stop)  즉시 정지. 래치 아님 (요청이 사라지면 복귀)
[3] manual 소스           모드 무관 최우선 (안전요원 개입)
[4] 구동 = 고정 우선순위   라이다 > 카메라 > GPS
[5] 조향 = 모드 게이트     주차 모드에서만 라이다, 그 외 카메라

구동과 조향은 독립적으로 결정된다.
조향 소스가 값을 안 줘도 구동은 멈추지 않는다 (0도 직진으로 폴백).

발행
----
/mcu_drive   Float32   단계값 (브릿지가 시리얼로 변환)
/mcu_wheel   Int32     조향각 [도]
/mcu_stop    Bool      true = 브릿지가 램프 무시하고 즉시 정지
"""

import math
import time
from functools import partial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int32, String

from .arbitration import (
    ArbitrationStatus,
    InputManager,
    PrioritySelector,
    SafetyManager,
    WheelGate,
    CENTER_SOURCE,
    FAILSAFE,
)


# 실측 속도 (5m 주행 기준). 3단은 선형 외삽 추정치.
STAGE_MPS = {1: 0.229, 2: 0.526, 3: 0.823}


class ManagerNode(Node):

    def __init__(self):
        super().__init__("mcu_manager")

        # ---------- 소스 ----------
        self.declare_parameter("source_names", ["lidar", "camera", "gps", "manual"])

        # ---------- 구동: 고정 우선순위 ----------
        self.declare_parameter("drive_priority", ["lidar", "camera", "gps"])

        # ---------- 조향: 모드 게이트 ----------
        self.declare_parameter("wheel_owner_default", "camera")
        self.declare_parameter(
            "wheel_owner_overrides", ["T_PARK:lidar", "PARALLEL_PARK:lidar"])

        # 대회 구간 모드 문자열 화이트리스트.
        # 여기 없는 모드가 /vehicle_mode 로 오면 ERROR 를 찍고 무시한다.
        # (조용히 기본 소유자로 넘어가면 주차 구간 오타를 못 잡는다)
        # A bare [] is inferred as BYTE_ARRAY by rclpy Jazzy and then rejects
        # the STRING_ARRAY supplied by YAML. Keep a typed string default.
        # An explicitly empty YAML list may still be used to disable checking.
        self.declare_parameter("known_modes", [
            "IDLE", "START", "SLOPE", "CRANK", "INTERSECTION_1",
            "S_COURSE", "INTERSECTION_2", "T_PARK", "INTERSECTION_3",
            "ACCELERATION", "PARALLEL_PARK", "FINISH", "SUDDEN_STOP",
        ])

        # 알 수 없는 모드가 왔을 때:
        #   keep    = 직전 모드 유지 (권장. 갑자기 조향 권한이 바뀌지 않는다)
        #   default = 기본 소유자로 (기존 동작)
        self.declare_parameter("unknown_mode_policy", "keep")
        # 권한 소스가 값을 안 줄 때: center(0도) | hold_last
        self.declare_parameter("wheel_failsafe_mode", "center")

        # ---------- 급정거 ----------
        # 이 소스들의 /<src>_stop 이 true 면 즉시 정지
        self.declare_parameter("stop_sources", ["lidar", "camera"])
        # 정지 신호 유효시간. 이보다 오래되면 해제로 본다.
        self.declare_parameter("stop_timeout_s", 0.5)

        # ---------- 수동 개입 ----------
        self.declare_parameter("manual_override_enabled", True)
        self.declare_parameter("manual_source_name", "manual")

        # ---------- 공통 ----------
        self.declare_parameter("initial_mode", "IDLE")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("drive_timeout_s", 0.5)
        self.declare_parameter("wheel_timeout_s", 0.5)

        # ---------- 값 검증 ----------
        self.declare_parameter("drive_validation_mode", "allowed_values")
        self.declare_parameter("drive_allowed_values", [-1.0, 0.0, 1.0, 2.0, 3.0])
        self.declare_parameter("drive_min", -1.0)
        self.declare_parameter("drive_max", 3.0)
        self.declare_parameter("wheel_min", -27)
        self.declare_parameter("wheel_max", 27)

        # m/s → 단계 변환, Twist 변환에 사용
        self.declare_parameter("max_steer_deg", 27.0)
        self.declare_parameter("wheelbase_m", 0.77)
        self.declare_parameter("mps_deadband", 0.05)

        # ---------- 토픽 ----------
        self.declare_parameter("mode_topic", "/vehicle_mode")
        self.declare_parameter("estop_topic", "/estop_lock")
        self.declare_parameter("output_drive_topic", "/mcu/cmd_drive")
        self.declare_parameter("output_wheel_topic", "/mcu/cmd_wheel")
        self.declare_parameter("output_stop_topic", "/mcu/cmd_stop")
        self.declare_parameter("status_mode_topic", "/mcu/current_mode")
        self.declare_parameter("status_drive_source_topic", "/mcu/active_drive_source")
        self.declare_parameter("status_wheel_source_topic", "/mcu/active_wheel_source")
        self.declare_parameter("status_safety_topic", "/mcu/safety_state")
        self.declare_parameter("status_ready_topic", "/mcu/ready")

        gp = lambda n: self.get_parameter(n).value
        self.source_names = [str(v).strip() for v in gp("source_names")]
        self.drive_timeout = float(gp("drive_timeout_s"))
        self.wheel_timeout = float(gp("wheel_timeout_s"))
        self.stop_timeout = float(gp("stop_timeout_s"))
        self.stop_sources = [str(v).strip().lower() for v in gp("stop_sources")]
        self.manual_override = bool(gp("manual_override_enabled"))
        self.manual_name = str(gp("manual_source_name")).strip().lower()

        self.max_steer_deg = float(gp("max_steer_deg"))
        self.wheelbase = float(gp("wheelbase_m"))
        self.mps_deadband = float(gp("mps_deadband"))

        self.wheel_failsafe_mode = str(gp("wheel_failsafe_mode")).strip().lower()
        if self.wheel_failsafe_mode not in ("center", "hold_last"):
            raise ValueError("wheel_failsafe_mode must be center or hold_last")

        unknown_stop = set(self.stop_sources) - set(self.source_names)
        if unknown_stop:
            raise ValueError("stop_sources 에 알 수 없는 소스: %s" % sorted(unknown_stop))
        if self.manual_override and self.manual_name not in self.source_names:
            self.get_logger().warn(
                "manual_override_enabled=true 이지만 source_names 에 '%s' 없음 → 비활성화"
                % self.manual_name)
            self.manual_override = False

        publish_hz = float(gp("publish_hz"))
        if publish_hz <= 0:
            raise ValueError("publish_hz must be > 0")

        self.inputs = InputManager(self.source_names)
        self.drive_sel = PrioritySelector(gp("drive_priority"), self.source_names)
        self.known_modes = [str(m).strip().upper() for m in (gp("known_modes") or [])]
        self.unknown_mode_policy = str(gp("unknown_mode_policy")).strip().lower()
        if self.unknown_mode_policy not in ("keep", "default"):
            raise ValueError("unknown_mode_policy 는 keep 또는 default")
        self.wheel_gate = WheelGate(
            gp("wheel_owner_default"), gp("wheel_owner_overrides"),
            self.source_names, self.known_modes)
        self.unknown_mode_seen = ""     # 같은 오타를 한 번만 찍기 위한 기록
        self.safety = SafetyManager(
            drive_validation_mode=gp("drive_validation_mode"),
            drive_allowed_values=gp("drive_allowed_values"),
            drive_min=gp("drive_min"), drive_max=gp("drive_max"),
            wheel_min=gp("wheel_min"), wheel_max=gp("wheel_max"))

        self.mode = str(gp("initial_mode")).strip().upper()
        self.estop_asserted = False
        self.last_wheel_output = 0
        self.last_status = ArbitrationStatus(mode=self.mode)

        # ---------- 소스별 입력 단위/타입 ----------
        # 팀마다 토픽 이름·타입·단위가 다르므로 소스별로 맞춘다.
        #   drive_unit  : "stage"(0/1/2/3/-1) | "mps"(m/s)
        #   wheel_type  : "int"(Int32 도) | "float"(Float32 도)
        #                 | "norm"(Float32 -1.0~+1.0)
        #   cmd_vel_topic: 비우지 않으면 Twist 를 구독해 자동 변환
        self.src_cfg = {}
        for source in self.source_names:
            for pname, default in (
                    ("%s_drive_topic" % source, "/%s_drive" % source),
                    ("%s_wheel_topic" % source, "/%s_wheel" % source),
                    ("%s_stop_topic" % source, "/%s_stop" % source),
                    ("%s_drive_unit" % source, "stage"),
                    ("%s_wheel_type" % source, "int"),
                    ("%s_cmd_vel_topic" % source, "")):
                self.declare_parameter(pname, default)

            cfg = {
                "drive_unit": str(
                    self.get_parameter("%s_drive_unit" % source).value).lower(),
                "wheel_type": str(
                    self.get_parameter("%s_wheel_type" % source).value).lower(),
            }
            if cfg["drive_unit"] not in ("stage", "mps"):
                raise ValueError("%s_drive_unit 은 stage 또는 mps" % source)
            if cfg["wheel_type"] not in ("int", "float", "norm"):
                raise ValueError("%s_wheel_type 은 int/float/norm" % source)
            self.src_cfg[source] = cfg

            d_topic = str(self.get_parameter("%s_drive_topic" % source).value)
            w_topic = str(self.get_parameter("%s_wheel_topic" % source).value)
            s_topic = str(self.get_parameter("%s_stop_topic" % source).value)
            cv_topic = str(self.get_parameter("%s_cmd_vel_topic" % source).value).strip()

            if cv_topic:
                # Nav2 계열: /cmd_vel (Twist) 하나로 구동+조향이 같이 온다
                self.create_subscription(
                    Twist, cv_topic, partial(self._cb_cmdvel, source), 10)
                self.get_logger().info(
                    "%s: Twist 입력 %s → 자전거모델 변환" % (source, cv_topic))
            else:
                self.create_subscription(
                    Float32, d_topic, partial(self._cb_drive, source), 10)
                if cfg["wheel_type"] == "int":
                    self.create_subscription(
                        Int32, w_topic, partial(self._cb_wheel, source), 10)
                else:
                    self.create_subscription(
                        Float32, w_topic, partial(self._cb_wheel_f, source), 10)
                self.get_logger().info(
                    "%s: drive=%s(%s) wheel=%s(%s)"
                    % (source, d_topic, cfg["drive_unit"],
                       w_topic, cfg["wheel_type"]))

            if source in self.stop_sources:
                self.create_subscription(
                    Bool, s_topic, partial(self._cb_stop, source), 10)

        self.create_subscription(String, str(gp("mode_topic")), self._cb_mode, 10)
        self.create_subscription(Bool, str(gp("estop_topic")), self._cb_estop, 10)

        # ---------- 발행 ----------
        self.pub_drive = self.create_publisher(Float32, str(gp("output_drive_topic")), 10)
        self.pub_wheel = self.create_publisher(Int32, str(gp("output_wheel_topic")), 10)
        self.pub_stop = self.create_publisher(Bool, str(gp("output_stop_topic")), 10)
        self.pub_mode = self.create_publisher(String, str(gp("status_mode_topic")), 10)
        self.pub_dsrc = self.create_publisher(
            String, str(gp("status_drive_source_topic")), 10)
        self.pub_wsrc = self.create_publisher(
            String, str(gp("status_wheel_source_topic")), 10)
        self.pub_safety = self.create_publisher(
            String, str(gp("status_safety_topic")), 10)
        self.pub_ready = self.create_publisher(Bool, str(gp("status_ready_topic")), 10)

        self.timer = self.create_timer(1.0 / publish_hz, self._tick)

        self.get_logger().info(
            "mcu_manager v3: 구동우선순위=%s 조향기본=%s 조향override=%s "
            "정지소스=%s %.0fHz 모드검증=%s"
            % (self.drive_sel.priority, self.wheel_gate.default_owner,
               self.wheel_gate.overrides, self.stop_sources, publish_hz,
               ("%d개" % len(self.known_modes)) if self.known_modes else "없음"))

    # ============================================================
    # 콜백
    # ============================================================

    def _mps_to_stage(self, mps):
        """m/s → 가장 가까운 단계값. 실측 속도 기준."""
        if abs(mps) < self.mps_deadband:
            return 0.0
        if mps < 0:
            return -1.0
        best, best_err = 0.0, float("inf")
        for stage, speed in STAGE_MPS.items():
            err = abs(abs(mps) - speed)
            if err < best_err:
                best, best_err = float(stage), err
        return best

    def _cb_drive(self, source, msg):
        now = time.monotonic()
        value = float(msg.data)
        if self.src_cfg[source]["drive_unit"] == "mps":
            value = self._mps_to_stage(value)
        valid, reason = self.safety.validate_drive(value)
        self.inputs.update_drive(source, value, now, valid, reason)
        if not valid:
            self.get_logger().warn(
                "%s drive 거부: %s (%s)" % (source, value, reason))

    def _cb_wheel_f(self, source, msg):
        """Float32 조향. 도 단위이거나 -1.0~+1.0 정규화."""
        raw = float(msg.data)
        if self.src_cfg[source]["wheel_type"] == "norm":
            raw = raw * self.max_steer_deg
        self._apply_wheel(source, int(round(raw)))

    def _cb_wheel(self, source, msg):
        self._apply_wheel(source, int(msg.data))

    def _cb_cmdvel(self, source, msg):
        """Twist → 단계값 + 조향각 (자전거 모델).

        delta = atan(L * omega / v).  v 가 0 에 가까우면 조향 계산 불가.
        """
        now = time.monotonic()
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        stage = self._mps_to_stage(v)
        valid, reason = self.safety.validate_drive(stage)
        self.inputs.update_drive(source, stage, now, valid, reason)

        if abs(v) < self.mps_deadband:
            # 제자리 회전은 Ackermann 이 불가능하다. 조향만 중앙으로.
            deg = 0
        else:
            # ROS(REP-103): angular.z 양수 = 반시계 = 좌회전
            # 우리 규약   : 양수 = 우측
            # → 부호를 뒤집어야 한다. 안 그러면 좌우가 반대로 꺾인다.
            delta_ros = math.atan(self.wheelbase * w / v)   # v 부호 포함
            deg = int(round(-math.degrees(delta_ros)))
        deg = max(-int(self.max_steer_deg), min(int(self.max_steer_deg), deg))
        self._apply_wheel(source, deg)

    def _apply_wheel(self, source, deg):
        now = time.monotonic()
        valid, reason = self.safety.validate_wheel(deg)
        self.inputs.update_wheel(source, deg, now, valid, reason)
        if not valid:
            self.get_logger().warn(
                "%s wheel 거부: %s (%s)" % (source, deg, reason))

    def _cb_wheel_unused(self, source, msg):
        now = time.monotonic()
        value = int(msg.data)
        valid, reason = self.safety.validate_wheel(value)
        self.inputs.update_wheel(source, value, now, valid, reason)
        if not valid:
            self.get_logger().warn(
                "%s wheel 거부: %s (%s)" % (source, value, reason))

    def _cb_stop(self, source, msg):
        prev = self.inputs.stop_asserted(source, time.monotonic(), self.stop_timeout)
        self.inputs.update_stop(source, bool(msg.data), time.monotonic())
        if bool(msg.data) and not prev:
            self.get_logger().warn("급정거 요청: %s" % source)

    def _cb_mode(self, msg):
        new_mode = str(msg.data).strip().upper()

        # ---- 모드 문자열 검증 ----
        # 오타 하나가 조향 권한을 통째로 바꾼다. 조용히 넘기지 않는다.
        if not self.wheel_gate.is_known(new_mode):
            if new_mode != self.unknown_mode_seen:
                self.get_logger().error(
                    "알 수 없는 모드 '%s' — known_modes 에 없다. "
                    "policy=%s. 허용: %s"
                    % (new_mode, self.unknown_mode_policy,
                       ", ".join(self.known_modes)))
                self.unknown_mode_seen = new_mode
            if self.unknown_mode_policy == "keep":
                return                      # 직전 모드 유지
        else:
            self.unknown_mode_seen = ""

        if new_mode != self.mode:
            old_owner = self.wheel_gate.owner(self.mode)
            new_owner = self.wheel_gate.owner(new_mode)
            self.get_logger().info(
                "모드 %s -> %s (조향권한 %s -> %s)"
                % (self.mode, new_mode, old_owner, new_owner))
            self.mode = new_mode

    def _cb_estop(self, msg):
        """E-stop 상태를 그대로 반영한다.

        ★ 래치하지 않는다.
          래치는 브릿지 한 곳에서만 한다(/mcu/reset_estop 로 해제).
          매니저까지 래치하면 해제 수단이 두 개 필요해지고,
          실제로 '서비스는 성공했는데 매니저가 안 풀리는' 상황이 생겼다.
          브릿지가 시리얼 직전 마지막 관문이므로 거기 래치 하나로 충분하다.
        """
        new_state = bool(msg.data)
        if new_state != self.estop_asserted:
            if new_state:
                self.get_logger().error(
                    "E-STOP 입력 — 정지. 브릿지 래치 해제는 /mcu/reset_estop")
            else:
                self.get_logger().info("E-STOP 입력 해제")
        self.estop_asserted = new_state

    # ============================================================
    # 주기 실행
    # ============================================================

    def _failsafe_wheel(self):
        return 0 if self.wheel_failsafe_mode == "center" else self.last_wheel_output

    def _tick(self):
        now = time.monotonic()

        # ---- [1] E-STOP ----
        if self.estop_asserted:
            self._publish(0.0, self._failsafe_wheel(), True,
                          "stop", "stop", "ESTOP", False)
            return

        # ---- [2] 급정거 ----
        stopping = self.inputs.any_stop(self.stop_sources, now, self.stop_timeout)
        if stopping:
            self._publish(0.0, self._failsafe_wheel(), True,
                          "stop", "stop",
                          "EMERGENCY_STOP(%s)" % ",".join(stopping), False)
            return

        # ---- [5] 조향: 모드 게이트 (구동과 독립) ----
        wheel_value, wheel_used, wheel_ok, wheel_reason = self.wheel_gate.resolve(
            self.mode, self.inputs, now, self.wheel_timeout, self._failsafe_wheel())

        faults = []

        # ---- [3] 수동 개입 ----
        override = False
        if self.manual_override:
            m_ok, _, m_val = self.inputs.drive_status(
                self.manual_name, now, self.drive_timeout)
            if m_ok:
                override = True
                drive_value = m_val
                drive_src = self.manual_name
                w_ok, _, w_val = self.inputs.wheel_status(
                    self.manual_name, now, self.wheel_timeout)
                if w_ok:
                    wheel_value, wheel_used, wheel_ok = int(w_val), self.manual_name, True
                else:
                    wheel_value, wheel_used = 0, CENTER_SOURCE

        # ---- [4] 구동: 고정 우선순위 ----
        if not override:
            src, drive_value, tried = self.drive_sel.select(
                self.inputs, now, self.drive_timeout)
            if src is None:
                drive_value = 0.0
                drive_src = "stop"
                faults.append("NO_DRIVE_SOURCE(%s)" % ",".join(tried))
            else:
                drive_src = src

        if not wheel_ok:
            faults.append("WHEEL_FALLBACK(%s)" % wheel_reason)

        if wheel_used not in (CENTER_SOURCE, FAILSAFE):
            self.last_wheel_output = int(wheel_value)

        if override:
            state = "MANUAL_OVERRIDE" + (";" + ";".join(faults) if faults else "")
        elif faults:
            state = ";".join(faults)
        else:
            state = "OK"

        self._publish(drive_value, wheel_value, False,
                      drive_src, wheel_used, state,
                      state in ("OK", "MANUAL_OVERRIDE"))

    # ============================================================

    def _publish(self, drive_value, wheel_value, stop_flag,
                 drive_src, wheel_src, safety_state, ready):
        d = Float32(); d.data = float(drive_value); self.pub_drive.publish(d)
        w = Int32();   w.data = int(wheel_value);   self.pub_wheel.publish(w)
        s = Bool();    s.data = bool(stop_flag);    self.pub_stop.publish(s)

        status = ArbitrationStatus(
            mode=self.mode, drive_source=drive_src, wheel_source=wheel_src,
            safety_state=safety_state, ready=ready)

        m = String(); m.data = status.mode; self.pub_mode.publish(m)
        m = String(); m.data = status.drive_source; self.pub_dsrc.publish(m)
        m = String(); m.data = status.wheel_source; self.pub_wsrc.publish(m)
        m = String(); m.data = status.safety_state; self.pub_safety.publish(m)
        r = Bool(); r.data = status.ready; self.pub_ready.publish(r)

        if status != self.last_status:
            self.get_logger().info(
                "mode=%s drive=%s wheel=%s safety=%s"
                % (status.mode, status.drive_source,
                   status.wheel_source, status.safety_state))
            self.last_status = status


def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()
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
