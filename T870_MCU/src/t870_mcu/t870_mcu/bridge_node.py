#!/usr/bin/env python3
"""
mcu_bridge_node.py  —  T870 Arduino v28 시리얼 브릿지 (대회용 v2)

v1 대비 수정 사항
-----------------
[치명] 1. STATUS 파싱 실패로 텔레메트리 전멸
   protocol.parse_status 가 fault 필드를 숫자로만 읽어서, v28 이 출력하는
   "NONE" 문자열에서 예외 → 모든 STATUS 라인 폐기.
   /drive, /rpm, /arduino/raw_status, /odom 이 전혀 안 나왔고
   펌웨어 fault 기반 E-stop 래치도 동작하지 않았다.
   → protocol.py 에서 수정. 개별 필드가 깨져도 라인 전체를 버리지 않는다.

[중대] 2. 재연결 시 노드 2초 프리즈
   open_serial() 안의 time.sleep(2.0) 이 타이머 콜백을 블록해서,
   재연결 시도마다 구독/발행/워치독이 전부 멈췄다.
   → 논블로킹 연결 상태머신(CLOSED / WAIT_RESET / READY)으로 분리.

[중대] 3. 시작 시 조향 상태 불명
   wheel_dirty=False 로 시작해 조향 명령을 한 번도 보내지 않았다.
   펌웨어 조향은 시간 기반이라 이전 세션 위치를 그대로 물고 있어,
   부팅 직후 실제 조향각을 아무도 모르는 상태였다.
   → center_steer_on_connect (기본 true) 로 연결 시 W0 1회 전송.

[중대] 4. 코스팅 거리 누락
   cmd_stage==0 이면 엔코더 delta 를 0 으로 버렸다. 이 차량은 브레이크가
   없어 정지 명령 후에도 상당 거리를 굴러가는데(감속 램프만 0.5초),
   그 거리가 odom 에서 통째로 사라졌다.
   → coasting_policy 로 선택. 기본은 직전 진행 방향을 유지해 적산.

[보완] 5. 수신 버퍼 무한 증가 방어, 파라미터 타입 완화,
        시리얼 통신 두절 감시 추가.

v3 (2026-08-28) — 급정거 정상화 + 하드코딩 제거
-----------------------------------------------
[치명] 6. 급정거가 급정거가 아니었다
   estop_serial_command 기본값이 v28 시절의 "1.00"(감속 램프 정지)로 남아
   있었다. v29 부터 램프를 건너뛰는 제동 명령 B 가 있는데 쓰지 않았다.
   → 기본값을 "B" 로 바꿨다.

[치명] 7. 제동 명령을 반복 전송하면 차가 뒤로 간다
   펌웨어 requestBrake() 는 호출될 때마다 제동 펄스 타이머를 다시 찍는다.
   브릿지 전송 주기(기본 100ms)가 BRAKE_PULSE_MS(150ms)보다 짧아서,
   매 주기 B 를 보내면 펄스 종료 조건이 영원히 성립하지 않고 역토크가
   계속 걸린 채 유지된다 → 역주행.
   → 제동은 상승 에지에 1회만. 이후에는 정지 유지 명령만 반복해
     펌웨어 워치독을 먹인다. (estop_hold_command / estop_repeat_s)

[중대] 8. 포트 하드코딩 제거
   setup.sh 가 udev 로 /dev/t870_mcu 심볼릭 링크를 만드는데 코드가 그걸
   보지 않았다. 이제 심볼릭 링크를 1순위로 본다(port_symlinks).
   못 찾았을 때 /dev/ttyACM0 을 무작정 여는 동작도 없앴다 — 그 자리에
   GPS 가 앉아 있을 수 있다. fallback_port 를 명시할 때만 시도한다.

[중대] 9. 발행 토픽 기본값을 전부 /mcu/* 로
   기본값이 /rpm, /inpulse, /steer_angle 같은 최상위 이름이라 yaml 없이
   노드를 띄우면 다른 팀 토픽과 충돌했다. 팀 배포용으로 안전하지 않다.

[보완] 10. /estop_lock, /mcu/reset_estop 도 파라미터로 뺐다.
"""

import math
import os
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
    TF_OK = False

import serial

from .diagnostics import check_subscriptions
from .protocol import (
    encoder_sanity,
    parse_drive_stage,
    drive_serial_command,
    valid_wheel_deg,
    wheel_serial_command,
    parse_status,
)

# 연결 상태
CONN_CLOSED = 0        # 포트 닫힘
CONN_WAIT_RESET = 1    # 포트 열림, 아두이노 USB 리셋 대기 중
CONN_READY = 2         # 통신 가능

MAX_RX_BUF = 8192      # 개행 없는 쓰레기 유입 시 메모리 방어

# 펌웨어가 제동을 유지할 수 있는 최대 시간 (v33 BRAKE_MAX_MS = 300ms).
# 제동 재시도 간격이 이보다 짧으면 앞 제동이 끝나기 전에 다음 B 가 들어와
# 펄스가 끊기지 않는다. 여유를 두고 0.35 로 잡는다.
#
# ⚠ 간격을 더 좁히고 싶어도 참을 것.
#   제동 방향은 driveForward(마지막으로 "명령된" 방향) 기준이다. 실제 움직임이
#   아니다. 차가 멈춘 뒤 조금이라도 뒤로 밀리면, 다음 제동 펄스도 여전히
#   뒤쪽으로 인가되어 후진을 제동하는 게 아니라 후진을 가속한다.
#   A상만 세므로 엔코더로 방향을 알 수 없어 펌웨어도 이걸 구분하지 못한다.
BRAKE_PULSE_GUARD_S = 0.35


# ==========================================================
# 포트 자동 탐색
#
#  USB 를 꽂는 순서에 따라 /dev/ttyACM 번호가 바뀐다.
#  GPS 를 같이 꽂으면 아두이노가 ACM0 이 되기도 ACM1 이 되기도 한다.
#  실제로 이것 때문에 반나절을 날린 적이 있으므로 번호를 믿지 않고
#  USB 장치 정보(제조사/시리얼)로 아두이노를 직접 찾는다.
# ==========================================================

ARDUINO_HINTS = ("arduino", "mega", "ch340", "ch910", "wch", "usb2.0-serial")
NOT_ARDUINO_HINTS = ("u-blox", "u_blox", "gnss", "gps")


def find_arduino_port(logger=None, symlinks=()):
    """아두이노 시리얼 포트를 찾는다.

    탐색 순서
      1) udev 심볼릭 링크 (기본 /dev/t870_mcu — setup.sh 가 만든다)
         벤더 ID 기준이라 어느 컴퓨터에서도 같은 이름이 나온다.
         팀원 PC 마다 포트 번호가 다른 문제를 여기서 끝낸다.
      2) /dev/serial/by-id 의 장치 이름 매칭 (심볼릭 링크가 없을 때)

    반환: 포트 실제 경로. 못 찾으면 None.
    """
    for link in symlinks:
        link = str(link).strip()
        if link and os.path.exists(link):
            real = os.path.realpath(link)
            if logger is not None:
                logger.info("udev 심볼릭 링크 사용: %s -> %s" % (link, real))
            return real

    base = "/dev/serial/by-id"
    if not os.path.isdir(base):
        return None

    candidates = []
    for name in sorted(os.listdir(base)):
        low = name.lower()
        if any(bad in low for bad in NOT_ARDUINO_HINTS):
            continue                       # GPS 등은 제외
        score = 0
        if "arduino" in low or "mega" in low:
            score = 100                    # 정품 아두이노 우선
        elif any(h in low for h in ARDUINO_HINTS):
            score = 50                     # 클론 칩
        if score:
            real = os.path.realpath(os.path.join(base, name))
            candidates.append((score, real, name))

    if not candidates:
        return None
    candidates.sort(reverse=True)
    if logger is not None and len(candidates) > 1:
        logger.warn("아두이노 후보가 여럿이다. 첫 번째 사용: %s"
                    % [c[1] for c in candidates])
    return candidates[0][1]


class McuBridge(Node):

    def __init__(self):
        super().__init__("mcu_bridge")

        # ---------- 파라미터 ----------
        self.declare_parameter("port", "auto")

        # port:"auto" 일 때 가장 먼저 볼 udev 심볼릭 링크 목록.
        # setup.sh 가 /dev/t870_mcu 를 만든다. 다른 이름을 쓰면 여기에 적는다.
        self.declare_parameter("port_symlinks", ["/dev/t870_mcu"])

        # 자동 탐색이 실패했을 때 시도할 포트. 비워두는 것이 기본이다.
        # ★ 예전에는 /dev/ttyACM0 을 무작정 열었는데, 거기 GPS 가 앉아 있으면
        #   GPS 포트에 구동 명령을 쏘게 된다. 이제는 명시할 때만 시도한다.
        self.declare_parameter("fallback_port", "")

        self.declare_parameter("baud", 115200)
        self.declare_parameter("send_hz", 10.0)
        self.declare_parameter("drive_timeout_s", 0.5)
        self.declare_parameter("wheel_timeout_s", 0.5)
        self.declare_parameter("status_poll_hz", 5.0)
        self.declare_parameter("max_steer_deg", 27.0)
        self.declare_parameter("steer_limit_ms", 440)
        self.declare_parameter("steer_cmd_mode", "W")
        self.declare_parameter("wheel_timeout_policy", "hold_last")

        # ---------- 직진 보정 ----------
        # 차량이 한쪽으로 치우쳐 갈 때 모든 조향 명령에 더하는 고정 각도.
        # 원인이 조향 영점이든 뒷축 틀어짐이든 결과적으로 같은 방식으로 보정된다.
        # +는 우, -는 좌.  예) 오른쪽으로 흘러가면 음수를 넣는다.
        self.declare_parameter("steer_offset_deg", 0.0)
        self.declare_parameter("latch_on_firmware_fault", True)

        # /estop_lock 의 true 를 "살아있는 주장" 으로 볼 유효시간 [초].
        #
        # ★ 이 값이 없으면 복구 불가능한 상태가 생긴다.
        #   래치 해제 서비스는 "지금도 /estop_lock 이 true 인가" 를 보고
        #   막는데, 어떤 노드가 true 를 한 번 쏘고 죽어버리면 마지막 값이
        #   영원히 true 로 남는다. → 리셋이 영구히 거부되고 브릿지를
        #   재시작하는 것 외에 복구 수단이 없어진다.
        #   (실제로 팀 노드를 켜자마자 E-Stop 이 걸려서 안 풀리는 일이 있었다)
        #
        # 여기서 막는 것은 "해제 허용 여부" 뿐이고 래치 자체는 그대로다.
        # 0 이면 타임아웃 없음(예전 동작).
        self.declare_parameter("estop_assert_timeout_s", 1.0)
        self.declare_parameter("reconnect_delay_s", 1.0)
        self.declare_parameter("arduino_reset_wait_s", 2.0)

        # 연결/재연결 시 조향을 0도로 한 번 맞출지.
        # 펌웨어 조향은 시간 기반이라 이전 세션 위치가 남아 있다.
        self.declare_parameter("center_steer_on_connect", True)

        # 텔레메트리 두절 판정 [초]. 0 이면 감시 안 함.
        self.declare_parameter("telemetry_timeout_s", 2.0)

        # 코스팅 처리:
        #   last_direction = 정지 명령 후에도 직전 진행 방향으로 적산 (권장)
        #   drop           = 정지 중 delta 폐기 (v1 동작)
        self.declare_parameter("coasting_policy", "last_direction")

        # ---------- 급정거 ----------
        #
        # ★ 제동 명령(B)은 "상승 에지에 1회"만 보낸다. 반복 전송 금지.
        #   펌웨어 requestBrake() 는 호출될 때마다 제동 펄스 타이머를 다시
        #   찍는다. 전송 주기(100ms)가 BRAKE_PULSE_MS(150ms)보다 짧으므로
        #   매 주기 보내면 펄스가 끝나지 않고 역토크가 유지되어 차가 뒤로 간다.
        #
        # 급정거가 걸린 순간 1회 보낼 명령. v29+ 펌웨어의 다이내믹 브레이킹.
        self.declare_parameter("estop_serial_command", "B")

        # 제동 이후 정지를 유지하며 매 주기 반복할 명령.
        # 펌웨어 워치독(2초)을 먹이는 역할도 겸한다.
        self.declare_parameter("estop_hold_command", "1.00")

        # 아직 굴러가고 있을 때를 대비한 제동 재시도 간격 [초].
        # 0 이면 재시도하지 않는다.
        #
        # v33 펌웨어는 엔코더로 정지를 감지해 스스로 제동을 끊고, 못 끊으면
        # BRAKE_MAX_MS(300ms)에 강제 종료한다. 재시도 간격은 그보다 넉넉해야
        # 한다(BRAKE_PULSE_GUARD_S). 이미 정지해 있으면 펌웨어가 역토크 없이
        # 넘긴다 (BRAKE_OK,ALREADY_STOPPED).
        self.declare_parameter("estop_repeat_s", 0.5)

        # 급정거 신호 유효시간. 이보다 오래되면 해제로 본다.
        self.declare_parameter("stop_timeout_s", 0.5)

        # 오도메트리
        self.declare_parameter("counts_per_meter", 0.0)
        self.declare_parameter("wheelbase_m", 0.73)
        self.declare_parameter("encoder_signed", False)
        #  엔코더 누적값이 1초에 이보다 많이 변하면 시리얼이 깨진 것으로 본다.
        #  199.8 counts/m 기준 2000 = 10 m/s. 이 차의 최고속(약 0.8m/s)의 12배라
        #  정상 주행은 절대 안 걸리고, 필드가 밀린 값만 걸러진다.
        #  0 으로 두면 이 검사를 끈다.
        self.declare_parameter("encoder_max_counts_per_s", 2000.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        # ---------- 오도메트리 신뢰도 (공분산) ----------
        #
        # ★ 이전에는 covariance 를 한 번도 채우지 않아 전부 0 이었다.
        #   0 은 "오차가 없다"는 뜻이다. EKF(robot_localization 등)로 융합하면
        #   필터가 이 값을 절대 신뢰하고 GPS 를 무시한다.
        #   우리 heading 은 조향각을 시간으로 추측한 값이라 가장 못 믿을 값인데,
        #   그걸 "완벽하다"고 선언하고 있었다.
        #
        # 오차는 이동할수록 쌓이므로 리셋 이후 누적해서 키운다.
        #
        # ⚠ 아래 기본값은 잠정값이다. 실측 후 조정할 것.
        #    dist_ratio 0.15 는 counts_per_meter 실측 편차 15.2% 에서 왔다.
        #    yaw 쪽은 조향각을 측정하지 못하므로 근거 없는 보수적 추정이다.

        # 이동거리 1m 당 위치 오차 비율
        self.declare_parameter("odom_dist_error_ratio", 0.15)
        # 회전량 1rad 당 방위 오차 비율 (조향각이 개루프라 크게 잡는다)
        self.declare_parameter("odom_yaw_error_ratio", 0.5)
        # 직진 1m 당 방위 드리프트 [rad] (조향 영점 오차, 토 틀어짐 등)
        self.declare_parameter("odom_yaw_drift_per_m", 0.010)
        # 공분산 상한. 무한히 커지면 필터가 수치적으로 불안정해진다.
        self.declare_parameter("odom_max_pos_var", 25.0)      # m^2  (=5m)
        self.declare_parameter("odom_max_yaw_var", 1.0)       # rad^2 (=1rad)
        # 속도 공분산: 비율 + 바닥값
        self.declare_parameter("odom_vel_error_ratio", 0.15)
        self.declare_parameter("odom_vel_error_floor", 0.02)  # m/s

        # ---------- 조향 보정 ----------
        # 엔코더가 앞축(조향축)에 있어 선회 시 뒤축보다 더 돈다.
        # 그래서 d_rear = d_front x cos(delta) 로 환산한다.
        #
        # ★ 문제: delta 는 측정값이 아니라 시간 기반 추정이다.
        #   steer_ms 가 실제와 다르면 이 보정이 오히려 거리를 왜곡한다.
        #   예) 직진 중인데 steer_ms 가 440(27도)로 남아 있으면
        #       cos(27도)=0.891 → 거리가 11% 짧게 나온다.
        #
        # 거리가 실제와 안 맞을 때 false 로 두고 다시 재보면
        # 조향 보정이 원인인지 바로 갈린다.
        # AS5600 으로 실제 조향각을 재게 되면 true 가 정답이다.
        self.declare_parameter("odom_steer_compensation", True)

        gp = lambda name: self.get_parameter(name).value
        # port: "auto" 면 USB 장치 정보로 아두이노를 직접 찾는다.
        #       고정하려면 "/dev/ttyACM1" 처럼 경로를 직접 넣으면 된다.
        port_param = str(gp("port")).strip()
        self.port_symlinks = [str(v) for v in (gp("port_symlinks") or [])]
        self.fallback_port = str(gp("fallback_port")).strip()
        if port_param.lower() in ("auto", "", "none"):
            self.auto_port = True
            found = find_arduino_port(self.get_logger(), self.port_symlinks)
            if found:
                self.port = found
                self.get_logger().info("포트 자동 탐색: %s" % found)
            elif self.fallback_port:
                self.port = self.fallback_port
                self.get_logger().warn(
                    "아두이노를 못 찾았다. fallback_port=%s 로 시도한다."
                    % self.fallback_port)
            else:
                self.port = ""
                self.get_logger().warn(
                    "아두이노를 못 찾았다. 연결될 때까지 계속 재탐색한다. "
                    "(./setup.sh 를 한 번 실행하면 /dev/t870_mcu 로 고정된다)")
        else:
            self.port = port_param
            self.auto_port = False
        self.baud = int(gp("baud"))
        send_hz = float(gp("send_hz"))
        if send_hz <= 0.0:
            raise ValueError("send_hz must be > 0")
        self.send_period = 1.0 / send_hz
        self.drive_timeout = float(gp("drive_timeout_s"))
        self.wheel_timeout = float(gp("wheel_timeout_s"))
        status_hz = float(gp("status_poll_hz"))
        self.status_period = 1.0 / status_hz if status_hz > 0 else 0.0
        self.max_deg = float(gp("max_steer_deg"))
        self.steer_limit_ms = int(gp("steer_limit_ms"))
        self.steer_mode = str(gp("steer_cmd_mode")).upper()
        self.wheel_timeout_policy = str(gp("wheel_timeout_policy")).lower()
        self.steer_offset = float(gp("steer_offset_deg"))
        self.latch_on_firmware_fault = bool(gp("latch_on_firmware_fault"))
        self.estop_assert_timeout = float(gp("estop_assert_timeout_s"))
        self.reconnect_delay = float(gp("reconnect_delay_s"))
        self.reset_wait = float(gp("arduino_reset_wait_s"))
        self.center_on_connect = bool(gp("center_steer_on_connect"))
        self.telemetry_timeout = float(gp("telemetry_timeout_s"))
        self.coasting_policy = str(gp("coasting_policy")).lower()
        self.estop_cmd = str(gp("estop_serial_command")).strip()
        self.estop_hold_cmd = str(gp("estop_hold_command")).strip()
        self.estop_repeat_s = float(gp("estop_repeat_s"))
        self.stop_timeout = float(gp("stop_timeout_s"))
        self.cpm = float(gp("counts_per_meter"))
        self.wheelbase = float(gp("wheelbase_m"))
        self.encoder_signed = bool(gp("encoder_signed"))
        self.enc_max_cps = float(gp("encoder_max_counts_per_s"))
        self.odom_frame = str(gp("odom_frame"))
        self.base_frame = str(gp("base_frame"))
        self.publish_tf = bool(gp("publish_tf"))
        self.odom_dist_ratio = float(gp("odom_dist_error_ratio"))
        self.odom_yaw_ratio = float(gp("odom_yaw_error_ratio"))
        self.odom_yaw_drift = float(gp("odom_yaw_drift_per_m"))
        self.odom_max_pos_var = float(gp("odom_max_pos_var"))
        self.odom_max_yaw_var = float(gp("odom_max_yaw_var"))
        self.odom_vel_ratio = float(gp("odom_vel_error_ratio"))
        self.odom_vel_floor = float(gp("odom_vel_error_floor"))
        self.odom_steer_comp = bool(gp("odom_steer_compensation"))
        self._steer_range_warned = False

        if self.wheel_timeout_policy not in ("hold_last", "center"):
            raise ValueError("wheel_timeout_policy must be hold_last or center")
        if self.coasting_policy not in ("last_direction", "drop"):
            raise ValueError("coasting_policy must be last_direction or drop")
        if not self.estop_hold_cmd:
            raise ValueError("estop_hold_command 는 비울 수 없다 (워치독 급식용)")
        if 0.0 < self.estop_repeat_s < BRAKE_PULSE_GUARD_S:
            raise ValueError(
                "estop_repeat_s 가 %.2f 초보다 작으면 제동 펄스가 끝나지 않아 "
                "역주행한다. 0(재시도 없음) 또는 %.2f 이상으로 둘 것."
                % (BRAKE_PULSE_GUARD_S, BRAKE_PULSE_GUARD_S))
        if self.max_deg <= 0:
            raise ValueError("max_steer_deg must be > 0")
        self.ms_per_deg = self.steer_limit_ms / self.max_deg

        # ---------- 연결 상태 ----------
        self.ser = None
        self.ser_lock = threading.Lock()
        self.conn_state = CONN_CLOSED
        self._last_connect_attempt = 0.0
        self._port_opened_at = 0.0

        # ---------- 명령 상태 ----------
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

        # ---------- 안전 래치 ----------
        self.hard_stop = False          # /mcu_stop 최신값
        self.hard_stop_rx = 0.0         # 마지막 /mcu_stop 수신 시각
        self.hard_stop_active = False   # 실제 적용 중인지
        self.ros_estop_asserted = False
        self.ros_estop_rx = 0.0         # 마지막 /estop_lock 수신 시각
        self.estop_flip_count = 0       # true<->false 전환 횟수 (플래핑 감지)
        self.estop_flip_window = 0.0
        self.estop_latched = False
        self._brake_pulse_at = None     # 마지막 제동 송신 시각 (None = 미발사)
        self.firmware_fault = 0
        self.firmware_fault_text = ""

        # ---------- 텔레메트리 ----------
        self.last_status_rx = 0.0
        self.telemetry_ok = False
        self._telemetry_warned = False
        # 펌웨어 메시지 로그 폭주 방지 (초당 상한)
        self._fw_msg_window = 0.0
        self._fw_msg_count = 0

        # ---------- 오도메트리 ----------
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.distance_m = 0.0
        self.prev_count = None
        self.prev_count_t = None
        self.last_motion_dir = 0      # 코스팅 시 사용할 직전 진행 방향
        # 누적 공분산 (리셋 이후 이동하면서 커진다)
        self.var_x = 0.0
        self.var_y = 0.0
        self.var_yaw = 0.0
        self._tf_warned = False       # TF 실패 경고를 한 번만
        self._odom_warned = False     # odom 실패 경고도 한 번만

        # 🔴 엔코더 누적값 검증 상태 (0829)
        self._enc_last_good = None
        self._enc_last_good_t = time.monotonic()
        self._enc_bad = 0
        self._enc_warn_at = 0.0
        self._last_rx_error = ""      # 같은 수신 오류를 반복해 찍지 않기 위해
        self._last_fw_state = ""      # 펌웨어 상태 전환 감지용

        # ---------- ROS I/O ----------
        self.declare_parameter("input_drive_topic", "/mcu/cmd_drive")
        self.declare_parameter("input_wheel_topic", "/mcu/cmd_wheel")
        self.declare_parameter("input_stop_topic", "/mcu/cmd_stop")
        self.in_drive_topic = str(self.get_parameter("input_drive_topic").value)
        self.in_wheel_topic = str(self.get_parameter("input_wheel_topic").value)
        _st = str(self.get_parameter("input_stop_topic").value)
        self.sub_drive = self.create_subscription(
            Float32, self.in_drive_topic, self.cb_drive, 10)
        self.sub_wheel = self.create_subscription(
            Int32, self.in_wheel_topic, self.cb_wheel, 10)
        self.declare_parameter("estop_topic", "/estop_lock")
        self._estop_topic_name = str(self.get_parameter("estop_topic").value)
        self.sub_estop = self.create_subscription(
            Bool, self._estop_topic_name, self.cb_estop, 10)
        self.sub_stop = self.create_subscription(Bool, _st, self.cb_stop, 10)

        #  타입·QoS 불일치는 에러 없이 메시지를 삼킨다. 주기적으로 확인한다.
        self._sub_specs = [
            (self.in_drive_topic, "std_msgs/msg/Float32", "구동 명령"),
            (self.in_wheel_topic, "std_msgs/msg/Int32", "조향 명령"),
            (_st, "std_msgs/msg/Bool", "급정거"),
            (self._estop_topic_name, "std_msgs/msg/Bool", "비상정지"),
        ]
        self._diag_seen = set()
        self.diag_timer = self.create_timer(
            3.0,
            lambda: check_subscriptions(self, self._sub_specs, self._diag_seen))

        # ---------- 발행 토픽 (전부 파라미터) ----------
        # ★ 기본값을 전부 /mcu/ 네임스페이스로 옮겼다.
        #   예전 기본값은 /rpm, /inpulse, /steer_angle 처럼 최상위 이름이라
        #   yaml 없이 노드만 띄우면 다른 팀 토픽과 이름이 겹쳤다.
        #   팀에 배포하는 코드의 기본값이 남의 이름을 점유하면 안 된다.
        #   (/odom 만 예외 — Nav2 표준 이름이라 그대로 둔다)
        pubdefs = [
            ("pub_topic_encoder",     "/mcu/encoder"),
            ("pub_topic_steer_angle", "/mcu/steer_deg"),
            ("pub_topic_rpm",         "/mcu/rpm"),
            ("pub_topic_steer_a0",    "/mcu/steer_a0"),
            ("pub_topic_steer_ms",    "/mcu/steer_ms"),
            ("pub_topic_speed_mps",   "/mcu/speed_mps"),
            ("pub_topic_speed_kph",   "/mcu/speed_kph"),
            ("pub_topic_distance",    "/mcu/distance_m"),
            ("pub_topic_speed_valid", "/mcu/speed_valid"),
            ("pub_topic_connected",   "/mcu/connected"),
            ("pub_topic_status",      "/mcu/fw_state"),
            ("pub_topic_raw_status",  "/mcu/raw_status"),
            ("pub_topic_fw_message",  "/mcu/fw_message"),
            ("pub_topic_feedback_valid", "/mcu/telemetry_ok"),
            ("pub_topic_fault",       "/mcu/fault"),
            ("pub_topic_fault_text",  "/mcu/fault_text"),
            ("pub_topic_ready",       "/mcu/ready"),
            ("pub_topic_iface_status","/mcu/iface_state"),
            ("pub_topic_estop",       "/mcu/estop_latched"),
            ("pub_topic_hard_stop",   "/mcu/hard_stop_active"),
            ("pub_topic_odom",        "/odom"),
        ]
        for name, default in pubdefs:
            self.declare_parameter(name, default)
        pt = lambda n: str(self.get_parameter(n).value)

        # 구버전 이름(/drive, /wheel, /arduino/telemetry_ok)으로도 같이 발행할지.
        # ★ 기본 false. 최상위 이름이라 다른 팀과 충돌할 수 있다.
        self.declare_parameter("publish_legacy_names", False)
        self.legacy = bool(self.get_parameter("publish_legacy_names").value)

        self.pub_conn = self.create_publisher(Bool, pt("pub_topic_connected"), 10)
        self.pub_status = self.create_publisher(String, pt("pub_topic_status"), 10)
        self.pub_raw = self.create_publisher(String, pt("pub_topic_raw_status"), 10)
        # STATUS 가 아닌 펌웨어 출력 (MCU_BOOT, BRAKE_OK, BRAKE_DONE, CAL_* ...)
        self.pub_fw_msg = self.create_publisher(String, pt("pub_topic_fw_message"), 10)
        self.pub_fault = self.create_publisher(Int32, pt("pub_topic_fault"), 10)
        self.pub_fault_text = self.create_publisher(String, pt("pub_topic_fault_text"), 10)
        self.pub_tele_ok = self.create_publisher(
            Bool, pt("pub_topic_feedback_valid"), 10)
        self.pub_drive = self.create_publisher(Int32, pt("pub_topic_encoder"), 10)
        self.pub_wheel = self.create_publisher(Float32, pt("pub_topic_steer_angle"), 10)
        self.pub_rpm = self.create_publisher(Float32, pt("pub_topic_rpm"), 10)
        self.pub_a0 = self.create_publisher(Int32, pt("pub_topic_steer_a0"), 10)
        self.pub_steer_ms = self.create_publisher(Int32, pt("pub_topic_steer_ms"), 10)
        self.pub_estop = self.create_publisher(Bool, pt("pub_topic_estop"), 10)
        self.pub_hard_stop = self.create_publisher(Bool, pt("pub_topic_hard_stop"), 10)
        self.pub_speed = self.create_publisher(Float32, pt("pub_topic_speed_mps"), 10)
        self.pub_speed_kph = self.create_publisher(Float32, pt("pub_topic_speed_kph"), 10)
        self.pub_distance = self.create_publisher(Float32, pt("pub_topic_distance"), 10)
        self.pub_speed_valid = self.create_publisher(
            Bool, pt("pub_topic_speed_valid"), 10)
        self.pub_iface_ready = self.create_publisher(Bool, pt("pub_topic_ready"), 10)
        self.pub_iface_status = self.create_publisher(
            String, pt("pub_topic_iface_status"), 10)

        # 구버전 호환 (우리 문서·도구에서 쓰던 이름)
        if self.legacy:
            self.pub_drive_legacy = self.create_publisher(Int32, "/drive", 10)
            self.pub_wheel_legacy = self.create_publisher(Float32, "/wheel", 10)
            self.pub_tele_legacy = self.create_publisher(
                Bool, "/arduino/telemetry_ok", 10)
        else:
            self.pub_drive_legacy = None
            self.pub_wheel_legacy = None
            self.pub_tele_legacy = None
        self.pub_odom = self.create_publisher(Odometry, pt("pub_topic_odom"), 10)
        self.tf_bc = TransformBroadcaster(self) if (TF_OK and self.publish_tf) else None

        self.declare_parameter("reset_estop_service", "/mcu/reset_estop")
        self._reset_service_name = str(
            self.get_parameter("reset_estop_service").value)
        self.reset_srv = self.create_service(
            Trigger, self._reset_service_name, self.cb_reset_estop)

        # 오도메트리 원점 재설정. 구간 시작마다 0 으로 맞추고 싶을 때 쓴다.
        #   ros2 service call /mcu/reset_odom std_srvs/srv/Trigger '{}'
        self.declare_parameter("reset_odom_service", "/mcu/reset_odom")
        _os = str(self.get_parameter("reset_odom_service").value)
        self.reset_odom_srv = self.create_service(
            Trigger, _os, self.cb_reset_odom)

        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()
        self.tx_timer = self.create_timer(self.send_period, self.tx_tick)
        self.conn_timer = self.create_timer(0.2, self.conn_tick)

        self.get_logger().info(
            "mcu_bridge v3: %s @ %d, send=%.1fHz, drive_timeout=%.2fs, "
            "wheel_timeout=%.2fs, 1deg=%.2fms, 직진보정=%+.1f도, cpm=%s, "
            "급정거=%s(1회) → %s(반복), 재시도=%s"
            % (self.port or "탐색 중", self.baud, send_hz, self.drive_timeout,
               self.wheel_timeout, self.ms_per_deg, self.steer_offset,
               ("%.2f" % self.cpm) if self.cpm > 0 else "미측정(odom 비활성)",
               self.estop_cmd, self.estop_hold_cmd,
               ("%.2fs" % self.estop_repeat_s) if self.estop_repeat_s > 0 else "없음")
        )
        if self.cpm <= 0:
            self.get_logger().warn(
                "counts_per_meter=0 → /odom, /vehicle/speed_mps, "
                "/vehicle/distance_m 비활성. 실측 후 설정하세요.")

    # ============================================================
    # 구독 콜백
    # ============================================================

    def cb_drive(self, msg: Float32):
        stage = parse_drive_stage(float(msg.data))
        now = time.monotonic()
        if stage is None:
            self.get_logger().error(
                "invalid drive=%r (%s); 허용값 -1,0,1,2,3. 거부하고 정지."
                % (msg.data, self.in_drive_topic))
            self.cmd_stage = 0
            self.have_drive = True
            self.last_drive_rx = now
            return
        self.cmd_stage = stage
        self.have_drive = True
        self.last_drive_rx = now
        self.drive_timeout_warned = False

    def cb_wheel(self, msg: Int32):
        deg = int(msg.data)
        if not valid_wheel_deg(deg, self.max_deg):
            self.get_logger().error(
                "invalid wheel=%d (%s); 허용범위 ±%d. 거부."
                % (deg, self.in_wheel_topic, int(self.max_deg)))
            return
        if deg != self.cmd_deg:
            self.wheel_dirty = True
        self.cmd_deg = deg
        self.have_wheel = True
        self.last_wheel_rx = time.monotonic()
        self.wheel_timeout_warned = False

    def cb_stop(self, msg: Bool):
        """급정거. E-stop 과 달리 래치하지 않는다.

        장애물이 치워지면 다시 출발해야 하므로, 요청이 사라지면 자동 복귀.
        """
        was = self.hard_stop
        self.hard_stop = bool(msg.data)
        self.hard_stop_rx = time.monotonic()
        if self.hard_stop and not was:
            self.get_logger().warn("급정거 요청 수신 → 즉시 정지 (cmd=%s)" % self.estop_cmd)
        elif was and not self.hard_stop:
            self.get_logger().info("급정거 해제")

    def cb_estop(self, msg: Bool):
        now = time.monotonic()
        new_state = bool(msg.data)
        changed = (new_state != self.ros_estop_asserted)

        self.ros_estop_asserted = new_state
        self.ros_estop_rx = now

        # ---- 플래핑 감지 ----
        # true/false 를 빠르게 오가면 발행 노드가 조건을 잘못 짠 것이다.
        # 매니저는 래치를 안 하므로 그대로 깜빡이고, 브릿지는 래치라서
        # 첫 true 에 굳는다. 같은 신호가 두 곳에서 다르게 보여 혼란스럽다.
        if changed:
            if now - self.estop_flip_window > 5.0:
                self.estop_flip_window = now
                self.estop_flip_count = 0
            self.estop_flip_count += 1
            if self.estop_flip_count == 6:
                self.get_logger().error(
                    "%s 가 5초 안에 여러 번 뒤집혔다 — 발행 노드의 조건이 "
                    "잘못됐을 가능성이 높다. 발행자: %s"
                    % (self._estop_topic_name, self._estop_publishers()))

        if msg.data and not self.estop_latched:
            # ★ 누가 걸었는지 남긴다.
            #   /estop_lock 은 한 번만 true 가 와도 영구 래치라서,
            #   "누가 쐈는지" 를 모르면 원인을 절대 못 찾는다.
            #   실제로 팀 통합 시 "E-Stop 이 자꾸 걸린다" 는데 발행자를
            #   특정할 수 없어 시간을 버린 적이 있다.
            self.get_logger().error(
                "ROS E-STOP 래치됨 (%s) — 발행자: %s"
                % (self._estop_topic_name, self._estop_publishers()))
            self.get_logger().error(
                "해제: ros2 service call %s std_srvs/srv/Trigger '{}'"
                % self._reset_service_name)
        if msg.data:
            self.estop_latched = True
        # false 는 의도적으로 래치를 풀지 않는다. /mcu/reset_estop 필요.

    def _estop_publishers(self):
        """/estop_lock 을 발행 중인 노드 이름 목록."""
        try:
            infos = self.get_publishers_info_by_topic(self._estop_topic_name)
        except Exception as exc:
            return "조회 실패(%s)" % exc
        names = []
        for info in infos:
            ns = getattr(info, "node_namespace", "") or ""
            name = getattr(info, "node_name", "?")
            names.append((ns.rstrip("/") + "/" + name) if ns != "/" else "/" + name)
        return ", ".join(sorted(names)) if names else "(발행자 없음 — 이미 종료된 노드)"

    def _ros_estop_live(self):
        """/estop_lock 의 true 가 "지금도 유효한 주장" 인가.

        타임아웃이 지난 true 는 무시한다. 그렇지 않으면 true 를 한 번 쏘고
        죽은 노드 때문에 래치를 영원히 못 푼다.
        """
        if not self.ros_estop_asserted:
            return False
        if self.estop_assert_timeout <= 0.0:
            return True
        return (time.monotonic() - self.ros_estop_rx) <= self.estop_assert_timeout

    def cb_reset_estop(self, _request, response):
        if self._ros_estop_live():
            response.success = False
            response.message = (
                "reset denied: %s 가 아직 true 다. 발행자: %s"
                % (self._estop_topic_name, self._estop_publishers()))
            self.get_logger().warn(response.message)
            return response

        if self.ros_estop_asserted:
            # 값은 true 인데 오래된 것 — 발행 노드가 멈췄거나 죽었다.
            age = time.monotonic() - self.ros_estop_rx
            self.get_logger().warn(
                "%s 의 마지막 값이 true 지만 %.1f초 전 것이라 무시하고 해제한다 "
                "(발행 노드가 멈춘 것으로 본다)"
                % (self._estop_topic_name, age))
            self.ros_estop_asserted = False
        if self.firmware_fault != 0:
            response.success = False
            response.message = "reset denied: Arduino fault=%s" % self.firmware_fault_text
            return response
        self.estop_latched = False
        self._brake_pulse_at = None     # 다음 E-Stop 때 제동을 다시 1회 발사
        self.get_logger().info("E-stop latch cleared")
        response.success = True
        response.message = "E-stop latch cleared; fresh ROS commands are still required"
        return response

    def _sane_encoder(self, value):
        """엔코더 누적값을 검증한다. 못 믿으면 None.

        두 가지를 본다.
          1. 파싱 실패(None)          → 필드가 깨졌다
          2. 물리적으로 불가능한 점프  → 필드가 밀렸거나 두 줄이 붙었다

        둘 다 시리얼 노이즈에서 온다. 특히 제동(역토크) 순간에 몰린다.
        믿을 수 없으면 발행하지 않고 직전 값을 유지한다.
        """
        now = time.monotonic()
        dt = now - self._enc_last_good_t

        ok, why = encoder_sanity(value, self._enc_last_good, dt, self.enc_max_cps)
        if not ok:
            self._enc_bad += 1
            self._warn_encoder(why)
            return None

        self._enc_last_good = value
        self._enc_last_good_t = now
        self._enc_bad = 0
        return value

    def _warn_encoder(self, why):
        """같은 경고로 로그를 도배하지 않는다."""
        now = time.monotonic()
        if now - self._enc_warn_at < 2.0:
            return
        self._enc_warn_at = now
        self.get_logger().warn(
            "%s — 이 STATUS 는 버린다 (연속 %d회). "
            "시리얼 노이즈일 가능성이 크다. 제동 순간에 몰리면 "
            "엔코더 배선을 모터선과 분리할 것." % (why, self._enc_bad))

    def cb_reset_odom(self, _request, response):
        """오도메트리를 원점으로. 누적 공분산도 함께 0 으로 되돌린다."""
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.distance_m = 0.0
        self.var_x = 0.0
        self.var_y = 0.0
        self.var_yaw = 0.0
        self.prev_count = None      # 다음 STATUS 를 새 기준점으로
        self.get_logger().info("오도메트리 원점 재설정 (공분산도 초기화)")
        response.success = True
        response.message = "odometry reset to origin"
        return response

    # ============================================================
    # 연결 (논블로킹 상태머신)
    # ============================================================

    def _try_open(self):
        now = time.monotonic()
        if now - self._last_connect_attempt < self.reconnect_delay:
            return
        self._last_connect_attempt = now

        # 자동 모드면 재연결 때마다 다시 찾는다.
        # USB 를 뽑았다 꽂으면 번호가 바뀔 수 있다.
        if self.auto_port:
            found = find_arduino_port(None, self.port_symlinks)
            if found and found != self.port:
                self.get_logger().info(
                    "포트 확정: %s -> %s" % (self.port or "(없음)", found))
                self.port = found
            elif not found and not self.fallback_port:
                self.port = ""      # 아직 안 꽂혔다. 다음 주기에 다시 찾는다.

        if not self.port:
            return

        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except serial.SerialException as exc:
            msg = str(exc)
            if "busy" in msg.lower() or "denied" in msg.lower():
                self.get_logger().error(
                    "포트 점유/권한: %s | sudo chmod 666 %s | sudo fuser -k %s"
                    % (msg, self.port, self.port))
            else:
                self.get_logger().warn("serial unavailable: %s (재시도 중)" % msg)
            return
        with self.ser_lock:
            self.ser = ser
        self._port_opened_at = now
        self.conn_state = CONN_WAIT_RESET

        # 재연결 정책: 연결 이전 명령은 전부 폐기하고 정지부터 시작
        self.cmd_stage = 0
        self.cmd_deg = 0
        self.have_drive = False
        self.have_wheel = False
        self.wheel_dirty = False
        self.last_drive_rx = 0.0
        self.last_wheel_rx = 0.0
        self.prev_count = None          # 엔코더 기준점 재설정
        self.last_motion_dir = 0
        self._brake_pulse_at = None     # 제동 에지 재무장
        self.get_logger().info(
            "포트 열림: %s — 아두이노 리셋 %.1fs 대기" % (self.port, self.reset_wait))

    def _finish_connect(self):
        self.conn_state = CONN_READY
        self.send_line("1.00")
        if self.center_on_connect:
            cmd = wheel_serial_command(
                self.steer_offset, self.max_deg,
                self.steer_limit_ms, self.steer_mode)
            self.send_line(cmd)
            self.get_logger().info(
                "시리얼 준비 완료: STOP + 조향중앙(%s) 전송, 새 ROS 명령 대기" % cmd)
        else:
            self.get_logger().info(
                "시리얼 준비 완료: STOP 전송, 새 ROS 명령 대기 "
                "(조향은 이전 위치 유지 — center_steer_on_connect=false)")

    def close_serial(self):
        with self.ser_lock:
            ser = self.ser
            self.ser = None
        self.conn_state = CONN_CLOSED
        self.telemetry_ok = False
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
        self.get_logger().error("serial TX 실패; 연결 해제")
        self.close_serial()
        return False

    def conn_tick(self):
        now = time.monotonic()

        if self.conn_state == CONN_CLOSED:
            self._try_open()
        elif self.conn_state == CONN_WAIT_RESET:
            if now - self._port_opened_at >= self.reset_wait:
                self._finish_connect()

        # 텔레메트리 감시: 연결돼 있는데 STATUS 가 안 오면 경고
        if self.telemetry_timeout > 0 and self.conn_state == CONN_READY:
            fresh = (self.last_status_rx > 0.0
                     and now - self.last_status_rx <= self.telemetry_timeout)
            if fresh != self.telemetry_ok:
                if fresh:
                    self.get_logger().info("텔레메트리 복구")
                    self._telemetry_warned = False
                self.telemetry_ok = fresh
            if not fresh and not self._telemetry_warned and self.last_status_rx > 0.0:
                self.get_logger().error(
                    "STATUS 수신 두절 %.1fs — 엔코더/상태 갱신 없음. "
                    "구동 명령은 계속 전송 중이므로 주의." % self.telemetry_timeout)
                self._telemetry_warned = True

        m = Bool(); m.data = (self.conn_state == CONN_READY); self.pub_conn.publish(m)
        m = Bool(); m.data = self.estop_latched; self.pub_estop.publish(m)
        m = Bool(); m.data = self.telemetry_ok; self.pub_tele_ok.publish(m)
        if self.pub_tele_legacy:
            self.pub_tele_legacy.publish(m)

    # ============================================================
    # 송신
    # ============================================================

    def _emit_brake(self, now):
        """급정거 명령 송신 — 제동은 1회, 이후에는 정지 유지만.

        ★ 이 함수가 존재하는 이유
          펌웨어 requestBrake() 는 호출될 때마다 brakeStartMs 를 다시 찍는다.
          updateBrake() 는 (now - brakeStartMs >= BRAKE_PULSE_MS) 일 때만
          펄스를 끝내는데, 브릿지가 매 주기(100ms) B 를 보내면 그 조건이
          영원히 성립하지 않는다. 결과는 PWM 120 역토크가 계속 걸린 상태 —
          즉 급정거를 걸었는데 차가 뒤로 간다.

          그래서 제동 명령은 상승 에지에 한 번만 보내고, 그 뒤에는
          estop_hold_command("1.00")만 반복해 워치독을 먹인다.

          estop_repeat_s > 0 이면 그 간격으로 제동을 재시도한다. 아직
          굴러가고 있을 때를 위한 것이고, 이미 서 있으면 펌웨어가
          BRAKE_OK,ALREADY_STOPPED 로 넘겨 역토크를 걸지 않는다.

          ★ 간격을 좁혀 제동을 강하게 만들고 싶은 유혹이 있는데 하지 말 것.
            제동 방향은 마지막으로 명령된 주행 방향 기준이라, 차가 멈춘 뒤
            뒤로 밀리기 시작하면 다음 펄스가 그 후진을 가속한다.
        """
        if self._brake_pulse_at is None:
            fire = True                                   # 상승 에지 — 첫 발
        elif self.estop_repeat_s > 0.0:
            fire = (now - self._brake_pulse_at) >= self.estop_repeat_s
        else:
            fire = False

        if fire:
            if self.send_line(self.estop_cmd):
                self._brake_pulse_at = now
            return

        self.send_line(self.estop_hold_cmd)

    def tx_tick(self):
        if self.conn_state != CONN_READY:
            return
        now = time.monotonic()

        drive_fresh = self.have_drive and (now - self.last_drive_rx <= self.drive_timeout)
        wheel_fresh = self.have_wheel and (now - self.last_wheel_rx <= self.wheel_timeout)

        # 급정거는 최우선. 오래된 true 는 해제로 본다(장애물이 치워졌을 수 있음).
        stop_fresh = self.hard_stop and (now - self.hard_stop_rx <= self.stop_timeout)
        if stop_fresh != self.hard_stop_active:
            self.hard_stop_active = stop_fresh
        if stop_fresh:
            self._emit_brake(now)
            m = Bool(); m.data = True; self.pub_hard_stop.publish(m)
            if self.status_period > 0 and now - self.last_status_req >= self.status_period:
                if self.send_line("S"):
                    self.last_status_req = now
            return

        m = Bool(); m.data = False; self.pub_hard_stop.publish(m)

        if self.estop_latched:
            # E-Stop 은 물리 비상이다. 급정거와 같은 경로로 제동을 쓴다.
            # (예전에는 감속 램프 정지만 나갔다)
            self._emit_brake(now)
        else:
            # 급정거·래치가 모두 없으면 다음 급정거를 위해 에지를 재무장한다.
            self._brake_pulse_at = None

            if drive_fresh:
                stage_to_send = self.cmd_stage
            else:
                stage_to_send = 0
                if self.have_drive and not self.drive_timeout_warned:
                    self.get_logger().warn(
                        "구동 명령 타임아웃 → 정지 (조향은 영향 없음)")
                    self.drive_timeout_warned = True

            # 펌웨어 워치독(2초) 급식. 매 주기 전송.
            self.send_line(drive_serial_command(stage_to_send))

        # 조향 타임아웃은 구동을 절대 멈추지 않는다.
        if self.have_wheel and not wheel_fresh and not self.wheel_timeout_warned:
            self.get_logger().warn(
                "조향 명령 타임아웃 → policy=%s; 구동은 독립적으로 계속"
                % self.wheel_timeout_policy)
            self.wheel_timeout_warned = True
            if self.wheel_timeout_policy == "center" and self.cmd_deg != 0:
                self.cmd_deg = 0
                self.wheel_dirty = True

        # E-stop 중에는 조향을 현재 위치에 동결한다 (자동 복귀 없음).
        if not self.estop_latched and self.wheel_dirty:
            # 직진 보정을 더한 뒤 물리 한계로 클램프한다.
            deg = self.cmd_deg + self.steer_offset
            deg = max(-self.max_deg, min(self.max_deg, deg))
            cmd = wheel_serial_command(
                deg, self.max_deg, self.steer_limit_ms, self.steer_mode)
            if self.send_line(cmd):
                self.wheel_dirty = False

        if self.status_period > 0 and now - self.last_status_req >= self.status_period:
            if self.send_line("S"):
                self.last_status_req = now

    # ============================================================
    # 수신
    # ============================================================

    def rx_loop(self):
        buf = b""
        while rclpy.ok():
            with self.ser_lock:
                ser = self.ser
            if ser is None:
                buf = b""
                time.sleep(0.1)
                continue
            try:
                chunk = ser.read(256)
            except serial.SerialException:
                self.close_serial()
                time.sleep(0.1)
                continue
            except Exception:
                time.sleep(0.1)
                continue
            if not chunk:
                continue
            buf += chunk

            # 개행 없는 쓰레기가 계속 들어오는 경우 메모리 방어
            if len(buf) > MAX_RX_BUF:
                buf = buf[-1024:]

            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                try:
                    self.handle_line(raw.decode("utf-8", "replace").strip())
                except Exception as exc:
                    # 같은 오류가 초당 5회씩 쏟아지면 정작 봐야 할
                    # [MCU] 메시지가 묻힌다. 같은 내용은 한 번만 찍는다.
                    text = "%s: %s" % (type(exc).__name__, exc)
                    if text != self._last_rx_error:
                        self._last_rx_error = text
                        self.get_logger().error("수신 처리 오류: %s" % text)

    def handle_line(self, line: str):
        status = parse_status(line)
        if status is None:
            # ★ 예전에는 여기서 그냥 버렸다.
            #   펌웨어가 내보내는 MCU_BOOT, BRAKE_OK, BRAKE_DONE, CAL_*,
            #   FAULT 안내 등이 전부 조용히 사라졌다. 그래서 ROS 로 띄운
            #   상태에서는 제동이 어떻게 끝났는지 볼 방법이 없었다.
            #   (시리얼 포트는 한 프로세스만 쓸 수 있어 모니터도 못 붙인다)
            self._emit_fw_message(line)
            return

        self.last_status_rx = time.monotonic()

        m = String(); m.data = status["raw"]; self.pub_raw.publish(m)
        m = String(); m.data = status["state"]; self.pub_status.publish(m)
        m = Int32(); m.data = status["fault"]; self.pub_fault.publish(m)
        m = String(); m.data = status["fault_text"]; self.pub_fault_text.publish(m)
        m = Int32(); m.data = status["adc"]; self.pub_a0.publish(m)
        m = Float32(); m.data = status["rpm"]; self.pub_rpm.publish(m)
        #  🔴 엔코더 누적값은 깨졌으면 발행하지 않는다 (0829)
        #    예전에는 못 읽으면 0 이 나갔다. 누적 카운터에 0 이 튀면
        #    구독자 쪽에서는 "주행 중 갑자기 원점으로 초기화" 로 보인다.
        #    제동 순간 역토크 펄스가 시리얼에 노이즈를 실어 정확히 그때 났다.
        enc = self._sane_encoder(status["encoder_count"])
        if enc is not None:
            m = Int32(); m.data = enc; self.pub_drive.publish(m)
            if self.pub_drive_legacy:
                self.pub_drive_legacy.publish(m)
        m = Int32(); m.data = status["steer_ms"]; self.pub_steer_ms.publish(m)

        steer_deg = status["steer_ms"] / self.ms_per_deg
        m = Float32(); m.data = float(round(steer_deg, 1)); self.pub_wheel.publish(m)
        if self.pub_wheel_legacy:
            self.pub_wheel_legacy.publish(m)

        # ★ 물리 E-Stop(D24) 은 펌웨어 state 만 ESTOP 으로 바꾸고
        #   fault 필드는 NONE 이라 브릿지가 조용히 지나쳤다.
        #   "차가 안 움직이는데 아무 메시지도 없다" 가 되므로 여기서 알린다.
        #   NC 접점이라 선이 빠지거나 진동으로 순간 끊겨도 눌린 것과 같다.
        if status["state"] != self._last_fw_state:
            if status["state"] == "ESTOP":
                self.get_logger().error(
                    "펌웨어 상태 ESTOP — 물리 E-Stop 핀(D24)이 HIGH. "
                    "버튼이 눌렸거나 배선이 끊겼다(NC 접점이라 단선도 같다). "
                    "복구 후 시리얼로 RESET.")
            elif self._last_fw_state == "ESTOP":
                self.get_logger().info(
                    "펌웨어 E-Stop 해제 → %s" % status["state"])
            self._last_fw_state = status["state"]

        # 차량 인터페이스 상태 (카메라팀 규약)
        r = Bool()
        r.data = (self.conn_state == CONN_READY and not self.estop_latched
                  and status["fault"] == 0)
        self.pub_iface_ready.publish(r)
        s2 = String(); s2.data = status["state"]; self.pub_iface_status.publish(s2)

        # ★ 안전 처리를 오도메트리보다 먼저 한다.
        #   예전에는 update_odom 이 먼저였는데, 거기서 예외가 나면 아래
        #   fault 감지가 통째로 건너뛰어졌다. TF 오타 때문에 실제로 매
        #   STATUS 마다 그 일이 벌어지고 있었다.
        #   안전에 관한 것은 무슨 일이 있어도 먼저 처리한다.
        if self.firmware_fault != status["fault"]:
            if status["fault"] != 0:
                self.get_logger().error(
                    "Arduino fault=%s (state=%s)"
                    % (status["fault_text"], status["state"]))
            else:
                self.get_logger().info("Arduino fault 해소: %s" % status["fault_text"])
        self.firmware_fault = status["fault"]
        self.firmware_fault_text = status["fault_text"]

        if self.latch_on_firmware_fault and self.firmware_fault != 0:
            if not self.estop_latched:
                self.get_logger().error(
                    "Arduino fault=%s: E-stop 래치" % self.firmware_fault_text)
            self.estop_latched = True

        # 오도메트리는 마지막. 여기서 문제가 나도 안전·구동에는 영향이 없다.
        try:
            if enc is not None:
                self.update_odom(enc, status["steer_ms"])
        except Exception as exc:
            if not self._odom_warned:
                self._odom_warned = True
                self.get_logger().error(
                    "오도메트리 계산 실패: %s — odom 없이 계속한다. "
                    "구동·안전은 영향 없다." % exc)

    def _emit_fw_message(self, line: str):
        """STATUS 가 아닌 펌웨어 출력을 토픽으로 내보내고 로그에 찍는다.

        MCU_BOOT,v33 / BRAKE_OK,MAX300 / BRAKE_DONE,STOPPED,160 같은 것들.
        현장에서 이것 없이는 제동이 어떻게 끝났는지 알 수 없다.

        펌웨어가 무언가를 폭주 출력해도 로그가 묻히지 않도록 초당 상한을 둔다.
        토픽 발행은 막지 않는다 (ros2 topic echo 로는 다 보인다).
        """
        text = line.strip()
        if not text:
            return

        m = String(); m.data = text; self.pub_fw_msg.publish(m)

        now = time.monotonic()
        if now - self._fw_msg_window >= 1.0:
            self._fw_msg_window = now
            self._fw_msg_count = 0
        self._fw_msg_count += 1
        if self._fw_msg_count <= 20:
            self.get_logger().info("[MCU] %s" % text)
        elif self._fw_msg_count == 21:
            self.get_logger().warn(
                "[MCU] 출력이 초당 20줄을 넘었다. 로그를 잠시 줄인다 "
                "(전체는 /mcu/fw_message 에서 볼 수 있다)")

    # ============================================================
    # 오도메트리
    # ============================================================

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

        # 부호 결정. 펌웨어 카운트가 무부호면 진행 방향을 브릿지가 붙인다.
        if not self.encoder_signed:
            if self.cmd_stage > 0:
                self.last_motion_dir = 1
                delta = abs(delta)
            elif self.cmd_stage < 0:
                self.last_motion_dir = -1
                delta = -abs(delta)
            else:
                # 정지 명령 상태. 브레이크가 없어 실제로는 코스팅 중일 수 있다.
                if self.coasting_policy == "last_direction" and self.last_motion_dir != 0:
                    delta = abs(delta) * self.last_motion_dir
                else:
                    delta = 0
        else:
            if delta > 0:
                self.last_motion_dir = 1
            elif delta < 0:
                self.last_motion_dir = -1

        steer_deg = steer_ms / self.ms_per_deg
        steer_rad = math.radians(steer_deg)

        # 조향각 추정이 물리적으로 불가능한 값이면 알린다.
        # 시간 기반이라 어긋나면 계속 어긋난 채로 남는다.
        if abs(steer_deg) > self.max_deg * 1.05 and not self._steer_range_warned:
            self._steer_range_warned = True
            self.get_logger().warn(
                "조향각 추정치가 한계를 넘었다: %.1f도 (한계 ±%.0f도, steer_ms=%d). "
                "시간 기반 추정이 어긋난 상태다. C(중앙 복귀) 후 다시 볼 것."
                % (steer_deg, self.max_deg, steer_ms))

        # 엔코더가 앞축(조향축)에 있으므로 뒤축 이동거리로 환산: d_rear = d_front·cos(δ)
        # odom_steer_compensation=false 면 환산하지 않는다 (조향각을 못 믿을 때).
        d_front = delta / self.cpm
        d = d_front * math.cos(steer_rad) if self.odom_steer_comp else d_front
        speed = d / dt
        self.distance_m += abs(d)

        dtheta = d * math.tan(steer_rad) / self.wheelbase
        if abs(dtheta) > 1e-9:
            radius = d / dtheta
            self.x += radius * (math.sin(self.th + dtheta) - math.sin(self.th))
            self.y -= radius * (math.cos(self.th + dtheta) - math.cos(self.th))
        else:
            self.x += d * math.cos(self.th)
            self.y += d * math.sin(self.th)
        self.th = math.atan2(math.sin(self.th + dtheta), math.cos(self.th + dtheta))

        m = Float32(); m.data = float(speed); self.pub_speed.publish(m)
        m = Float32(); m.data = float(speed * 3.6); self.pub_speed_kph.publish(m)
        m = Float32(); m.data = float(self.distance_m); self.pub_distance.publish(m)

        # ---- 공분산 누적 ----
        #
        #  위치 오차는 이동거리에 비례해 쌓인다 (counts_per_meter 편차).
        #  방위 오차는 두 갈래로 쌓인다.
        #    1) 회전할 때 — 조향각을 측정하지 못해 회전량 자체가 부정확
        #    2) 직진할 때 — 조향 영점 오차, 토 틀어짐으로 조금씩 휜다
        #  방위가 틀어지면 그 뒤의 위치 오차가 거리에 비례해 커지므로,
        #  방위 분산을 위치 분산에도 반영한다.
        sigma_d = self.odom_dist_ratio * abs(d)
        sigma_th = (self.odom_yaw_ratio * abs(dtheta)
                    + self.odom_yaw_drift * abs(d))

        self.var_yaw = min(self.var_yaw + sigma_th * sigma_th,
                           self.odom_max_yaw_var)
        # 순수 거리 오차만 누적한다 (counts_per_meter 편차)
        pos_var = min(self.var_x + sigma_d * sigma_d, self.odom_max_pos_var)
        self.var_x = pos_var
        self.var_y = pos_var

        # 실제 위치 오차는 방위 오차가 지배한다.
        # 방위가 sigma_yaw 만큼 틀어진 채로 distance_m 를 갔다면
        # 횡방향으로 대략 (distance_m x sigma_yaw) 만큼 벌어진다.
        # 이건 누적값이 아니라 현재 상태에서 매번 계산한다.
        lateral = self.distance_m * math.sqrt(self.var_yaw)
        pos_var_with_yaw = min(pos_var + lateral * lateral,
                               self.odom_max_pos_var)

        sigma_v = self.odom_vel_ratio * abs(speed) + self.odom_vel_floor
        # 각속도도 조향각 추정에 의존하므로 같은 비율로 본다
        sigma_w = (self.odom_yaw_ratio
                   * abs(speed * math.tan(steer_rad) / self.wheelbase)
                   + self.odom_vel_floor)

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

        # 6x6 행렬, 행 우선. 순서는 x y z roll pitch yaw.
        # 2D 차량이라 z/roll/pitch 는 쓰지 않으므로 큰 값을 넣어
        # 융합 필터가 그 축을 무시하도록 한다.
        big = 1e6
        pose_cov = [0.0] * 36
        pose_cov[0]  = max(pos_var_with_yaw, 1e-4)   # x
        pose_cov[7]  = max(pos_var_with_yaw, 1e-4)   # y
        pose_cov[14] = big                            # z
        pose_cov[21] = big                            # roll
        pose_cov[28] = big                            # pitch
        pose_cov[35] = max(self.var_yaw, 1e-5)        # yaw
        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0]  = sigma_v * sigma_v             # vx
        twist_cov[7]  = big                            # vy (횡슬립은 모델에 없음)
        twist_cov[14] = big
        twist_cov[21] = big
        twist_cov[28] = big
        twist_cov[35] = sigma_w * sigma_w             # wz
        odom.twist.covariance = twist_cov

        self.pub_odom.publish(odom)

        if self.tf_bc:
            # ★ 0829 수정: send_transform (X) → sendTransform (O)
            #   tf2_ros 파이썬 API 는 카멜케이스다. 이 오타 하나 때문에
            #   매 STATUS 마다 예외가 나면서 둘이 같이 죽어 있었다.
            #     1) TF 가 한 번도 발행되지 않음
            #     2) handle_line 에서 이 뒤에 있던 펌웨어 fault 감지가 건너뛰어짐
            #        → 아두이노가 FAULT 를 내도 E-Stop 래치가 안 걸렸다
            #   TF 실패가 다른 처리를 막지 않도록 여기서 따로 잡는다.
            try:
                tf = TransformStamped()
                tf.header = odom.header
                tf.child_frame_id = self.base_frame
                tf.transform.translation.x = self.x
                tf.transform.translation.y = self.y
                tf.transform.rotation.z = odom.pose.pose.orientation.z
                tf.transform.rotation.w = odom.pose.pose.orientation.w
                self.tf_bc.sendTransform(tf)
            except Exception as exc:
                if not self._tf_warned:
                    self._tf_warned = True
                    self.get_logger().error(
                        "TF 발행 실패: %s — TF 없이 계속한다 "
                        "(/odom 토픽은 정상)" % exc)

    # ============================================================

    def shutdown(self):
        try:
            self.send_line("1.00")
            time.sleep(0.05)
        finally:
            self.close_serial()


def main(args=None):
    rclpy.init(args=args)
    node = McuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
