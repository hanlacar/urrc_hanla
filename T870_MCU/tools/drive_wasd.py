#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
drive_wasd_v1_0826.py  —  T870 키보드 수동 조종 (WASD)

브릿지·중재기가 떠 있는 상태에서 /manual_drive, /manual_wheel, /manual_stop
으로 발행한다. manual 소스는 모드와 무관하게 최우선이라 언제든 개입된다.

실행
----
    터미널 1 (브릿지, 계속 켜둠)
        cd <워크스페이스> && source install/setup.bash
        ros2 launch t870_mcu t870_mcu.launch.py

    터미널 2 (이 도구)
        source <워크스페이스>/install/setup.bash
        python3 tools/drive_wasd.py

키
--
    W  전진 (단계 올림: 정지 -> 1 -> 2 -> 3)
    S  후진 (한 번 더 누르면 정지)
    A  조향 좌 3도
    D  조향 우 3도
    C  조향 중앙

    0      ★ 조종권 토글 (대기 <-> 키보드 전용)
    Space  정지
    F      급정거 토글 (드라이브 0 과 달리 램프를 건너뛴다)
    E      E-Stop 토글
    R      E-Stop 래치 해제 (서비스 호출까지)

조종권 (0 키)
-------------
    대기(STANDBY)   카메라/라이다/GPS 가 차를 몬다. 이 도구는 보고만 있는다.
                    구동·조향을 발행하지 않는다.
    조종(TAKEOVER)  키보드가 차를 몬다. 팀 명령은 전부 무시된다.
                    manual 소스가 모드·우선순위와 무관하게 최우선이기 때문.

    0 을 누르면 전환된다. 처음 진입할 때는 정지 + 조향 중앙에서 시작한다.
    조종 중에 0 을 누르면 먼저 정지만 하고, 한 번 더 눌러야 팀에게 넘긴다
    (달리는 상태로 넘기지 않기 위해).

    ⚠ F(급정거)와 E(E-Stop)는 대기 중에도 항상 동작한다. 안전요원 역할.

    ※ 이전 버전은 실행하는 순간부터 무조건 조종 상태였다. 팀 주행을 지켜보다
      필요할 때만 잡는 것이 불가능했다.

    Z  구간 측정 시작
    X  구간 측정 끝 (결과 출력)
    Q  종료 (정지 명령 보내고 나감)

주의
----
    브릿지가 시리얼 포트를 쓰므로 measure.py 와 동시 실행 불가.
    첫 주행은 반드시 1단으로, 앞 공간을 확보하고 할 것.

토픽 이름 바꾸기 (0828: 하드코딩 제거)
--------------------------------------
    기본값은 manual 소스 규약(/manual_drive 등)이다. 다른 이름을 쓰려면:

        python3 tools/drive_wasd.py --source safety      # /safety_drive ...
        python3 tools/drive_wasd.py --mcu-ns /mcu2       # 발행 토픽 접두어
        python3 tools/drive_wasd.py --estop-topic /my_estop

    --source 는 매니저 yaml 의 source_names 에 있는 이름이어야 중재된다.
"""

import argparse
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger


FWD = [0.0, 1.0, 2.0, 3.0]
STAGE_MPS = {0.0: 0.0, 1.0: 0.229, 2.0: 0.526, 3.0: 0.823, -1.0: 0.229}
STEER_STEP = 3
STEER_MAX = 27


class Drive(Node):

    def __init__(self, hz, distance, source="manual", mcu_ns="/mcu",
                 estop_topic="/estop_lock", reset_service=None,
                 takeover=False):
        super().__init__("drive_wasd")

        # 내 명령
        self.drive = 0.0
        self.wheel = 0
        self.stop = False
        self.estop = False

        # 구간 측정
        self.distance = distance
        self.mark_base = None
        self.mark_t0 = None
        self.runs = []

        # 차량 상태
        self.enc = None
        self.rpm = 0.0
        self.dist_m = None
        self.speed = None
        self.fw_state = "?"
        self.connected = False
        self.tele_ok = False
        self.estop_latched = False
        self.safety = "?"
        self.act_drive = "?"
        self.act_wheel = "?"
        self.mode = "?"
        self.out_drive = 0.0
        self.out_wheel = 0
        self.raw = ""
        self.raw_at = 0.0
        self.msg = ""

        # 조종권. False 면 구동·조향을 발행하지 않아 팀이 차를 몬다.
        self.takeover = bool(takeover)

        # 토픽 이름은 전부 인자로 받는다. 코드에 박아두면 팀원이 소스 이름을
        # 바꿀 때마다 파일을 고쳐야 한다.
        ns = mcu_ns.rstrip("/")
        self.source = source
        self.mcu_ns = ns
        if reset_service is None:
            reset_service = ns + "/reset_estop"

        self.pub_d = self.create_publisher(Float32, "/%s_drive" % source, 10)
        self.pub_w = self.create_publisher(Int32, "/%s_wheel" % source, 10)
        self.pub_s = self.create_publisher(Bool, "/%s_stop" % source, 10)
        self.pub_e = self.create_publisher(Bool, estop_topic, 10)
        self.cli_reset = self.create_client(Trigger, reset_service)

        sub = self.create_subscription
        sub(Int32, ns + "/encoder", lambda m: setattr(self, "enc", int(m.data)), 10)
        sub(Float32, ns + "/rpm", lambda m: setattr(self, "rpm", float(m.data)), 10)
        sub(Float32, ns + "/distance_m",
            lambda m: setattr(self, "dist_m", float(m.data)), 10)
        sub(Float32, ns + "/speed_mps",
            lambda m: setattr(self, "speed", float(m.data)), 10)
        sub(String, ns + "/fw_state",
            lambda m: setattr(self, "fw_state", str(m.data)), 10)
        sub(Bool, ns + "/connected",
            lambda m: setattr(self, "connected", bool(m.data)), 10)
        sub(Bool, ns + "/telemetry_ok",
            lambda m: setattr(self, "tele_ok", bool(m.data)), 10)
        sub(Bool, ns + "/estop_latched",
            lambda m: setattr(self, "estop_latched", bool(m.data)), 10)
        sub(String, ns + "/safety_state",
            lambda m: setattr(self, "safety", str(m.data)), 10)
        sub(String, ns + "/active_drive_source",
            lambda m: setattr(self, "act_drive", str(m.data)), 10)
        sub(String, ns + "/active_wheel_source",
            lambda m: setattr(self, "act_wheel", str(m.data)), 10)
        sub(String, ns + "/current_mode",
            lambda m: setattr(self, "mode", str(m.data)), 10)
        sub(Float32, ns + "/cmd_drive",
            lambda m: setattr(self, "out_drive", float(m.data)), 10)
        sub(Int32, ns + "/cmd_wheel",
            lambda m: setattr(self, "out_wheel", int(m.data)), 10)
        sub(String, ns + "/raw_status", self._cb_raw, 10)

        self.create_timer(1.0 / hz, self._publish)

    def _cb_raw(self, m):
        self.raw = str(m.data)
        self.raw_at = time.monotonic()

    # ---------- 발행 ----------

    def _publish(self):
        # ---- 구동·조향은 조종권을 잡았을 때만 ----
        #
        # 발행을 멈추면 0.5초 뒤 manual 소스가 죽은 것으로 판정되어
        # 매니저가 팀 우선순위(라이다 > 카메라 > GPS)로 돌아간다.
        # 그래서 "안 보내는 것" 자체가 조종권을 놓는 방법이다.
        if self.takeover:
            d = Float32(); d.data = float(self.drive); self.pub_d.publish(d)
            w = Int32();   w.data = int(self.wheel);   self.pub_w.publish(w)

        # ---- 정지·E-Stop 은 대기 중에도 항상 보낸다 (안전요원) ----
        s = Bool();    s.data = bool(self.stop);   self.pub_s.publish(s)
        # estop 은 현재 상태를 항상 보낸다.
        # false 를 안 보내면 매니저 쪽이 계속 true 로 남는다.
        e = Bool();    e.data = bool(self.estop);  self.pub_e.publish(e)

    def toggle_takeover(self):
        """0 키. 대기 <-> 조종 전환."""
        if not self.takeover:
            # 진입: 정지 + 조향 중앙에서 시작한다. 팀이 주던 값을 이어받지 않는다.
            self.drive = 0.0
            self.wheel = 0
            self.stop = False
            self.takeover = True
            self.msg = "조종권 획득 — 팀 명령 무시. 정지/중앙에서 시작"
            return
        if self.drive != 0.0:
            # 달리는 상태로 팀에게 넘기지 않는다. 먼저 세운다.
            self.drive = 0.0
            self.msg = "정지했다. 0 을 한 번 더 누르면 팀에게 넘긴다"
            return
        self.takeover = False
        self.msg = "조종권 반납 — 0.5초 뒤 팀 명령으로 돌아간다"

    def request_reset(self):
        self.estop = False
        for _ in range(5):
            e = Bool(); e.data = False; self.pub_e.publish(e)
            time.sleep(0.05)
        if not self.cli_reset.wait_for_service(timeout_sec=1.0):
            self.msg = "reset 서비스 없음 (브릿지 실행 중인가?)"
            return
        fut = self.cli_reset.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 2.0:
            time.sleep(0.05)
        if fut.done() and fut.result() is not None:
            r = fut.result()
            self.msg = "래치 해제됨" if r.success else "거부: %s" % r.message
        else:
            self.msg = "응답 없음"

    # ---------- 구간 측정 ----------

    def mark_start(self):
        if self.enc is None:
            self.msg = "엔코더 값이 아직 없다"
            return
        self.mark_base = self.enc
        self.mark_t0 = time.monotonic()
        self.msg = "측정 시작: %d — %.1fm 주행 후 X" % (self.mark_base, self.distance)

    def mark_end(self):
        if self.mark_base is None:
            self.msg = "먼저 Z 로 시작을 표시하라"
            return
        if self.enc is None:
            self.msg = "엔코더 값이 아직 없다"
            return
        delta = self.enc - self.mark_base
        sec = time.monotonic() - self.mark_t0
        cpm = delta / self.distance if self.distance > 0 else 0.0
        self.runs.append((delta, sec, cpm))
        self.mark_base = None
        self.msg = ("%d회: %d카운트 %.1fs %.3fm/s cpm %.1f"
                    % (len(self.runs), delta, sec,
                       self.distance / sec if sec else 0, cpm))

    def summary_lines(self):
        if not self.runs:
            return []
        vals = [r[2] for r in self.runs]
        avg = sum(vals) / len(vals)
        spread = (max(vals) - min(vals)) / avg * 100 if avg else 0
        out = ["%d회 평균 counts_per_meter = %.1f (편차 %.1f%%)"
               % (len(vals), avg, spread)]
        if avg > 0:
            out.append("1카운트 %.1fmm  회전당 %.0f" % (1000.0 / avg, avg * 0.817))
        return out

    def all_stop(self):
        """종료 시 정지 명령을 확실히 보낸다.

        ⚠ 조종권을 잡은 채로 종료하면, 이 도구가 발행을 멈춘 0.5초 뒤
          매니저가 팀 명령으로 돌아간다. 차를 확실히 세워두려면
          E 로 E-Stop 을 걸고 나가야 한다 (그건 래치라 남는다).
        """
        was_takeover = self.takeover
        self.drive = 0.0
        self.stop = False
        self.takeover = True          # 종료 순간에는 반드시 정지를 명령한다
        for _ in range(6):
            self._publish()
            time.sleep(0.05)
        self.takeover = was_takeover


# ============================================================

def render(n):
    L = []
    w = L.append
    w("\033[2J\033[H")
    w("┌─ T870 수동 조종 (WASD) ──────────────────────────────────")
    w("│")
    if n.takeover:
        w("│  \033[30;42m  조종 중 — 키보드 전용 (팀 명령 무시)  \033[0m   0=반납")
    else:
        w("│  \033[30;43m  대기 중 — 팀이 주행 (카메라/라이다/GPS)  \033[0m 0=조종")
    w("│")

    if n.stop:
        dtxt = "\033[31m급정거\033[0m"
    elif n.drive == 0.0:
        dtxt = "정지"
    else:
        dtxt = "%s %d단  %.3f m/s" % (
            "전진" if n.drive > 0 else "후진",
            abs(int(n.drive)), STAGE_MPS.get(n.drive, 0))
    w("│  내 명령   \033[1m%-26s\033[0m" % dtxt)

    bar = ""
    for i in range(-27, 28, 3):
        bar += "◆" if abs(i - n.wheel) < 2 else ("│" if i == 0 else "·")
    w("│  조향      L %s R   \033[1m%+d도\033[0m" % (bar, n.wheel))
    w("│")

    w("│  ── 중재 ──────────────────────────────────────────")
    w("│   모드 \033[36m%-14s\033[0m 구동 \033[36m%-8s\033[0m 조향 \033[36m%s\033[0m"
      % (n.mode, n.act_drive, n.act_wheel))
    col = "\033[32m" if n.safety.startswith(("OK", "MANUAL")) else "\033[31m"
    w("│   안전 %s%s\033[0m" % (col, n.safety))
    w("│   출력  drive \033[1m%+.0f\033[0m   wheel \033[1m%+d\033[0m"
      % (n.out_drive, n.out_wheel))
    w("│")

    w("│  ── 차량 ──────────────────────────────────────────")
    conn = "\033[32m연결\033[0m" if n.connected else "\033[31m끊김\033[0m"
    tele = "\033[32mOK\033[0m" if n.tele_ok else "\033[31m두절\033[0m"
    w("│   아두이노 %s  텔레메트리 %s  상태 %s" % (conn, tele, n.fw_state))
    w("│   엔코더 \033[1m%-10s\033[0m RPM %.1f"
      % ("----" if n.enc is None else n.enc, n.rpm))
    if n.dist_m is not None:
        w("│   거리 \033[1m%.3f m\033[0m   속도 %.3f m/s"
          % (n.dist_m, n.speed if n.speed is not None else 0))
    lat = "\033[31m래치됨 (R 로 해제)\033[0m" if n.estop_latched else "정상"
    myE = "\033[31mON\033[0m" if n.estop else "off"
    w("│   E-Stop  내 명령 %s   차량 %s" % (myE, lat))

    if n.mark_base is not None and n.enc is not None:
        w("│   \033[33m측정 중\033[0m  Δ\033[1m%d\033[0m  %.1f초"
          % (n.enc - n.mark_base, time.monotonic() - n.mark_t0))
    elif n.runs:
        w("│   측정 %d회 — 마지막 cpm %.1f" % (len(n.runs), n.runs[-1][2]))
    w("│")

    w("│  ── 키 ────────────────────────────────────────────")
    w("│   \033[1mW\033[0m 전진   \033[1mS\033[0m 후진   \033[1mA\033[0m 좌   \033[1mD\033[0m 우   \033[1mC\033[0m 중앙")
    w("│   Space 정지    F 급정거    E E-Stop    R 래치해제")
    w("│   1 2 3 단계직접   \033[1m0 조종권 토글\033[0m")
    w("│   Z 측정시작   X 측정끝   Q 종료")
    w("│")
    if n.msg:
        w("│   \033[33m%s\033[0m" % n.msg)
    age = time.monotonic() - n.raw_at if n.raw_at else 999
    if age < 2:
        w("│   %s" % n.raw[:58])
    else:
        w("│   \033[31mSTATUS 수신 없음\033[0m")
    w("└──────────────────────────────────────────────────────────")
    sys.stdout.write("\r\n".join(L) + "\r\n")
    sys.stdout.flush()


def read_key(timeout):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return None
    ch = sys.stdin.read(1)
    if ch != "\x1b":
        return ch
    # 방향키도 함께 받아준다 (ESC[A / ESCOA 두 형태)
    seq = ""
    deadline = time.monotonic() + 0.05
    while time.monotonic() < deadline and len(seq) < 4:
        r, _, _ = select.select([sys.stdin], [], [], 0.02)
        if not r:
            break
        seq += sys.stdin.read(1)
        if seq[-1] in "ABCD~":
            break
    if not seq:
        return "ESC"
    return {"A": "w", "B": "s", "C": "d", "D": "a"}.get(seq[-1])


DRIVE_KEYS = ("w", "W", "s", "S", "a", "A", "d", "D", "c", "C", "1", "2", "3", " ")


def handle(n, k):
    n.msg = ""

    # 대기 중에는 주행 키가 아무 일도 하지 않는다. 조용히 무시하지 않고 알린다.
    # (예전에는 무조건 발행돼서, 켜 두기만 해도 팀 명령이 막혔다)
    if not n.takeover and k in DRIVE_KEYS:
        n.msg = "대기 중이다. 0 을 눌러 조종권을 잡아라 (F/E 는 항상 동작)"
        return

    if k in ("w", "W"):
        if n.drive < 0:
            n.drive = 0.0
            n.msg = "후진 -> 정지. 다시 W 로 전진"
        else:
            i = FWD.index(n.drive) if n.drive in FWD else 0
            n.drive = FWD[min(i + 1, len(FWD) - 1)]

    elif k in ("s", "S"):
        if n.drive > 0:
            n.drive = 0.0
            n.msg = "전진 -> 정지. 다시 S 로 후진"
        elif n.drive == 0.0:
            n.drive = -1.0
        else:
            n.drive = 0.0

    elif k in ("a", "A"):
        n.wheel = max(-STEER_MAX, n.wheel - STEER_STEP)
    elif k in ("d", "D"):
        n.wheel = min(STEER_MAX, n.wheel + STEER_STEP)
    elif k in ("c", "C"):
        n.wheel = 0

    elif k == "0":
        n.toggle_takeover()
        return

    elif k in ("1", "2", "3"):
        n.drive = float(k)
    elif k == " ":
        n.drive = 0.0
        n.stop = False

    elif k in ("f", "F"):
        n.stop = not n.stop
        if n.stop:
            n.drive = 0.0

    elif k in ("e", "E"):
        n.estop = not n.estop
        if n.estop:
            n.drive = 0.0
            n.msg = "E-Stop 걸림. R 로 해제"

    elif k in ("r", "R"):
        n.request_reset()

    elif k in ("z", "Z"):
        n.mark_start()
    elif k in ("x", "X"):
        n.mark_end()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--distance", type=float, default=10.0,
                    help="Z~X 구간의 실제 거리 [m]")
    ap.add_argument("--source", default="manual",
                    help="발행할 소스 이름. /<source>_drive 등으로 발행한다. "
                         "매니저 yaml 의 source_names 에 있어야 한다")
    ap.add_argument("--mcu-ns", default="/mcu",
                    help="MCU 발행 토픽 접두어")
    ap.add_argument("--estop-topic", default="/estop_lock")
    ap.add_argument("--reset-service", default=None,
                    help="기본값은 <mcu-ns>/reset_estop")
    ap.add_argument("--takeover", action="store_true",
                    help="처음부터 조종권을 잡은 상태로 시작한다. "
                         "팀 노드 없이 혼자 시험할 때 편하다. "
                         "기본은 대기 상태 — 0 을 눌러야 조종이 시작된다")
    args = ap.parse_args()

    rclpy.init()
    node = Drive(args.hz, args.distance, args.source, args.mcu_ns,
                 args.estop_topic, args.reset_service, args.takeover)
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    last = 0.0
    try:
        tty.setraw(fd)
        while True:
            k = read_key(0.05)
            if k in ("q", "Q", "\x03"):
                break
            if k:
                handle(node, k)
            now = time.monotonic()
            if now - last > 0.15:
                render(node)
                last = now
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.all_stop()
        node.destroy_node()
        rclpy.shutdown()
        print("\n정지 명령 전송 후 종료했습니다.")
        if node.takeover:
            print("⚠ 조종권을 잡은 채로 종료했습니다. 0.5초 뒤 팀 명령으로 "
                  "돌아갑니다.\n  차를 확실히 세워두려면 E 로 E-Stop 을 걸고 "
                  "나가야 합니다 (래치라 남습니다).")
        if node.runs:
            print("\n" + "=" * 52)
            print("  구간 측정 (%.1fm 기준)" % node.distance)
            print("=" * 52)
            for i, (d, sec, cpm) in enumerate(node.runs, 1):
                print("   %d회: %5d 카운트 / %5.1fs / cpm %.1f" % (i, d, sec, cpm))
            for line in node.summary_lines():
                print("   " + line)
            vals = [r[2] for r in node.runs]
            print("\n   yaml 에 넣을 값:  counts_per_meter: %.1f"
                  % (sum(vals) / len(vals)))
            print()


if __name__ == "__main__":
    main()
