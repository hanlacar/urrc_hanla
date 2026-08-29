#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
team_monitor.py  —  통합 시험 현장 진단

각 팀이 실제로 뭘 보내고 있는지, 중재기가 뭘 골랐는지, 차량이 뭘 받았는지
한 화면에서 본다. 통합 첫날 문제 지점을 빠르게 찾기 위한 도구.

실행 (터미널):
    source /opt/ros/jazzy/setup.bash
    source <워크스페이스>/install/setup.bash
    python3 team_monitor.py

토픽 이름이 다르면:
    python3 team_monitor.py --camera-drive /cmd_driving --lidar-drive /lidar/cmd
"""

import argparse
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String


class Chan:
    """한 토픽의 최신값 + 수신율."""

    def __init__(self, label):
        self.label = label
        self.value = None
        self.last = 0.0
        self.stamps = deque(maxlen=30)

    def update(self, value):
        now = time.monotonic()
        self.value = value
        self.last = now
        self.stamps.append(now)

    def hz(self):
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / span if span > 0 else 0.0

    def age(self):
        return time.monotonic() - self.last if self.last else 999.0

    def alive(self, timeout=0.5):
        return self.age() <= timeout


class TeamMonitor(Node):

    def __init__(self, topics):
        super().__init__("team_monitor")
        self.ch = {}

        def sub(key, topic, msgtype, conv):
            self.ch[key] = Chan(topic)
            self.create_subscription(
                msgtype, topic,
                lambda m, k=key, c=conv: self.ch[k].update(c(m)), 10)

        f = lambda m: float(m.data)
        i = lambda m: int(m.data)
        b = lambda m: bool(m.data)
        s = lambda m: str(m.data)

        # 팀 입력
        for src in ("camera", "lidar", "gps"):
            sub("%s_drive" % src, topics["%s_drive" % src], Float32, f)
            sub("%s_wheel" % src, topics["%s_wheel" % src], Int32, i)
        sub("lidar_stop", topics["lidar_stop"], Bool, b)
        sub("camera_stop", topics["camera_stop"], Bool, b)
        sub("mode", topics["mode"], String, s)

        # 중재 출력
        sub("mcu_drive", "/mcu/cmd_drive", Float32, f)
        sub("mcu_wheel", "/mcu/cmd_wheel", Int32, i)
        sub("mcu_stop", "/mcu/cmd_stop", Bool, b)
        sub("act_drive", "/mcu/active_drive_source", String, s)
        sub("act_wheel", "/mcu/active_wheel_source", String, s)
        sub("safety", "/mcu/safety_state", String, s)

        # 차량 피드백
        sub("enc", "/mcu/encoder", Int32, i)
        sub("rpm", "/mcu/rpm", Float32, f)
        sub("ard_state", "/mcu/fw_state", String, s)
        sub("connected", "/mcu/connected", Bool, b)
        sub("tele_ok", "/mcu/telemetry_ok", Bool, b)

        self.create_timer(0.2, self.render)

    # ---------- 화면 ----------

    def render(self):
        C = self.ch
        L = []
        w = L.append

        def mark(key, timeout=0.5):
            c = C[key]
            if c.value is None:
                return "\033[90m○ 없음\033[0m"
            if not c.alive(timeout):
                return "\033[31m✗ %.1fs 끊김\033[0m" % c.age()
            return "\033[32m● %.0fHz\033[0m" % c.hz()

        def val(key, fmt="%s", default="-"):
            v = C[key].value
            return default if v is None else fmt % v

        w("\033[2J\033[H")
        w("┌─ 통합 진단 ─────────────────────────────────────────────────")
        w("│")

        mode = val("mode", "%s", "없음")
        w("│  모드  \033[1;36m%-15s\033[0m  %s" % (mode, mark("mode", 2.0)))
        owner = "lidar" if mode in ("T_PARK", "PARALLEL_PARK") else "camera"
        w("│  조향 권한  \033[33m%s\033[0m" % owner)
        w("│")

        w("│  ── 팀 입력 ────────────────────────────────────────────")
        w("│         drive              wheel             stop")
        for src in ("camera", "lidar", "gps"):
            dk, wk = "%s_drive" % src, "%s_wheel" % src
            sk = "%s_stop" % src if "%s_stop" % src in C else None
            stop_txt = ""
            if sk:
                sv = C[sk].value
                stop_txt = ("\033[31mSTOP\033[0m" if sv else
                            ("\033[90moff\033[0m" if sv is not None else "-"))
            gate = "" if src == owner or src == "gps" else " \033[90m(조향무시)\033[0m"
            w("│  %-7s %-6s %-13s %-5s %-13s %s%s"
              % (src, val(dk, "%+.0f"), mark(dk),
                 val(wk, "%+d"), mark(wk), stop_txt, gate))
        w("│")

        w("│  ── 중재 출력 ──────────────────────────────────────────")
        stopv = C["mcu_stop"].value
        w("│   /mcu/cmd_drive \033[1m%-6s\033[0m /mcu/cmd_wheel \033[1m%-5s\033[0m /mcu/cmd_stop %s"
          % (val("mcu_drive", "%+.0f"), val("mcu_wheel", "%+d"),
             "\033[31mTRUE\033[0m" if stopv else "false"))
        w("│   구동소스 \033[36m%-10s\033[0m 조향소스 \033[36m%s\033[0m"
          % (val("act_drive"), val("act_wheel")))
        sfy = val("safety")
        col = "\033[32m" if sfy.startswith("OK") else "\033[31m"
        w("│   안전상태 %s%s\033[0m" % (col, sfy))
        w("│   중재기   %s" % mark("mcu_drive", 0.5))
        w("│")

        w("│  ── 차량 ───────────────────────────────────────────────")
        conn = C["connected"].value
        tele = C["tele_ok"].value
        w("│   아두이노 %s   텔레메트리 %s   상태 %s"
          % ("\033[32m연결\033[0m" if conn else "\033[31m끊김\033[0m",
             "\033[32mOK\033[0m" if tele else "\033[31m두절\033[0m",
             val("ard_state")))
        w("│   엔코더 \033[1m%-10s\033[0m RPM %-8s %s"
          % (val("enc", "%d"), val("rpm", "%.1f"), mark("enc", 1.0)))
        w("│")
        w("│   q 또는 Ctrl-C 로 종료 (감시 전용. 아무것도 발행하지 않음)")
        w("└──────────────────────────────────────────────────────────────")

        sys.stdout.write("\n".join(L) + "\n")
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    for src in ("camera", "lidar", "gps"):
        ap.add_argument("--%s-drive" % src, default="/%s_drive" % src)
        ap.add_argument("--%s-wheel" % src, default="/%s_wheel" % src)
    ap.add_argument("--lidar-stop", default="/lidar_stop")
    ap.add_argument("--camera-stop", default="/camera_stop")
    ap.add_argument("--mode", default="/vehicle_mode")
    a = ap.parse_args()

    topics = {
        "camera_drive": a.camera_drive, "camera_wheel": a.camera_wheel,
        "lidar_drive": a.lidar_drive, "lidar_wheel": a.lidar_wheel,
        "gps_drive": a.gps_drive, "gps_wheel": a.gps_wheel,
        "lidar_stop": a.lidar_stop, "camera_stop": a.camera_stop,
        "mode": a.mode,
    }

    rclpy.init()
    node = TeamMonitor(topics)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("\n종료")


if __name__ == "__main__":
    main()
