#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
odom_calib.py — odom 거리가 실제와 안 맞을 때 원인을 분리해서 보여준다.

    /mcu/distance_m = |엔코더델타 / counts_per_meter × cos(조향각)| 의 누적

  오차 후보가 셋이라 하나씩 갈라야 한다.
    ① counts_per_meter 가 틀림          → 엔코더델타 / 실제거리 로 직접 계산
    ② cos(조향각) 이 거리를 깎음         → 주행 중 steer_ms 를 감시해서 보여줌
    ③ 코스팅 적산                        → 정지 후에도 늘어나는 양을 따로 표시

사용
----
    터미널 1: ros2 launch t870_mcu t870_mcu.launch.py
    터미널 2: python3 tools/odom_calib.py

    1. 출발선을 바닥에 표시하고 차를 세운다
    2. Enter → 리셋
    3. 아무 거리나 직진 (C 로 조향 중앙, 2단 권장)
    4. 완전히 멈추고 2초 기다린 뒤 Enter
    5. 출발선에서 멈춘 자리까지 줄자로 재서 입력

  ★ "정확히 10m 에서 멈추기" 를 하지 않는다.
    이 차는 브레이크가 없어 코스팅으로 항상 넘어간다. 선에 맞추려 하면
    "실제 거리" 라는 전제부터 틀어진다. 그냥 굴리고 멈춘 자리를 재는 것이
    훨씬 정확하다.

  ★ 지킬 것
    - 직진만. 엔코더가 앞바퀴 하나에 달려 있어 선회하면 그 바퀴만 더 돈다
    - 2단 권장. 저속에서는 브러시 노이즈로 가짜 펄스가 생겨 카운트가 부푼다
    - 완전히 멈춘 뒤 읽기. 코스팅 구간 카운트가 빠지면 안 된다
    - 거리는 길수록 정확. 줄자 오차 ±2cm 가 2m 에선 1%, 20m 에선 0.1%

  ★ 3회 이상 반복할 것. 1회로는 편차를 알 수 없다.
"""

import argparse
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from std_srvs.srv import Trigger


class OdomCalib(Node):

    def __init__(self, mcu_ns="/mcu"):
        super().__init__("odom_calib")
        ns = mcu_ns.rstrip("/")
        self.enc = None
        self.dist = None
        self.steer_ms = 0
        self.steer_seen = []

        self.create_subscription(
            Int32, ns + "/encoder",
            lambda m: setattr(self, "enc", int(m.data)), 10)
        self.create_subscription(
            Float32, ns + "/distance_m",
            lambda m: setattr(self, "dist", float(m.data)), 10)
        self.create_subscription(
            Int32, ns + "/steer_ms", self._cb_steer, 10)

        self.cli_reset = self.create_client(Trigger, ns + "/reset_odom")

    def _cb_steer(self, m):
        self.steer_ms = int(m.data)
        self.steer_seen.append(self.steer_ms)

    def wait_data(self, timeout=10.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.enc is not None and self.dist is not None:
                return True
        return False

    def reset_odom(self):
        if not self.cli_reset.wait_for_service(timeout_sec=2.0):
            return False, "reset_odom 서비스 없음 (브릿지가 구버전일 수 있다)"
        fut = self.cli_reset.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if fut.done() and fut.result() is not None:
            return fut.result().success, fut.result().message
        return False, "응답 없음"

    def pump(self, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)


def ask(prompt):
    try:
        return input(prompt)
    except (EOFError, KeyboardInterrupt):
        print()
        sys.exit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--distance", type=float, default=0.0,
                    help="매번 같은 거리를 갈 때만 쓴다. 기본은 주행 후 입력")
    ap.add_argument("--cpm", type=float, default=533.1,
                    help="현재 yaml 의 counts_per_meter (비교용)")
    ap.add_argument("--mcu-ns", default="/mcu")
    args = ap.parse_args()

    rclpy.init()
    node = OdomCalib(args.mcu_ns)

    print("=" * 62)
    print("  odom 거리 검증")
    if args.distance > 0:
        print("  기준 거리 %.2f m (고정)" % args.distance)
    else:
        print("  주행 후 실측 거리를 입력하는 방식")
    print("=" * 62)
    print("  직진만 / 2단 권장 / 완전히 멈춘 뒤 읽기")

    if not node.wait_data():
        print("\n  토픽이 안 온다. 브릿지가 떠 있는지, 아두이노가 연결됐는지 확인.")
        print("  확인:  ros2 topic hz /mcu/encoder")
        sys.exit(1)

    runs = []
    n = 0
    while True:
        n += 1
        print("\n" + "-" * 62)
        print("  %d 회차" % n)
        print("-" * 62)
        print("  차를 출발선에 세우고, 조향은 C 로 중앙에 두세요.")
        ask("  준비되면 Enter (리셋합니다) > ")

        ok, msg = node.reset_odom()
        print("  리셋: %s (%s)" % ("성공" if ok else "실패", msg))
        node.pump(0.5)
        enc0 = node.enc
        dist0 = node.dist
        node.steer_seen = []
        print("  기준점  엔코더 %d  거리 %.3f m" % (enc0, dist0))

        if args.distance > 0:
            print("\n  ▶ %.2f m 주행하세요." % args.distance)
        else:
            print("\n  ▶ 직진 주행하세요. 거리는 자유입니다 (길수록 정확).")
        print("     완전히 멈춘 뒤 Enter (코스팅이 끝날 때까지 기다립니다).")
        ask("  > ")

        # 정지 직후 값
        node.pump(0.3)
        enc_stop = node.enc
        dist_stop = node.dist

        # 코스팅이 끝날 때까지 조금 더 본다
        node.pump(2.0)
        enc1 = node.enc
        dist1 = node.dist

        # ---- 실제 거리 입력 ----
        if args.distance > 0:
            real_m = args.distance
        else:
            while True:
                txt = ask("\n  출발선 ~ 멈춘 자리 실측 거리 [m] (예: 9.72) > ")
                try:
                    real_m = float(txt.strip())
                    if real_m > 0:
                        break
                except ValueError:
                    pass
                print("     숫자를 입력하세요. cm 가 아니라 m 단위입니다.")

        d_enc = enc1 - enc0
        d_odom = dist1 - dist0
        coast_enc = enc1 - enc_stop
        coast_odom = dist1 - dist_stop

        steer = node.steer_seen or [0]
        steer_max = max(abs(s) for s in steer)
        steer_avg = statistics.mean(abs(s) for s in steer)

        real_cpm = d_enc / real_m
        err_pct = (d_odom - real_m) / real_m * 100.0

        print()
        print("  엔코더 델타     %d 카운트" % d_enc)
        print("  odom 이 말한 거리 %.3f m   (실제 %.2f m, 오차 %+.1f%%)"
              % (d_odom, real_m, err_pct))
        print("  정지 후 추가분   엔코더 %d,  거리 %+.3f m  (코스팅)"
              % (coast_enc, coast_odom))
        print("  주행 중 steer_ms  평균 %.0f  최대 %.0f" % (steer_avg, steer_max))

        if steer_max > 30:
            import math
            # 브릿지 기본: 440ms = 27도
            deg = steer_max / (440.0 / 27.0)
            shrink = (1 - math.cos(math.radians(deg))) * 100
            print("     ⚠ 조향이 0 이 아니다. cos 보정으로 거리가 최대 %.1f%% 깎인다."
                  % shrink)
            print("       측정 전에 C 로 중앙 정렬했는지 확인할 것.")

        print()
        print("  실측 counts_per_meter = %.1f   (현재 설정 %.1f, 차이 %+.1f%%)"
              % (real_cpm, args.cpm, (real_cpm - args.cpm) / args.cpm * 100))

        runs.append((d_enc, d_odom, real_cpm, steer_max, real_m))

        again = ask("\n  한 번 더? (Enter=계속 / q=끝내고 결과) > ")
        if again.strip().lower() in ("q", "quit", "n"):
            break

    print("\n" + "=" * 62)
    print("  결과 (%d 회)" % len(runs))
    print("=" * 62)
    cpms = [r[2] for r in runs]
    for i, (de, do, c, sm, rm) in enumerate(runs, 1):
        print("   %d회  실측 %6.2f m   엔코더 %6d   odom %6.3f m   cpm %6.1f   steer_max %d"
              % (i, rm, de, do, c, sm))

    avg = statistics.mean(cpms)
    if len(cpms) > 1:
        spread = (max(cpms) - min(cpms)) / avg * 100
        print("\n   평균 counts_per_meter = %.1f   (편차 %.1f%%)" % (avg, spread))
        if spread > 5:
            print("   ⚠ 편차가 5%% 를 넘는다. 측정 자체가 불안정하다.")
            print("     바퀴 미끄러짐, 출발/정지 지점 표시, 조향 중앙 정렬을 확인할 것.")
    else:
        print("\n   counts_per_meter = %.1f  (1회 측정 — 반복 권장)" % avg)

    print("\n   1카운트 = %.2f mm    바퀴 1회전 = %.0f 카운트 (둘레 0.817m)"
          % (1000.0 / avg, avg * 0.817))
    print("\n   yaml 에 넣을 값:")
    print("       counts_per_meter: %.1f" % avg)
    print()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
