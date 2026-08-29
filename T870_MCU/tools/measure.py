#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
measure.py  —  T870 현장 측정 도구 (ROS 불필요, 시리얼 직결)

어제 10m 주행에서 22카운트만 나왔다. 정상이면 1000~2500 이 나와야 한다.
그래서 이 도구는 무엇보다 먼저 '엔코더가 살아 있는지' 부터 확인한다.

측정 항목
  0. 엔코더 생존 확인      손으로 1회전 → 회전당 카운트
  1. counts_per_meter      10m 손 밀기 (가장 정확)
  2. 단계별 속도            1/2/3 단 주행 시간
  3. 코스팅 거리            정지 명령 후 관성 거리
  4. B 급정거 거리          v29 이상에서만

실행:
    cd <워크스페이스>/tools
    python3 measure.py

포트가 다르면:
    python3 tools/measure.py --port /dev/t870_mcu

※ 브릿지·시리얼모니터가 켜져 있으면 안 된다 (포트 단일 점유)
    sudo fuser -k /dev/ttyACM*
"""

import argparse
import glob
import os
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial 없음:  sudo apt install python3-serial")


STAGE_CMD = {0: "1.00", 1: "2.00", 2: "3.00", 3: "4.00", -1: "6.00"}


# setup.sh 가 udev 로 만들어주는 고정 이름. 어느 PC 에서도 동일하다.
PORT_SYMLINKS = ("/dev/t870_mcu",)


def find_arduino_port():
    """아두이노 포트를 찾는다. 포트 번호는 꽂는 순서마다 바뀌므로 믿지 않는다.

    1) udev 심볼릭 링크 (/dev/t870_mcu)
    2) /dev/serial/by-id 장치 이름 매칭
    못 찾으면 None. ★ 예전에는 여기서 /dev/ttyACM* 의 첫 번째를 그냥 집었는데,
      거기 GPS 가 앉아 있으면 GPS 포트에 구동 명령을 쏘게 된다.
    """
    for link in PORT_SYMLINKS:
        if os.path.exists(link):
            return os.path.realpath(link)

    base = "/dev/serial/by-id"
    if os.path.isdir(base):
        best = None
        for name in sorted(os.listdir(base)):
            low = name.lower()
            if any(b in low for b in ("u-blox", "u_blox", "gnss", "gps")):
                continue
            if "arduino" in low or "mega" in low:
                return os.path.realpath(os.path.join(base, name))
            if any(h in low for h in ("ch340", "ch910", "wch", "usb2.0-serial")):
                best = os.path.realpath(os.path.join(base, name))
        if best:
            return best
    return None
FEED_HZ = 10.0
STATUS_HZ = 5.0
WHEEL_CIRC = 0.817          # 바퀴 둘레 [m] (지름 0.26)


class Mcu:
    """시리얼 연결 + 워치독 급식 + STATUS 파싱."""

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.count = 0
        self.rpm = 0.0
        self.state = "?"
        self.pwm = 0
        self.have = False
        self.fw = "?"

        self.stage = 0          # tx 루프가 계속 보낼 단계
        self.running = True
        self.lock = threading.Lock()
        self.raw_log = []

        # ★ USB 를 열면 아두이노가 리셋되고 약 2초간 부트로더가 돈다.
        #   그 동안 명령을 쏘면 부트로더가 방해받아 스케치가 안 뜬다.
        #   부팅이 끝날 때까지 송신을 막는다.
        self.tx_enabled = False
        self.opened_at = time.monotonic()

        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._tx, daemon=True).start()

    def wait_boot(self, seconds=3.0):
        """부팅이 끝날 때까지 조용히 기다린 뒤 송신을 연다."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            if self.fw != "?":          # MCU_BOOT 를 받으면 부팅 완료
                time.sleep(0.3)
                break
            time.sleep(0.1)
        self.tx_enabled = True
        # 첫 STATUS 를 받을 때까지 잠깐 더
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0 and not self.have:
            time.sleep(0.1)

    def send(self, cmd):
        with self.lock:
            try:
                self.ser.write((cmd + "\n").encode())
            except serial.SerialException:
                pass

    def _rx(self):
        buf = b""
        while self.running:
            try:
                chunk = self.ser.read(256)
            except Exception:
                time.sleep(0.1)
                continue
            if not chunk:
                continue
            buf += chunk
            if len(buf) > 8192:
                buf = buf[-1024:]
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._parse(line.decode("utf-8", "replace").strip())

    def _parse(self, line):
        if not line:
            return
        if line.startswith("MCU_BOOT"):
            self.fw = line.split(",")[-1]
            return
        if not line.startswith("STATUS,"):
            self.raw_log.append(line)
            if len(self.raw_log) > 20:
                self.raw_log.pop(0)
            return
        f = line.split(",")
        if len(f) < 8:
            return
        try:
            self.state = f[1]
            self.pwm = int(float(f[5]))
            self.rpm = float(f[6])
            self.count = int(float(f[7]))
            self.have = True
        except ValueError:
            pass

    def _tx(self):
        last = 0.0
        while self.running:
            if not self.tx_enabled:
                time.sleep(0.05)
                continue
            self.send(STAGE_CMD.get(self.stage, "1.00"))
            now = time.monotonic()
            if now - last >= 1.0 / STATUS_HZ:
                self.send("S")
                last = now
            time.sleep(1.0 / FEED_HZ)

    def reset_odo(self):
        self.send("O")
        time.sleep(0.4)

    def stop(self):
        self.stage = 0
        self.send("1.00")

    def close(self):
        self.stage = 0
        self.running = False
        for _ in range(3):
            self.send("1.00")
            time.sleep(0.1)
        try:
            self.ser.close()
        except Exception:
            pass


# ============================================================

def hr(title):
    print("\n" + "=" * 62)
    print("  " + title)
    print("=" * 62)


def wait_enter(msg):
    try:
        input("  " + msg)
        return True
    except (EOFError, KeyboardInterrupt):
        return False


def live(mcu, msg, stop_key="Enter"):
    """카운트를 실시간 표시하며 Enter 대기."""
    done = threading.Event()

    def show():
        while not done.is_set():
            sys.stdout.write("\r  카운트 \033[1m%8d\033[0m  RPM %6.2f  PWM %3d  %-8s"
                             % (mcu.count, mcu.rpm, mcu.pwm, mcu.state))
            sys.stdout.flush()
            time.sleep(0.1)

    t = threading.Thread(target=show, daemon=True)
    t.start()
    print("  " + msg)
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        done.set()
        return False
    done.set()
    time.sleep(0.15)
    print()
    return True


# ============================================================
# 0. 엔코더 생존 확인
# ============================================================

def step_encoder_health(mcu):
    hr("0. 엔코더 생존 확인  ★ 이것부터")

    print("""
  어제 10m 에 22카운트만 나왔다. 정상이면 1000~2500 이다.
  주행 전에 엔코더가 살아 있는지부터 본다.

  준비: 구동 바퀴에 분필로 마크 하나
""")
    if not wait_enter("준비되면 Enter"):
        return None

    results = []
    for i in range(1, 4):
        mcu.reset_odo()
        base = mcu.count
        if not live(mcu, "%d회차: 바퀴를 손으로 정확히 1바퀴 돌리고 Enter" % i):
            return None
        delta = mcu.count - base
        results.append(delta)
        print("    → %d 카운트" % delta)

        if delta == 0:
            print("""
    \033[31m🔴 카운트가 전혀 안 올라간다. 엔코더가 죽었다.\033[0m
       확인할 것:
         - D2 (A상) 배선
         - 엔코더 VCC → 5V,  GND
         - 위에 설치한 부품 때문에 선이 눌리거나 빠졌나
       이 상태로는 어떤 측정도 의미가 없다.
""")
            return None

    avg = sum(results) / len(results)
    spread = (max(results) - min(results)) / avg * 100 if avg else 0
    print("\n  회전당 평균 \033[1m%.1f\033[0m 카운트   편차 %.0f%%" % (avg, spread))

    if avg < 20:
        print("  \033[31m⚠ 너무 적다. 펄스 소실 의심 (배선/디바운스)\033[0m")
    elif spread > 20:
        print("  \033[33m⚠ 편차가 크다. 손으로 정확히 1바퀴 맞추기 어려운 탓일 수 있다\033[0m")
    else:
        print("  \033[32m✓ 정상 범위\033[0m")

    print("""
  참고: 공중이면 오픈 디퍼렌셜 때문에 한쪽 바퀴만 돈다.
        그때 도는 바퀴는 캐리어의 2배로 회전하므로
        지면 기준 회전당 카운트는 이 값의 약 2배로 추정된다.
        → 지면 추정 %.0f 카운트/회전""" % (avg * 2))
    return avg


# ============================================================
# 1. counts_per_meter (손 밀기)
# ============================================================

def step_cpm_push(mcu, distance):
    hr("1. counts_per_meter — 손 밀기 %.1fm  ★최우선" % distance)

    print("""
  🔴 시작 전에 반드시:
     - MD30C 의 MA/MB 선을 뽑는다 (역기전력이 드라이버를 태운다)
     - 아두이노 USB 는 연결 유지
     - 조향 중앙, 앞바퀴 똑바른지 눈으로 확인
     - 뒷바퀴 축을 출발선에 정렬

  ※ 걷는 속도 이상으로 민다. 천천히 밀면 브러시 노이즈로 부풀려진다.
""")
    if not wait_enter("준비되면 Enter"):
        return None

    runs = []
    for i in range(1, 4):
        mcu.reset_odo()
        base = mcu.count
        if not live(mcu, "%d회차: %.1fm 밀고 도착선에 뒷바퀴 축 맞춘 뒤 Enter"
                    % (i, distance)):
            return None
        delta = mcu.count - base
        cpm = delta / distance
        runs.append(cpm)
        print("    → %d 카운트   counts_per_meter = %.2f" % (delta, cpm))

    avg = sum(runs) / len(runs)
    spread = (max(runs) - min(runs)) / avg * 100 if avg else 0
    print("\n  \033[1mcounts_per_meter = %.1f\033[0m   편차 %.1f%% %s"
          % (avg, spread, "\033[32mOK\033[0m" if spread <= 5
             else "\033[31m★5%% 초과 — 재측정 권장\033[0m"))
    print("  1카운트 = %.1f mm" % (1000.0 / avg))
    print("  회전당 %.0f 카운트 (둘레 %.3fm 기준)" % (avg * WHEEL_CIRC, WHEEL_CIRC))
    return avg


# ============================================================
# 2. 단계별 속도 + 3. 코스팅
# ============================================================

def step_drive(mcu, distance, stage, cpm):
    hr("2-%d. 단계 %d 주행 + 코스팅" % (stage, stage))

    print("""
  🔴 MA/MB 를 다시 연결한다.
     %.1fm 직선, 도착선 뒤로 코스팅 여유 3m 이상.
     E-Stop 담당 대기.
""" % distance)
    if not wait_enter("준비되면 Enter (취소는 Ctrl-C)"):
        return None

    mcu.reset_odo()
    base = mcu.count
    t0 = time.monotonic()
    mcu.stage = stage
    print("  출발. 단계 %d (%s)" % (stage, STAGE_CMD[stage]))

    if not live(mcu, "도착선을 지나는 순간 Enter"):
        mcu.stop()
        return None
    c_finish = mcu.count
    t1 = time.monotonic()
    mcu.stop()
    print("  정지 명령 전송")

    if not live(mcu, "차가 완전히 멈추면 Enter"):
        return None
    c_stop = mcu.count

    d_run = c_finish - base
    d_coast = c_stop - c_finish
    elapsed = t1 - t0
    speed = distance / elapsed if elapsed else 0

    print("\n  주행 %d 카운트 / %.1fm / %.2f초" % (d_run, distance, elapsed))
    print("  \033[1m평균 속도 %.3f m/s\033[0m  (%.2f km/h)" % (speed, speed * 3.6))
    print("  코스팅 %d 카운트" % d_coast)
    if cpm and cpm > 0:
        print("  \033[1m코스팅 거리 %.3f m\033[0m" % (d_coast / cpm))
        print("  주행 구간 환산 %.2f m (오차 %.1f%%)"
              % (d_run / cpm, (d_run / cpm - distance) / distance * 100))
    return {"stage": stage, "counts": d_run, "sec": elapsed,
            "mps": speed, "coast_counts": d_coast}


# ============================================================
# 4. B 급정거
# ============================================================

def step_brake(mcu, stage, cpm):
    hr("4. B 급정거 거리 (v29 이상)")

    if mcu.fw not in ("v29",) and mcu.fw != "?":
        print("  \033[33m⚠ 펌웨어가 %s 다. B 명령은 v29 부터.\033[0m" % mcu.fw)
        print("  건너뛰려면 Ctrl-C")

    print("""
  달리다가 B 를 보내 즉시 정지시킨다.
  램프 정지와 비교해 얼마나 짧아지는지 본다.
""")
    if not wait_enter("준비되면 Enter"):
        return None

    mcu.reset_odo()
    mcu.stage = stage
    if not live(mcu, "충분히 가속되면 Enter (그 순간 B 전송)"):
        mcu.stop()
        return None

    c0 = mcu.count
    mcu.stage = 0
    mcu.send("B")
    print("  B 전송")

    if not live(mcu, "완전히 멈추면 Enter"):
        return None
    d = mcu.count - c0
    print("\n  제동 %d 카운트" % d)
    if cpm and cpm > 0:
        print("  \033[1m제동 거리 %.3f m\033[0m" % (d / cpm))
    return d


# ============================================================

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="auto",
                    help='기본 auto = 아두이노를 스스로 찾는다')
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--distance", type=float, default=10.0)
    ap.add_argument("--cpm", type=float, default=0.0,
                    help="이미 아는 counts_per_meter (1단계 건너뛸 때)")
    args = ap.parse_args()

    print("=" * 62)
    print("  T870 현장 측정")
    print("=" * 62)
    port = args.port
    if port.strip().lower() in ("auto", "", "none"):
        port = find_arduino_port()
        if not port:
            sys.exit("\n아두이노를 못 찾았다. USB 연결을 확인하거나 --port 로 지정하라.")
        print("  포트 자동 탐색: %s" % port)
    print("  포트 %s  측정거리 %.1fm" % (port, args.distance))
    print("  ※ 브릿지·시리얼모니터가 켜져 있으면 안 됨")
    print("     sudo fuser -k <포트>")

    try:
        mcu = Mcu(port, args.baud)
    except serial.SerialException as e:
        sys.exit("\n포트 열기 실패: %s\n  sudo chmod 666 %s\n  sudo fuser -k %s"
                 % (e, port, port))

    print("\n  아두이노 부팅 대기 (약 3초, 이 동안은 아무것도 보내지 않음)...")
    mcu.wait_boot()

    if not mcu.have:
        print("  \033[31m⚠ STATUS 수신 없음\033[0m")
        if mcu.fw != "?":
            print("     부팅 메시지는 받았다 (펌웨어 %s). S 응답만 없다." % mcu.fw)
        else:
            print("     부팅 메시지도 못 받았다. USB·전원을 확인하라.")
        if mcu.raw_log:
            print("     최근 수신:")
            for l in mcu.raw_log[-5:]:
                print("       %s" % l)
        print("\n  이 상태로는 측정이 불가능하다. Ctrl-C 로 나가라.")
    else:
        print("  \033[32m✓\033[0m 펌웨어 %s / 상태 %s / 카운트 %d"
              % (mcu.fw, mcu.state, mcu.count))

    result = {}
    cpm = args.cpm
    try:
        result["cpr_air"] = step_encoder_health(mcu)
        if result["cpr_air"] is None:
            raise KeyboardInterrupt

        if cpm <= 0:
            cpm = step_cpm_push(mcu, args.distance)
            if cpm is None:
                raise KeyboardInterrupt
        result["cpm"] = cpm

        for stage in (1, 2, 3):
            r = step_drive(mcu, args.distance, stage, cpm)
            if r:
                result["stage%d" % stage] = r

        result["brake"] = step_brake(mcu, 2, cpm)

    except KeyboardInterrupt:
        print("\n\n  중단됨")
    finally:
        mcu.close()

    # ---- 요약 ----
    hr("측정 요약")
    if result.get("cpm"):
        c = result["cpm"]
        print("\n  config/t870_mcu.yaml 에 넣을 값:")
        print("    \033[1mcounts_per_meter: %.1f\033[0m" % c)
        print("\n  1카운트 = %.1f mm" % (1000.0 / c))
    for s in (1, 2, 3):
        r = result.get("stage%d" % s)
        if r:
            coast = r["coast_counts"] / result["cpm"] if result.get("cpm") else 0
            print("  %d단: %.3f m/s   코스팅 %.3f m" % (s, r["mps"], coast))
    if result.get("brake") and result.get("cpm"):
        print("  B 급정거: %.3f m" % (result["brake"] / result["cpm"]))
    print()


if __name__ == "__main__":
    main()
