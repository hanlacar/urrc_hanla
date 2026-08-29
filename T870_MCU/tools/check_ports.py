#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_ports_v1_0826.py  —  어느 포트가 뭔지 먼저 확인

포트 번호는 꽂는 순서에 따라 바뀐다.
GPS·라이다·아두이노가 자리를 바꿔 앉는 일이 실제로 있었으므로,
측정이나 주행을 시작하기 전에 이걸 먼저 돌린다.

실행:
    python3 tools/check_ports.py

udev 규칙까지 만들려면:
    python3 tools/check_ports.py --udev
"""

import argparse
import glob
import os
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial 없음:  sudo apt install python3-serial")


def by_id_map():
    """/dev/serial/by-id 를 읽어 실제 장치명 → 포트 로 정리."""
    out = {}
    base = "/dev/serial/by-id"
    if not os.path.isdir(base):
        return out
    for name in os.listdir(base):
        real = os.path.realpath(os.path.join(base, name))
        out[real] = name
    return out


def guess(name):
    low = name.lower()
    if "arduino" in low:
        return "Arduino (MCU)", "t870_mcu"
    if "u-blox" in low or "u_blox" in low or "gnss" in low:
        return "u-blox GPS", "t870_gps"
    if "cp2102" in low or "silicon_labs" in low:
        return "CP2102 (라이다 가능성)", "t870_lidar"
    if "ch340" in low or "ch910" in low:
        return "CH340", None
    return "알 수 없음", None


def probe(port, seconds=3.0):
    """포트를 열어 무엇이 흘러나오는지 본다. 아무것도 안 보낸다."""
    try:
        ser = serial.Serial(port, 115200, timeout=0.3)
    except Exception as e:
        return "열기 실패: %s" % e, ""
    time.sleep(0.3)
    ser.reset_input_buffer()
    buf = b""
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        try:
            buf += ser.read(256)
        except Exception:
            break
        if len(buf) > 2000:
            break
    ser.close()

    text = buf.decode("utf-8", "replace")
    if "MCU_BOOT" in text or "STATUS," in text:
        verdict = "\033[32mArduino 확인 (STATUS/MCU_BOOT)\033[0m"
    elif "$GN" in text or "$GP" in text:
        verdict = "\033[33mGPS 확인 (NMEA)\033[0m"
    elif buf:
        verdict = "데이터는 나오는데 정체 불명"
    else:
        verdict = "조용함 (요청해야 응답하는 장치일 수 있음)"

    sample = text.strip().replace("\r", "").split("\n")
    sample = [s for s in sample if s][:2]
    return verdict, "  |  ".join(sample)[:100]


def make_udev(found):
    """by-id 이름에서 시리얼 번호를 뽑아 udev 규칙을 만든다."""
    lines = []
    for port, name, kind, symlink in found:
        if not symlink:
            continue
        serial_no = None
        parts = name.split("_")
        for p in reversed(parts):
            if len(p) >= 8 and p.replace("-", "").isalnum():
                serial_no = p.split("-")[0]
                break
        if serial_no:
            lines.append(
                'SUBSYSTEM=="tty", ATTRS{serial}=="%s", SYMLINK+="%s"'
                % (serial_no, symlink))

    if not lines:
        print("\n  udev 규칙을 만들 수 없다 (시리얼 번호 추출 실패)")
        return

    rule = "\n".join(lines) + "\n"
    print("\n" + "=" * 62)
    print("  udev 규칙 (포트 번호가 바뀌어도 이름이 고정된다)")
    print("=" * 62)
    print(rule)
    print("  아래를 그대로 터미널에 붙여넣으면 적용된다:\n")
    print("sudo tee /etc/udev/rules.d/99-t870.rules > /dev/null << 'EOF'")
    print(rule.rstrip())
    print("EOF")
    print("sudo udevadm control --reload-rules && sudo udevadm trigger")
    print("ls -l /dev/t870_*")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--udev", action="store_true", help="udev 규칙도 출력")
    ap.add_argument("--probe", action="store_true",
                    help="각 포트를 열어 실제 데이터를 확인 (느림)")
    args = ap.parse_args()

    print("=" * 62)
    print("  T870 포트 확인")
    print("=" * 62)

    # setup.sh 의 udev 규칙이 만들어주는 고정 이름부터 보여준다.
    # 이게 살아 있으면 포트 번호는 신경 쓸 필요가 없다.
    fixed = sorted(glob.glob("/dev/t870_*"))
    if fixed:
        print("\n  고정 이름 (udev):")
        for link in fixed:
            print("     %-16s -> %s" % (link, os.path.realpath(link)))
    else:
        print("\n  고정 이름 없음 — ./setup.sh 를 한 번 실행하면"
              " /dev/t870_mcu 로 고정된다")

    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if not ports:
        sys.exit("\n  시리얼 포트가 하나도 없다. USB 연결을 확인하라.")

    idmap = by_id_map()
    found = []

    for port in ports:
        name = idmap.get(port, "")
        kind, symlink = guess(name) if name else ("by-id 없음", None)
        found.append((port, name, kind, symlink))

    print()
    for port, name, kind, symlink in found:
        mark = "\033[32m★\033[0m" if "Arduino" in kind else " "
        print(" %s %-16s %s" % (mark, port, kind))
        if name:
            print("     %s" % name)
        if args.probe:
            verdict, sample = probe(port)
            print("     → %s" % verdict)
            if sample:
                print("       %s" % sample)
        print()

    mcu = [p for p, n, k, s in found if "Arduino" in k]
    if mcu:
        print("=" * 62)
        print("  \033[1m아두이노 포트: %s\033[0m" % mcu[0])
        print("=" * 62)
        print("""
  브릿지 실행:
    cd <워크스페이스> && source install/setup.bash
    ros2 launch t870_mcu t870_mcu.launch.py port:=%s

  권한이 없으면:
    sudo chmod 666 %s
""" % (mcu[0], mcu[0]))
    else:
        print("=" * 62)
        print("  \033[31m아두이노를 못 찾았다\033[0m")
        print("=" * 62)
        print("""
  USB 연결과 전원을 확인하라.
  --probe 옵션으로 각 포트에서 실제로 뭐가 나오는지 볼 수 있다:
    python3 tools/check_ports.py --probe
""")

    if args.udev:
        make_udev(found)


if __name__ == "__main__":
    main()
