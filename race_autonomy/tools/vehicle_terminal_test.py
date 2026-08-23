#!/usr/bin/env python3
"""Single-terminal, low-stage vehicle test controller for Mega v21."""

import argparse
import glob
import os
import re
import select
import sys
import termios
import time
import tty

import serial


HELP = """
키 조작
  w : 전진 1→2→3단계     r : 후진 1→2→3단계
  a : 좌 조향 L1→L2→L3   d : 우 조향 R1→R2→R3
      (반대 방향 키는 해당 방향 1단계부터 시작)
  c : 논리 조향 중앙     t : A0 센서 기반 중앙복귀
  s/Space/x : 즉시 정지
  i : Arduino 상태 조회  q : 정지 후 종료
"""


def find_port(requested):
    if requested:
        return requested
    candidates = sorted(glob.glob("/dev/serial/by-id/*Arduino*"))
    candidates += sorted(glob.glob("/dev/ttyACM*"))
    if not candidates:
        raise RuntimeError("Arduino serial port를 찾지 못했습니다")
    return candidates[0]


class TerminalDrive:
    def __init__(self, port, baudrate):
        self.serial = serial.Serial(port, baudrate, timeout=0, write_timeout=0.5)
        self.drive_command = b"1.00\n"
        self.last_heartbeat = 0.0
        self.buffer = bytearray()
        self.latest_encoder = None
        self.latest_rpm = None
        self.status_pattern = re.compile(
            r"\[상태\].*?RPM=([-+]?\d+(?:\.\d+)?).*?오도=(-?\d+)"
        )

    def send(self, payload):
        self.serial.write(payload)
        self.serial.flush()

    def stop(self):
        self.drive_command = b"1.00\n"
        self.send(b"X\n1.00\n")
        self.last_heartbeat = time.monotonic()
        print("\n[STOP] 구동 및 조향 정지")

    def set_drive(self, command, label):
        self.drive_command = command
        self.send(command)
        self.last_heartbeat = time.monotonic()
        print(f"\n[DRIVE] {label}")

    def heartbeat(self):
        now = time.monotonic()
        if now - self.last_heartbeat >= 0.5:
            self.send(self.drive_command)
            self.last_heartbeat = now

    def read_lines(self):
        lines = []
        waiting = self.serial.in_waiting
        if waiting:
            self.buffer.extend(self.serial.read(waiting))
        while b"\n" in self.buffer:
            raw, _, remainder = self.buffer.partition(b"\n")
            self.buffer = bytearray(remainder)
            text = raw.rstrip(b"\r").decode("utf-8", errors="replace")
            if text:
                print(f"\n[ARDUINO] {text}")
                lines.append(text)
                match = self.status_pattern.search(text)
                if match:
                    self.latest_rpm = float(match.group(1))
                    self.latest_encoder = int(match.group(2))
        return lines

    def close(self):
        try:
            self.stop()
            time.sleep(0.2)
        finally:
            self.serial.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="", help="기본값: Arduino 자동 탐색")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--steer-once",
        choices=("L1", "L2", "L3", "R1", "R2", "R3", "C", "T"),
        help="조향 명령을 한 번 실행하고 상태 확인 후 종료",
    )
    parser.add_argument(
        "--steer-sequence",
        nargs="+",
        choices=("L1", "L2", "L3", "R1", "R2", "R3", "C", "T"),
        help="같은 연결에서 조향 명령들을 순서대로 실행하고 종료",
    )
    parser.add_argument(
        "--neutral-repeat-calibration",
        action="store_true",
        help="R2-C와 L2-C를 각 3회 반복하고 C 직후/2초 후 A0 기록",
    )
    parser.add_argument(
        "--measure-run",
        action="store_true",
        help="10Hz 상태 기록과 정지 시 엔코더 변화량을 표시",
    )
    parser.add_argument(
        "--measure-stage",
        type=int,
        choices=(1, 2, 3),
        default=1,
        help="--measure-run에서 w 키로 실행할 전진 단계 (기본 1)",
    )
    parser.add_argument(
        "--measure-distance-m",
        type=float,
        default=5.0,
        help="평균속도 계산용 실측 거리 (기본 5.0m)",
    )
    args = parser.parse_args()
    port = find_port(args.port)
    print(f"Arduino 연결: {port} @ {args.baudrate}")
    print("다른 ROS/Arduino 시리얼 브리지는 종료되어 있어야 합니다.")
    print(HELP)

    controller = TerminalDrive(port, args.baudrate)
    if args.neutral_repeat_calibration:
        status_patterns = (
            re.compile(r"\[상태\].*?A0=(-?\d+)"),
            re.compile(r"^STATUS,[^,]*,[^,]*,(-?\d+),"),
        )

        def drain(seconds):
            lines = []
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                lines.extend(controller.read_lines())
                time.sleep(0.05)
            return lines

        def query_a0(label):
            controller.send(b"S\n")
            lines = drain(0.7)
            values = []
            for line in lines:
                for pattern in status_patterns:
                    match = pattern.search(line)
                    if match:
                        values.append(int(match.group(1)))
                        break
            value = values[-1] if values else None
            print(f"\n[MEASURE] {label}: A0={value}")
            return value

        results = []
        try:
            time.sleep(2.5)
            controller.stop()
            drain(0.4)
            for direction in ("R2", "L2"):
                for repeat in range(1, 4):
                    controller.send((direction + "\n").encode("ascii"))
                    print(f"\n[CALIBRATION] {direction} #{repeat}")
                    drain(1.5)
                    controller.send(b"C\n")
                    print("\n[CALIBRATION] C")
                    drain(1.5)
                    immediate = query_a0(f"{direction} #{repeat} C 직후")
                    time.sleep(2.0)
                    delayed = query_a0(f"{direction} #{repeat} C 2초 후")
                    results.append((direction, repeat, immediate, delayed))
            print("\n[CALIBRATION SUMMARY]")
            for direction, repeat, immediate, delayed in results:
                print(
                    f"{direction} #{repeat}: immediate={immediate}, "
                    f"after_2s={delayed}"
                )
        finally:
            controller.close()
            print("종료 완료: 최종 정지 명령 전송됨")
        return
    if args.steer_once or args.steer_sequence:
        commands = args.steer_sequence or [args.steer_once]
        try:
            time.sleep(2.5)
            controller.stop()
            time.sleep(0.3)
            for command in commands:
                controller.send((command + "\n").encode("ascii"))
                print(f"\n[STEER] {command}")
                end = time.monotonic() + 1.5
                while time.monotonic() < end:
                    controller.read_lines()
                    time.sleep(0.05)
            controller.send(b"S\n")
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                controller.read_lines()
                time.sleep(0.05)
        finally:
            controller.close()
            print("종료 완료: 최종 정지 명령 전송됨")
        return
    old_settings = termios.tcgetattr(sys.stdin.fileno())
    measurement_start_encoder = None
    measurement_started_at = None
    last_measurement_poll = 0.0
    drive_direction = None
    drive_stage = 0
    steer_direction = None
    steer_stage = 0
    try:
        # Let the Mega bootloader finish, then enter a known safe state.
        time.sleep(2.5)
        controller.stop()
        tty.setcbreak(sys.stdin.fileno())
        while True:
            controller.heartbeat()
            controller.read_lines()
            now = time.monotonic()
            if args.measure_run and now - last_measurement_poll >= 0.1:
                controller.send(b"S\n")
                last_measurement_poll = now
            readable, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not readable:
                continue
            key = os.read(sys.stdin.fileno(), 1).decode(errors="ignore").lower()
            if key == "w":
                measurement_start_encoder = controller.latest_encoder
                measurement_started_at = time.monotonic()
                stage_commands = {1: b"2.00\n", 2: b"3.00\n", 3: b"4.00\n"}
                if args.measure_run:
                    drive_stage = args.measure_stage
                else:
                    drive_stage = min(drive_stage + 1, 3) if drive_direction == "forward" else 1
                drive_direction = "forward"
                controller.set_drive(
                    stage_commands[drive_stage],
                    f"전진 {drive_stage}단계",
                )
                if args.measure_run:
                    print(
                        f"\n[MEASURE START] encoder={measurement_start_encoder}; "
                        f"stage={drive_stage}; "
                        f"{args.measure_distance_m:g}m 지점에서 Space를 누르세요"
                    )
            elif key == "r":
                stage_commands = {1: b"6.00\n", 2: b"7.00\n", 3: b"8.00\n"}
                drive_stage = min(drive_stage + 1, 3) if drive_direction == "reverse" else 1
                drive_direction = "reverse"
                controller.set_drive(stage_commands[drive_stage], f"후진 {drive_stage}단계")
            elif key == "a":
                steer_stage = min(steer_stage + 1, 3) if steer_direction == "left" else 1
                steer_direction = "left"
                command = f"L{steer_stage}"
                controller.send((command + "\n").encode("ascii"))
                print(f"\n[STEER] 좌 {command}")
            elif key == "d":
                steer_stage = min(steer_stage + 1, 3) if steer_direction == "right" else 1
                steer_direction = "right"
                command = f"R{steer_stage}"
                controller.send((command + "\n").encode("ascii"))
                print(f"\n[STEER] 우 {command}")
            elif key == "c":
                controller.send(b"C\n")
                steer_direction = None
                steer_stage = 0
                print("\n[STEER] 중앙")
            elif key == "t":
                controller.send(b"T\n")
                steer_direction = None
                steer_stage = 0
                print("\n[STEER] A0 센서 기반 중앙복귀")
            elif key in ("s", "x", " "):
                stopped_at = time.monotonic()
                controller.stop()
                drive_direction = None
                drive_stage = 0
                steer_direction = None
                steer_stage = 0
                if args.measure_run and measurement_started_at is not None:
                    controller.send(b"S\n")
                    end = time.monotonic() + 0.5
                    while time.monotonic() < end:
                        controller.read_lines()
                        time.sleep(0.05)
                    elapsed = stopped_at - measurement_started_at
                    average_speed = args.measure_distance_m / elapsed
                    delta = None
                    if (
                        measurement_start_encoder is not None
                        and controller.latest_encoder is not None
                    ):
                        delta = controller.latest_encoder - measurement_start_encoder
                    print(
                        f"\n[MEASURE STOP] elapsed={elapsed:.2f}s, "
                        f"encoder={controller.latest_encoder}, delta={delta}, "
                        f"rpm={controller.latest_rpm}, "
                        f"average_speed={average_speed:.3f}m/s"
                    )
                    measurement_started_at = None
            elif key == "i":
                controller.send(b"S\n")
            elif key == "q":
                break
    except KeyboardInterrupt:
        print("\nCtrl+C 감지")
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)
        controller.close()
        print("종료 완료: 최종 정지 명령 전송됨")


if __name__ == "__main__":
    main()
