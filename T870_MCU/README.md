# T870 MCU

BROON T870 개조 자율주행 차량의 하위제어 계층.
각 팀(카메라·라이다·GPS)의 명령을 중재해 아두이노로 보내고,
차량 상태를 ROS 2 토픽으로 되돌린다.

```
카메라 ─┐
라이다 ─┼─▶ manager ─▶ bridge ─▶ Arduino Mega ─▶ 모터
GPS   ─┘                  ▲
                          └── STATUS (엔코더·조향·상태)
```

---

## 설치

```bash
git clone https://github.com/hanlacar/T870_MCU.git ~/mcu_ws   # 위치는 자유
cd <워크스페이스>
chmod +x setup.sh && ./setup.sh
colcon build --symlink-install
source install/setup.bash
```

`setup.sh` 가 하는 일:
- `python3-serial`, `colcon` 설치
- `dialout` 그룹 추가
- **udev 규칙 등록 — 포트 이름 고정** (`/dev/t870_mcu`)
- 현재 연결된 포트에 즉시 권한 부여

---

## 실행

```bash
ros2 launch t870_mcu t870_mcu.launch.py
```

**포트를 적을 필요가 없다.** 브릿지가 USB 장치 정보로 아두이노를 스스로 찾는다.
번호(`/dev/ttyACM0`, `ACM1`)는 꽂는 순서마다 바뀌므로 믿지 않는다.

특정 포트를 강제하려면:
```bash
ros2 launch t870_mcu t870_mcu.launch.py port:=/dev/t870_mcu
```

### 수동 조종

```bash
python3 tools/drive_wasd.py
```

```
W 전진   S 후진   A 좌   D 우   C 중앙
Space 정지   F 급정거   R E-Stop 해제   Q 종료
```

---

## 토픽 계약

### 각 팀이 발행

```
/camera_drive   /camera_wheel   /camera_stop
/lidar_drive    /lidar_wheel    /lidar_stop
/gps_drive      /gps_wheel      /gps_stop
/vehicle_mode   (GPS팀)
/estop_lock     (공통)
```

| 타입 | 값 |
|---|---|
| `drive` Float32 | `0` 정지 / `1` `2` `3` 전진 / `-1` 후진 |
| `wheel` Int32 | `-27` ~ `+27` 도, **+ 가 우측** |
| `stop` Bool | `true` = 즉시 정지 |

**최소 4Hz 연속 발행.** 0.5초 끊기면 그 소스는 죽은 것으로 본다.

### 각 팀이 구독

```
/mcu/encoder        Int32     엔코더 카운트   ★ 반드시 Int32
/mcu/distance_m     Float32   누적 거리 [m]
/mcu/speed_mps      Float32   속도 [m/s]
/mcu/speed_valid    Bool      위 값 유효 여부
/mcu/steer_deg      Float32   조향각 추정
/mcu/fw_state       String    READY / ACTIVE / FAULT / ESTOP
/mcu/safety_state   String    중재 상태
/odom               Odometry
```

`/mcu/encoder` 를 `Float32` 로 구독하면 에러도 콜백도 없이 조용히 실패한다.

### 내부 전용 — 발행 금지

```
/mcu/cmd_drive   /mcu/cmd_wheel   /mcu/cmd_stop
```

---

## 중재 규칙

```
[1] /estop_lock       래치. /mcu/reset_estop 서비스로만 해제
[2] /<팀>_stop        급정거. 모드·우선순위 무시
[3] manual            최우선 (안전요원 개입)
[4] 구동 = 라이다 > 카메라 > GPS
[5] 조향 = 모드가 정한 팀만
```

| 모드 | 조향 권한 |
|---|---|
| `T_PARK`, `PARALLEL_PARK` | 라이다 |
| 그 외 | 카메라 |

---

## 실측값 (2026-08-26)

```
바퀴 지름        0.26 m   (둘레 0.817 m)
축거             0.73 m
최대 조향각      27도
최소 회전반경    1.43 m
counts_per_meter 543.0    (실차 엔코더 측정값)
속도 1단         0.229 m/s
속도 2단         0.526 m/s
```

---

## 펌웨어

`firmware/T870_MCU_v30.ino` 를 Arduino IDE 로 업로드.
Board: **Arduino Mega or Mega 2560** / 115200 baud

부팅 로그로 확인:
```
MCU_BOOT,v30
```

---

## 도구

| 파일 | 용도 |
|---|---|
| `tools/drive_wasd.py` | 키보드 수동 조종 |
| `tools/check_ports.py` | 어느 포트가 뭔지 확인 |
| `tools/measure.py` | counts_per_meter 실측 (시리얼 직결) |
| `tools/team_monitor.py` | 각 팀 토픽 수신 상태 |
| `tools/mode_sim.py` | 모드 전환 시뮬레이션 |

---

## 문제 해결

| 증상 | 조치 |
|---|---|
| 포트를 못 찾음 | `python3 tools/check_ports.py` |
| `Permission denied` | `./setup.sh` 재실행 또는 `sudo chmod 666 /dev/ttyACM*` |
| `Device or resource busy` | `sudo fuser -k /dev/ttyACM*` |
| 텔레메트리 두절 | 포트가 아두이노가 아닐 가능성. `check_ports` |
| `ESTOP` 안 풀림 | 조종 화면에서 `R` |
| 차가 0.5초마다 섬 | 팀 노드 발행 주기 부족 (4Hz 이상 필요) |

---

## 아직 안 된 것

- 조향각 센서(A0 포텐셔미터) 변화폭이 4카운트뿐이라 사용 불가.
  조향 모터 엔코더를 D18 에 배선하면 v30 이 자동으로 읽는다.
- 코스팅 거리, 실측 선회반경 미측정.
- 펌웨어 `ENCODER_CPR` 이 71 로 잘못 설정 → `/mcu/rpm` 이 약 6배 크게 나온다.
