# T870 MCU

BROON T870 개조 자율주행 차량(24V, Ackermann)의 하위제어 계층.
각 팀(카메라·라이다·GPS)의 명령을 중재해 아두이노로 보내고,
차량 상태를 ROS 2 토픽으로 되돌린다.

```
카메라 ─┐
라이다 ─┼─▶ manager ─▶ bridge ─▶ Arduino Mega 2560 ─▶ 모터
GPS   ─┘                  ▲
                          └── STATUS (엔코더·조향·상태)
```

ROS 2 Jazzy / Ubuntu 24.04 / Arduino Mega 2560 (115200 baud)

---

## 설치 — 한 번만

배포 zip 을 받았다면 그 안의 `설치.sh` 하나면 끝난다.
옛 소스와 빌드 산출물을 **지우지 않고** `~/t870_backup_<날짜>/` 로 옮긴 뒤
새 파일을 넣고 빌드하고 정합성 감사까지 돌린다.

```bash
cd ~/Downloads && unzip -o T870_MCU_0829_팀배포_v6.zip
cd T870_MCU_0829
./설치.sh              # 미리보기 — 아무것도 바꾸지 않는다
./설치.sh --apply      # 실제 설치
```

저장소에서 직접 받는 경우:

```bash
git clone https://github.com/hanlacar/T870_MCU.git ~/mcu_ws
cd ~/mcu_ws
chmod +x setup.sh && ./setup.sh      # 패키지·dialout·udev 규칙
colcon build --symlink-install
```

`setup.sh` 가 하는 일: `python3-serial` 설치, `dialout` 그룹 추가,
**udev 규칙 등록으로 포트 이름 고정**(`/dev/t870_mcu`), 현재 포트 권한 부여.

---

## 실행 — 매번 이것만

```bash
cd ~/mcu_ws

./실행.sh 점검      # 이 PC 환경 점검 — 안 될 때 제일 먼저
./실행.sh           # 브릿지 + 매니저 (평소 이것)
./실행.sh 조종      # 터미널로 직접 운전 (WASD)
./실행.sh 상태      # 지금 뭐가 도는지 한눈에
```

**`source` 도 `export` 도 스크립트가 알아서 한다.** 터미널을 새로 열어도
따로 칠 것이 없다. `ros2 run` 으로 브릿지·매니저를 따로 띄우지 말 것 —
두 번 뜨면 시리얼 포트를 서로 뺏고 params-file 을 빠뜨리기 쉽다.

포트는 적을 필요가 없다. 브릿지가 USB 장치 정보(`/dev/serial/by-id`)로
아두이노를 스스로 찾는다. **번호(`ttyACM0`, `ACM1`)는 꽂는 순서마다 바뀌므로
믿지 않는다.**

### 수동 조종

```bash
./실행.sh 조종
```

기본은 **대기(STANDBY)** 다. 팀 노드를 방해하지 않으려고 구동·조향을
발행하지 않는다. **`0` 을 눌러야 조종권을 잡는다.**

```
0 조종권 잡기/놓기   W 전진   S 후진   A 좌   D 우   C 중앙
Space 정지   F 급정거   E 비상정지   R E-Stop 해제   Q 종료
```

---

## ⚠ ROS_DOMAIN_ID 와 RMW — 1순위 사고 원인

ROS 2 는 이 둘 중 **하나만 달라도 노드끼리 서로 존재조차 모른다.**
`ros2 node list` 에 안 보이고, 토픽도 안 보이고, **에러도 안 난다.**

값은 `t870_env.sh` **한 곳에만** 있다.

```bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`./실행.sh` 는 이걸 자동으로 읽는다. **다른 워크스페이스**(`mmission_ws`,
라이다 등)를 띄우는 터미널에서는 이 한 줄을 먼저 칠 것:

```bash
source ~/mcu_ws/t870_env.sh
```

각자 고치지 말 것. 바꿔야 하면 MCU 담당에게 말하고 전원이 같이 받는다.

---

## 토픽 계약

### 각 팀이 발행

```
/camera_drive   /camera_wheel   /camera_stop
/lidar_drive    /lidar_wheel    /lidar_stop
/gps_drive      /gps_wheel      /gps_stop
/drive_mode     (GPS팀 — String, 구간 이름)
/estop_lock     (공통)
```

| 타입 | 값 |
|---|---|
| `drive` **Float32** | `0` 정지 / `1` `2` `3` 전진 / `-1` 후진 |
| `wheel` **Int32** | `-27` ~ `+27` 도, **+ 가 우측** |
| `stop` **Bool** | `true` = 즉시 정지 |

**최소 4Hz 연속 발행.** 0.5초 끊기면 그 소스는 죽은 것으로 본다.

> **타입을 정확히 맞출 것.** `Float32` 로 쏴야 할 곳에 `Int32` 를 쓰면
> ROS 는 에러를 내지 않고 **메시지만 조용히 안 온다.**
> 브릿지·매니저가 3초마다 확인해 `[ERROR] ... 메시지 타입이 다르다` 로 찍는다.

### 각 팀이 구독

```
/mcu/encoder        Int32     엔코더 누적 카운트 (부호 있음)   ★ 반드시 Int32
/mcu/distance_m     Float32   누적 거리 [m]
/mcu/speed_mps      Float32   속도 [m/s]
/mcu/speed_valid    Bool      위 값 유효 여부
/mcu/steer_deg      Float32   조향각 추정
/mcu/fw_state       String    READY / ACTIVE / FAULT / ESTOP
/mcu/safety_state   String    중재 상태
/mcu/current_mode   String    매니저가 받아들인 구간 모드
/mcu/ready          Bool      아두이노 통신 정상 + READY
/mcu/manager_ready  Bool      중재기가 유효한 명령을 내보내는 중
/mcu/odom           Odometry  참고용 (TF 는 발행하지 않는다)
```

### 내부 전용 — 발행 금지

```
/mcu/cmd_drive   /mcu/cmd_wheel   /mcu/cmd_stop
```

### odom / TF 주인

`/odom` 과 `odom → base_footprint` TF 는 **`mission_manager/wheel_odom` 이
담당한다.** 브릿지는 `publish_tf: false` 이고 `/mcu/odom` 에만 참고용으로 낸다.
(wheel_odom 이 IMU yaw 를 쓰고, 우리 조향각 추정은 아직 신뢰할 수 없다)

⚠ `base_footprint → base_link` 를 누군가 반드시 발행해야 Nav2 가 돈다.

---

## 중재 규칙

```
[1] /estop_lock       래치. /mcu/reset_estop 서비스로만 해제
[2] /<팀>_stop        급정거. 모드·우선순위 무시
[3] manual            최우선 (안전요원 개입)
[4] 구동 = 라이다 > 카메라 > GPS
[5] 조향 = 모드가 정한 팀만
```

### 구간 모드 (GPS팀 이름 기준)

```
IDLE  NORMAL  SLOPE  D_COURSE  INTERSECTION
S_COURSE  T_PARK  ACCEL  PARALLEL_PARK  OUT
```

| 모드 | 조향 권한 |
|---|---|
| `T_PARK`, `PARALLEL_PARK` | 라이다 |
| 그 외 | 카메라 |

목록의 주인은 `config/t870_mcu.yaml` **한 곳**이다. 코드에 박지 말 것.
이름이 바뀌면 `known_modes` 와 `wheel_owner_overrides` 두 줄만 고친다.

---

## 실측값

```
축거              0.77 m      ✅ 실측
counts_per_meter  199.8       ✅ 실측 (955카운트 / 4.78m, 0829)
최대 조향각       27도        ⚠ 무부하 실측
바퀴 지름         0.26 m      ⚠ 스펙값 (둘레 0.817m 는 여기서 계산)
최소 회전반경     1.43 m      ⚠ 계산값
속도 1단          0.229 m/s
속도 2단          0.526 m/s   (하중 실린 상태 0.455 실측)
```

> **`counts_per_meter` 와 펌웨어 `ENCODER_DEBOUNCE_US` 는 한 쌍이다.**
> 199.8 은 디바운스 **200us** 조건에서 측정된 값이다.
> 디바운스를 바꾸면 반드시 `counts_per_meter` 를 다시 측정할 것.
> (v34 에서 2000us 로 올렸다가 카운트를 99% 잃었다. v36 에서 되돌렸다)

wheel_odom 을 쓸 때: `encoder_m_per_tick = 1 / 199.8 = 0.005005`

---

## 펌웨어

`firmware/T870_MCU_v36.ino` 를 Arduino IDE 로 업로드.
Board: **Arduino Mega or Mega 2560** / 115200 baud

> **⚠ 펌웨어 업로드는 MCU 담당만 한다.** 서로 다른 버전이 올라가면
> 어느 코드가 도는지 아무도 모르게 된다.

부팅 로그로 확인:

```
MCU_BOOT,v36
ENCODER,CPR,163.0,DEBOUNCE_US,200,UPDATE_MS,100
```

### 주요 기능

| 기능 | 명령 | 내용 |
|---|---|---|
| 급제동 | `B` | 역토크 펄스. 엔코더가 정지를 감지하면 끊는다 (상한 300ms) |
| 경사로 홀딩 | `H40` / `H0` | 지정 PWM 으로 버팀. 8초 자동 해제 |
| **자동 안티롤백** | `AR1` `AR2` `AR0` | 정지 중 밀리면 스스로 토크를 넣어 버틴다 |
| 안티롤백 세기 | `ARP70` | 기본 70, 상한 90 |
| 오도 리셋 | `O` | 캘리브레이션용 |
| 도움말 | `?` | 전체 명령 목록 |

**안티롤백**은 기본 꺼짐이다. 경사로에서 `AR1`(오르막·전진 버팀) /
`AR2`(내리막·후진 버팀)로 켠다. **방향을 반대로 켜면 차가 스스로 밀려나간다.**
그 경우 1.0m/s 이상 미끄러지는 것을 감지해 0.2초 안에 스스로 꺼진다.

---

## 도구

| 파일 | 용도 |
|---|---|
| `실행.sh` | 실행·조종·점검 (평소 쓰는 것) |
| `설치.sh` | 설치·백업·빌드·감사 |
| `tools/preflight.py` | 이 PC 환경 점검 (DOMAIN·권한·빌드·중복·토픽) |
| `tools/audit.py` | 소스 ↔ yaml 정합성 감사 (ROS 불필요) |
| `tools/drive_wasd.py` | 키보드 수동 조종 |
| `tools/check_ports.py` | 어느 포트가 뭔지 확인 |
| `tools/odom_calib.py` | counts_per_meter 검증 (ROS) |
| `tools/measure.py` | counts_per_meter 실측 (시리얼 직결) |
| `tools/team_monitor.py` | 각 팀 토픽 수신 상태 |
| `tools/mode_sim.py` | 모드 전환 시뮬레이션 |

---

## 문제 해결

| 증상 | 조치 |
|---|---|
| **뭘 해도 안 됨** | `./실행.sh 점검` — 대부분 여기서 나온다 |
| 상대 노드가 안 보임 | DOMAIN_ID / RMW 불일치. `source t870_env.sh` |
| 소스를 고쳤는데 안 바뀜 | `install/` 이 옛것. `colcon build --symlink-install` |
| `colcon build` 가 패키지 중복으로 실패 | `touch ~/t870_backup_*/COLCON_IGNORE` |
| 포트를 못 찾음 | `python3 tools/check_ports.py` |
| `Permission denied` | `sudo usermod -aG dialout $USER` 후 재로그인 |
| `Device or resource busy` | `sudo fuser -k /dev/ttyACM*` — 다른 노드가 잡고 있다 |
| 텔레메트리 두절 | 포트가 아두이노가 아닐 가능성 |
| `ESTOP` 안 풀림 | 조종 화면에서 `R` |
| 차가 0.5초마다 섬 | 팀 노드 발행 주기 부족 (4Hz 이상 필요) |
| 명령을 쐈는데 반응 없음 | 로그에 `메시지 타입이 다르다` / `QoS 가 안 맞는다` 가 있는지 |
| 엔코더가 안 오름 | 펌웨어 배너의 `DEBOUNCE_US` 확인 (200 이어야 한다) |

---

## 아직 안 된 것

- **조향각 센서.** A0 포텐셔미터 가동폭이 너무 좁아 폐루프 불가.
  작년에는 같은 부품으로 875카운트를 썼다 — 부품이 아니라 **기구 연결** 문제다.
  AS5600(자기식 절대각) 도입 검토 중. 피니언 회전수 실측 필요.
- **쿼드러처 미사용.** B상 배선(D3)은 이미 있다. 노이즈 때문에 껐는데
  작년에는 같은 모터에서 동작했다 — 배선 문제일 가능성이 크다.
- 센서 3개(라이다·카메라·GPS) 장착 위치 실측 → `t870_frames.yaml` 은 추정치.
- 코스팅 거리, 지면 조향각, 잭업 회전수 미측정.
- 안티롤백(v36) 경사로 실차 검증 미실시.
