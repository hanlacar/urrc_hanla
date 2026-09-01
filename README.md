# URRC Hanla 자율주행 차량

이 저장소는 카메라 경로 생성·Pure Pursuit 제어·구간 미션 판단과
T870 MCU 명령 중재기를 포함한다. ROS 2 Jazzy를 사용하며 모든 팀 노드는
동일한 `ROS_DOMAIN_ID=10`에서 실행해야 한다.

## 전체 데이터 흐름

```text
카메라 영상 → YOLO/도로·차선 분할 → BEV 경로 → Pure Pursuit 조향
                                      ↓
GPS 구간 번호 → course_mission → /camera_drive, /camera_wheel
                                      ↓
라이다 정지/회피 ───────────────→ T870 MCU manager → Arduino v37 → 실차
                                      ↑
                        /mcu/odom, 속도, 엔코더 상태
```

최종 구동·조향·정지 판단은 MCU manager가 한다. 카메라 노드는 후보 명령을
발행하며 `/lidar_stop`, `/camera_stop`, `/manual_stop`, `/estop_lock`은
일반 경로 명령보다 우선한다.

## 실행 전 준비

각 터미널에서 다음 환경을 먼저 적용한다.

```bash
export ROS_DOMAIN_ID=10
set +u
source /opt/ros/jazzy/setup.bash
source /home/parkjinwoo/urrc_hanla/race_autonomy/ros2_ws/install/setup.bash
```

MCU 워크스페이스는 다음 스크립트로 실행한다.

```bash
cd /home/parkjinwoo/urrc_hanla/T870_MCU
./실행.sh
```

실차 시험 전 반드시 확인한다.

```bash
ros2 topic echo /mcu/safety_state
ros2 topic echo /mcu/current_mode
ros2 topic echo /camera_drive
ros2 topic echo /camera_wheel
ros2 topic echo /camera/path_valid
```

차량을 지면에 내려놓기 전 E-stop, 수동 정지 및 `/lidar_stop`이 실제로
0단 정지를 발생시키는지 잭업 상태에서 먼저 검증한다.

## 구간 번호

| 번호 | 구간 | 기본 조향 | 주요 판단 |
|---:|---|---|---|
| 1 | 출발 직선 | 카메라 | 정지선과 traffic20 무시, 경로추종 |
| 2 | 경사로 | 카메라 | IMU pitch, 정지선 2개, MCU odom |
| 3 | D자 코스 | 카메라 | 경로 유효성 및 곡률 기반 감속/정지 |
| 4 | 교차로 | 카메라 | 정지선 2m와 직진 신호 |
| 5 | S자 코스 | 라이다 회피 시 라이다 | 카메라 경로 + 장애물 회피 게이트 |
| 6 | 교차로 | 카메라 | 정지선 2m와 직진 신호 |
| 7 | T 주차 | 라이다 | 라이다 주차 명령 |
| 8 | 좌회전 교차로 | 카메라 | 좌회전 신호만 통행 허용 |
| 9 | 가속 구간 | 카메라 | traffic20 재검출과 odom 간격 |
| 10 | 평행 주차 | 라이다 | 라이다 주차 명령 |
| 11 | 마지막 교차로 | 카메라 | 정지선 2m와 최종 신호차 |

구간은 다음처럼 수동 전환해 시험할 수 있다.

```bash
ros2 topic pub --once /mission/section std_msgs/msg/Int8 '{data: 3}'
```

## 객체 검출 안정화

정지선, traffic20, 신호등은 한 프레임 검출만으로 확정하지 않는다.

1. 같은 종류의 객체가 3개 추론 프레임에 연속으로 나타나야 한다.
2. 연속 박스의 IoU가 `0.3` 이상이어야 같은 위치의 객체로 인정한다.
3. 위치가 크게 이동하거나 객체가 사라지면 연속 프레임 카운트를 초기화한다.

이 조건은 순간 오검출과 화면 흔들림을 줄이기 위한 것이다. 프레임 수가 아니라
카메라 입력 시간으로 판단하지 않으므로 추론 FPS가 낮으면 확정도 늦어진다.

## 1번 출발 구간

1번에서는 정지선과 traffic20 토픽이 발행돼도 주행 판단에 사용하지 않는다.
카메라 경로와 곡률 속도계획만 사용한다. 다만 다음 안전정지는 항상 유효하다.

- 카메라 경로 또는 속도계획 무효
- 카메라 명령 timeout
- 라이다·카메라·수동 stop
- E-stop

## 2번 경사로

경사로 진입은 pitch `+15도` 이상이 0.5초 유지될 때 확정한다. 정지선은
다음 순서로 구분한다.

1. 첫 번째 정지선을 `3프레임 + 위치 일치`로 확정한다.
2. 그 순간의 누적 `/mcu/odom` 이동거리를 저장한다.
3. 정지선이 0.5초 이상 사라져야 재검출을 허용한다.
4. 첫 번째 선 이후 1.5m 이상 이동한 뒤 검출된 선만 두 번째로 인정한다.
5. 두 번째 정지선의 카메라 추정거리가 2.0m 이내면 정지한다.
6. 정확히 3초 정지한 뒤 카메라 경로추종 2단으로 출발한다.

관련 설정은 `race_control/config/course_mission.yaml`에 있다.

```yaml
ramp_pitch_deg: 15.0
ramp_pitch_confirm_sec: 0.5
ramp_stop_line_min_separation_m: 1.5
stop_line_trigger_distance_m: 2.0
ramp_second_line_stop_sec: 3.0
ramp_post_stop_drive_sec: 0.0
```

첫 정지선 검출 시 odom이 없거나 0.5초 이상 오래되면
`RAMP:STOP_LINE_ODOM_INVALID`로 정지한다. 현재 wheel odom은 구동륜 슬립의
영향을 받으므로 1.5m 값은 반드시 저속 실차에서 재확인해야 한다.

## 3번 D자 코스 안전장치

- 경로 또는 조향 입력이 0.5초 이상 끊기면 정지
- 경로 신뢰도 0.45 미만이면 속도계획 무효로 정지
- 경로 곡률 0.25/m 이상이면 1단 감속
- 경로 곡률 0.60/m 이상이면 정지하고 1초 유지
- 구간 출력은 최대 1단
- 조향각은 최대 ±27도
- 조향 변화율은 최대 20도/초
- 모든 구간 공통 stop과 E-stop은 항상 최우선

`/camera/path_jump_detected`는 현재 진단용이며 그 값만으로 직접 정지하지는
않는다. D자 구간의 라이다는 장애물 급정지는 할 수 있지만 조향 우회 권한은
기본적으로 카메라에 있다.

## Pure Pursuit

기본 lookahead는 1.0m이며 속도에 따라 증가하고 최대 1.5m로 제한된다.

```text
lookahead = clamp(1.0 + 현재 목표속도 × 1.0, 1.0, 1.5)
```

| 상태 | lookahead |
|---|---:|
| 정지·초저속 | 1.0m |
| 1단 0.229m/s | 약 1.23m |
| 2단 0.526m/s | 1.5m 상한 |

## 9번 traffic20 재검출

traffic20은 `3프레임 + 위치 일치` 후 인정한다. 첫 번째 표지 위치의 누적
odom을 저장하고, 표지가 0.5초 이상 사라진 다음 2.0m 이상 이동한 위치에서
재검출돼야 두 번째 표지로 인정한다. odom이 유효하지 않으면 가속 판단을
진행하지 않고 안전정지한다.

## MCU 펌웨어

사용할 펌웨어는 `T870_MCU/firmware/T870_MCU_v37/T870_MCU_v37.ino`이다.
v29~v36은 혼동 방지를 위해 저장소에서 제거했다.

Arduino IDE 설정:

- Board: Arduino Mega or Mega 2560
- Baud rate: 115200
- 업로드 후 부팅 로그에서 `MCU_BOOT,v37` 확인

## 저장 영상 녹화

```bash
ffmpeg -f v4l2 -input_format yuyv422 -framerate 60 -video_size 640x480 \
  -i /dev/video6 -c:v libx264 -preset ultrafast -crf 18 -pix_fmt yuv420p \
  "$HOME/urrc_hanla/recordings/raw/$(date +%Y%m%d_%H%M%S).mp4"
```

실제 카메라 시험에서는 저장 영상 publisher를 동시에 켜면 안 된다. 두 노드가
같은 `/camera/image_raw`를 발행하면 프레임이 섞여 경로가 튈 수 있다.
