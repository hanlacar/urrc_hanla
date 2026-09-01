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

## 처음 보는 사람을 위한 시스템 설명

자율주행은 하나의 AI가 차량을 직접 움직이는 구조가 아니다. 이 프로젝트는
인지, 판단, 경로·조향 제어, 차량제어의 네 계층으로 나뉘며 각 계층은 ROS 2
토픽으로 결과를 다음 계층에 전달한다.

| 계층 | 질문 | 대표 노드 | 출력 |
|---|---|---|---|
| 인지 | 화면에 도로·차선·표지·신호가 있는가? | `camera_yolo_inference_node` | 마스크, 객체, 유효성 |
| 판단 | 지금 어느 구간이며 멈추거나 가야 하는가? | `course_mission_node` | 주행 단계, 조향 후보, 상태 |
| 경로·조향 제어 | 차량이 어느 선을 따라 얼마나 꺾어야 하는가? | `camera_path_planner_node`, `camera_path_controller_node` | 경로, 목표 조향각·속도 |
| 차량제어 | 여러 팀 명령 중 무엇을 실제 MCU로 보낼 것인가? | `mcu_manager`, `mcu_bridge` | 모터 단계, 조향각, 정지 명령 |

### 1. 인지: 카메라 영상에서 주행 정보를 만드는 과정

입력은 `/camera/image_raw` 영상과 `/camera/camera_info` 카메라 내부
파라미터다. `camera_yolo_inference_node`가 한 번의 추론 결과에서 다음 정보를
만든다.

| 인지 결과 | 토픽 | 사용 목적 |
|---|---|---|
| 도로 영역 | `/camera/road_mask` | 차량이 주행 가능한 면적 계산 |
| 흰색 차선 | `/camera/white_line_mask` | 일반 차선 경계 계산 |
| 노란색 차선 | `/camera/yellow_line_mask` | 중앙선·S자 안전 경계 계산 |
| 전체 객체 | `/perception/detections_json` | 신호등·정지선·표지 후처리 |
| 정지선 | `/perception/stop_detected` | 경사로·교차로 정지 판단 |
| traffic20 | `/perception/traffic20_detected` | 9번 가속구간 단계 전환 |
| 처리 상태 | `/camera/perception_valid` | 최신 영상과 추론 결과의 유효성 |
| 시각화 | `/perception/detections_image` | 사람이 ROI·마스크·경로 확인 |

검출됐다는 사실과 실제 주행에 사용한다는 것은 다르다. 예를 들어 1번 구간에서
정지선과 traffic20을 검출하고 토픽을 발행할 수는 있지만, 미션 판단기가 해당
입력을 의도적으로 무시한다.

영상이 `max_image_age_sec`보다 오래됐거나 추론 지연이 제한을 넘으면 인지
유효성을 내린다. 실제 카메라와 저장 영상 publisher가 동시에 같은 토픽을
발행하면 서로 다른 프레임이 섞이므로 반드시 하나만 실행한다.

### 2. BEV 변환과 경로 생성

카메라 마스크는 원근이 포함된 2차원 화면이므로 픽셀 중앙을 그대로 따라가면
안 된다. `camera_path_planner_node`는 카메라 위치·자세와 지면 모델을 이용해
마스크를 차량 기준 좌표계의 BEV(Bird's-Eye View)로 바꾼다.

차량 좌표계에서 `x`는 차량 전방, `y`는 좌우 방향이다. 현재 주요 실측값은
축거 0.73m, 뒷바퀴 중심에서 카메라 중심까지 0.41m다. 생성되는
`/camera/path`는 차량이 앞으로 따라갈 `(x, y)` 점들의 목록이다.

경로 생성기는 상황에 따라 다음 자료를 조합한다.

- 양쪽 경계가 보이면 두 경계의 중앙
- 한쪽 경계만 보이면 차선 폭과 안전여유를 적용한 평행 경로
- 도로만 보이면 주행 가능 영역의 중심
- S자 구간에서는 노란선과 장애물 사이의 통과 가능한 corridor
- 교차로에서는 정해진 진행 방향과 도로 영역

새 프레임 경로를 그대로 사용하지 않고 공간 필터와 프레임 간 이동 제한을
적용한다. 경로가 너무 짧거나, 급격히 꺾이거나, 이전 경로에서 과도하게 이동한
경우 `/camera/path_valid=false`로 만든다. 출력 토픽은 다음과 같다.

| 토픽 | 의미 |
|---|---|
| `/camera/path` | 차량 기준 목표 경로 |
| `/camera/path_valid` | 제어에 사용 가능한 경로인지 |
| `/camera/path_confidence` | 현재 경로 근거의 신뢰도 |
| `/camera/path_mode` | 실제 제어에 사용 중인 경로 모드 |
| `/camera/path_observed_mode` | 현재 프레임에서 관측한 후보 모드 |
| `/camera/path_status` | 경로 생성·거부 이유 |
| `/camera/path_debug_image` | BEV와 목표 경로 확인 화면 |

### 3. 판단: 구간에 따라 같은 인지를 다르게 사용하는 과정

`course_mission_node`는 `/mission/section`의 1~11 번호를 기준으로 상태기계를
선택한다. 입력은 카메라 경로, 곡률 속도계획, IMU pitch, 정지선 거리,
신호등, traffic20, MCU odom이다.

판단 결과는 다음 후보 명령으로 발행된다.

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/camera_drive` | `Float32` | `0` 정지, `1~3` 전진 단계 |
| `/camera_wheel` | `Int32` | 목표 조향각 `-27~+27도` |
| `/camera_stop` | `Bool` | 입력 스트림 상실 등 명시적 긴급정지 |
| `/drive_mode` | `String` | MCU가 사용할 현재 구간 번호 |
| `/mission/status` | `String` | 왜 그 명령을 냈는지 설명 |
| `/mission/active_section` | `Int8` | 다른 카메라 노드가 사용할 구간 번호 |

일반 신호대기나 곡률 정지는 `/camera_drive=0`으로 표현한다. `/camera_stop`은
MCU의 독립적인 hard-stop 채널이므로 모든 일반 정지에 남용하지 않는다.

교차로 4·6번은 정지선이 2m 이내일 때 GREEN만 출발을 허용한다. RED,
YELLOW 또는 확정 신호가 없으면 정지한다. 8번은 LEFT만 허용하고 다른 신호는
무시한다. 11번은 최종 신호차의 GREEN만 출발을 허용한다.

### 4. 경로추종과 조향 계산

`camera_path_controller_node`는 Pure Pursuit로 경로 위의 목표점을 고른다.
lookahead가 짧으면 경로에 민감하지만 조향이 흔들릴 수 있고, 길면 부드럽지만
급커브를 늦게 따라간다. 현재 실행 설정은 1.0~1.5m 가변 lookahead다.

목표점이 정해지면 축거 0.73m인 자전거 모델로 앞바퀴 조향각을 계산한다.
계산값에는 두 가지 제한이 적용된다.

- 물리 조향 한계: `-27~+27도`
- 시간당 변화 한계: 최대 `20도/초`

따라서 한 프레임의 경로가 옆으로 움직여도 조향 명령이 즉시 반대쪽 끝까지
뛰지 않는다. 제어 노드는 20Hz로 계산하며 다음을 발행한다.

| 토픽 | 의미 |
|---|---|
| `/camera/target_steering_deg` | Pure Pursuit가 계산한 제한 적용 조향각 |
| `/camera/target_speed_mps` | 경로 종류·곡률 기반 목표속도 |
| `/control/path_status` | 목표점, 원시·제한 조향각, 경로 상태 |

별도의 `curvature_speed_planner_node`는 전체 경로의 최대 곡률을 보고
`/control/curvature_drive_stage`를 만든다. 완만하면 순항, 급하면 1단 감속,
과도한 곡률이면 0단 정지를 요청한다. 경로가 0.5초 이상 오래되거나 신뢰도가
0.45 미만이면 계획을 유효하지 않은 것으로 처리한다.

### 5. 차량제어: MCU 중재기가 최종 명령을 선택하는 과정

카메라·라이다·GPS는 모터를 직접 제어하지 않고 각자의 후보 명령만 발행한다.
`mcu_manager`가 안전 우선순위와 현재 구간을 적용해 하나의 최종 명령을 고른다.

```text
E-stop
  ↓ 없을 때
각 센서의 stop
  ↓ 없을 때
수동 조종 override
  ↓ 없을 때
구동 우선순위와 구간별 조향 권한
  ↓
/mcu/cmd_drive, /mcu/cmd_wheel, /mcu/cmd_stop
```

구동 후보의 기본 우선순위는 `lidar > camera > gps`다. 조향은 기본적으로
카메라가 담당하고 5·7·10번은 YAML의 권한 설정에 따라 라이다가 담당한다.
5번에서 라이다 회피 명령은 `/avoidance/active=true`가 최신 상태일 때만
통과한다. 단, `/lidar_stop`은 회피 게이트와 무관하게 항상 적용된다.

후보 명령은 연속 스트림이다. drive나 wheel이 0.5초 이상 갱신되지 않으면
오래된 명령을 계속 사용하지 않는다. 조향 권한자의 값이 끊기면 바퀴를 중앙으로
보내고 `stop_on_wheel_source_loss=true` 설정에 따라 정지한다.

`mcu_bridge`는 최종 ROS 명령을 115200 baud 시리얼 프로토콜로 Arduino Mega에
전달하고, 펌웨어 상태·엔코더·속도·odom을 다시 ROS로 발행한다. Arduino v37은
실제 구동 PWM, 조향 액추에이터, 급제동과 안티롤백을 담당한다.

### 6. 안전 설계 원칙

안전 조건은 특정 구간보다 높은 우선순위로 처리한다.

1. `/estop_lock=true`이면 다른 모든 명령을 무시하고 정지한다.
2. 라이다·카메라·수동 stop 중 하나라도 최신 `true`이면 즉시 정지한다.
3. 경로·속도계획·조향 권한자의 입력이 끊기면 정지한다.
4. 허용 범위를 벗어난 drive와 wheel 값은 MCU 중재 단계에서 제한한다.
5. 상태 판단 이유는 `/mission/status`와 `/mcu/safety_state`로 공개한다.

`/camera/path_jump_detected`처럼 현재 진단에만 쓰이는 토픽도 있다. 진단값이
존재한다고 자동 정지 기능까지 구현된 것은 아니므로 README의 구간별 설명과
실제 `/mission/status`, `/mcu/safety_state`를 함께 확인해야 한다.

### 7. 문제가 생겼을 때 계층별 확인 순서

| 증상 | 먼저 볼 토픽 | 의미 |
|---|---|---|
| 객체가 안 보임 | `/perception/detections_image`, `/camera/inference_status` | 모델·입력영상·추론 문제 |
| ROI는 보이지만 경로가 없음 | `/camera/path_status`, `/camera/path_valid` | BEV·마스크·검증 문제 |
| 경로는 있는데 조향이 없음 | `/camera/target_steering_deg`, `/camera_wheel` | 제어 또는 미션 입력 문제 |
| 카메라 명령은 있는데 차가 안 감 | `/mcu/safety_state`, `/mcu/active_drive_source` | MCU 우선순위·stop·timeout 문제 |
| 직진만 함 | `/mcu/active_wheel_source`, `/drive_mode` | 구간별 조향 권한자 문제 |
| 갑자기 정지 | `/mission/status`, `/mcu/safety_state` | 어느 계층이 정지를 요청했는지 확인 |

한 번에 최종 모터만 보지 말고 위에서 아래 순서로 확인하면 오류가 인지,
판단, 제어, MCU 중 어디에서 시작됐는지 구분할 수 있다.

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
