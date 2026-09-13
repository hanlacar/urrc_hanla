# lidar_parking_planner

## LiDAR 정면 규칙 (LiDAR FRONT CONVENTION)

RPLIDAR A2M12에서 차량의 정면은 라이다 본체 기준 선의 반대편이다. A2M12 관련
launch는 아래 규칙을 기본값으로 사용한다.

```text
flip_x_axis=true
LaserScan frame=laser
LaserScan topic=/scan
LaserScan 0°=차량 정면
```

`flip_x_axis=true`가 LaserScan 자체를 이미 180° 보정하므로,
`base_link -> laser` TF에 방향 보정용 `yaw=pi`를 추가하지 않는다. TF의 위치와
자세는 실제 장착값만 사용해야 하며, 두 보정을 함께 적용하면 double flip이
발생한다.

이 패키지에는 목적이 다른 네 실행 구성이 있다.

- `front_lidar.launch.py`: 전방 RPLIDAR를 `/scan`으로 발행하며, 기본적으로
  `flip_x_axis=true`를 적용해 스캔의 0도 기준을 정확히 180도 이동한다.
- `parking_planner.launch.py`: 기존의 `base_link` 고정 A/B 영역과 front/rear
  LaserScan을 사용하는 scan-only 주차 상태 머신이다. 기존 동작과 토픽을
  유지하기 위한 legacy 기능이다.
- `constant_drive_test.launch.py`: 실차 safety gate 검증을 위해 2.0의 주행
  요청을 별도 토픽으로 만들고, 기존 planner만 최종 `/cmd_drive`를 발행하게
  하는 임시 시험 구성이다.
- `slam_parking_memory.launch.py`: `slam_toolbox` online asynchronous mapping과
  `map` 좌표계 주차 공간 기억 전용 구성이다. 기존
  `parking_planner_node`를 실행하지 않으며 속도, 조향 또는 실제 주차 명령을
  발행하지 않는다.

이번 단계에는 Nav2, AMCL, 경로 생성, 자동 조향/속도 제어, 카메라, 가짜
odometry, 임의의 라이다 static TF가 포함되지 않는다.

## 왜 기존 노드는 공간을 기억하지 못했는가

기존 노드는 front/rear 포인트를 `base_link`의 고정 사각형 A/B에 넣어 개수만
센다. `selected_slot`도 노드 메모리의 문자열이며 미션을 다시 시작할 때
초기화된다. `/map`, 로봇의 map pose, 과거 관측과 현재 관측 사이의 data
association이 없고 마커도 `base_link` 기준이므로 차량이 움직이면 검출
영역과 마커가 함께 움직인다.

새 memory 노드는 `OccupancyGrid`에서 제한된 ROI만 NumPy로 검사한다.
unknown(`-1`)은 free로 보지 않고, occupied cell이 하나라도 후보 내부에
들어오면 후보를 거부한다. free/unknown 비율, 차량 크기와 안전 여유, 지도
경계, 후보 외곽의 장애물 지지를 점수에 반영한다. 가까운 후보는 NMS로
병합하고 위치·방향·크기가 association 허용 오차 안에 있으면 같은 slot
ID를 갱신한다.

## 선행 조건과 TF

`slam_toolbox`와 memory 노드가 필요한 실제 TF 체인은 다음과 같다.

```text
map -> odom                 slam_toolbox가 추정하여 발행
odom -> base_link           실제 odometry 공급자가 발행해야 함
base_link -> <scan_frame>   실제 라이다 설치 위치에 맞는 TF가 있어야 함
```

`<scan_frame>`은 선택한 `LaserScan.header.frame_id`이다. 이 패키지는 라이다
위치를 추측하거나 static TF를 만들지 않는다. `odom -> base_link`도 이
패키지에서 생성하지 않는다.

memory 노드는 조건에 따라 `WAIT_MAP`, `WAIT_SCAN`, `WAIT_ODOM_TF`,
`WAIT_LIDAR_TF`, `MAPPING`, `MEMORY_ACTIVE`, `ERROR`를
`/parking/memory/status`에 `STATE: detail` 형식으로 발행한다. 필수 TF가
없으면 계속 재시도하며 종료하지 않고, 그동안
`/parking/memory/space_found=false`를 유지하여 잘못된 map 좌표를 저장하지
않는다.

실행 전에 확인한다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix slam_toolbox
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link <실제_scan_frame>
```

`slam_toolbox`가 없다면 자동 설치하지 말고 다음 명령으로 설치한다.

```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox
```

## 빌드

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lidar_parking_planner --symlink-install
source install/setup.bash
```

## 실행

터미널 1에서는 기존 RPLIDAR node를 먼저 종료한 뒤 전방 라이다를 실행한다.
기존 `rplidar_a1_launch.py`와 아래 launch를 동시에 실행하면
`/dev/ttyUSB0` 포트 충돌이 발생할 수 있다. 프로세스를 강제로 종료하지 말고
기존 launch를 실행한 터미널에서 정상 종료한다.

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch lidar_parking_planner front_lidar.launch.py
```

전방 라이다 기본 파라미터는 다음과 같다.

```text
channel_type=serial
serial_port=/dev/ttyUSB0
serial_baudrate=256000
frame_id=laser
inverted=false
angle_compensate=true
flip_x_axis=true
scan_mode=Sensitivity
```

토픽 remap을 하지 않으므로 LaserScan은 `/scan`으로 발행된다. 원래 방향과
비교할 때만 다음과 같이 180도 이동을 끌 수 있다.

```bash
ros2 launch lidar_parking_planner front_lidar.launch.py flip_x_axis:=false
```

터미널 2에서는 SLAM과 parking-space memory를 실행한다. `scan_topic`의
기본값이 `/scan`이므로 인자를 생략할 수 있다.

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch lidar_parking_planner slam_parking_memory.launch.py
```

같은 기본값을 모두 명시하면 다음과 같다.

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch lidar_parking_planner slam_parking_memory.launch.py \
  scan_topic:=/scan \
  map_frame:=map \
  odom_frame:=odom \
  base_frame:=base_link \
  use_sim_time:=false
```

후방 라이다 하나로 SLAM을 수행하려면 실제 TF가 준비된 뒤 다음처럼 선택한다.
두 scan을 합친 가상 LaserScan은 만들지 않는다.

```bash
ros2 launch lidar_parking_planner slam_parking_memory.launch.py \
  scan_topic:=/rear/scan
```

RViz를 별도로 실행할 때는 launch의 RViz를 끈다.

```bash
ros2 launch lidar_parking_planner slam_parking_memory.launch.py use_rviz:=false
rviz2 -d "$(ros2 pkg prefix lidar_parking_planner)/share/lidar_parking_planner/rviz/parking_slam_memory.rviz"
```

기존 scan-only 상태 머신은 별도로 실행한다.

```bash
ros2 launch lidar_parking_planner parking_planner.launch.py localization_mode:=scan_only
```

두 launch는 목적이 다르다. SLAM 지도 작성 중에는 legacy launch를 함께
실행하지 않아야 자동 주차용 명령 발행을 확실히 피할 수 있다.

## 토픽

입력:

- `/map` (`nav_msgs/msg/OccupancyGrid`)
- 선택한 `scan_topic` (`sensor_msgs/msg/LaserScan`)
- `/tf`, `/tf_static` (tf2 listener가 사용)

출력은 기존 노드와 충돌하지 않도록 `/parking/memory/`에 한정된다.

- `/parking/memory/status` (`std_msgs/msg/String`)
- `/parking/memory/space_found` (`std_msgs/msg/Bool`)
- `/parking/memory/selected_slot_id` (`std_msgs/msg/String`)
- `/parking/memory/selected_slot_pose` (`geometry_msgs/msg/PoseStamped`)
- `/parking/memory/slots_json` (`std_msgs/msg/String`)
- `/parking/memory/slot_markers` (`visualization_msgs/msg/MarkerArray`)

`selected_slot_pose`와 모든 Marker의 frame은 항상 `map_frame`이다.
`slots_json`은 `slot_id`, `parking_type`, 중심, yaw, 폭, 길이, confidence,
관측 횟수, 최초/마지막 시각과 `CANDIDATE`, `CONFIRMED`,
`TEMPORARILY_LOST`, `OCCUPIED` 상태를 포함한다. 추후 경로 생성기는
`/parking/memory/selected_slot_pose`와 `/parking/memory/slots_json`을
입력으로 사용한다.

상태를 확인한다.

```bash
ros2 topic echo /parking/memory/status
ros2 topic echo /parking/memory/slots_json
ros2 topic echo /parking/memory/selected_slot_pose
ros2 topic echo /parking/memory/space_found
ros2 topic echo /map --once
```

## 슬롯 저장, 불러오기, 삭제

기본 파일은
`~/.ros/lidar_parking_planner/parking_slots.json`이다. 저장 파일에는 format
version, map frame, resolution, origin, 크기, 저장 시각과 모든 슬롯이
들어간다. load 시 현재 map frame/resolution/origin과 호환되지 않으면
자동 적용하지 않고 실패 응답과 경고를 낸다.

```bash
ros2 service call /parking/memory/save std_srvs/srv/Trigger "{}"
ros2 service call /parking/memory/load std_srvs/srv/Trigger "{}"
ros2 service call /parking/memory/clear std_srvs/srv/Trigger "{}"
```

`clear`는 내부 슬롯을 지우고 이전 Marker ID에 `Marker.DELETE`를 발행한다.

SLAM 지도/pose graph 저장 서비스의 실제 Jazzy 인터페이스는 다음 명령으로
확인할 수 있다.

```bash
ros2 service list | grep slam_toolbox
ros2 interface list | grep slam_toolbox
ros2 interface show slam_toolbox/srv/SerializePoseGraph
ros2 interface show slam_toolbox/srv/SaveMap
```

이 환경의 설치본에서 `SerializePoseGraph` 요청 필드는 `string filename`,
`SaveMap` 요청 필드는 `std_msgs/String name`이다. 노드가 활성화된 뒤
서비스명도 위 명령으로 확인하고 다음처럼 저장한다.

```bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/.ros/parking_posegraph'}"

ros2 service call /slam_toolbox/save_map \
  slam_toolbox/srv/SaveMap \
  "{name: {data: '$HOME/.ros/parking_map'}}"
```

pose graph와 슬롯 JSON은 별도 파일이므로 동일한 지도 세션에 대해 함께
관리해야 한다.

## RViz

`parking_slam_memory.rviz`의 Fixed Frame은 `map`이며 다음 display가
활성화되어 있다.

- `/map`
- `/scan` (launch의 `scan_topic` remap을 따라감)
- TF
- `/parking/memory/slot_markers`

슬롯 색/투명도는 후보(반투명 주황), 확정(선명한 초록), 일시 유실(옅은
회색), 점유(빨강)로 구분한다. 라벨에는 slot ID, 주차 방식과 confidence가
표시된다.

## 주요 조정 파라미터

실차 투입 전 `config/parking_space_memory.yaml`에서 다음을 반드시 실측값으로
바꾼다.

- `vehicle_length_m`, `vehicle_width_m`, `safety_margin_m`
- `t_slot_width_m`, `t_slot_depth_m`
- `parallel_slot_length_m`, `parallel_slot_width_m`
- 우측 탐색 ROI와 sample/NMS 간격
- association 오차, 확정 관측 횟수, 유실/삭제 시간
- map metadata 호환 오차와 저장 경로

라이다의 위치·방향과 `header.frame_id`가 확정되면 외부 bringup/robot
description에서 실제 `base_link -> scan_frame` TF를 제공해야 한다. 이
패키지 YAML에는 설치 위치가 하드코딩되어 있지 않다. 실제 설치 후에는
라이다 원점의 `x`, `y`, `z`와 자세의 `roll`, `pitch`, `yaw`를 측정하여
`base_link -> laser` extrinsic으로 설정한다. `flip_x_axis=true`가 이미
스캔의 정면을 바꾸므로 방향 보정만을 위한 추가 `yaw=pi`를 넣지 않는다.
그렇게 하면 180도 보정이 두 번 적용되어 원래 방향으로 돌아간다.

## 원시 LaserScan 180도 보정 검증

SLAM이나 TF의 영향을 배제하려면 RViz에서 `Fixed Frame=laser`,
`LaserScan Topic=/scan`으로 설정하고 Map과 Parking Slots를 끈 뒤 LaserScan만
켠다. 라이다의 기존 정면(선이 있는 방향)에 박스, 사람 또는 벽처럼 위치가
분명한 물체를 둔다.

먼저 아래 설정에서 물체 위치를 확인한다.

```bash
ros2 launch lidar_parking_planner front_lidar.launch.py flip_x_axis:=false
```

해당 노드를 정상 종료하고 다음 설정으로 다시 실행한다.

```bash
ros2 launch lidar_parking_planner front_lidar.launch.py flip_x_axis:=true
```

동일한 물체가 RViz에서 정확히 180도 반대편에 표시되어야 한다. 함께 다음을
확인한다.

```bash
ros2 node list | grep rplidar
ros2 param get /front_rplidar_node flip_x_axis
ros2 topic list | grep scan
ros2 topic hz /scan
ros2 topic echo /scan --once | grep frame_id
```

정상 상태는 `flip_x_axis`가 `True`, 토픽이 `/scan`, frame ID가 `laser`이고
스캔이 지속적으로 발행되는 상태다.

## Nav2 또는 경로 생성 단계 전 필요한 조건

다음 단계로 넘어가기 전에 실제 wheel/IMU 기반의 안정적인
`odom -> base_link`, 검증된 라이다 extrinsic TF, 충분히 수렴하고 저장된
SLAM map/pose graph, 실측 차량 footprint와 안전 여유, 주차 공간 검출의
현장 정밀도/재현율, 선택 슬롯의 진입 방향 정의가 필요하다. 그 후에도
Nav2 localization/planner/controller 또는 별도 저속 주차 경로 추종기는 이
memory 노드와 분리해 설계한다.

## 전방 LiDAR 장애물 추적과 속도 안전 정지

`parking_planner_node`는 검증된 `/scan`의 0 rad를 차량 정면으로 사용한다.
유효 scan point를 Cartesian 좌표로 바꾸고, 연속 점 사이 거리가 기본
0.15 m 이하인 점 3개 이상을 하나의 cluster로 묶는다. 현재 cluster와 기존
track의 예측 위치, 각도, 폭을 함께 비교하여 `OBS_0001` 형식의 ID를
유지한다. 한 frame을 놓쳐도 바로 지우지 않고 기본 0.5초 동안 association을
시도한다. 넓은 cluster는 centroid가 전방 밖에 있어도 point extent 일부가
`-30`도부터 `+30`도에 들어오면 전방 장애물로 취급한다.

같은 track의 대표 거리로 다음 값을 계산한다.

```text
raw_range_rate = (current_range - previous_range) / dt
ego_radial_speed = ego_speed * cos(object_angle)
object_radial_speed = raw_range_rate + ego_radial_speed
```

차량 전진 속도는 양수, 후진은 음수이며 거리가 줄면 range rate는 음수다.
EMA를 적용한 물체 속도가 0.15 m/s 이하면 `STATIC`, -0.30 m/s 이하면
`DYNAMIC_APPROACHING`, +0.30 m/s 이상이면 `DYNAMIC_RECEDING`, 그 사이는
`UNKNOWN` 후보가 된다. 같은 후보가 기본 3 frame 연속 확인되어야 상태가
바뀐다. 5 m/s를 넘는 단일 속도 측정치는 이상치로 거부한다.

정적/동적/미확정 장애물의 stop 거리는 각각 조정할 수 있고 기본값은 모두
0.50 m다. 따라서 분류 결과나 encoder 유효 여부와 무관하게 0.50 m 이내의
cluster는 정지 대상이다. 기존 point-level gate도 함께 유지하여 cluster가
되기 전의 유효 근거리 point를 놓치지 않는다. 연속 2 scan에서 위험이
확인되면 기존 최종 `cmd_drive` 발행 직전에 속도를 0으로 clamp한다. 별도의
`/cmd_drive` publisher를 만들지 않으며 안전 기능은 steering 값을 변경하지
않는다. 정지 후 0.60 m 이상이 연속 3 scan에서 확인되어야 다시 주행한다.
`/scan`이 0.5초 넘게 끊기면 `LIDAR_TIMEOUT`으로 정지한다.

추적/분류 설정은 `config/obstacle_motion.yaml`에 있다. TTC는 JSON에 항상
계산 가능하게 포함되지만 기본 정지에는 사용하지 않는다.
`enable_ttc_stop:=true`로 설정한 경우에만 `ttc_stop_sec`가 적용된다.

### Encoder 확인 전 fail-safe 상태

2026-08-16 조사 당시 실행 중 ROS graph에는 `/parameter_events`와 `/rosout`
외의 토픽이 없어 실제 encoder publisher, encoder 원시 단위, 좌우 분리 여부,
resolution, wheel radius, gear ratio와 부호를 확인할 수 없었다. 읽기 전용
소스에서는 `lidar_motion_detector`가 `/ego_speed_mps`
(`std_msgs/msg/Float32`)를 m/s 입력으로 소비하는 것만 확인되었고, 이 값의
publisher나 encoder 변환 과정은 워크스페이스에 없었다.

그러므로 기본 설정은 다음과 같이 안전하게 잠겨 있다.

```yaml
encoder_speed_topic: /ego_speed_mps
encoder_speed_unit: mps
encoder_forward_sign: 1.0
encoder_source_verified: false
```

`encoder_source_verified=false`이면 메시지를 받더라도 ego 보정에 사용하지
않고 모든 물체를 `UNKNOWN`으로 둔다. 근거리/scan-timeout 정지는 계속
동작한다. 차량 bringup을 실행한 상태에서 아래를 확인하고, 해당 토픽이 실제
wheel encoder에서 변환된 m/s이며 전진이 양수임을 실측한 후에만
`encoder_source_verified: true`로 바꾼다.

```bash
ros2 topic list | grep -iE "encoder|wheel|speed|velocity|odom"
ros2 topic type /ego_speed_mps
ros2 topic info /ego_speed_mps -v
ros2 topic echo /ego_speed_mps --once
ros2 topic type /cmd_drive
ros2 topic info /cmd_drive -v
ros2 topic type /cmd_wheel
ros2 topic info /cmd_wheel -v
```

실제 encoder가 tick만 발행한다면 `encoder_source_verified`를 켜지 말고 먼저
resolution, wheel radius/diameter, gear ratio, sampling period와 좌우/부호를
확정해야 한다. 현재 코드에는 확인되지 않은 tick-to-m/s 상수가 없다.

기존 RPLIDAR launch와 경쟁하는 라이다 노드를 실행하지 않은 상태에서 다음을
실행한다.

```bash
ros2 launch lidar_parking_planner front_lidar.launch.py
ros2 launch lidar_parking_planner parking_planner.launch.py
```

상태를 확인한다.

```bash
ros2 topic echo /parking/safety/status
ros2 topic echo /parking/safety/obstacle_detected
ros2 topic echo /parking/safety/front_min_distance
ros2 topic echo /parking/obstacles/tracks_json
ros2 topic echo /parking/obstacles/nearest_distance
ros2 topic echo /parking/obstacles/nearest_type
ros2 topic echo /cmd_drive
```

실차에서는 먼저 바퀴를 띄우거나 아주 낮은 속도로 시험한다. 정면 1.0 m와
0.6 m에서는 `CLEAR`인지 확인하고, 장애물을 0.5 m 이내로 옮겼을 때
`OBSTACLE_STOP`과 `cmd_drive.data=0.0`을 확인한다. 다시 0.6 m보다 멀리
옮긴 뒤 연속 3 scan 후 `CLEAR`로 복귀하는지 확인한다.

정적/동적 분류 실차 검증은 별도로 수행한다. 차량 정지+정적 벽,
차량 저속 전진+정적 벽, 차량 정지+사람 접근/이탈 순으로 시험한다. 특히
두 번째 시험에서 raw range rate가 음수여도 ego 보정 후 object speed가 0에
가까워지고 `STATIC`인지 확인한다. 회전 중에는 선속도 하나만으로 sensor
offset과 yaw motion을 제거할 수 없다. 따라서 명령 조향각이 기본 5도를
넘으면 분류를 `UNKNOWN`으로 강등하며, 그 밖에도 직진/저조향 주행을
전제로 한다.

## 임시 Constant Drive 2.0 safety gate 실차 검증

`constant_drive_test_node`는 `std_msgs/msg/Float32` 형식의
`/parking/test_drive_request`만 기본 20 Hz로 발행한다. 이 노드는
`/cmd_drive`와 `/cmd_wheel`을 발행하지 않는다. `parking_planner_node`가 요청을
받아 기존 point-level gate와 cluster/motion gate를 차례로 적용한 뒤 유일한
최종 `/cmd_drive` publisher로 동작한다.

`constant_drive_test_enabled`의 기본값은 `false`이므로 기존
`parking_planner.launch.py` 동작은 바뀌지 않는다. 전용 launch에서만 이를
활성화한다. 요청이 `test_command_timeout_sec`의 기본값 0.5초보다 오래 끊기면
effective requested drive를 0으로 만든다. 안전 상태가 `CLEAR`이면 요청 2.0이
그대로 통과하고, 0.50 m 이내의 `STATIC`, `DYNAMIC_APPROACHING`,
`DYNAMIC_RECEDING`, `UNKNOWN`, 또는 LaserScan timeout이면 0.0이 된다. 정지
후 0.60 m 이상이 기존 `clear_confirm_frames`만큼 확인되면 자동으로 2.0으로
복귀한다.

처음에는 반드시 바퀴를 지면에서 띄우고 차량 구동 전원 차단 수단을 바로
사용할 수 있는 상태에서 토픽 전환을 검증한다. 그 다음에만 장애물이 없는
넓고 통제된 공간에서 지면 주행을 시험한다.

터미널 1 — LiDAR:

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch lidar_parking_planner front_lidar.launch.py
```

터미널 2 — encoder source 확인 전의 보수적 시험:

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch lidar_parking_planner constant_drive_test.launch.py \
  drive_value:=2.0 \
  encoder_source_verified:=false
```

실제 `/ego_speed_mps` publisher, 단위, 전진 부호를 실차에서 확인한 후에만
다음처럼 분류 보정을 활성화한다.

```bash
ros2 launch lidar_parking_planner constant_drive_test.launch.py \
  drive_value:=2.0 \
  encoder_source_verified:=true
```

시험 중 다음 토픽을 각각 확인한다.

```bash
ros2 topic echo /parking/test_drive_request
ros2 topic echo /parking/test/requested_drive
ros2 topic echo /parking/test/final_drive
ros2 topic echo /cmd_drive
ros2 topic echo /parking/safety/status
ros2 topic echo /parking/obstacles/tracks_json
ros2 topic echo /ego_speed_mps
ros2 topic info /cmd_drive -v
```

마지막 명령에서 `/cmd_drive` publisher가 `parking_planner_node` 하나인지
확인한다. 장애물 없음에서 requested/final/cmd가 모두 2.0인지, 0.50 m 이내
장애물과 LaserScan 중단 시 final/cmd가 0.0인지, 장애물을 0.60 m 이상으로
치운 뒤 다시 2.0인지 확인하고 나서 지면 주행으로 넘어간다.

## `/drive` 누적 엔코더 count 확인

`encoder_monitor_node`는 하드웨어의 `/drive`를 반드시
`std_msgs/msg/Int32`로 구독한다. 이 값은 속도가 아니라 누적 raw pulse
count이다. 첫 메시지는 기준값으로만 저장하고, 다음 메시지부터
`delta_count = current_count - previous_count`와
`count_rate = delta_count / dt`를 계산한다. `dt`는 ROS clock으로 측정하며
기본 `min_dt_sec=0.001`보다 작거나 유효하지 않으면 잘못된 rate를 발행하지
않는다. `count_rate`의 단위는 `count/s`이며 `m/s`가 아니다.

현재 `counts_per_meter` 실측값이 없으므로 이 노드는 거리나 속도의 SI 단위
변환을 수행하지 않는다. 또한 raw `/drive`를 기존 obstacle motion classifier의
ego speed로 연결하지 않는다. 향후 하드웨어 bridge가 발행할
`/vehicle/distance_m`, `/vehicle/speed_mps`, `/odom` 중 실제 LiDAR ego-motion
compensation에는 `/vehicle/speed_mps`를 사용해야 한다.

엔코더가 앞축(조향축) 모터에 있으므로 선회 중 앞축 이동거리는 뒤축 또는
차량 중심 이동거리보다 크게 측정될 수 있다. 따라서 raw count로 차량 중심
속도를 직접 계산하지 말고, 향후 bridge에서 조향 기하를 반영해 보정해야 한다.

빌드 후 monitor를 실행한다.

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run lidar_parking_planner encoder_monitor_node
```

필요한 경우 최소 시간 간격만 변경할 수 있다.

```bash
ros2 run lidar_parking_planner encoder_monitor_node \
  --ros-args -p min_dt_sec:=0.001
```

진단 출력 토픽은 다음과 같다.

- `/parking/encoder/count` (`std_msgs/msg/Int32`)
- `/parking/encoder/delta_count` (`std_msgs/msg/Int32`)
- `/parking/encoder/count_rate` (`std_msgs/msg/Float32`, 단위 `count/s`)

원본 토픽의 타입과 약 5 Hz 주기를 먼저 확인한다.

```bash
ros2 topic echo /drive
ros2 topic type /drive
ros2 topic info /drive --verbose
ros2 topic hz /drive
```

`ros2 topic type /drive`의 정상 출력은 `std_msgs/msg/Int32`이다. monitor의
계산 결과는 다음 명령으로 확인한다.

```bash
ros2 topic echo /parking/encoder/count
ros2 topic echo /parking/encoder/delta_count
ros2 topic echo /parking/encoder/count_rate
```

전진/후진 부호 규칙은 아직 확정하지 않는다. 안전한 실차 조건에서 아래
순서로 확인해 기록한다.

1. 정지 상태에서 `/drive` count와 `delta_count`가 거의 변하지 않는지 본다.
2. 차량을 매우 천천히 전진시키고 `delta_count`의 부호를 기록한다.
3. 차량을 매우 천천히 후진시키고 전진과 반대 부호인지 확인한다.

실측 전에는 전진을 양수 또는 음수로 코드에 하드코딩하지 않는다.
