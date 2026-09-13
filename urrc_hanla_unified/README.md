# URRC Hanla Unified Workspace

카메라, 라이다, DR의 후보 명령을 한 판단 노드에서 선택해 별도 MCU
워크스페이스로 최종 속도(m/s)와 조향각(deg)을 전달하는 ROS 2 Jazzy
워크스페이스다.

```bash
cd /home/werwerwer/urrc_hanla/urrc_hanla_unified
./build.sh
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=10
ros2 launch hanla_unified system.launch.py \
  front_lidar_port:=/dev/serial/by-id/라이다_ID
```

DR을 사용하려면 실제 차량에서 기록한 경로를 지정한다. 빈 경로로 DR 노드를
켜면 시작할 수 없으므로 기본값은 꺼짐이다.

```bash
ros2 launch hanla_unified system.launch.py \
  enable_dr:=true \
  dr_route_csv:=$PWD/configured_routes/a-c_dr.csv
ros2 service call /dr_route/start std_srvs/srv/Trigger '{}'
```

최종 명령 토픽은 `/mcu/target_speed_mps`(Float32)와
`/mcu/target_steering_deg`(Float32)다. 이 워크스페이스는 MCU 프로세스나
시리얼 포트를 소유하지 않는다. 별도 `T870_MCU` 워크스페이스에서는 manager를
끄고 bridge만 실행한다. `/mcu/emergency_stop`은 라이다 돌발상황에서 동적 제동을 수행하는
안전 입력이며, `/mcu/slope_hold`은 경사로 정차 중 펌웨어의 제한시간 홀딩
기능을 사용한다.

두 워크스페이스는 같은 `ROS_DOMAIN_ID`를 사용해야 한다. MCU 터미널 실행법은
`../T870_MCU/README_UNIFIED.md`에 정리했다.

카메라와 TensorRT YOLO의 최대 영상 처리율을 확인할 때는 대용량 이미지용
Fast DDS 공유메모리와 RGB 전용 설정을 함께 적용하는 스크립트를 사용한다.
이 모드는 Depth와 IMU를 끄므로 정지선 거리 측정이나 자세 추정이 필요한 실제
미션용 launch를 대신하지 않는다.

```bash
cd /home/werwerwer/urrc_hanla/urrc_hanla_unified
ROS_DOMAIN_ID=10 ./run_camera_yolo_fast.sh
```

RQT 창 없이 처리율만 확인하려면 `SHOW_RQT=0`을 앞에 붙인다. 같은 카메라를
여러 launch에서 동시에 열면 `Device or resource busy`가 발생하므로 카메라
launch는 하나만 실행한다.

구간은 `/mission/section`(Int8)으로 선택한다. 카메라 ROI에 필요한 동일 구간과
회전 방향은 통합 판단 계층이 계속 발행해야 한다. 현재 A/B 및 A-A/A-B 선택은
기존 주차 노드의 선택 토픽을 사용하며, 실제 대회 경로와 선택 waypoint가
제공되기 전에는 자동 실행하지 않는다.

## 구간 선택 규칙

| 구간 | 선택 순서와 게이트 |
|---|---|
| 1 | DR → 카메라 |
| 2 | 카메라 → DR → 라이다; pitch 5도 이상과 경사 정지 waypoint에서 3초 홀드 |
| 3 | 신뢰도 0.8 이상 카메라 → DR → 라이다 |
| 4, 6, 8 | DR 경로; 카메라 신호가 GO/GREEN일 때만 진행 |
| 5 | 라이다 우회 → DR → 카메라 |
| 7 | 라이다 T 주차 → DR → 카메라 |
| 9 | 라이다 돌발정지 최우선; 그 외 DR → 카메라 |
| 10 | 라이다 평행주차 → DR → 카메라 |
| 11 | DR → 카메라 → 라이다; 출차 waypoint에서 5초 정지 후 GREEN=A-A, RED=A-B 선택 |

명령 또는 센서 토픽이 0.5초 이상 끊기면 다음 후보로 전환하며, 유효 후보가
없으면 속도 0과 긴급정지를 발행한다.

`stage_3_speed_mps`, 실제 DR 경로, 정지 waypoint, 주차 A/B와 출차 A-A/A-B
선택 기준은 실차 데이터가 필요해 설정·입력 토픽으로 남겨 두었다.

## mmission 경로 입력

첨부 ZIP에서 전체 경로 4개와 구간 index 설정 4개를
`imported_routes/mmission_ws-main/routes/`에 가져왔다. 원본은 보존하고 DR
follower 형식으로 변환한 결과는 `configured_routes/`에 둔다.

| 선택 | DR 경로 |
|---|---|
| A-C | `configured_routes/a-c_dr.csv` |
| A-D | `configured_routes/a-d_dr.csv` |
| B-C | `configured_routes/b-c_dr.csv` |
| B-D | `configured_routes/b-d_dr.csv` |

원본 segment의 1~8번은 같은 번호로, SEG09·SEG10은 가속구간 9번으로,
SEG11은 평행주차 10번으로, SEG12는 출차 11번으로 변환한다.
출발·도착 조합은 실차 출발 위치에 맞춰 A-C, A-D, B-C, B-D 중 선택한다.

T 주차 또는 평행주차 Nav2를 준비할 때는 둘 중 하나만 켠다. 시작 시에는
`auto_start=false`, `execute=false`로 고정되어 차량이 움직이지 않는다.

```bash
ros2 launch hanla_unified system.launch.py enable_t_parking:=true \
  parking_map:=/절대경로/map.yaml
# 또는 enable_parallel_parking:=true
```

11번은 OpenCV 신호 토픽 `/perception/traffic_light_state`를 사용한다. 출차
waypoint에서 5초 정지한 뒤 GREEN(GO)이면 `A-A`, RED이면 `A-B`를
`/mission/exit_route_request`로 발행한다. 실제 경로 실행기는 이 요청에 맞는
기록 경로를 연결해야 한다.

7번과 10번의 A/B는 라이다 주차 노드가 각각 `/t_parking/selected_slot`,
`/parallel_parking/selected_slot`으로 결정한다. 통합 판단기는 선택값을
`/mission/parking_slot`으로 중계하고 `/parking/drive_cmd`,
`/parking/wheel_cmd`를 최종 속도·조향 후보로 사용한다.
