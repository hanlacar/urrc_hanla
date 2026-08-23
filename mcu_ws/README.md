# mcu_ws

중앙 명령 선택기(`mcu_manager`)와 T870 Arduino v28 시리얼 브릿지(`mcu_bridge`)를 포함합니다.

## 데이터 흐름

센서/알고리즘 → `mcu_manager` → `/mcu_drive`, `/mcu_wheel` → `mcu_bridge` → `/dev/ttyACM0` → Arduino

## mcu_bridge 확정 설정

- Port: `/dev/ttyACM0`
- Baud: `115200`
- TX: `10 Hz`
- Drive timeout: `0.5 s`
- Wheel timeout: `0.5 s`, Drive와 독립
- Drive: `-1, 0, 1, 2, 3`
- Arduino drive serial: `6.00, 1.00, 2.00, 3.00, 4.00`
- Wheel: `-27..+27 deg`, `-` 좌 / `+` 우
- Wheel serial: 기존 v28 `W<ms>` 방식
- 범위 초과/NaN/Inf: clamp하지 않고 거부
- E-stop: `/estop_lock=true` 또는 Arduino `STATUS` fault!=0 시 latch
- 해제: `/mcu/reset_estop` 서비스가 성공해야 함
- 재연결: 기존 명령 폐기, 정지부터 시작, fresh ROS 명령 대기

## 빌드

```bash
cd ~/mcu_ws
source /opt/ros/jazzy/setup.bash
sudo apt install -y python3-serial
colcon build --symlink-install
source install/setup.bash
```

## 브릿지만 실행

```bash
ros2 launch mcu_bridge mcu_bridge.launch.py
```

## E-stop reset

```bash
ros2 service call /mcu/reset_estop std_srvs/srv/Trigger '{}'
```

`/estop_lock`이 아직 true이거나 Arduino STATUS fault가 0이 아니면 reset은 거부됩니다.
