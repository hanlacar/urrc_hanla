#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$repo_root/urrc_hanla_unified"
route="${RACE_DR_ROUTE_CSV:-$workspace/configured_routes/a-c_dr.csv}"
lidar_port="${RACE_LIDAR_PORT:-/dev/ttyUSB0}"
mcu_port="${RACE_MCU_PORT:-/dev/ttyACM0}"

cd "$workspace"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/jazzy/setup.bash
source "$workspace/install/local_setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"

ros2 run t870_cmd_bridge bridge --ros-args -p port:="$mcu_port" &
mcu_pid=$!
trap 'kill "$mcu_pid" 2>/dev/null || true' EXIT
sleep 2

exec ros2 launch hanla_unified system.launch.py \
  enable_camera:=true \
  enable_lidar:=true \
  enable_dr:=true \
  dr_route_csv:="$route" \
  dr_auto_start:=true \
  dr_launch_rviz:=true \
  front_lidar_port:="$lidar_port" \
  enable_mcu_simple_compat:=true \
  launch_rqt:=true
