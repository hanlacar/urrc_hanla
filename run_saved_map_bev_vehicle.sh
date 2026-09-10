#!/usr/bin/env bash
set -eo pipefail

workspace="${RACE_WORKSPACE:-/home/parkjinwoo/urrc_hanla/race_autonomy/ros2_ws}"
map_dir="${RACE_MAP_DIR:-/home/parkjinwoo/urrc_hanla/maps/test_20260909_191813_8xh0Le}"
# This tracked model is byte-for-byte identical to the latest
# hanla_yolo11n_seg_best.pt used during vehicle validation.
model="$workspace/src/camera_yolo_inference/models/hanla_competition_11class_best.pt"
manifest="$workspace/src/camera_yolo_inference/config/class_manifest.yaml"

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash

if [[ ! -f "$workspace/install/setup.bash" ]]; then
    echo "ROS workspace is not built: $workspace/install/setup.bash" >&2
    exit 1
fi

if [[ ! -f "$map_dir/map.db" || ! -f "$map_dir/route_optimized.json" ]]; then
    echo "Saved map or route is missing under: $map_dir" >&2
    echo "Set RACE_MAP_DIR to the directory containing map.db and route_optimized.json." >&2
    exit 1
fi
source "$workspace/install/setup.bash"
set -u

prefix="$(ros2 pkg prefix race_control)"
expected="$workspace/install/race_control"
if [[ "$prefix" != "$expected" ]]; then
    echo "Wrong race_control selected: $prefix" >&2
    echo "Expected: $expected" >&2
    exit 1
fi

exec ros2 launch race_control saved_map_bev_pad_v6_vehicle.launch.py \
    database:="$map_dir/map.db" \
    route_file:="$map_dir/route_optimized.json" \
    segmentation_model_path:="$model" \
    class_manifest_path:="$manifest" \
    device:=cpu \
    require_cuda:=false \
    mcu_port:=/dev/ttyACM0 \
    target_speed_mps:=0.15 \
    launch_rqt:=true
