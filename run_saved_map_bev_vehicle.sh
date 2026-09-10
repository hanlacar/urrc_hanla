#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="${RACE_WORKSPACE:-$repo_root/race_autonomy/ros2_ws}"
map_dir="${RACE_MAP_DIR:-$repo_root/maps/test_20260909_191813_8xh0Le}"
# This tracked model is byte-for-byte identical to the latest
# hanla_yolo11n_seg_best.pt used during vehicle validation.
model="$workspace/src/camera_yolo_inference/models/hanla_competition_11class_best.pt"
manifest="$workspace/src/camera_yolo_inference/config/class_manifest.yaml"
python_executable="$workspace/.yolo_runtime/bin/python"
camera_serial="${RACE_CAMERA_SERIAL:-338122302896}"
mcu_port="${RACE_MCU_PORT:-/dev/ttyACM0}"
target_speed="${RACE_TARGET_SPEED_MPS:-0.15}"
device="${RACE_DEVICE:-cpu}"
require_cuda="${RACE_REQUIRE_CUDA:-false}"
launch_rqt="${RACE_LAUNCH_RQT:-true}"
export ROS_DOMAIN_ID="${RACE_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash

if [[ ! -f "$workspace/install/setup.bash" ]]; then
    echo "ROS workspace is not built: $workspace/install/setup.bash" >&2
    exit 1
fi

if [[ ! -f "$map_dir/map.db" && -f "$map_dir/map.db.xz" ]]; then
    echo "Restoring the bundled RTAB-Map database..."
    xz -dk "$map_dir/map.db.xz"
fi

if [[ ! -f "$map_dir/map.db" || ! -f "$map_dir/route_optimized.json" ]]; then
    echo "Saved map or route is missing under: $map_dir" >&2
    echo "Set RACE_MAP_DIR to the directory containing map.db and route_optimized.json." >&2
    exit 1
fi

(
    cd "$map_dir"
    sha256sum --check --status map.db.sha256
    sha256sum --check --status route_optimized.json.sha256
) || {
    echo "Bundled saved map or route failed its checksum." >&2
    exit 1
}

if [[ ! -x "$python_executable" ]]; then
    echo "YOLO environment is missing. Run: $repo_root/setup_saved_map_bev_vehicle.sh" >&2
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
    python_executable:="$python_executable" \
    device:="$device" \
    require_cuda:="$require_cuda" \
    serial_no:="$camera_serial" \
    mcu_port:="$mcu_port" \
    target_speed_mps:="$target_speed" \
    launch_rqt:="$launch_rqt"
