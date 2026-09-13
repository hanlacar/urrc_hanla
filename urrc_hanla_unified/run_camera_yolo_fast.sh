#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/install/local_setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$workspace_dir/config/fastdds_shm_64mb.xml"

camera_pid=""
yolo_pid=""
viewer_pid=""

stop_processes() {
    for pid in "$viewer_pid" "$yolo_pid" "$camera_pid"; do
        [[ -n "$pid" ]] || continue
        kill -INT -- "-$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "$viewer_pid" "$yolo_pid" "$camera_pid"; do
        [[ -n "$pid" ]] || continue
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
}
trap stop_processes EXIT INT TERM

setsid ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/camera -r __node:=camera \
  -p serial_no:="'338122302896'" \
  -p enable_color:=true \
  -p enable_depth:=false \
  -p enable_infra1:=false -p enable_infra2:=false \
  -p enable_gyro:=false -p enable_accel:=false \
  -p rgb_camera.color_profile:=640x480x60 \
  -p rgb_camera.enable_auto_exposure:=true \
  -p color_qos:=SENSOR_DATA \
  -p enable_sync:=false \
  -p align_depth.enable:=false \
  -p publish_tf:=false &
camera_pid=$!

setsid ros2 launch camera_yolo_inference yolo_inference.launch.py \
  input_image_topic:=/camera/camera/color/image_raw \
  input_camera_info_topic:=/camera/camera/color/camera_info \
  inference_fps:=60.0 \
  detections_image_fps:=0.0 \
  launch_rqt:=false &
yolo_pid=$!

if [[ "${SHOW_RQT:-1}" == "1" ]]; then
    sleep 6
    setsid ros2 run rqt_image_view rqt_image_view /perception/detections_image &
    viewer_pid=$!
fi

wait "$camera_pid" "$yolo_pid"
