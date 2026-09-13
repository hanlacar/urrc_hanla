#!/usr/bin/env bash
# ROS setup files probe unset variables, so enable nounset only after sourcing.
set -o pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
else
  echo 'FAIL /opt/ros/jazzy/setup.bash is missing' >&2
  exit 1
fi
if [[ -f "${workspace}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${workspace}/install/setup.bash"
else
  echo 'FAIL build the workspace before running smoke_test.sh' >&2
  exit 1
fi
set -eu

# Keep ROS launch logs inside a writable, isolated test directory.
export ROS_LOG_DIR
ROS_LOG_DIR="$(mktemp -d /tmp/lidar_ws_plus_roslog.XXXXXX)"

python3 - "${workspace}/src/lidar_ws_plus_bringup/config" <<'PY'
from pathlib import Path
import sys
import yaml

for path in sorted(Path(sys.argv[1]).glob('*.yaml')):
    with path.open(encoding='utf-8') as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise SystemExit(f'FAIL YAML root is not a mapping: {path}')
    print(f'PASS YAML {path.name}')
PY

for launch_file in \
    real_vehicle.launch.py lidar_only.launch.py avoidance.launch.py \
    t_parking.launch.py parallel_parking.launch.py; do
  ros2 launch lidar_ws_plus_bringup "${launch_file}" --show-args >/dev/null
  echo "PASS launch arguments ${launch_file}"
done

for executable in \
    'lidar_ws_plus_bringup command_mux' \
    'rplidar_ros rplidar_node' \
    'lidar_motion_detector motion_detector_node' \
    'avoidance_lidar lidar_safety' \
    'avoidance_planner avoidance_coordinator' \
    'avoidance_route route_follower' \
    't_parking_sim cmd_vel_to_lidar_cmd.py'; do
  package="${executable%% *}"
  name="${executable#* }"
  if ! ros2 pkg executables "${package}" | awk '{print $2}' | grep -qx "${name}"; then
    echo "FAIL missing executable: ${executable}" >&2
    exit 1
  fi
  echo "PASS executable ${executable}"
done

log_file="$(mktemp /tmp/lidar_ws_plus_smoke.XXXXXX.log)"
set +e
timeout 5s ros2 launch lidar_ws_plus_bringup real_vehicle.launch.py \
  enable_lidar:=false enable_rear_lidar:=false \
  enable_lidar_tf:=false enable_rear_tf:=false \
  enable_motion_detector:=false enable_avoidance:=false \
  enable_mux:=true >"${log_file}" 2>&1
status=$?
set -e
if [[ "${status}" != 0 && "${status}" != 124 ]]; then
  sed -n '1,160p' "${log_file}" >&2
  echo "FAIL hardware-free launch exited with ${status}" >&2
  exit 1
fi
if rg -n 'Traceback|Caught exception|process has died|InvalidLaunchFileError' \
    "${log_file}"; then
  echo 'FAIL hardware-free launch logged an exception' >&2
  exit 1
fi
echo 'PASS hardware-free real_vehicle launch'
