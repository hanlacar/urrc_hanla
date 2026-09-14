#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$repo_root/urrc_hanla_unified"

if [[ ! -d "$workspace/src" ]]; then
  echo "Unified workspace not found: $workspace" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y ros-jazzy-desktop ros-dev-tools python3-venv python3-pip xz-utils

set +u
source /opt/ros/jazzy/setup.bash
set -u
rosdep update
rosdep install --from-paths "$workspace/src" --ignore-src -r -y --rosdistro jazzy

python3 -m venv --system-site-packages "$workspace/.yolo_runtime"
"$workspace/.yolo_runtime/bin/python" -m pip install --upgrade pip
"$workspace/.yolo_runtime/bin/python" -m pip install -r "$repo_root/requirements-vehicle.txt"

cd "$workspace"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/jazzy/setup.bash
set -u
colcon build --base-paths "$workspace/src" --symlink-install

echo "Unified vehicle setup complete."
