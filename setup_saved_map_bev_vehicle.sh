#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -d "$repo_root/race_autonomy/ros2_ws" ]]; then
    workspace="$repo_root/race_autonomy/ros2_ws"
else
    workspace="$repo_root/race_autonomy/ros_ws"
fi
map_dir="$repo_root/maps/test_20260909_191813_8xh0Le"

if [[ "$(uname -s)" != Linux ]]; then
    echo "This vehicle setup supports Ubuntu 24.04 only." >&2
    exit 1
fi
. /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 24.04 ]]; then
    echo "Ubuntu 24.04 is required for ROS 2 Jazzy binary packages." >&2
    exit 1
fi

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    sudo apt-get update
    sudo apt-get install -y software-properties-common curl
    sudo add-apt-repository -y universe
    ros_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | awk -F'"' '/tag_name/{print $4; exit}')"
    if [[ -z "$ros_apt_version" ]]; then
        echo "Could not determine the current ROS apt source version." >&2
        exit 1
    fi
    ubuntu_codename="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"
    ros_apt_deb="/tmp/ros2-apt-source.deb"
    curl -fL -o "$ros_apt_deb" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.${ubuntu_codename}_all.deb"
    sudo dpkg -i "$ros_apt_deb"
fi

sudo apt-get update
sudo apt-get install -y \
    ros-jazzy-desktop ros-dev-tools python3-venv python3-pip xz-utils \
    ros-jazzy-realsense2-camera ros-jazzy-rtabmap-launch ros-jazzy-rqt-image-view

if ! id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
    sudo usermod -aG dialout "$USER"
    echo "Added $USER to dialout. Log out and back in before using the MCU serial port."
fi

# ROS Jazzy's setup script references optional variables that may be unset.
set +u
source /opt/ros/jazzy/setup.bash
set -u
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update
rosdep install --from-paths "$workspace/src" --ignore-src -r -y --rosdistro jazzy

python3 -m venv --system-site-packages "$workspace/.yolo_runtime"
"$workspace/.yolo_runtime/bin/python" -m pip install --upgrade pip
if [[ "${RACE_INSTALL_CUDA:-false}" == true ]]; then
    echo "Installing the GPU PyTorch runtime (this can download several GB)..."
    "$workspace/.yolo_runtime/bin/python" -m pip install torch torchvision tensorrt
else
    echo "Installing the CPU-only PyTorch runtime..."
    "$workspace/.yolo_runtime/bin/python" -m pip install \
        --ignore-installed --index-url https://download.pytorch.org/whl/cpu \
        torch torchvision
fi
"$workspace/.yolo_runtime/bin/python" -m pip install -r "$repo_root/requirements-vehicle.txt"

if [[ ! -f "$map_dir/map.db" ]]; then
    xz -dk "$map_dir/map.db.xz"
fi
(
    cd "$map_dir"
    sha256sum --check map.db.sha256
    sha256sum --check route_optimized.json.sha256
)

cd "$workspace"
colcon build --symlink-install --packages-select \
    race_interfaces camera_bringup camera_navigation camera_yolo_inference \
    race_perception race_vehicle_interface race_control

echo
echo "Setup complete. Connect the D456 and vehicle MCU, then run:"
echo "  $repo_root/run_saved_map_bev_vehicle.sh"
