#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo 'ROS 2 Jazzy가 필요합니다: /opt/ros/jazzy/setup.bash' >&2
    exit 1
fi

# 다른 작업 공간의 패키지가 통합 빌드에 섞이지 않도록 한다.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash

# 저장소 안의 기존/백업 워크스페이스는 검색하지 않는다.
colcon build --base-paths "$workspace_dir/src" --symlink-install \
    --parallel-workers 2 "$@" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
