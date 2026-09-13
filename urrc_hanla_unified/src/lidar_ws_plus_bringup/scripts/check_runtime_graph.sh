#!/usr/bin/env bash
set -uo pipefail

failures=0

check_topic_count() {
  local topic="$1" expected="$2"
  local output count
  if ! output="$(ros2 topic info "${topic}" -v 2>&1)"; then
    echo "FAIL ${topic}: ${output}" >&2
    failures=$((failures + 1))
    return
  fi
  printf '%s\n' "${output}"
  count="$(printf '%s\n' "${output}" | sed -n 's/^Publisher count: *//p' | head -n1)"
  if [[ "${count}" != "${expected}" ]]; then
    echo "FAIL ${topic}: publisher count=${count:-unknown}, expected=${expected}" >&2
    failures=$((failures + 1))
  else
    echo "PASS ${topic}: publisher count=${count}"
  fi
}

check_tf() {
  local parent="$1" child="$2" required="$3"
  if timeout 4s ros2 run tf2_ros tf2_echo "${parent}" "${child}" >/tmp/lidar_ws_plus_tf.log 2>&1; then
    echo "PASS TF ${parent} -> ${child}"
  elif rg -q 'At time|Translation:|Rotation:' /tmp/lidar_ws_plus_tf.log; then
    echo "PASS TF ${parent} -> ${child} (sample received before timeout)"
  elif [[ "${required}" == true ]]; then
    echo "FAIL TF ${parent} -> ${child}" >&2
    sed -n '1,20p' /tmp/lidar_ws_plus_tf.log >&2
    failures=$((failures + 1))
  else
    echo "WARN optional TF ${parent} -> ${child} unavailable"
  fi
}

for topic in /lidar_drive /lidar_wheel /lidar_stop /front/scan /odom; do
  check_topic_count "${topic}" 1
done

for topic in /lidar_drive /lidar_wheel /lidar_stop; do
  if ! ros2 topic info "${topic}" -v | rg -q 'Node name: command_mux'; then
    echo "FAIL ${topic}: command_mux is not the reported owner" >&2
    failures=$((failures + 1))
  fi
done

check_tf base_link front_laser true
check_tf odom base_link true
check_tf map odom false

tf_owner_candidates="$(ros2 node list 2>/dev/null | rg -c \
  'base_to_front_laser_static_tf|real_robot_state_publisher' || true)"
if (( tf_owner_candidates > 1 )); then
  echo 'FAIL multiple front-LiDAR TF owner candidates are running' >&2
  failures=$((failures + 1))
fi

if ros2 pkg executables tf2_tools 2>/dev/null | rg -q 'view_frames'; then
  output_dir="${PWD}/runtime_graph_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "${output_dir}"
  (cd "${output_dir}" && timeout 8s ros2 run tf2_tools view_frames) || true
  echo "TF graph artifacts: ${output_dir}"
fi

if (( failures > 0 )); then
  echo "runtime graph check: FAIL (${failures})" >&2
  exit 1
fi
echo 'runtime graph check: PASS'
