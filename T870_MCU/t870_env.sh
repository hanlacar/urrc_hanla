#!/usr/bin/env bash
# ==========================================================
#  T870 공통 환경 — 팀 전체가 같은 값이어야 한다
#
#  ROS 2 는 DOMAIN_ID 나 RMW 가 하나라도 다르면
#  **노드끼리 서로 존재조차 모른다.** 에러도 안 난다.
#  "나는 되는데 너는 안 되는" 상황의 1순위 원인이다.
#
#  ⚠ 이 파일은 팀에서 한 벌만 유지한다. 각자 고치지 말 것.
#    값을 바꿔야 하면 MCU 담당(SJ)에게 말하고 전원이 같이 받는다.
# ==========================================================

# ---- 팀 합의값 ----
#  2026-08-29 기준 실제로 팀이 쓰고 있는 값을 그대로 넣었다.
#  현재 차량 전체 노드 합의값: ROS_DOMAIN_ID=10
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 다른 PC 의 노드가 보여야 하므로 켜 두면 안 된다
unset ROS_LOCALHOST_ONLY

# ---- ROS 본체 ----
if [ -f /opt/ros/jazzy/setup.bash ]; then
  . /opt/ros/jazzy/setup.bash
fi

# ---- 이 워크스페이스 ----
#  T870_WS 가 미리 잡혀 있으면 그것을 쓰고, 없으면 이 파일 위치에서 찾는다.
if [ -z "${T870_WS:-}" ]; then
  T870_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi
if [ -f "$T870_WS/install/setup.bash" ]; then
  . "$T870_WS/install/setup.bash"
fi
export T870_WS
