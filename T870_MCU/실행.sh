#!/usr/bin/env bash
# ==========================================================
#  T870 MCU 실행 — 매번 이것만 치면 된다
#
#    ./실행.sh          브릿지 + 매니저 실행 (평소 이거)
#    ./실행.sh 조종     터미널로 직접 운전 (WASD)
#    ./실행.sh 점검     이 PC 환경 점검 (안 될 때 먼저 이것)
#    ./실행.sh 상태     지금 뭐가 도는지 한눈에
#
#  source 도 export 도 이 스크립트가 알아서 한다.
#  터미널을 새로 열 때마다 뭘 칠 필요 없다.
# ==========================================================
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export T870_WS="$HERE"

if [ ! -f "$HERE/t870_env.sh" ]; then
  echo "t870_env.sh 가 없다. 설치가 덜 됐다."
  echo "  → ./설치.sh --apply 를 먼저 실행할 것"
  exit 1
fi
# shellcheck disable=SC1090
. "$HERE/t870_env.sh"

# ROS setup scripts may inspect optional variables that are not defined yet.
# Enable nounset only after the environment has been sourced.
set -u

MODE="${1:-실행}"

#  ★ 점검은 "안 될 때" 돌리는 것이라 빌드가 없어도 실행되어야 한다.
#    나머지 명령만 빌드를 요구한다.
if [ "$MODE" != "점검" ] && [ "$MODE" != "check" ] \
   && [ ! -f "$HERE/install/setup.bash" ]; then
  echo "빌드가 안 되어 있다 (install/ 없음)."
  echo "  → cd $HERE && colcon build --symlink-install"
  echo "  또는  ./실행.sh 점검   으로 무엇이 문제인지 먼저 볼 것"
  exit 1
fi

echo "───────────────────────────────────────────"
echo " 워크스페이스 : $T870_WS"
echo " DOMAIN_ID    : ${ROS_DOMAIN_ID:-미설정}"
echo " RMW          : ${RMW_IMPLEMENTATION:-기본}"
echo "───────────────────────────────────────────"

case "$MODE" in
  점검|check)
    exec python3 "$HERE/tools/preflight.py" --ws "$HERE"
    ;;

  상태|status)
    echo; echo "== 돌고 있는 노드 =="
    ros2 node list
    echo; echo "== 중요 토픽 발행자 수 (1 이 아니면 문제) =="
    for t in /odom /tf /mcu/cmd_drive /drive_mode /mcu/ready; do
      n=$(ros2 topic info "$t" 2>/dev/null | sed -n 's/.*Publisher count: //p')
      printf "  %-18s %s\n" "$t" "${n:-없음}"
    done
    ;;

  조종|drive)
    echo
    echo " ⚠ 창이 뜨면 반드시 0 키를 눌러 조종권을 잡아야 한다."
    echo "   그 전에는 팀 노드를 방해하지 않으려고 아무것도 발행하지 않는다."
    echo "   (W A S D 주행 / F 급정거 / E 비상정지 / Q 종료)"
    echo
    exec python3 "$HERE/tools/drive_wasd.py" --takeover
    ;;

  실행|run)
    echo
    echo " 브릿지 + 매니저를 띄운다. 끄려면 Ctrl-C."
    echo
    exec ros2 launch t870_mcu t870_mcu.launch.py
    ;;

  *)
    echo "모르는 명령: $MODE"
    echo "  ./실행.sh        브릿지+매니저"
    echo "  ./실행.sh 조종   터미널 운전"
    echo "  ./실행.sh 점검   환경 점검"
    echo "  ./실행.sh 상태   현재 상태"
    exit 1
    ;;
esac
