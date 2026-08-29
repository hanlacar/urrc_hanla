#!/usr/bin/env bash
# ==========================================================
#  T870 MCU 설치 스크립트
#
#  포트 권한과 udev 규칙을 한 번에 잡는다.
#  포트 번호(/dev/ttyACM0, ACM1 ...)는 USB 를 꽂는 순서에 따라
#  매번 바뀌므로, 번호 대신 장치 정보로 고정한다.
#
#  사용:
#      chmod +x setup.sh
#      ./setup.sh
#
#  ★ 이 스크립트는 중간에 실패해도 멈추지 않는다.
#    팀원 PC 에 깨진 apt 저장소(Spotify, VS Code 등)가 하나만 있어도
#    apt-get update 가 실패하는데, 그것 때문에 정작 중요한 udev 규칙이
#    안 깔리면 안 되기 때문이다. 실패한 단계는 경고로 표시하고 넘어간다.
#
#  --skip-apt  로 패키지 설치 단계를 아예 건너뛸 수 있다.
# ==========================================================

# set -e 를 쓰지 않는다. (위 이유)
SKIP_APT=0
for arg in "$@"; do
    case "$arg" in
        --skip-apt) SKIP_APT=1 ;;
        -h|--help)
            echo "사용: ./setup.sh [--skip-apt]"; exit 0 ;;
    esac
done

WARNINGS=()
warn() { WARNINGS+=("$1"); echo "      ⚠ $1"; }

# 워크스페이스 경로는 스크립트 위치에서 구한다.
# ★ ~/mcu_ws 로 박아두면 팀원이 다른 곳에 클론했을 때 안내가 전부 틀린다.
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================================="
echo "  T870 MCU 설치"
echo "  워크스페이스: $WS_DIR"
echo "=========================================================="

# ---------- 1. 의존 패키지 ----------
echo ""
echo "[1/5] 의존 패키지"

need_pkgs=()
python3 -c "import serial" 2>/dev/null || need_pkgs+=(python3-serial)
command -v colcon >/dev/null 2>&1 || need_pkgs+=(python3-colcon-common-extensions)

if [ "$SKIP_APT" -eq 1 ]; then
    echo "      --skip-apt 지정됨 — 건너뛴다"
    [ ${#need_pkgs[@]} -gt 0 ] && warn "설치 필요: ${need_pkgs[*]}"
elif [ ${#need_pkgs[@]} -eq 0 ]; then
    echo "      python3-serial, colcon 이미 설치됨 — apt 생략"
else
    echo "      설치 필요: ${need_pkgs[*]}"
    # apt-get update 실패는 대개 우리와 무관한 서드파티 저장소 문제다.
    # 실패해도 설치는 시도해 본다 (캐시에 있으면 그대로 깔린다).
    if ! sudo apt-get update -qq 2>/dev/null; then
        warn "apt-get update 실패 — 깨진 저장소가 있는 것 같다. 설치는 계속 시도한다"
        echo "        (확인: sudo apt-get update  로 어느 저장소인지 볼 수 있다)"
    fi
    if sudo apt-get install -y "${need_pkgs[@]}" >/dev/null 2>&1; then
        echo "      설치 완료"
    else
        warn "패키지 설치 실패: ${need_pkgs[*]}"
        echo "        수동 설치: sudo apt-get install ${need_pkgs[*]}"
    fi
fi

# ---------- 2. dialout 그룹 ----------
echo ""
echo "[2/5] 시리얼 포트 권한"
if groups "$USER" | grep -qw dialout; then
    echo "      이미 dialout 그룹에 속해 있다"
else
    sudo usermod -aG dialout "$USER"
    echo "      dialout 그룹에 추가했다"
    echo "      ★ 로그아웃 후 다시 로그인해야 적용된다"
    echo "        (지금 바로 쓰려면 아래 chmod 를 쓴다)"
fi

# ---------- 3. udev 규칙 ----------
echo ""
echo "[3/5] udev 규칙 — 포트 이름 고정"
sudo tee /etc/udev/rules.d/99-t870.rules > /dev/null << 'RULES'
# T870 자율주행 차량 — 포트 고정
#
# 꽂는 순서와 무관하게 항상 같은 이름을 쓴다.
#   /dev/t870_mcu    아두이노 Mega (하위제어기)
#   /dev/t870_gps    u-blox GNSS
#   /dev/t870_lidar  RPLIDAR (CP2102)
#
# 벤더 ID 기준이라 어느 컴퓨터에서도 동일하게 동작한다.

# 아두이노 (정품: 2341 / 클론 CH340: 1a86)
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", MODE="0666", SYMLINK+="t870_mcu"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2a03", MODE="0666", SYMLINK+="t870_mcu"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666", SYMLINK+="t870_mcu"

# u-blox GNSS
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", MODE="0666", SYMLINK+="t870_gps"

# CP2102 (RPLIDAR 등)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666", SYMLINK+="t870_lidar"
RULES

if sudo udevadm control --reload-rules && sudo udevadm trigger; then
    echo "      /etc/udev/rules.d/99-t870.rules 적용"
else
    warn "udev 규칙 재적용 실패 — 재부팅하면 적용된다"
fi

# ---------- 4. 현재 포트에 즉시 권한 ----------
echo ""
echo "[4/5] 현재 연결된 포트에 즉시 권한 부여"
found=0
for p in /dev/ttyACM* /dev/ttyUSB*; do
    [ -e "$p" ] || continue
    sudo chmod 666 "$p"
    echo "      $p"
    found=1
done
[ $found -eq 0 ] && echo "      연결된 시리얼 포트가 없다"

# ---------- 5. 확인 ----------
echo ""
echo "[5/5] 결과"
echo ""
if [ -e /dev/t870_mcu ]; then
    echo "      /dev/t870_mcu  ->  $(readlink -f /dev/t870_mcu)"
else
    echo "      /dev/t870_mcu 없음 — 아두이노가 연결되지 않았거나"
    echo "      USB 를 뽑았다 다시 꽂아야 규칙이 적용된다"
fi
[ -e /dev/t870_gps ]   && echo "      /dev/t870_gps    ->  $(readlink -f /dev/t870_gps)"
[ -e /dev/t870_lidar ] && echo "      /dev/t870_lidar  ->  $(readlink -f /dev/t870_lidar)"

echo ""
echo "=========================================================="
if [ ${#WARNINGS[@]} -eq 0 ]; then
    echo "  설치 완료"
else
    echo "  설치 완료 (경고 ${#WARNINGS[@]}건)"
    for w in "${WARNINGS[@]}"; do echo "    ⚠ $w"; done
    echo ""
    echo "  경고가 있어도 /dev/t870_mcu 가 만들어졌으면 사용에는 지장이 없다."
fi
echo "=========================================================="
cat << NEXT

  빌드:
      cd "$WS_DIR" && colcon build --symlink-install
      source "$WS_DIR/install/setup.bash"

  실행 (포트를 적을 필요가 없다. 자동으로 찾는다):
      ros2 launch t870_mcu t870_mcu.launch.py

  조종:
      python3 "$WS_DIR/tools/drive_wasd.py"

  포트가 안 잡히면:
      python3 "$WS_DIR/tools/check_ports.py"

NEXT
