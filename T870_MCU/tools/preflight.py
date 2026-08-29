#!/usr/bin/env python3
"""T870 MCU 출발 전 점검 — "나는 되는데 팀원은 안 되는" 원인을 찾는다.

각자 자기 PC 에서 돌린다. ROS 가 안 켜져 있어도 절반은 검사된다.

    python3 tools/preflight.py
    python3 tools/preflight.py --ws ~/T870_MCU

정적 감사(tools/audit.py)는 소스와 yaml 의 아귀를 본다.
이건 그 소스가 "이 PC 에서 실제로 어떻게 도는지" 를 본다.
"""

import argparse
import glob
import io
import os
import re
import shutil
import subprocess
import sys

OK, WARN, BAD = [], [], []


def ok(m):
    OK.append(m)


def warn(m, fix=""):
    WARN.append((m, fix))


def bad(m, fix=""):
    BAD.append((m, fix))


def run(cmd, timeout=10):
    """명령 실행. (성공여부, 출력)"""
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True,
                           text=True, timeout=timeout)
        return r.returncode == 0, (r.stdout + r.stderr).strip()
    except Exception as exc:
        return False, str(exc)



def _expected_env():
    """t870_env.sh 에서 팀 합의값을 읽는다. (DOMAIN_ID, RMW)"""
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, "..", "t870_env.sh"),
                 os.path.join(here, "t870_env.sh"),
                 os.path.expanduser("~/T870_MCU/t870_env.sh")):
        if not os.path.isfile(cand):
            continue
        try:
            text = io.open(cand, encoding="utf-8").read()
        except Exception:
            continue
        d = re.search(r"^\s*export\s+ROS_DOMAIN_ID=(\S+)", text, re.M)
        r = re.search(r"^\s*export\s+RMW_IMPLEMENTATION=(\S+)", text, re.M)
        return (d.group(1).strip('"\'') if d else None,
                r.group(1).strip('"\'') if r else None)
    return (None, None)


# ==========================================================
# 1. 환경변수 — 팀끼리 다르면 서로 안 보인다
# ==========================================================
def check_env():
    print("\n[1] ROS 환경")

    distro = os.environ.get("ROS_DISTRO", "")
    if not distro:
        bad("ROS 가 source 안 되어 있다.", "source /opt/ros/jazzy/setup.bash")
    else:
        ok("ROS_DISTRO = %s" % distro)

    #  기대값은 t870_env.sh 한 곳에서만 읽는다.
    #  여기에 또 적으면 두 곳이 어긋나는 순간 이 검사 자체가 거짓말이 된다.
    want_domain, want_rmw = _expected_env()

    did = os.environ.get("ROS_DOMAIN_ID", "")
    print("    ROS_DOMAIN_ID   = %s" % (did or "(미설정 = 0)"))
    print("    ※ 팀 전체가 같아야 한다. 다르면 노드끼리 아예 안 보인다.")
    if want_domain:
        print("      팀 합의값(t870_env.sh): %s" % want_domain)
        if did != want_domain:
            warn("ROS_DOMAIN_ID 가 '%s' 다 — 팀 합의값은 %s"
                 % (did or "미설정", want_domain),
                 "터미널에서 ./실행.sh 를 쓰면 자동으로 맞춰진다. "
                 "다른 워크스페이스 터미널에서는 "
                 "source ~/T870_MCU/t870_env.sh")
        else:
            ok("ROS_DOMAIN_ID = %s (합의값과 일치)" % did)
    else:
        warn("t870_env.sh 를 못 찾아 합의값을 확인할 수 없다")

    rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    print("    RMW             = %s" % (rmw or "(기본 rmw_fastrtps_cpp)"))
    print("    ※ 이것도 팀끼리 같아야 한다. Fast-DDS 와 Cyclone 은 서로 안 보인다.")
    if want_rmw:
        cur = rmw or "rmw_fastrtps_cpp"
        if cur != want_rmw:
            warn("RMW 가 '%s' 다 — 팀 합의값은 %s" % (cur, want_rmw),
                 "source ~/T870_MCU/t870_env.sh")
        else:
            ok("RMW = %s (합의값과 일치)" % cur)

    if os.environ.get("ROS_LOCALHOST_ONLY") == "1":
        warn("ROS_LOCALHOST_ONLY=1 — 다른 PC 의 노드가 안 보인다.",
             "unset ROS_LOCALHOST_ONLY")

    if "cyclonedds" in rmw and not os.environ.get("CYCLONEDDS_URI"):
        # docker/veth 가 많으면 Cyclone 이 엉뚱한 인터페이스를 잡는다
        okk, out = run("ip -br link show 2>/dev/null | grep -c veth")
        n = int(out) if okk and out.isdigit() else 0
        if n > 0:
            warn("Cyclone 인데 veth 인터페이스가 %d개 있다 (Docker). "
                 "같은 PC 안에서도 노드끼리 못 찾을 수 있다." % n,
                 "CYCLONEDDS_URI 로 실제 네트워크 인터페이스를 지정할 것")


# ==========================================================
# 2. 시리얼 포트 권한 — permission denied 의 90%
# ==========================================================
def check_serial():
    print("\n[2] 시리얼 포트")

    okk, groups = run("id -nG")
    if okk and "dialout" in groups.split():
        ok("dialout 그룹에 속해 있다")
    else:
        bad("dialout 그룹에 없다 → 포트를 열 때 permission denied 가 난다.",
            "sudo usermod -aG dialout $USER  (그 다음 반드시 로그아웃/재로그인)")

    sym = "/dev/t870_mcu"
    if os.path.exists(sym):
        ok("%s 있음 (udev 규칙 설치됨)" % sym)
    else:
        warn("%s 없음 — udev 규칙이 안 깔렸다. by-id 로 자동탐색은 된다." % sym,
             "setup.sh 실행")

    byid = sorted(glob.glob("/dev/serial/by-id/*"))
    if byid:
        for b in byid:
            print("    %s -> %s" % (b, os.path.realpath(b)))
    else:
        warn("USB 시리얼 장치가 하나도 안 보인다. 아두이노 케이블을 확인할 것.")

    # 포트를 누가 잡고 있나
    for dev in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):
        if not shutil.which("fuser"):
            break
        okk, out = run("fuser -v %s 2>&1" % dev)
        if out and "cannot" not in out.lower() and dev in out:
            bad("%s 를 다른 프로세스가 잡고 있다:\n      %s" % (dev, out),
                "그 프로그램을 끄거나  sudo fuser -k %s" % dev)


# ==========================================================
# 3. 🔴 install/ 이 src/ 보다 오래됐나 — 최다 원인
# ==========================================================
def check_build(ws):
    print("\n[3] 빌드 상태")

    src = os.path.join(ws, "src", "t870_mcu")
    if not os.path.isdir(src):
        bad("%s 가 없다. 워크스페이스 경로가 맞는지 확인할 것." % src)
        return

    inst = os.path.join(ws, "install")
    if not os.path.isdir(inst):
        bad("install/ 이 없다. 빌드를 안 했다.",
            "cd %s && colcon build --symlink-install" % ws)
        return

    def newest(root, pat):
        t, who = 0, ""
        for p in glob.glob(os.path.join(root, pat), recursive=True):
            m = os.path.getmtime(p)
            if m > t:
                t, who = m, p
        return t, who

    ts, ws_ = newest(src, "**/*.py")
    ti, wi_ = newest(inst, "**/t870_mcu/**/*.py")

    if ts == 0:
        warn("src 에서 .py 를 못 찾았다")
    elif ti == 0:
        bad("install/ 안에 t870_mcu 코드가 없다. 빌드가 실패했을 수 있다.",
            "cd %s && colcon build --symlink-install" % ws)
    elif ts > ti + 1:
        import datetime
        f = "%Y-%m-%d %H:%M:%S"
        bad("🔴 install/ 이 src/ 보다 오래됐다. ros2 run 은 install/ 안의 "
            "코드를 실행하므로, 지금 고친 소스가 아니라 옛날 코드가 돈다.\n"
            "      src     최신 %s  (%s)\n"
            "      install 최신 %s  (%s)"
            % (datetime.datetime.fromtimestamp(ts).strftime(f),
               os.path.basename(ws_),
               datetime.datetime.fromtimestamp(ti).strftime(f),
               os.path.basename(wi_)),
            "cd %s && colcon build --symlink-install" % ws)
    else:
        ok("install/ 이 src/ 와 같거나 더 새것이다")

    # config 가 share 에 깔렸나
    shared = glob.glob(os.path.join(inst, "**", "share", "t870_mcu",
                                    "config", "t870_mcu.yaml"), recursive=True)
    if shared:
        ok("config/t870_mcu.yaml 이 share 에 설치됨")
    else:
        warn("share 에 t870_mcu.yaml 이 없다. launch 기본 경로가 깨진다.",
             "colcon build 를 다시 할 것")


# ==========================================================
# 4. 패키지가 여러 군데 깔려 있나
# ==========================================================
def check_duplicates(ws):
    print("\n[4] 패키지 중복")

    home = os.path.expanduser("~")
    found = []
    for depth in range(1, 6):
        pat = os.path.join(home, *(["*"] * depth), "src", "t870_mcu")
        found += [p for p in glob.glob(pat) if os.path.isdir(p)]
    found = sorted(set(found))

    for p in found:
        print("    소스 %s" % p)
    if len(found) > 1:
        bad("t870_mcu 소스가 %d군데 있다. 어느 것이 도는지 아무도 모른다."
            % len(found),
            "쓰지 않는 쪽을 다른 이름으로 옮길 것 (지우지 말고)")
    elif found:
        ok("소스는 한 군데뿐")

    prefixes = [p for p in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if p]
    hits = [p for p in prefixes
            if os.path.isdir(os.path.join(p, "share", "t870_mcu"))]
    for h in hits:
        print("    설치 %s" % h)
    if len(hits) > 1:
        bad("t870_mcu 가 AMENT_PREFIX_PATH 에 %d군데 있다. 앞의 것이 이긴다."
            % len(hits),
            "새 터미널을 열고 원하는 워크스페이스 하나만 source 할 것")
    elif hits:
        ok("설치 경로도 한 군데뿐")
    elif prefixes:
        warn("t870_mcu 가 source 되어 있지 않다.",
             "cd %s && source install/setup.bash" % ws)


# ==========================================================
# 5. 지금 돌고 있는 노드 / 토픽 (ROS 가 켜져 있을 때만)
# ==========================================================
def check_runtime():
    print("\n[5] 실행 중인 노드 · 토픽")

    if not shutil.which("ros2"):
        warn("ros2 명령이 없다. 이 항목은 건너뛴다.")
        return

    okk, out = run("ros2 node list", timeout=15)
    nodes = [l.strip() for l in out.splitlines() if l.strip().startswith("/")]
    if not nodes:
        print("    돌고 있는 노드 없음 — 노드를 띄운 뒤 다시 실행하면"
              " 토픽 충돌까지 검사한다.")
        return
    for n in nodes:
        print("    %s" % n)

    dup = {n for n in nodes if nodes.count(n) > 1}
    for n in dup:
        bad("노드 이름 '%s' 가 중복이다. 같은 노드를 두 번 띄웠다." % n,
            "하나를 Ctrl-C 로 끌 것")

    # 🔴 한 토픽에 발행자가 둘 이상이면 값이 섞인다
    KEY = ["/odom", "/mcu/ready", "/mcu/manager_ready", "/mcu/cmd_drive",
           "/mcu/cmd_wheel", "/mcu/cmd_stop", "/vehicle_mode", "/estop_lock",
           "/tf"]
    for t in KEY:
        okk, info = run("ros2 topic info %s" % t, timeout=10)
        if not okk or "Publisher count" not in info:
            continue
        m = re.search(r"Publisher count:\s*(\d+)", info)
        s = re.search(r"Subscription count:\s*(\d+)", info)
        np = int(m.group(1)) if m else 0
        ns = int(s.group(1)) if s else 0
        print("    %-22s 발행 %d / 구독 %d" % (t, np, ns))
        if t == "/tf":
            if np > 1:
                warn("/tf 발행자가 %d개다. odom→base 를 두 곳이 쏘면 "
                     "TF 트리가 깨진다. 아래로 누가 쏘는지 확인할 것." % np,
                     "ros2 topic info /tf --verbose")
        elif np > 1:
            bad("'%s' 발행자가 %d개다. 서로 다른 값이 번갈아 들어간다." % (t, np),
                "ros2 topic info %s --verbose  로 누구인지 확인" % t)
        elif np == 0 and ns > 0:
            warn("'%s' 는 구독자만 있고 아무도 발행하지 않는다." % t)

    # TF 트리
    okk, out = run("ros2 run tf2_tools view_frames --help", timeout=5)
    print("\n    TF 확인은 이렇게:")
    print("      ros2 run tf2_ros tf2_echo odom base_link")
    print("      (Nav2 를 쓰려면 odom -> base_link 가 반드시 이어져야 한다)")


# ==========================================================
def main():
    ap = argparse.ArgumentParser(description="T870 MCU 출발 전 점검")
    ap.add_argument("--ws", default=os.path.expanduser("~/T870_MCU"),
                    help="워크스페이스 경로 (기본 ~/T870_MCU)")
    ap.add_argument("--skip-runtime", action="store_true",
                    help="ROS 노드 검사 건너뛰기")
    args = ap.parse_args()

    ws = os.path.abspath(os.path.expanduser(args.ws))
    print("=" * 72)
    print("T870 MCU 출발 전 점검")
    print("워크스페이스: %s" % ws)
    print("=" * 72)

    check_env()
    check_serial()
    check_build(ws)
    check_duplicates(ws)
    if not args.skip_runtime:
        check_runtime()

    print("\n" + "=" * 72)
    if BAD:
        print("🔴 반드시 고칠 것 %d개" % len(BAD))
        for m, fix in BAD:
            print("\n  • %s" % m)
            if fix:
                print("    → %s" % fix)
    if WARN:
        print("\n🟡 확인할 것 %d개" % len(WARN))
        for m, fix in WARN:
            print("\n  • %s" % m)
            if fix:
                print("    → %s" % fix)
    if not BAD and not WARN:
        print("문제 없음. 출발해도 된다.")
    print("\n통과 %d개 / 경고 %d개 / 치명 %d개" % (len(OK), len(WARN), len(BAD)))
    return 1 if BAD else 0


if __name__ == "__main__":
    sys.exit(main())
