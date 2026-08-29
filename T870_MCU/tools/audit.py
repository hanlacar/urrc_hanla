#!/usr/bin/env python3
"""T870 MCU 정합성 감사 v2 — "나는 되는데 팀원은 안 되는" 상황을 미리 잡는다.

ROS 없이 소스와 yaml 만 읽는다. 루프로 선언되는 파라미터도 인식한다.
"""
import ast, io, os, re, sys

ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG  = os.path.join(ROOT, "src", "t870_mcu")
SRC  = os.path.join(PKG, "t870_mcu")
CFG  = os.path.join(PKG, "config", "t870_mcu.yaml")
TOOLS = os.path.join(ROOT, "tools")

TQ_D = chr(34) * 3
TQ_S = chr(39) * 3

problems = []
def bad(m, c=""):  problems.append(("🔴", c, m))
def warn(m, c=""): problems.append(("🟡", c, m))
def note(m, c=""): problems.append(("ℹ️", c, m))

def ros_type(v):
    if v is None:            return "NOT_SET"
    if isinstance(v, bool):  return "BOOL"
    if isinstance(v, int):   return "INTEGER"
    if isinstance(v, float): return "DOUBLE"
    if isinstance(v, str):   return "STRING"
    if isinstance(v, (list, tuple)):
        if len(v) == 0: return "빈배열(타입불명)"
        if all(isinstance(x, bool)  for x in v): return "BOOL_ARRAY"
        if all(isinstance(x, int)   for x in v): return "INTEGER_ARRAY"
        if all(isinstance(x, float) for x in v): return "DOUBLE_ARRAY"
        if all(isinstance(x, str)   for x in v): return "STRING_ARRAY"
        return "혼합배열(불가)"
    return "?"

import yaml
Y = yaml.safe_load(io.open(CFG, encoding="utf-8"))

# ---------- 소스 파싱 ----------
def scan(path, sources):
    """노드이름, {param:(기본값,라인)}, 루프선언 파라미터 집합, 모든 문자열"""
    txt = io.open(path, encoding="utf-8").read()
    tree = ast.parse(txt)
    name, lit, loop, strings = None, {}, set(), set()

    for n in ast.walk(tree):
        if isinstance(n, ast.Constant) and isinstance(n.value, str):
            strings.add(n.value)

    # bridge 의 pubdefs 처럼 [("이름", 기본값), ...] 를 돌며 선언하는 형태.
    # 튜플 자체가 선언 정보이므로 기본값 타입까지 같이 얻는다.
    pairs = {}
    for n in ast.walk(tree):
        if not isinstance(n, (ast.List, ast.Tuple)):
            continue
        for e in n.elts:
            if not (isinstance(e, ast.Tuple) and len(e.elts) == 2):
                continue
            k = e.elts[0]
            if not (isinstance(k, ast.Constant) and isinstance(k.value, str)):
                continue
            try:
                v = ast.literal_eval(e.elts[1])
            except Exception:
                continue
            pairs[k.value] = (v, e.lineno)

    for n in ast.walk(tree):
        if not isinstance(n, ast.Call):
            continue
        f = n.func
        if (isinstance(f, ast.Attribute) and f.attr == "__init__"
                and n.args and isinstance(n.args[0], ast.Constant)):
            name = n.args[0].value
        if not (isinstance(f, ast.Attribute) and f.attr == "declare_parameter"):
            continue
        if not n.args:
            continue
        a0 = n.args[0]
        if isinstance(a0, ast.Constant):            # 리터럴 선언
            d = "<없음>"
            if len(n.args) > 1:
                try:    d = ast.literal_eval(n.args[1])
                except Exception: d = "<식>"
            lit[a0.value] = (d, n.lineno)
        else:
            # 루프 선언: "%s_drive_topic" % source 형태를 모두 전개한다
            # "%s_drive_topic" 같은 템플릿만 고른다.
            # 로그 포맷 문자열(%d, %r, 공백 포함)은 파라미터 이름이 아니다.
            for s in strings:
                if (s.count("%") == 1 and "%s" in s
                        and " " not in s and not s.startswith("/")
                        and re.fullmatch(r"[%A-Za-z0-9_]+", s)):
                    for src in sources:
                        loop.add(s % src)
            # 이름이 변수면 pairs 에서 찾은 이름들도 선언으로 인정한다
            if isinstance(a0, ast.Name):
                for k, v in pairs.items():
                    lit.setdefault(k, v)
    return name, lit, loop, strings

mgr_cfg = (Y.get("mcu_manager") or {}).get("ros__parameters", {}) or {}
SOURCES = mgr_cfg.get("source_names", ["lidar", "camera", "gps", "manual"])

nodes = {}
for fn in sorted(os.listdir(SRC)):
    if not fn.endswith("_node.py"): continue
    nm, lit, loop, strings = scan(os.path.join(SRC, fn), SOURCES)
    if nm: nodes[nm] = dict(file=fn, lit=lit, loop=loop, strings=strings)

print("=" * 72)
print("T870 MCU 정합성 감사")
print("=" * 72)
for nm, d in nodes.items():
    print("  %-13s %-18s 리터럴 %d / 루프 %d"
          % (nm, d["file"], len(d["lit"]), len(d["loop"])))
print()

# [1] yaml 섹션 == 노드 이름
for sec in Y:
    if sec not in nodes:
        bad("yaml 섹션 '%s:' 에 맞는 노드가 없다 → 그 섹션 전체가 무시된다." % sec, "[1] 섹션이름")
for nm in nodes:
    if nm not in Y:
        bad("노드 '%s' 용 yaml 섹션이 없다 → 전부 기본값." % nm, "[1] 섹션이름")

# [2] 타입 불일치 (노드가 시작하자마자 죽는 부류)
for sec, d in nodes.items():
    yd = (Y.get(sec) or {}).get("ros__parameters", {}) or {}
    for k, yv in yd.items():
        if k not in d["lit"]: continue
        dflt, line = d["lit"][k]
        if dflt in ("<없음>", "<식>"): continue
        td, ty = ros_type(dflt), ros_type(yv)
        if td.startswith("빈배열"):
            bad("%s:%d  '%s' 기본값이 빈 리스트 → rclpy 가 타입을 못 정한다. 노드가 죽는다."
                % (d["file"], line, k), "[2] 타입")
        elif td != ty:
            bad("%s:%d  '%s' 타입 불일치 — 코드 %s(%r) vs yaml %s(%r). 노드가 죽는다."
                % (d["file"], line, k, td, dflt, ty, yv), "[2] 타입")

# [3] yaml 에만 있는 키 (오타 → 조용히 무시)
for sec, d in nodes.items():
    yd = (Y.get(sec) or {}).get("ros__parameters", {}) or {}
    known = set(d["lit"]) | d["loop"]
    for k in yd:
        if k not in known:
            bad("yaml [%s] 의 '%s' 를 코드가 선언하지 않는다 → 에러 없이 무시된다."
                % (sec, k), "[3] 유령키")

# [4] 빈 컨테이너 기본값
for sec, d in nodes.items():
    for k, (dflt, line) in d["lit"].items():
        if isinstance(dflt, (list, tuple)) and len(dflt) == 0:
            bad("%s:%d  '%s' 기본값이 빈 리스트 (yaml 없이 띄우면 터진다)"
                % (d["file"], line, k), "[4] 빈기본값")

# [5] ★ 노드끼리 같은 토픽을 발행하는가
PUBKEY = re.compile(r"^(pub_topic_|status_|output_)")
pubs = {}
for sec in nodes:
    yd = (Y.get(sec) or {}).get("ros__parameters", {}) or {}
    for k, v in yd.items():
        if PUBKEY.match(k) and isinstance(v, str) and v.startswith("/"):
            pubs.setdefault(v, []).append("%s.%s" % (sec, k))
for topic, who in sorted(pubs.items()):
    if len(who) > 1:
        bad("토픽 '%s' 를 두 곳이 발행한다 → %s. 구독자는 두 뜻이 섞인 값을 받는다."
            % (topic, " / ".join(who)), "[5] 토픽충돌")

# [6] 매니저 출력 ↔ 브릿지 입력
brg = (Y.get("mcu_bridge") or {}).get("ros__parameters", {}) or {}
LINK = [("구동", "output_drive_topic", "input_drive_topic"),
        ("조향", "output_wheel_topic", "input_wheel_topic"),
        ("정지", "output_stop_topic",  "input_stop_topic"),
        ("비상", "estop_topic",        "estop_topic")]
for what, mk, bk in LINK:
    mv = mgr_cfg.get(mk, nodes.get("mcu_manager", {}).get("lit", {}).get(mk, ("?",))[0])
    bv = brg.get(bk,    nodes.get("mcu_bridge",  {}).get("lit", {}).get(bk, ("?",))[0])
    if mv != bv:
        bad("%s 명령이 안 간다 — 매니저 %s='%s' ≠ 브릿지 %s='%s'"
            % (what, mk, mv, bk, bv), "[6] 배선")

# [7] 하드코딩
HARD = [(r"/dev/tty[A-Z]+[0-9]", "시리얼 포트"),
        (r"/home/[a-z][a-z0-9_-]*", "홈 경로"),
        (r"192\.168\.\d+\.\d+", "고정 IP")]
for base in (SRC, TOOLS):
    if not os.path.isdir(base): continue
    for fn in sorted(os.listdir(base)):
        if not fn.endswith(".py"): continue
        in_doc = False
        for i, line in enumerate(io.open(os.path.join(base, fn), encoding="utf-8"), 1):
            # 삼중따옴표 독스트링 안은 설명문이라 검사에서 뺀다
            q = line.count(TQ_D) + line.count(TQ_S)
            if in_doc:
                if q % 2 == 1:
                    in_doc = False
                continue
            if q % 2 == 1:
                in_doc = True
                continue
            code = line.split("#", 1)[0]
            for pat, what in HARD:
                if re.search(pat, code):
                    warn("%s:%d  %s → %s" % (fn, i, what, code.strip()[:64]), "[7] 하드코딩")

# [8] package.xml
pkgxml = io.open(os.path.join(PKG, "package.xml"), encoding="utf-8").read()
declared = set(re.findall(r"<(?:exec_|build_)?depend>([^<]+)</", pkgxml))
ROSMODS = ["rclpy", "std_msgs", "geometry_msgs", "nav_msgs", "std_srvs",
           "tf2_ros", "rcl_interfaces", "sensor_msgs"]
used = set()
for fn in os.listdir(SRC):
    if not fn.endswith(".py"): continue
    s = io.open(os.path.join(SRC, fn), encoding="utf-8").read()
    for m in ROSMODS:
        if re.search(r"^\s*(from|import)\s+%s\b" % m, s, re.M):
            used.add(m)
for m in sorted(used - declared):
    bad("package.xml 에 <depend>%s</depend> 가 없다. 내 PC 엔 이미 깔려 있어 "
        "돌지만 팀원이 rosdep 으로 깔면 빠진다." % m, "[8] 의존성")

# [9] setup.py
setup = io.open(os.path.join(PKG, "setup.py"), encoding="utf-8").read()
for what in ("config", "launch"):
    if what not in setup:
        bad("setup.py 가 %s/ 를 설치하지 않는다 → launch 기본 경로가 깨진다." % what, "[9] 설치")
for exe in ("bridge", "manager"):
    if "%s = t870_mcu" % exe not in setup:
        bad("setup.py entry_points 에 '%s' 가 없다 → ros2 run 불가." % exe, "[9] 설치")

# [10] launch 이름
ls = io.open(os.path.join(PKG, "launch", "t870_mcu.launch.py"), encoding="utf-8").read()
for m in re.finditer(r'executable="([^"]+)",\s*\n\s*name="([^"]+)"', ls):
    exe, nm = m.groups()
    if nm not in nodes:
        bad("launch 의 name=\"%s\" 에 맞는 노드가 없다 → 파라미터가 통째로 무시된다." % nm, "[10] 런치")
    if "%s = t870_mcu" % exe not in setup:
        bad("launch 의 executable=\"%s\" 가 setup.py 에 없다." % exe, "[10] 런치")

# [11] 파이썬 문법
import py_compile
for base in (SRC, TOOLS):
    if not os.path.isdir(base): continue
    for fn in sorted(os.listdir(base)):
        if not fn.endswith(".py"): continue
        try: py_compile.compile(os.path.join(base, fn), doraise=True, cfile="/tmp/x.pyc")
        except Exception as e:
            bad("%s 문법 오류: %s" % (fn, e), "[11] 문법")

order = {"🔴": 0, "🟡": 1, "ℹ️": 2}
problems.sort(key=lambda x: (order[x[0]], x[1]))
cur = None
for sev, cat, msg in problems:
    if cat != cur:
        print("\n--- %s ---" % (cat or "기타")); cur = cat
    print("%s %s" % (sev, msg))
nb = sum(1 for p in problems if p[0] == "🔴")
nw = sum(1 for p in problems if p[0] == "🟡")
print("\n" + "=" * 72)
print("치명 %d개 / 경고 %d개" % (nb, nw))
sys.exit(1 if nb else 0)
