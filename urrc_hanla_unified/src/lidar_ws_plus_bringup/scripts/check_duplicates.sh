#!/usr/bin/env bash
set -uo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
src_dir="${workspace}/src"
failures=0

echo '[1/5] package.xml names'
package_rows="$({
  find "${src_dir}" -name package.xml -print0 |
    while IFS= read -r -d '' manifest; do
      name="$(sed -n 's:.*<name>\([^<]*\)</name>.*:\1:p' "${manifest}" | head -n1)"
      printf '%s|%s\n' "${name}" "${manifest#${workspace}/}"
    done
} | sort)"
printf '%s\n' "${package_rows}"
duplicates="$(printf '%s\n' "${package_rows}" | cut -d'|' -f1 | uniq -d)"
if [[ -n "${duplicates}" ]]; then
  echo "FAIL duplicate package names: ${duplicates}" >&2
  failures=$((failures + 1))
fi
rplidar_count="$(printf '%s\n' "${package_rows}" | cut -d'|' -f1 | grep -cx rplidar_ros || true)"
if [[ "${rplidar_count}" != 1 ]]; then
  echo "FAIL rplidar_ros count=${rplidar_count}, expected 1" >&2
  failures=$((failures + 1))
fi

echo '[2/5] executable-name candidates'
python3 - "${src_dir}" <<'PY'
from collections import defaultdict
from pathlib import Path
import re
import sys

root = Path(sys.argv[1])
owners = defaultdict(set)
for setup in root.glob('*/setup.py'):
    text = setup.read_text(errors='replace')
    for name in re.findall(r"['\"]([A-Za-z0-9_.-]+)\s*=\s*[^'\"]+:[^'\"]+['\"]", text):
        owners[name].add(setup.parent.name)
for cmake in root.glob('*/CMakeLists.txt'):
    text = cmake.read_text(errors='replace')
    for block in re.findall(r'install\s*\(PROGRAMS(.*?)DESTINATION', text, re.S):
        for item in re.findall(r'(?:scripts/)?([A-Za-z0-9_.-]+\.py)', block):
            owners[item].add(cmake.parent.name)
for name, packages in sorted(owners.items()):
    if len(packages) > 1:
        print(f'WARN executable candidate {name}: {sorted(packages)}')
PY

echo '[3/5] final MCU publisher candidates'
publisher_hits="$(python3 - "${src_dir}" <<'PY'
import ast
from pathlib import Path
import re
import sys

root = Path(sys.argv[1])
final = {'/lidar_drive', '/lidar_wheel', '/lidar_stop'}
for path in root.rglob('*.py'):
    if 'test' in path.parts or path.name == 'command_mux.py':
        continue
    try:
        tree = ast.parse(path.read_text(errors='replace'))
    except SyntaxError:
        continue
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Attribute) or node.func.attr != 'create_publisher':
            continue
        for argument in node.args:
            if isinstance(argument, ast.Constant) and argument.value in final:
                print(f'{path}:{node.lineno}:{argument.value}')
for path in list(root.rglob('*.cpp')) + list(root.rglob('*.hpp')):
    text = path.read_text(errors='replace')
    for match in re.finditer(
            r'create_publisher\s*<[^>]+>\s*\(\s*["\"]'
            r'(/lidar_(?:drive|wheel|stop))["\"]', text):
        print(f'{path}:{text.count(chr(10), 0, match.start()) + 1}:{match.group(1)}')
PY
)"
if [[ -n "${publisher_hits}" ]]; then
  printf '%s\n' "${publisher_hits}"
  echo 'FAIL final MCU topic publisher exists outside command_mux' >&2
  failures=$((failures + 1))
else
  echo 'PASS only command_mux has final-topic publisher declarations'
fi

echo '[4/5] static TF owner candidates (legacy launches are inventory only)'
rg -n "static_transform_publisher|robot_state_publisher" "${src_dir}" \
  -g '*.launch.py' -g '!**/test/**' || true

echo '[5/5] hardcoded old workspace paths'
home_word='home'
home_pattern="/${home_word}/"
lidar_name='Lidar_ws'
avoidance_name='avoidance_sim'
parking_name='t_parking_ws'
old_pattern="~/(${lidar_name}|${avoidance_name}|${parking_name})|${lidar_name}/install|${avoidance_name}/install|${parking_name}/install"
path_hits="$({
  rg -n "${home_pattern}[^/]+/" "${src_dir}" \
    -g '!**/.git/**' -g '!**/build/**' -g '!**/install/**' -g '!**/log/**' \
    -g '!check_duplicates.sh' || true
  rg -n "${old_pattern}" "${src_dir}" \
    -g '!**/.git/**' -g '!**/build/**' -g '!**/install/**' -g '!**/log/**' \
    -g '!check_duplicates.sh' || true
})"
if [[ -n "${path_hits}" ]]; then
  printf '%s\n' "${path_hits}"
  echo 'FAIL hardcoded historical workspace path found' >&2
  failures=$((failures + 1))
else
  echo 'PASS no hardcoded historical workspace paths'
fi

if (( failures > 0 )); then
  echo "duplicate/static contract check: FAIL (${failures})" >&2
  exit 1
fi
echo 'duplicate/static contract check: PASS'
