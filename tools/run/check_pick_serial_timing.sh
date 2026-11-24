#!/usr/bin/env bash
# STOP-latched or verified unprepared controller queries; never stops an owner.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
task_host="${JETSON_HOST:-hyper@192.168.35.2}"
mkdir -p "$task_root/output/pick-serial-timing"
task_output="$(mktemp -d "$task_root/output/pick-serial-timing/check-XXXXXXXX")"
python3 - "$task_root" "$task_output/probe.pyz" <<'PY'
from pathlib import Path
from zipfile import ZipFile
import sys
root=Path(sys.argv[1])
with ZipFile(sys.argv[2],'w') as archive:
    archive.write(root/'tools/diagnostics/probe_pick_serial_timing.py','__main__.py')
    package=root/'ros2_ws/src/so101_arm_bridge/so101_arm_bridge'
    for path in package.rglob('*.py'):
        archive.write(path,'so101_arm_bridge/'+path.relative_to(package).as_posix())
PY
task_args="$(python3 -c 'import shlex,sys;print(shlex.join(sys.argv[1:]))' "$@")"
task_remote='set -eu
task_python=/home/hyper/robot/shared/venvs/s4-jazzy/bin/python
if [ ! -x "$task_python" ]; then task_python=python3; fi
task_probe=$(mktemp)
trap '\''rm -f -- "$task_probe"'\'' EXIT
cat > "$task_probe"
timeout 50 "$task_python" "$task_probe"'
task_status=0
ssh -T -o ConnectTimeout=5 -o ServerAliveInterval=10 -o ServerAliveCountMax=2 \
    "$task_host" "$task_remote $task_args" < "$task_output/probe.pyz" \
    > "$task_output/report.json" 2> "$task_output/ssh.log" || task_status=$?
if [ -s "$task_output/report.json" ]; then
python3 - "$task_output/report.json" <<'PY'
import json,sys
r=json.load(open(sys.argv[1]))
print('STATUS:',r['status'])
print('MODE:',r['mode'])
if r.get('lift_session_queries'):print('LIFT_QUERIES:',r['lift_session_queries'])
if r.get('error'):print('ERROR:',r['error'])
for phase,groups in r.get('phases',{}).items():
    for name,g in groups.items():
        print(f"{phase} {name}: max={g['maximum_ms']:.1f}ms, >=50ms={g['over_50ms']}/{g['count']}")
print('MOTION_COMMANDS:',r['motion_commands'])
PY
fi
if [ -s "$task_output/ssh.log" ]; then cat "$task_output/ssh.log" >&2; fi
printf '\nREPORT: %s/report.json\n' "$task_output"
exit "$task_status"
