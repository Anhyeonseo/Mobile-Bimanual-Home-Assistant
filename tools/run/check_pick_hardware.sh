#!/usr/bin/env bash
# Read Jetson status and save a local report. No deployment or motor commands.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
task_host="${JETSON_HOST:-hyper@192.168.35.2}"
task_parent="$task_root/output/pick-hardware-check"
mkdir -p "$task_parent"
task_output="$(mktemp -d "$task_parent/check-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"
# Stream the same diagnostic and headless camera reader used in the local
# checkout, including on Jetsons without the web pick station installed.
python3 - "$task_root" "$task_output/inspect.pyz" <<'PY'
from pathlib import Path
import sys
from zipfile import ZipFile
root = Path(sys.argv[1])
with ZipFile(sys.argv[2], 'w') as bundle:
    bundle.write(root/'tools/diagnostics/inspect_pick_hardware.py', '__main__.py')
    for package in ('tools', 'tools/diagnostics'):
        bundle.writestr(package+'/__init__.py', '')
    for name in ('realsense_viewer', 'realsense_depth_preview'):
        relative = 'tools/diagnostics/'+name+'.py'
        bundle.write(root/relative, relative)
PY
task_python_arg="$(python3 -c 'import shlex,sys; print(shlex.quote(sys.argv[1]))' "${JETSON_PICK_PYTHON:-}")"
source "$task_root/tools/lib/pick_ros_env.sh"
task_remote='pick_source_ros >&2
if [ -z "$task_python" ]; then
    task_python=/home/hyper/robot/shared/venvs/s4-jazzy/bin/python
    if [ ! -x "$task_python" ]; then task_python=python3; fi
fi
task_probe=$(mktemp)
trap '\''rm -f -- "$task_probe"'\'' EXIT
cat > "$task_probe"
timeout 60 "$task_python" "$task_probe"'
task_args="$(python3 -c 'import shlex, sys; print(shlex.join(sys.argv[1:]))' "$@")"
if ! ssh -T -o ConnectTimeout=10 -o ServerAliveInterval=10 -o ServerAliveCountMax=2 \
    "$task_host" "set -e
$(declare -f pick_source_ros)
task_python=$task_python_arg
$task_remote $task_args" \
    < "$task_output/inspect.pyz" \
    > "$task_output/jetson.json" 2> "$task_output/ssh.log"; then
    cat "$task_output/ssh.log" >&2
    printf 'Jetson inspection did not complete. Log: %s\n' "$task_output/ssh.log" >&2
    exit 1
fi
python3 "$task_root/tools/diagnostics/inspect_pick_hardware.py" \
    --compare "$task_output/jetson.json" --root "$task_root"
