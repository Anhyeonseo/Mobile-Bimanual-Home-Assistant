#!/usr/bin/env bash
# Run on the Jetson; the SSH terminal owns recording start/stop.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
task_python="${REALSENSE_PYTHON:-$HOME/robot/shared/venvs/room-scan-bag/bin/python}"
if [[ ! -x "$task_python" ]]; then
  echo "Install requirements/room_scan.txt in a recording venv; set REALSENSE_PYTHON if its path differs." >&2
  exit 1
fi
exec "$task_python" -u "$task_root/tools/diagnostics/record_room_rgbd.py" "$@"
