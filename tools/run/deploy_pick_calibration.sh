#!/usr/bin/env bash
# Install calibration into a new Jetson runtime copy. Never starts robot control.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
task_host="${JETSON_HOST:-hyper@192.168.35.2}"
task_archive="$task_root/output/pick-calibration-20261117/pick-calibration-20261117.tar.gz"
test -f "$task_archive"
task_arguments="$(python3 -c 'import shlex, sys; print(shlex.join(sys.argv[1:]))' "$@")"
scp "$task_archive" "$task_host:/tmp/pick-calibration-20261117.tar.gz"
task_remote_script='set -eu
task_stage=$(mktemp -d /home/hyper/robot/shared/pick-calibration-20261117-XXXXXX)
tar -xzf /tmp/pick-calibration-20261117.tar.gz -C "$task_stage"
/home/hyper/robot/shared/venvs/s4-jazzy/bin/python "$task_stage/pick-calibration-20261117/apply.py"'
ssh -T "$task_host" "$task_remote_script $task_arguments"
