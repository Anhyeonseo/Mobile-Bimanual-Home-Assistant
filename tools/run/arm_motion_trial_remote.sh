#!/usr/bin/env bash
# No arguments: local offline preview. --execute: interactive supervised trial.
# Reuse an existing runtime without modifying its files or stopping its owner.
set -euo pipefail
trial_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
trial_remote=false
for trial_arg in "$@"; do
    if [[ "$trial_arg" == --execute || "$trial_arg" == --check-runtime ]]; then trial_remote=true; fi
done
if [[ "$trial_remote" == false ]]; then
    exec python3 "$trial_repo/tools/run/arm_motion_trial.py" "$@"
fi
trial_host=${JETSON_HOST:-hyper@192.168.35.2}
trial_runtime=${JETSON_PICK_ROOT:-/home/hyper/robot/releases/pick-station-GuTuEVP6}
trial_python=${JETSON_PICK_PYTHON:-/home/hyper/realsense-grounding/.venv/bin/python}
trial_dir=$(ssh -o ConnectTimeout=8 "$trial_host" \
    'mkdir -p /home/hyper/robot/diagnostics && mktemp -d /home/hyper/robot/diagnostics/arm-motion-XXXXXXXX')
if [[ ! "$trial_dir" =~ ^/home/hyper/robot/diagnostics/arm-motion-[a-zA-Z0-9]+$ ]]; then
    echo 'Unexpected remote diagnostic directory.' >&2
    exit 2
fi
scp -q "$trial_repo/tools/run/arm_motion_trial.py" "$trial_host:$trial_dir/arm_motion_trial.py"
trial_command=$(python3 - "$trial_runtime" "$trial_python" "$trial_dir" "$@" <<'PY'
import shlex, sys
from pathlib import PurePosixPath
root, python, folder, *args = sys.argv[1:]
if not all(PurePosixPath(p).is_absolute() for p in (root, python, folder)):
    raise SystemExit('Absolute runtime/python/output paths required')
script = '\n'.join([
    'set -eo pipefail',
    'cd '+shlex.quote(root),
    'source tools/lib/pick_ros_env.sh',
    'pick_source_ros',
    'export PYTHONPATH="/home/hyper/realsense-grounding/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}:/home/hyper/robot/shared/venvs/s4-jazzy/lib/python3.12/site-packages"',
    'export ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1 HF_HUB_OFFLINE=1',
    shlex.join(['export', 'ROS_LOG_DIR='+folder+'/ros']),
    'exec '+shlex.join([python, folder+'/arm_motion_trial.py', '--runtime-root',root,
                       '--output',folder+'/result',*args])])
print('bash -lc '+shlex.quote(script))
PY
)
printf '시험 기록 폴더: %s/result\n' "$trial_dir"
exec ssh -t -o ConnectTimeout=8 -o ServerAliveInterval=10 -o ServerAliveCountMax=2 \
    "$trial_host" "$trial_command"
