#!/usr/bin/env bash
# Reuse the exact Python/library combination from the saved GraspNet experiment.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
task_source="${GRASP_SOURCE_REPO:-/home/an-hyeonseo/Documents/GitHub/SO101-Can-Towel-Manipulation}"
task_workspace="$task_source/workspaces/home-assistant"
task_python="$task_source/.venv-realsense/bin/python"
task_packages="$task_workspace/.venv-room-reconstruction/lib/python3.12/site-packages"
if [[ ! -x "$task_python" || ! -d "$task_packages" ]]; then
    echo 'Previous GraspNet environment is missing; set GRASP_SOURCE_REPO to its repository.' >&2
    exit 2
fi
export PYTHONPATH="$task_packages${PYTHONPATH:+:$PYTHONPATH}"
exec "$task_python" "$task_root/tools/diagnostics/benchmark_grasp_backends.py" \
    --artifacts "$task_workspace/artifacts" "$@"
