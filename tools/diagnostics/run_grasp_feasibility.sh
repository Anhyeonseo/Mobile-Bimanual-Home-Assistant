#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
task_source="${GRASP_SOURCE_REPO:-/home/an-hyeonseo/Documents/GitHub/SO101-Can-Towel-Manipulation}"
task_workspace="$task_source/workspaces/home-assistant"
task_python="$task_source/.venv-realsense/bin/python"
task_packages="$task_workspace/.venv-room-reconstruction/lib/python3.12/site-packages"
source /opt/ros/jazzy/setup.bash
# Offline MoveIt only. Isolate discovery from any live robot ROS graph.
export ROS_DOMAIN_ID=197
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$task_root/output/grasp-feasibility-ros-logs"
mkdir -p "$ROS_LOG_DIR"
export PYTHONPATH="$task_packages${PYTHONPATH:+:$PYTHONPATH}"
exec "$task_python" "$task_root/tools/diagnostics/benchmark_grasp_feasibility.py" \
    --artifacts "$task_workspace/artifacts" \
    --mobile-mesh-dir "$task_workspace/tmp/s2-official/AlohaMini/AlohaMini1/simulation/src/Aloha/meshes" "$@"
