#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
task_source="${GRASP_SOURCE_REPO:-/home/an-hyeonseo/Documents/GitHub/SO101-Can-Towel-Manipulation}"
task_workspace="$task_source/workspaces/home-assistant"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=198
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$task_root/output/grasp-rviz-logs"
mkdir -p "$ROS_LOG_DIR"
export PYTHONPATH="$task_workspace/.venv-room-reconstruction/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
exec "$task_source/.venv-realsense/bin/python" "$task_root/tools/diagnostics/replay_grasp_rviz.py" \
  --mobile-mesh-dir "$task_workspace/tmp/s2-official/AlohaMini/AlohaMini1/simulation/src/Aloha/meshes" "$@"
