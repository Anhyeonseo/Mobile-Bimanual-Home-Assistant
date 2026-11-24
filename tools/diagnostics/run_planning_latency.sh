#!/usr/bin/env bash
# Standalone offline comparison. Does not launch controllers or contact hardware.
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
task_build="$task_root/output/planning-accel-workspace"
task_source="${GRASP_SOURCE_REPO:-/home/an-hyeonseo/Documents/GitHub/SO101-Can-Towel-Manipulation}"
source /opt/ros/jazzy/setup.bash
# Native experiments are opt-in. Baseline/pipelined need no C++ server/build.
task_native=false
for task_arg in "$@"; do
  if [[ "$task_arg" == batch || "$task_arg" == parallel ]]; then task_native=true; fi
done
if $task_native; then
  colcon --log-base "$task_build/log" build \
  --base-paths "$task_root/ros2_ws/src/so101_planning_accel" \
  --build-base "$task_build/build" --install-base "$task_build/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
  source "$task_build/install/setup.bash"
fi
export ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$task_root/output/planning-latency-ros-logs"
mkdir -p "$ROS_LOG_DIR"
exec /usr/bin/python3 "$task_root/tools/diagnostics/benchmark_planning_latency.py" \
  --prepared "$task_root/output/so101_photo_experiment/fair-comparison-all/set-1/so101/prepared.json" \
  --model-dir "$task_root/output/so101_photo_experiment/set-1/capture-model" \
  --snapshot "$task_root/output/so101_photo_experiment/set-1/rgbd-99" \
  --mobile-mesh-dir "$task_source/workspaces/home-assistant/tmp/s2-official/AlohaMini/AlohaMini1/simulation/src/Aloha/meshes" \
  "$@"
