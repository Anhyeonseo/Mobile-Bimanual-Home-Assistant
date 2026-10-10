#!/usr/bin/env bash
set -euo pipefail

task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
profile="${1:-far}"
if [[ $# -gt 0 ]]; then shift; fi
case "$profile" in
  far) depth_args=(--depth-width 1280 --depth-height 720 --min-depth 0.3 --max-depth 2.0) ;;
  near) depth_args=(--depth-width 640 --depth-height 360 --min-depth 0.2 --max-depth 0.8) ;;
  *) echo "Usage: $0 {far|near} [viewer arguments]" >&2; exit 2 ;;
esac

task_python="${REALSENSE_PYTHON:-$task_root/.venv/bin/python}"
if [[ ! -x "$task_python" ]]; then
  echo "Missing Jetson venv: $task_python (see docs/REALSENSE.md)" >&2
  exit 1
fi
cd "$task_root"
export HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
prompt_args=(--interactive-prompt)
for arg in "$@"; do
  case "$arg" in --prompt|--prompt=*) prompt_args=(); break ;; esac
done
exec "$task_python" tools/diagnostics/realsense_viewer.py \
  --device cuda "${prompt_args[@]}" "${depth_args[@]}" "$@"
