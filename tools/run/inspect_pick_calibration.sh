#!/usr/bin/env bash
# Download source files for comparison only. Does not deploy or access devices.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
task_host="${JETSON_HOST:-hyper@192.168.35.2}"
task_parent="$task_root/output/pick-calibration-20261117"
mkdir -p "$task_parent"
task_capture="$(mktemp -d "$task_parent/jetson-inspection-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"
ssh -T "$task_host" 'python3 -' < "$task_root/tools/diagnostics/inspect_pick_calibration.py" > "$task_capture/snapshot.json"
python3 "$task_root/tools/diagnostics/inspect_pick_calibration.py" \
  --compare "$task_capture/snapshot.json" \
  --package "$task_parent/pick-calibration-20261117" --output "$task_capture/review"
