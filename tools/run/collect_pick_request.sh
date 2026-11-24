#!/usr/bin/env bash
# Copy the latest completed web request. No camera, controller or motor access.
set -euo pipefail
task_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
task_host=${JETSON_HOST:-hyper@192.168.35.2}
task_selection=latest
if [[ ${1:-} == --executed && $# == 1 ]]; then task_selection=executed
elif [[ $# != 0 ]]; then echo 'Usage: collect_pick_request.sh [--executed]' >&2; exit 2; fi
task_remote=$(ssh -T -o BatchMode=yes -o ConnectTimeout=8 "$task_host" "python3 - '$task_selection'" <<'PY'
from pathlib import Path
import json,sys
root=Path.home()/'robot/releases'
reports=list(root.glob('pick-station-*/output/pick-station/*/*/result.json'))
for path in sorted(reports,key=lambda p:p.stat().st_mtime,reverse=True):
    if sys.argv[1]=='executed':
        result=json.loads(path.read_text())
        if not any(s.get('id')=='EXECUTING' and s.get('started_s') is not None
                   for s in result.get('stages',[])):
            continue
    print(path.parent.resolve())
    break
else:
    sys.exit('No completed Pick Station request matching the selection found')
PY
)
if [[ ! "$task_remote" =~ ^/home/[^/]+/robot/releases/pick-station-[a-zA-Z0-9]+/output/pick-station/[a-zA-Z0-9-]+/[a-f0-9]{32}$ ]]; then
    echo 'Unexpected Pick Station request path; nothing copied.' >&2
    exit 1
fi
mkdir -p "$task_root/output/pick-request-review"
task_output=$(mktemp -d "$task_root/output/pick-request-review/request-XXXXXXXX")
rsync -a --protect-args --include='*/' --include='*.json' --include='*.jsonl' \
    --include='color.png' --include='depth_aligned.png' --exclude='*' \
    "$task_host:$task_remote/" "$task_output/"
printf '%s\n' "$task_remote" > "$task_output/source-request.txt"
printf 'SAVED_REQUEST: %s\nMOTION_COMMANDS: 0\n' "$task_output"
