#!/usr/bin/env bash
# Stage a fresh immutable kit from the notebook, then run its device-free checks.
set -euo pipefail
if [[ $# -gt 1 || ( ${1:-} != '' && ${1:-} != --check-hardware ) ]]; then
    echo 'Usage: stage_pick_station.sh [--check-hardware]' >&2
    exit 2
fi
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
kit=$(cd "$repo/output/pick-kit/current" && pwd -P)
host=${JETSON_HOST:-hyper@192.168.35.2}
test -f "$kit/SHA256SUMS"
release=$(ssh -o ConnectTimeout=10 "$host" 'mkdir -p "$HOME/robot/releases" && mktemp -d "$HOME/robot/releases/pick-station-XXXXXXXX"')
printf 'Copying pick station to %s\n' "$release"
rsync -a --partial --info=progress2 --exclude=/output/ --exclude=/.venv/ --exclude=__pycache__/ "$kit/" "$host:$release/"
mkdir -p "$kit/output"
ssh -t "$host" "cd '$release' && bash tools/run/pick_kit.sh preflight"
printf '%s\n' "$release" > "$kit/output/last-jetson-release"
if [[ "${1:-}" == --check-hardware ]]; then
    bash "$kit/tools/run/check_pick_hardware.sh" --root "$release"
fi
