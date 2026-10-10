#!/usr/bin/env bash
# Run on the viewing PC. The SSH session owns both the Jetson server and tunnel.
set -euo pipefail
profile="${1:-near}"
case "$profile" in near|far) ;; *) echo "Usage: $0 {near|far}" >&2; exit 2 ;; esac
jetson_target="${JETSON_TARGET:-hyper@192.168.35.236}"
echo 'Open http://127.0.0.1:18765 — keep this connection running.'
exec ssh -tt -o BatchMode=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=5 \
  -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
  -L 127.0.0.1:18765:127.0.0.1:8765 "$jetson_target" \
  "cd ~/realsense-grounding && exec env HF_HUB_OFFLINE=1 .venv/bin/python tools/diagnostics/realsense_web.py --profile $profile"
