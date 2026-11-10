#!/bin/sh
# Explicit operator entry points. No automatic motion, firmware install, or
# fallback to the old controller. Server-side commands retain backend ownership.
set -eu
target=${ROBOT_SSH_TARGET:-hyper@192.168.35.2}
alias=${ROBOT_HOST_KEY_ALIAS:-192.168.35.236}
mode=${1:-status}
case "$mode" in
  status|info|help|stop-wheels|plan-hold|hold-here|check|keyboard|web) ;;
  *) echo 'status | info | stop-wheels | plan-hold | hold-here | check | keyboard | web' >&2; exit 2 ;;
esac
if test "$mode" = web; then
  exec ssh -t -o BatchMode=yes -o ConnectTimeout=5 -o HostKeyAlias="$alias" \
    -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=2 \
    -L 18766:127.0.0.1:18766 "$target" 'sh /home/hyper/robot/wheel-hold-control.sh web'
fi
exec ssh -t -o BatchMode=yes -o ConnectTimeout=5 -o HostKeyAlias="$alias" \
  -o ServerAliveInterval=5 -o ServerAliveCountMax=2 "$target" \
  "sh /home/hyper/robot/wheel-hold-control.sh $mode"
