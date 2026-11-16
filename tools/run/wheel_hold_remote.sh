#!/bin/sh
# Explicit operator entry points. No automatic motion, firmware install, or
# fallback to the old controller. Server-side commands retain backend ownership.
set -eu
target=${ROBOT_SSH_TARGET:-hyper@192.168.35.2}
root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
identity=${ROBOT_SSH_IDENTITY:-"$HOME/.ssh/id_ed25519_raccoonbot_jetson"}
known_hosts=${ROBOT_KNOWN_HOSTS:-"$root/output/s4-jetson-no-lidar/known_hosts"}
mode=${1:-status}
case "$mode" in
  status|info|help|stop-wheels|plan-hold|hold-here|check|keyboard|web) ;;
  *) echo 'status | info | stop-wheels | plan-hold | hold-here | check | keyboard | web' >&2; exit 2 ;;
esac
set -- -t -o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=2
if test -f "$identity"; then set -- "$@" -i "$identity" -o IdentitiesOnly=yes; fi
if test -f "$known_hosts"; then set -- "$@" -o "UserKnownHostsFile=$known_hosts"; fi
if test -n "${ROBOT_HOST_KEY_ALIAS:-}"; then set -- "$@" -o "HostKeyAlias=$ROBOT_HOST_KEY_ALIAS"; fi
if test "$mode" = web; then
  set -- "$@" -o ExitOnForwardFailure=yes -L 18766:127.0.0.1:18766
fi
exec ssh "$@" "$target" "sh /home/hyper/robot/wheel-hold-control.sh $mode"
