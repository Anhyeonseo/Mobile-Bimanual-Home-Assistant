#!/bin/sh
set -eu
umask 077
test "${1:-}" = --motor-power-off-confirmed || exit 2
release=@RELEASE@
logs=@DIAGNOSTICS@
tools=/home/hyper/robot-archive/20261029-cleanup/home-assistant-s0-debug-4_vjtnwn/root/usr
# Run this wrapper as hyper: keep the backend lock owned by the regular user.
test "$(id -u)" = 1000
exec 9<> /run/user/1000/so101-backend.lock
flock -n 9
if fuser /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_005100393235511438363730-if02 >/dev/null 2>&1; then exit 2; fi
printf '%s  %s\n' '@FIRMWARE_SHA256@' "$release/firmware.hex" | sha256sum -c -
test ! -e "$logs"
mkdir -p "$logs"
# Authenticate before redirecting the OpenOCD output so the prompt stays visible.
sudo -p 'S2 authentication: ' -v
code=0
sudo -n env LD_LIBRARY_PATH="$tools/lib/aarch64-linux-gnu" "$tools/bin/openocd" -s "$tools/share/openocd/scripts" -f "$release/program.cfg" > "$logs/program.log" 2>&1 || code=$?
cat "$logs/program.log"
if [ "$code" -ne 0 ]; then exit "$code"; fi
sudo -n python3 "$release/verify-backup.py"
