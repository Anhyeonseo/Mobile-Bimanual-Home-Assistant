#!/usr/bin/env python3
"""Stage a wheel/hold firmware candidate and host tools; never install or run."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[2]


def package(output):
    sources={
        'teleop_mobile_base.py':ROOT/'tools/run/teleop_mobile_base.py',
        'base_teleop_web.py':ROOT/'tools/run/base_teleop_web.py',
        **{'so101_arm_bridge/'+name:ROOT/'ros2_ws/src/so101_arm_bridge/so101_arm_bridge'/name
           for name in ('backend_lease.py','serial_port.py')},
    }
    memory=io.BytesIO()
    with zipfile.ZipFile(memory,'w',zipfile.ZIP_DEFLATED) as archive:
        for name,path in sorted(sources.items()):
            item=zipfile.ZipInfo(name,(2026,11,9,0,0,0));item.compress_type=zipfile.ZIP_DEFLATED
            archive.writestr(item,path.read_bytes())
        archive.writestr(zipfile.ZipInfo('so101_arm_bridge/__init__.py',(2026,11,9,0,0,0)),b'')
        archive.writestr(zipfile.ZipInfo('__main__.py',(2026,11,9,0,0,0)),b'from teleop_mobile_base import main\nraise SystemExit(main())\n')
    files={'wheel-hold.pyz':memory.getvalue(),
        'firmware.hex':(ROOT/'build/wheel-hold-teleop/stm32_g474_single_arm.hex').read_bytes()}
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources.values()}
    for name in ('mobile_base_teleop_app.c','wheel_hold_monitor.c','wheel_hold_setup.c','servo_transport.c','right_servo_bus.c','main.c'):
        path=ROOT/'firmware/stm32_g474_single_arm/Core/Src'/name
        hashes[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    files['build.json']=(json.dumps(dict(source_sha256=hashes,
        build_options=dict(MOBILE_BASE_TELEOP_ONLY=True,MOBILE_BASE_PRESERVE_HOLD=True,
            ACTUATOR_MOBILE_FIRST_ID=7,CMAKE_BUILD_TYPE='Release'),
        firmware='mobile-base-hold-teleop-v7',hardware_tested=False),indent=2)+'\n').encode()
    files['control.sh']=b'''#!/bin/sh
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
port=/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_005100393235511438363730-if02
case "${1:-status}" in
  status) if test -f @DIAGNOSTICS@/installation-status.json; then cat @DIAGNOSTICS@/installation-status.json; else cat release.json; fi ;;
  help) exec python3 wheel-hold.pyz --help ;;
  check) exec python3 wheel-hold.pyz --port "$port" --check-hold ;;
  info) exec python3 wheel-hold.pyz --port "$port" --preserve-hold ;;
  plan-hold) exec python3 wheel-hold.pyz --port "$port" --prepare-hold ;;
  stop-wheels) exec python3 wheel-hold.pyz --port "$port" --stop-wheels ;;
  hold-here) exec python3 wheel-hold.pyz --port "$port" --prepare-hold --execute ;;
  keyboard) exec python3 wheel-hold.pyz --port "$port" --preserve-hold --run ;;
  web) exec python3 wheel-hold.pyz --port "$port" --preserve-hold --web --bind 127.0.0.1 --web-port 18766 ;;
  *) echo 'status | help | info | stop-wheels | plan-hold (read only) | hold-here (torque ON at current pose) | check | keyboard | web' >&2; exit 2 ;;
esac
'''
    # Physical confirmation belongs immediately before installation, not during
    # packaging. This wrapper has no motor commands and never stops a holder.
    files['install.sh']=b'''#!/bin/sh
set -eu
umask 077
case "${1:-}" in
  --load-supported-motor-power-retained) ;;
  --motor-power-off) echo 'Motor power OFF: previous torque/goal continuity is not assumed. Hold setup is required after power ON.' ;;
  *) echo 'Installation resets STM32. Confirm --motor-power-off or --load-supported-motor-power-retained.' >&2; exit 2 ;;
esac
release=@RELEASE@
logs=@DIAGNOSTICS@
tools=/home/hyper/robot-archive/20261029-cleanup/home-assistant-s0-debug-4_vjtnwn/root/usr
test "$(id -u)" = 1000
exec 9<> /run/user/1000/so101-backend.lock
flock -n 9
if fuser /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_005100393235511438363730-if02 >/dev/null 2>&1; then
  echo 'Serial port is in use; holder was NOT stopped.' >&2; exit 2
fi
cd "$release"
sha256sum -c SHA256SUMS
test ! -e "$logs"
mkdir -p "$logs"
sudo -p 'Firmware installation authentication: ' -v
code=0
sudo -n env LD_LIBRARY_PATH="$tools/lib/aarch64-linux-gnu" "$tools/bin/openocd" -s "$tools/share/openocd/scripts" -f "$release/program.cfg" > "$logs/program.log" 2>&1 || code=$?
cat "$logs/program.log"
test "$code" -eq 0 || exit "$code"
sudo -n python3 "$release/verify-backup.py"
echo 'Firmware installed. Keep the load supported and run control.sh check before enabling wheels.'
'''
    program=(ROOT/'tools/setup/s2_deployment/program.cfg').read_text()
    program=program.replace('flash write_image erase',
        'if {[file size @DIAGNOSTICS@/preflash-full.bin] != 524288} {error "Incomplete firmware backup"}\nflash write_image erase')
    files['program.cfg']=program.encode()
    files['verify-backup.py']=(ROOT/'tools/setup/s2_deployment/verify-backup.py').read_bytes()
    files['README.md']=(ROOT/'docs/WHEEL_HOLD_TELEOP.md').read_bytes()
    checksum=lambda data:hashlib.sha256(data).hexdigest()
    identity=checksum(json.dumps({k:checksum(v) for k,v in sorted(files.items())},sort_keys=True).encode())[:12]
    name='wheel-hold-'+identity
    remote='/home/hyper/robot/releases/'+name
    diagnostics='/home/hyper/robot/shared/diagnostics/wheel-hold/'+name+'/install'
    files={k:v.replace(b'@RELEASE@',remote.encode()).replace(b'@DIAGNOSTICS@',diagnostics.encode())
           if k.endswith(('.sh','.cfg','.py')) else v for k,v in files.items()}
    files['release.json']=(json.dumps(dict(release=name,firmware='mobile-base-hold-teleop-v7',
        state='PREPARED_NOT_INSTALLED',hardware_tested=False,required_on_axes=13,
        teleop_motor_write_ids=[7,8,9],teleop_arm_gripper_lift_write_commands=0,
        explicit_stationary_setup='Preserve ON arm goals; anchor OFF arms; lift current-angle position mode with verified EEPROM relock',
        maintain_existing_goals=True,stop_and_disconnect_leave_existing_arm_lift_goals=True,
        require_read_only_hold_check_after_install=True,
        files_sha256={k:checksum(v) for k,v in sorted(files.items())}),indent=2)+'\n').encode()
    files['SHA256SUMS']=''.join(checksum(v)+'  '+k+'\n' for k,v in sorted(files.items())).encode()
    folder=output/name;folder.mkdir(parents=True,exist_ok=False)
    for key,value in files.items():(folder/key).write_bytes(value)
    return folder


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    print(package(parser.parse_args().output))
