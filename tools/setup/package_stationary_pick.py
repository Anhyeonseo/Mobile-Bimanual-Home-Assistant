#!/usr/bin/env python3
"""One content-addressed Jetson release, reusing the backup-preserving installer."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile
import tempfile

ROOT=Path(__file__).resolve().parents[2]


def package_s4_inspection(destination):
    """Reuse the S2 backup installer for the isolated, read-only mobile image."""
    build=ROOT/'build/s1-lift-measurement'
    cache=dict(line.split('=',1) for line in (build/'CMakeCache.txt').read_text().splitlines()
               if '=' in line and not line.startswith(('#','//')))
    if (cache.get('MOBILE_BUS_INSPECTION_ONLY:BOOL')!='ON'
            or cache.get('ACTUATOR_MOBILE_FIRST_ID:STRING')!='7'
            or cache.get('MOBILE_BOARD_PROFILE_HEADER:FILEPATH')!=''):
        raise ValueError('explicit read-only inspection build required')
    if any(value=='ON' and key!='MOBILE_BUS_INSPECTION_ONLY:BOOL'
           and key.endswith(('_ONLY:BOOL','_CANDIDATE:BOOL')) for key,value in cache.items()):
        raise ValueError('inspection package cannot contain another controller variant')
    firmware=build/'stm32_g474_single_arm.hex'
    sources=[p for p in (ROOT/'firmware/stm32_g474_single_arm/Core').rglob('*') if p.suffix in ('.c','.h')]
    if any(p.stat().st_mtime>firmware.stat().st_mtime for p in sources):
        raise ValueError('rebuild inspection firmware after source changes')
    paths=['tools/run/probe_mobile_bus.py','tools/diagnostics/analyze_wheel_capture.py']
    for package_name in ('home_robot_tasks','so101_arm_bridge'):
        paths += [str(p.relative_to(ROOT)) for p in (ROOT/'ros2_ws/src'/package_name/package_name).glob('*.py')]
    files={p:(ROOT/p).read_bytes() for p in paths}
    files['firmware.hex']=firmware.read_bytes()
    files['build.json']=(json.dumps(dict(preset='home-assistant-s1-lift-measurement',
        motor_writes_permitted=False,source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}),indent=2)+'\n').encode()
    templates={n:(ROOT/'tools/setup/s2_deployment'/n).read_bytes() for n in ('flash.sh','program.cfg','verify-backup.py')}
    templates['flash.sh']=templates['flash.sh'].replace(
        b'test "${1:-}" = --motor-power-off-confirmed || exit 2',
        b'case "${1:-}" in\n  --motor-power-off-confirmed|--load-supported-motor-power-retained) ;;\n  *) echo "Confirm supported arms and lowest/secured lift before controller reset." >&2; exit 2 ;;\nesac')
    templates['flash.sh']=templates['flash.sh'].replace(b'mkdir -p "$logs"\n',b'').replace(
        b"sudo -p 'S2 authentication: ' -v",
        b"sudo -p 'S4 inspection authentication: ' -v\nmkdir -p \"$logs\"")
    templates['program.cfg']=templates['program.cfg'].replace(b'flash write_image erase',
        b'if {[file size @DIAGNOSTICS@/preflash-full.bin] != 524288} {error "Incomplete firmware backup"}\nflash write_image erase')
    checksum=lambda raw:hashlib.sha256(raw).hexdigest()
    name='s4-inspection-'+checksum(json.dumps({k:checksum(v) for k,v in sorted({**files,**templates}.items())},sort_keys=True).encode())[:12]
    remote='/home/hyper/robot/releases/'+name
    logs='/home/hyper/robot/shared/diagnostics/s4-inspection/'+name
    for key,raw in templates.items():
        files[key]=raw.replace(b'@RELEASE@',remote.encode()).replace(b'@DIAGNOSTICS@',logs.encode()).replace(b'@FIRMWARE_SHA256@',checksum(files['firmware.hex']).encode())
    files['release.json']=(json.dumps(dict(release=name,firmware='mobile-bus-inspection-v2',
        wheel_samples_supported=True,device_inventory_supported=True,motor_writes_permitted=False,flashed=False,
        requires_controller_reset=True,requires_supported_mechanism=True,
        replaces_active_controller_after_explicit_install=True,
        not_navigation_firmware=True,files_sha256={k:checksum(v) for k,v in sorted(files.items())}),indent=2)+'\n').encode()
    files['SHA256SUMS']=''.join(checksum(v)+'  '+k+'\n' for k,v in sorted(files.items())).encode()
    destination.mkdir(parents=True,exist_ok=False);folder=destination/name
    for name_,raw in files.items():
        path=folder/name_;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw)
    with tarfile.open(destination/(name+'.tar.gz'),'w:gz') as archive:archive.add(folder,arcname=name)
    return folder


def package(destination,preview=None):
    destination.mkdir(parents=True,exist_ok=False)
    paths=[ROOT/'tools/run/stationary_pick.py']
    for package_name in ('home_robot_tasks','so101_arm_bridge'):
        paths.extend((ROOT/'ros2_ws/src'/package_name/package_name).rglob('*.py'))
    paths.extend((ROOT/'config/home_assistant').glob('*.json'))
    paths.extend((ROOT/'config/home_assistant/home_calibration').rglob('*.json'))
    # Keep the installed coordinate profile independently verifiable, including
    # pinned geometry sources and the control-model axis reference.
    paths.extend(p for p in (ROOT/'config/home_assistant/official_sources').rglob('*') if p.is_file())
    profile=json.loads((ROOT/'config/home_assistant/s2_coordinates.development.json').read_bytes())
    paths.extend(ROOT/name for name in profile['source_files_sha256'])
    files={str(p.relative_to(ROOT)):p.read_bytes() for p in paths}
    files['firmware.hex']=(ROOT/'build/s3-stationary-pick/stm32_g474_single_arm.hex').read_bytes()
    if preview:
        for p in preview.iterdir():
            if p.is_file():files['preview-bundle/'+p.name]=p.read_bytes()
    templates={name:(ROOT/'tools/setup/s2_deployment'/name).read_bytes() for name in ('flash.sh','program.cfg','verify-backup.py')}
    templates['control.sh']=b'''#!/bin/sh
set -eu
release=@RELEASE@
python=/home/hyper/robot/shared/venvs/room-scan-bag/bin/python
logs=/home/hyper/robot/shared/diagnostics/s3/stationary
mkdir -p "$logs"
cd "$release"
exec "$python" tools/run/stationary_pick.py "$@" --output "$logs/$(date -u +%Y%m%dT%H%M%SZ)-$$.json"
'''
    checksum=lambda data:hashlib.sha256(data).hexdigest()
    fingerprint={k:checksum(v) for k,v in sorted({**files,**templates}.items())}
    name='s3-stationary-'+checksum(json.dumps(fingerprint,sort_keys=True).encode())[:12]
    remote='/home/hyper/robot/releases/'+name
    logs='/home/hyper/robot/shared/diagnostics/s3/stationary/'+name+'/flash'
    for key,data in templates.items():files[key]=data.replace(b'@RELEASE@',remote.encode()).replace(b'@DIAGNOSTICS@',logs.encode()).replace(b'@FIRMWARE_SHA256@',fingerprint['firmware.hex'].encode())
    files['release.json']=(json.dumps(dict(release=name,firmware_version='0x00024911',motor_motion_tested=False,
        python_environment='shared/venvs/room-scan-bag',files_sha256={k:checksum(v) for k,v in sorted(files.items())}),indent=2)+'\n').encode()
    folder=destination/name
    for key,data in files.items():
        p=folder/key;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
    (folder/'SHA256SUMS').write_text(''.join(f'{checksum(v)}  {k}\n' for k,v in sorted(files.items())))
    with tarfile.open(destination/(name+'.tar.gz'),'w:gz') as archive:archive.add(folder,arcname=name)
    return folder


def package_s4(destination, *, host_only=False):
    """Portable source release for later deployment; no compiled PC libraries.

    Shares the existing content-addressed packaging entry point. S4 deliberately
    contains no active service, automatic SSH, firmware flash or controller reset.
    """
    destination=Path(destination)
    if destination.exists():raise ValueError('new release output directory required')
    roots=('ros2_ws/src','tools','home_perception','protocol','hardware','config','requirements','tests')
    allowed={'.py','.json','.yaml','.yml','.xml','.urdf','.xacro','.srdf','.stl','.STL',
             '.rviz','.ini','.msg','.srv','.action','.c','.h','.cpp','.hpp','.html','.css','.js','.txt','.cfg','.sh','.md'}
    paths=[]
    for root in roots:
        paths.extend(p for p in (ROOT/root).rglob('*') if p.is_file()
            and '.local.' not in p.name and not p.name.startswith('.env')
            and not any(part in ('__pycache__','.pytest_cache','node_modules','.git','build','install','log') for part in p.relative_to(ROOT/root).parts)
            and (p.suffix in allowed or p.name in ('CMakeLists.txt','setup.cfg','LICENSE') or p.parent.name=='resource'))
    paths.extend(p for p in (ROOT/'docs').glob('*.md'))
    paths.extend(p for name in ('LICENSE','README.md') if (p:=ROOT/name).is_file())
    # Retain the established controller sources for a later measured-profile
    # build, but do not present an uncommissioned binary as installed firmware.
    for root in ('firmware/stm32_actuator','firmware/stm32_g474_single_arm'):
        if host_only:
            continue
        paths.extend(p for p in (ROOT/root).rglob('*') if p.is_file()
            and '.local.' not in p.name and not p.name.startswith('.env')
            and not any(part in ('build','__pycache__','.git') for part in p.relative_to(ROOT/root).parts)
            and (p.suffix in {'.c','.h','.s','.S','.ld','.cmake','.json','.py','.md','.txt','.ioc'} or p.name=='CMakeLists.txt'))
    files={str(p.relative_to(ROOT)):p.read_bytes() for p in paths
           if not p.is_symlink() and p.resolve().is_relative_to(ROOT.resolve())}
    checksum=lambda value:hashlib.sha256(value).hexdigest()
    fingerprints={name:checksum(raw) for name,raw in sorted(files.items())}
    name='s4-local-'+checksum(json.dumps(fingerprints,sort_keys=True).encode())[:12]
    manifest=dict(schema_version=1,release=name,stage='S4',status='local_candidate_not_commissioned',
        hardware_enabled=False,physical_task_completed=False,jetson_deployed=False,
        target_runtime='build from source on target after OS/ROS/dependency verification',
        no_firmware_binary=True,files_sha256=fingerprints,
        controller_sources_included=not host_only,
        commissioning_profile='config/home_assistant/s4_commissioning.pending.json',
        instructions='docs/S4_LOCAL_PREPARATION.md')
    files['release.json']=(json.dumps(manifest,indent=2)+'\n').encode()
    files['SHA256SUMS']=''.join(f'{checksum(raw)}  {key}\n' for key,raw in sorted(files.items())).encode()
    destination.mkdir(parents=True)
    folder=destination/name
    for key,raw in files.items():
        target=folder/key;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(raw)
    with tarfile.open(destination/(name+'.tar.gz'),'w:gz') as archive:
        archive.add(folder,arcname=name)
    return folder

def package_unified(destination, profile):
    """Reuse host packaging and the full-flash/journal-preserving installer."""
    destination=Path(destination);profile=Path(profile).resolve()
    if destination.exists():raise ValueError('new package destination required')
    build=ROOT/'build/s4-unified-controller'
    cache=dict(line.split('=',1) for line in (build/'CMakeCache.txt').read_text().splitlines()
               if '=' in line and not line.startswith(('#','//')))
    if (cache.get('S4_UNIFIED_CANDIDATE:BOOL')!='ON' or cache.get('S3_STATIONARY_PICK_CANDIDATE:BOOL')!='ON'
        or cache.get('ACTUATOR_MOBILE_FIRST_ID:STRING')!='7'
        or Path(cache.get('MOBILE_BOARD_PROFILE_HEADER:FILEPATH','')).resolve()!=profile/'mobile_profile.h'):
        raise ValueError('exact S4 unified build/profile required')
    checksum=lambda b:hashlib.sha256(b).hexdigest()
    manifest=json.loads((profile/'manifest.json').read_text())
    if manifest['mode']!='commissioning':raise ValueError('explicit commissioning profile required')
    for name,digest in manifest['files'].items():
        if checksum((profile/name).read_bytes())!=digest:raise ValueError('profile hash mismatch')
    firmware_inputs=[p for base in ('firmware/stm32_actuator','firmware/stm32_g474_single_arm')
        for p in (ROOT/base).rglob('*') if p.is_file() and p.suffix in ('.c','.h','.s','.S','.txt','.ld','.cmake')]
    firmware_inputs.extend((profile/'mobile_profile.h',build/'CMakeCache.txt'))
    binary=build/'stm32_g474_single_arm.hex'
    if any(p.stat().st_mtime_ns>binary.stat().st_mtime_ns for p in firmware_inputs):
        raise ValueError('rebuild unified firmware after source/profile/config changes')
    with tempfile.TemporaryDirectory(prefix='s4-unified-') as temp:
        source=package_s4(Path(temp)/'source',host_only=True)
        files={str(p.relative_to(source)):p.read_bytes() for p in source.rglob('*') if p.is_file()
               and p.name not in ('release.json','SHA256SUMS')}
    files['firmware.hex']=binary.read_bytes()
    for name in ('mobile_profile.h','mobile_profile.json','manifest.json'):
        files['controller-profile/'+name]=(profile/name).read_bytes()
    for name in ('program.cfg','verify-backup.py'):
        files[name]=(ROOT/'tools/setup/s2_deployment'/name).read_bytes()
    files['program.cfg']=files['program.cfg'].replace(b'flash write_image erase',
        b'if {[file size @DIAGNOSTICS@/preflash-full.bin] != 524288} {error "Incomplete firmware backup"}\nflash write_image erase')
    files['install.sh']=b'''#!/bin/sh
set -eu
umask 077
case "${1:-}" in
  --motor-power-off-confirmed|--load-supported-motor-power-retained) ;;
  *) exit 2 ;;
esac
release=@RELEASE@
logs=@DIAGNOSTICS@
tools=/home/hyper/robot-archive/20261029-cleanup/home-assistant-s0-debug-4_vjtnwn/root/usr
test "$(id -u)" = 1000
exec 9<> /run/user/1000/so101-backend.lock
flock -n 9
if fuser /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_005100393235511438363730-if02 >/dev/null 2>&1; then
  echo 'Serial port is in use; owner was not stopped.' >&2; exit 2
fi
cd "$release"
sha256sum -c SHA256SUMS > /dev/null
if [ "${2:-}" = --authenticate-stdin ]; then
  sudo -S -p '' -v
else
  test -z "${2:-}"
  sudo -n true
fi
test ! -e "$logs"
mkdir -p "$logs"
code=0
sudo -n env LD_LIBRARY_PATH="$tools/lib/aarch64-linux-gnu" "$tools/bin/openocd" -s "$tools/share/openocd/scripts" -f "$release/program.cfg" > "$logs/program.log" 2>&1 || code=$?
cat "$logs/program.log"
test "$code" -eq 0 || exit "$code"
sudo -n python3 "$release/verify-backup.py"
sudo -n chown "$(id -u):$(id -g)" "$logs/preflash-full.bin" "$logs/boot-journal-after.bin" "$logs/backup-verification.json"
echo 'Installed with no automatic preparation, torque enable or homing.'
'''
    files['control.sh']=b'''#!/bin/sh
set -eu
cd @RELEASE@
exec /home/hyper/robot/shared/venvs/room-scan-bag/bin/python tools/run/prepare_unified_controller.py --port /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_005100393235511438363730-if02 "$@"
'''
    firmware_sources={str(p.relative_to(ROOT)):checksum(p.read_bytes())
        for base in ('firmware/stm32_actuator','firmware/stm32_g474_single_arm')
        for p in (ROOT/base).rglob('*') if p.is_file() and p.suffix in ('.c','.h','.txt','.ld','.cmake')}
    files['build.json']=(json.dumps(dict(firmware='0x0002492D',source_sha256=firmware_sources,
        profile_sha256=manifest['profile_sha256'],physical_tested=False,automatic_setup=False),indent=2)+'\n').encode()
    name='s4-unified-'+checksum(json.dumps({n:checksum(b) for n,b in sorted(files.items())},sort_keys=True).encode())[:12]
    remote='/home/hyper/robot/releases/'+name
    logs='/home/hyper/robot/shared/diagnostics/s4-unified/'+name+'/install'
    for name_in in ('install.sh','control.sh','program.cfg','verify-backup.py'):
        files[name_in]=files[name_in].replace(b'@RELEASE@',remote.encode()).replace(b'@DIAGNOSTICS@',logs.encode())
    files['release.json']=(json.dumps(dict(release=name,firmware='0x0002492D',hardware_authorized=False,
        physical_test_ready=False,automatic_setup=False,boot_motor_writes=0,
        files_sha256={n:checksum(b) for n,b in sorted(files.items())}),indent=2)+'\n').encode()
    files['SHA256SUMS']=''.join(checksum(b)+'  '+n+'\n' for n,b in sorted(files.items())).encode()
    folder=destination/name
    for n,b in files.items():
        p=folder/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b)
    with tarfile.open(destination/(name+'.tar.gz'),'w:gz') as archive:archive.add(folder,arcname=name)
    return folder

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--preview-bundle',type=Path)
    p.add_argument('--stage',choices=('s3','s4','s4-inspection','s4-unified'),default='s3')
    p.add_argument('--controller-profile',type=Path)
    p.add_argument('--host-only',action='store_true',help='S4 source bundle without controller firmware sources')
    a=p.parse_args()
    if a.host_only and a.stage!='s4':p.error('--host-only requires --stage s4')
    if a.stage!='s3' and a.preview_bundle:p.error('S4 package does not take a stationary preview')
    if (a.stage=='s4-unified')!=bool(a.controller_profile):p.error('--controller-profile is required only for s4-unified')
    if a.stage=='s4-unified':print(package_unified(a.output,a.controller_profile))
    elif a.stage=='s4-inspection':print(package_s4_inspection(a.output))
    else:print(package_s4(a.output,host_only=a.host_only) if a.stage=='s4' else package(a.output,a.preview_bundle))
