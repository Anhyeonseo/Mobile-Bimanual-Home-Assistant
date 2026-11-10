#!/usr/bin/env python3
"""One content-addressed Jetson release, reusing the backup-preserving installer."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile

ROOT=Path(__file__).resolve().parents[2]


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


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--preview-bundle',type=Path);a=p.parse_args();print(package(a.output,a.preview_bundle))
