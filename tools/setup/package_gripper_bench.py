#!/usr/bin/env python3
"""Package the isolated bench with the same backup-preserving installer as the observer."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ('tools/run/bench_left_gripper.py',
          'ros2_ws/src/so101_arm_bridge/so101_arm_bridge/backend_lease.py',
          'ros2_ws/src/so101_arm_bridge/so101_arm_bridge/gripper_bench_protocol.py',
          'ros2_ws/src/so101_arm_bridge/so101_arm_bridge/serial_port.py')


def package(dest):
    dest.mkdir(parents=True, exist_ok=False)
    files = {name:(ROOT/name).read_bytes() for name in ASSETS}
    files['firmware.hex'] = (ROOT/'build/s3-grip-bench/stm32_g474_single_arm.hex').read_bytes()
    templates = {s:(ROOT/'tools/setup/s2_deployment'/s).read_bytes()
                 for s in ('flash.sh','program.cfg','verify-backup.py')}
    def checksum(data):
        return hashlib.sha256(data).hexdigest()
    hashes = {name:checksum(data) for name,data in {**files,**templates}.items()}
    name = 's3-left-grip-'+checksum(json.dumps(hashes,sort_keys=True).encode())[:12]
    release = dest/name
    remote = '/home/hyper/robot/releases/'+name
    logs = '/home/hyper/robot/shared/diagnostics/s3/bench/'+name+'/flash'
    for template,src in templates.items():
        files[template] = src.replace(b'@RELEASE@',remote.encode()).replace(b'@DIAGNOSTICS@',logs.encode()).replace(
            b'@FIRMWARE_SHA256@',hashes['firmware.hex'].encode())
    for path,data in files.items():
        p=release/path;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
    (release/'SHA256SUMS').write_text(''.join(f'{checksum(data)}  {path}\n' for path,data in files.items()))
    with tarfile.open(dest/(name+'.tar.gz'),'w:gz') as tar:tar.add(release,arcname=name)
    return name


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    print(package(parser.parse_args().output))
