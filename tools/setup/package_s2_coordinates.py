#!/usr/bin/env python3
"""Create one content-addressed Jetson release; never flash or connect devices."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tarfile
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks import s2_development as dev, s2_commissioning as s2


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    profile=dev.build(ROOT);geometry=s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
    files={}
    for name in ['s2_development','s2_commissioning','s2_acquisition','s2_rgbd_acquisition','camera_registration','camera_evidence']:
        path=f'ros2_ws/src/home_robot_tasks/home_robot_tasks/{name}.py';files[path]=(ROOT/path).read_bytes()
    for name in ['backend_lease','serial_port']:
        path=f'ros2_ws/src/so101_arm_bridge/so101_arm_bridge/{name}.py';files[path]=(ROOT/path).read_bytes()
    for name in ['s2_development','capture_s2_observer']:
        path=f'tools/diagnostics/{name}.py';files[path]=(ROOT/path).read_bytes()
    files['config/home_assistant/official_geometry.json']=(ROOT/'config/home_assistant/official_geometry.json').read_bytes()
    files['config/home_assistant/camera_device.observed.json']=(ROOT/'config/home_assistant/camera_device.observed.json').read_bytes()
    for source in geometry['sources']:
        path='config/home_assistant/official_sources/'+source['file'];files[path]=(ROOT/path).read_bytes()
    files['profile.json']=(json.dumps(profile,indent=2,ensure_ascii=False)+'\n').encode()
    state={'raw':{side:profile['arms'][side]['reference_raw'] for side in s2.ARMS},'lift_height_m':0.,'camera_xyz_m':[0.,0.,.6],
           'note':'offline reference-pose example only; camera point and lift are illustrative'}
    files['example-state.json']=(json.dumps(state,indent=2)+'\n').encode()
    files['coordinates.urdf']=dev.export_urdf(profile,geometry).encode()
    files['firmware.hex']=(ROOT/'build/s2-observer/stm32_g474_single_arm.hex').read_bytes()
    manifest={name:hashlib.sha256(data).hexdigest() for name,data in files.items()}
    release='s2-coordinates-'+s2.digest(manifest)[:12]
    remote='/home/hyper/robot/releases/'+release
    logs='/home/hyper/robot/shared/diagnostics/s2/observer/'+release
    directory=a.output/release;directory.mkdir()
    for name,data in files.items():
        out=directory/name;out.parent.mkdir(parents=True,exist_ok=True);out.write_bytes(data)
    # Reuse the previously reviewed full-backup/boot-journal-preserving flash flow.
    template=ROOT/'tools/setup/s2_deployment'
    replacements={'@RELEASE@':remote,'@DIAGNOSTICS@':logs,'@FIRMWARE_SHA256@':manifest['firmware.hex']}
    for name in ['flash.sh','program.cfg','verify-backup.py']:
        text=(template/name).read_text()
        for old,new in replacements.items():text=text.replace(old,new)
        (directory/name).write_text(text)
    for name in ['flash.sh','program.cfg','verify-backup.py']:
        manifest[name]=hashlib.sha256((directory/name).read_bytes()).hexdigest()
    metadata={'release':release,'remote_directory':remote,'diagnostics_directory':logs,'files_sha256':manifest,
              'firmware_version':'s2-observer-v1','motor_writes_permitted':False,'flashed':False,
              'profile_id':profile['calibration_id'],'live_acquisition_verified':False}
    (directory/'manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
    (a.output/'release.json').write_text(json.dumps(metadata,indent=2)+'\n')
    with tarfile.open(a.output/(release+'.tar.gz'),'w:gz') as tar:tar.add(directory,arcname=release)
    print(json.dumps(metadata,indent=2))

if __name__=='__main__':main()
