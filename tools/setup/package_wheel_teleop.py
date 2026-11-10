#!/usr/bin/env python3
"""Package the existing wheel-only controller, without flashing or running it."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[2]


def package(output):
    files = {
        'teleop_mobile_base.py': (ROOT / 'tools/run/teleop_mobile_base.py').read_bytes(),
        'base_teleop_web.py': (ROOT / 'tools/run/base_teleop_web.py').read_bytes(),
        'so101_arm_bridge/__init__.py': b'',
    }
    for name in ('backend_lease.py', 'serial_port.py'):
        files['so101_arm_bridge/' + name] = (ROOT / 'ros2_ws/src/so101_arm_bridge/so101_arm_bridge' / name).read_bytes()
    files['__main__.py'] = b'from teleop_mobile_base import main\nraise SystemExit(main())\n'
    digest = hashlib.sha256(b''.join(k.encode()+v for k,v in sorted(files.items()))).hexdigest()[:12]
    destination = output / ('wheel-teleop-' + digest)
    destination.mkdir(parents=True, exist_ok=False)
    with zipfile.ZipFile(destination/'wheel-teleop.pyz', 'w', zipfile.ZIP_DEFLATED) as archive:
        for name, data in files.items():
            archive.writestr(name, data)
    status = dict(kind='wheel_only_teleop_candidate', required_firmware='mobile-base-teleop-v4',
        wheel_ids=[7,8,9], excluded=['arms','grippers','lift'],
        prepared_only=True, installed_firmware_verified=False, motion_tested=False,
        arm_hold_verified=False, controller_replaced=False,
        limitation='Current stationary-pick release is not this wheel-only firmware. Do not switch while carrying an object.',
        keys=dict(w='forward',s='backward',a='left',d='right',q='rotate-left',e='rotate-right',space='stop',x='exit'),
        source_sha256={name:hashlib.sha256(data).hexdigest() for name,data in sorted(files.items())})
    (destination/'status.json').write_text(json.dumps(status,ensure_ascii=False,indent=2)+'\n')
    (destination/'wheel-control.sh').write_text('''#!/bin/sh
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
case "${1:-status}" in
  status) cat status.json ;;
  help) python3 wheel-teleop.pyz --help ;;
  *)
    echo '현재 팔 제어기와 바퀴 전용 펌웨어의 호환 확인이 남았습니다.' >&2
    echo '이 준비 도구는 현재 팔·집게 유지 상태를 바꾸거나 모터를 활성화하지 않습니다.' >&2
    exit 2 ;;
esac
''')
    (destination/'README.txt').write_text('''바퀴 전용 텔레옵 준비 묶음

기존 teleop_mobile_base.py / base_teleop_web.py를 그대로 묶었습니다.
W/S 전후, A/D 좌우, Q/E 회전, Space 정지, X/Esc 종료.
U/J 리프트 조작은 없으며 팔·집게 목표도 보내지 않습니다.
웹은 기본 127.0.0.1 바인딩이며 SSH 포트 전달로 사용할 수 있습니다.

현재 젯슨의 stationary-pick 배포는 별도 펌웨어입니다.
이번 준비에서는 포트 조회, 구동, 리셋, 펌웨어 설치를 하지 않았습니다.
현재 리모컨 파지를 유지한 채 사용할 수 있다고 검증한 상태가 아닙니다.
팔에 명령을 안 보내는 것과 팔 토크를 지속적으로 보장하는 것은 다릅니다.
기존 팔 제어기의 베이스 제어 연결 또는 검증된 제어기 전환이 남았습니다.
실행 진입점은 이 확인 전까지 status/help만 제공합니다.
''')
    (destination/'SHA256SUMS').write_text(''.join(
        hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n'
        for p in sorted(destination.iterdir()) if p.is_file()))
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(package(args.output))
