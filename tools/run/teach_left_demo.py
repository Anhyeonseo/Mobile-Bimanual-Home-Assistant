#!/usr/bin/env python3
"""Teach only the five filming poses using the installed no-motion maintenance firmware."""
import argparse
import copy
import json
from pathlib import Path
import sys
import time
from calibrate_left_home import Link,save
from pose_locked_profile import STAGES,LABELS,new_record,validate_record,capture,compile_profile

def load(path):
    def unique(pairs):
        out={}
        for k,v in pairs:
            if k in out:raise ValueError('중복 JSON 항목: '+k)
            out[k]=v
        return out
    return json.loads(path.read_text(),object_pairs_hook=unique)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--file',type=Path,default=Path('output/left-five-poses.json'))
    p.add_argument('--calibration-record',type=Path)
    p.add_argument('--port',default='/dev/ttyACM0')
    p.add_argument('--check',action='store_true',help='검사만; 장치에 연결하지 않음')
    args=p.parse_args()
    if args.check:
        profile=compile_profile(load(args.file));print(json.dumps(profile,ensure_ascii=False,indent=2));return
    if not sys.stdin.isatty():raise ValueError('직접 입력 가능한 젯슨 터미널에서 실행하세요.')
    root=Path(__file__).resolve().parents[2]
    calibration_path=args.calibration_record or Path(load(root/'current-centering.json')['receipt_path'])
    expected=new_record(load(calibration_path))
    record=load(args.file) if args.file.exists() else expected
    validate_record(record,expected)
    args.file.parent.mkdir(parents=True,exist_ok=True)
    # Detect an unwritable destination before the operator begins teaching.
    save(args.file,record)
    import serial
    from so101_arm_bridge.backend_lease import acquire_backend_lease
    from so101_arm_bridge.serial_port import open_exclusive_serial
    print('왼팔 촬영 자세 저장: 모터는 힘이 풀린 상태여야 합니다. 팔과 물체를 받쳐주세요.')
    print('영점 보정 때 집게 중간과 달리, 여기의 home은 집게를 연 촬영 시작 자세입니다.')
    print(' | '.join(s+': '+label for s,label in zip(STAGES,LABELS)))
    with acquire_backend_lease('stm32',0),open_exclusive_serial(serial,args.port,921600,.1) as port:
        port.reset_input_buffer();link=Link(port)
        info=link.exchange('INFO')
        if info.get('firmware')!='left-calibration-v1' or info.get('motion_enabled') is not False:
            raise ValueError('왼팔 보정 전용 펌웨어가 아닙니다.')
        capture(link,record,sleep=time.sleep)
        while True:
            try:command=input('home / approach / grasp / closed / lift / show / done / q > ').strip().lower()
            except EOFError:break
            if command=='q':break
            if command=='show':
                print('저장됨: '+', '.join(s for s in STAGES if s in record['poses']));continue
            if command=='done':
                try:profile=compile_profile(record)
                except ValueError as e:print('확인 필요:',e);continue
                profile_path=args.file.with_suffix('.profile.json');save(profile_path,profile)
                print('5개 자세와 허용 이동 구간 검사 완료:',args.file.resolve())
                print('재생용 프로필:',profile_path.resolve())
                print('아직 자동 이동하지 않습니다. 이 파일에 맞춘 제어기 전환 후 재생합니다.')
                return
            if command not in STAGES:print('위의 자세 이름 또는 show / done / q를 입력하세요.');continue
            try:
                item=capture(link,record,sleep=time.sleep)
                updated=copy.deepcopy(record);updated['poses'][command]=item
                validate_record(updated,expected);save(args.file,updated);record=updated
                print(command+' 저장:',item['positions_raw'])
            except ValueError as e:print('저장하지 않음:',e)
    print('저장된 자세는 유지됩니다:',args.file.resolve())
if __name__=='__main__':
    try:main()
    except (Exception,KeyboardInterrupt) as e:
        print('종료:',e or '사용자 중단',file=sys.stderr);sys.exit(2)
