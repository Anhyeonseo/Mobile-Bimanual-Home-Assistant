#!/usr/bin/env python3
"""ID10 v5: explicit bottom reference, manual up/down and MCU-bounded up check."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import secrets
import select
import sys
import termios
import time
import tty
import teleop_lift as base
from so101_arm_bridge.encoder_units import LIFT_METERS_PER_TURN, lift_rise_m

INFO = dict(base.INFO, firmware='lift-teleop-v5', down_only=False,
            bottom_guard_counts=256, top_guard_counts=30000)


def set_bottom(port, path):
    base.inspect(port, INFO)
    base.send(port, b'STATUS\n'); before=base.receive(port,2)
    if before.get('armed') is not False or before.get('fault') is not False or before.get('reference_id') != 0:
        raise RuntimeError('오류 없는 대기 상태에서 새 최저점 기준만 설정할 수 있습니다.')
    nonce=secrets.randbelow(0xffffffff)+1
    report=dict(status='failed', before=before, replies=[], motion_authorized=False)
    try:
        for _ in range(20):
            base.send(port,base.frame(f'REF {nonce:08X}')); row=base.receive(port,2)
            report['replies'].append(row)
            if row.get('type')=='referenced':
                if (row.get('reference_id')!=nonce or row.get('relative_counts')!=0 or
                        row.get('fault') is not False or row.get('armed') is not False or
                        row.get('motor_write_count')!=before['motor_write_count']):
                    raise RuntimeError('최저점 기준 응답 불일치')
                report.update(status='bottom_referenced',reference_id=nonce,motor_writes=0)
                return report
            if (row.get('reason')!='REFERENCE_NOT_STATIONARY' or row.get('fault') is not False or
                    row.get('armed') is not False or row.get('motor_write_count')!=before['motor_write_count']):
                raise RuntimeError('최저점 설정 거부: '+json.dumps(row))
            time.sleep(.1)
        raise RuntimeError('최저점 정지 확인 실패')
    finally:
        base.save_new(path,report)


def command_for(keys, now, current, last_key):
    if len(keys)>64: raise RuntimeError('키 입력 과다')
    keys=keys.lower()
    if any(k in keys for k in ('x','\x03','\x1b')):return 'Z',now,'exit'
    if 'b' in keys:return 'Z',now,'mark'
    if ' ' in keys or ('w' in keys and 's' in keys):return 'Z',now,None
    if 'w' in keys:current,last_key='U',now
    elif 's' in keys:current,last_key='D',now
    if now-last_key>=base.INPUT_LEASE:current='Z'
    return current,last_key,None


def run(port,input_fd,path,*,check_up=False):
    if not os.isatty(input_fd):raise RuntimeError('SSH -t 대화형 터미널이 필요합니다.')
    path=Path(path);trace=Path(str(path)+'.jsonl')
    if path.exists() or trace.exists():raise RuntimeError('기존 기록을 덮어쓰지 않습니다.')
    base.inspect(port,INFO)
    base.send(port,b'STATUS\n');initial=base.receive(port,2)
    if initial.get('armed') is not False or initial.get('fault') is not False or not initial.get('reference_id'):
        raise RuntimeError('최저점 기준 설정(ref)과 오류 없는 대기 상태가 필요합니다.')
    reference=initial['reference_id']
    path.parent.mkdir(parents=True,exist_ok=True)
    report=dict(status='incomplete',started_utc=datetime.now(timezone.utc).isoformat(),
                reference_id=reference,check_up=check_up,scale_m_per_turn=LIFT_METERS_PER_TURN,
                calibration_valid=False,s2_complete=False)
    old=termios.tcgetattr(input_fd);attempted=False;error=None;stopped=None
    capture=base.Capture();nonce=secrets.randbelow(0xffffffff)+1
    with trace.open('x') as log:
        def record(row):
            log.write(json.dumps(dict(host_monotonic=time.monotonic(),reply=row))+'\n');log.flush()
        try:
            tty.setcbreak(input_fd);attempted=True
            base.arm_when_stationary(port,nonce,check_up,record)
            seq=0;ready=False;current='Z';last_key=float('-inf');last_motion='Z'
            pulse_started=False;finishing=False;finish_started=None
            started=last_print=time.monotonic()
            print('정지 확인 중… 준비 표시 후 W/S를 사용하세요.',flush=True)
            while True:
                now=time.monotonic()
                if now-started>1200:raise RuntimeError('20분 조작 시간 제한')
                keys=''
                if select.select([input_fd],[],[],0)[0]:
                    raw=os.read(input_fd,4096)
                    if not raw:break
                    keys=raw.decode('ascii')
                current,last_key,action=command_for(keys,now,current,last_key)
                if action=='exit':break
                if action=='mark' and ready and not check_up:
                    finishing=True;finish_started=finish_started or now
                if not ready or finishing:current='Z';last_key=float('-inf')
                if check_up:
                    if pulse_started and ' ' in keys:break
                    if ready and current=='U':pulse_started=True
                    current='U' if pulse_started else 'Z'
                elif current!='Z' and current!=last_motion and capture.stationary() is None:
                    current='Z'  # Hold zero until the opposite direction is stationary.
                seq+=1
                base.send(port,base.frame(f'D {nonce:08X} {seq:08X} {current}'))
                row=base.receive(port,.3);record(row)
                if row.get('reference_id')!=reference:raise RuntimeError('최저점 기준이 변경되거나 무효화됐습니다.')
                if row.get('type')=='stopped' and row.get('fault') is False and row.get('stop_confirmed') is True:
                    report['status']=row.get('reason','stopped');report['last']=row;break
                base.validate(row,'ack',nonce,seq);capture.add(row)
                if current!='Z':last_motion=current
                quiet=capture.stationary()
                if not ready and quiet is not None:
                    ready=True
                    print('준비 완료 | '+('W 한 번: 최대 2초 상승 검사 · Space/X 정지' if check_up else 'W 유지: 상승 · S 유지: 하강 · Space: 정지 · B: 중간점 저장 후 종료 · X: 종료'),flush=True)
                if finishing and quiet is not None:
                    report.update(status='intermediate_point_recorded',relative_counts=quiet,
                                  height_above_bottom_m=lift_rise_m(quiet),sample_tick_ms=row['sample_tick'])
                    break
                if finishing and now-finish_started>5:raise RuntimeError('중간점 정지 확인 실패')
                if ready and now-last_print>=1:
                    print(f"\r최저점 대비 {lift_rise_m(row['relative_counts'])*100:.2f} cm | {dict(U='상승',D='하강',Z='정지')[current]}    ",end='',flush=True)
                    last_print=now
                time.sleep(max(0,base.PERIOD-(time.monotonic()-now)))
        except base.StartupNotStationary as exc:
            attempted=False;error=str(exc)
        except BaseException as exc:error=str(exc) or type(exc).__name__
        finally:
            if attempted:
                try:
                    stopped=base.stop(port);record(stopped)
                    if stopped.get('fault') is not False:error=error or '제어기 오류 잠금'
                except Exception as exc:error=(error or '종료')+'; STOP 확인 실패: '+str(exc)
            termios.tcsetattr(input_fd,termios.TCSADRAIN,old)
            report.update(error=error,stop=stopped,finished_utc=datetime.now(timezone.utc).isoformat())
            if error:report['status']='failed'
            elif report['status']=='incomplete':report['status']='operator_stopped'
            base.save_new(path,report)
            print('\n기록: '+str(path),flush=True)
            if stopped and stopped.get('stop_confirmed'):print('정지 응답 확인 · 토크 유지',flush=True)
            elif attempted:print('정지 미확인: 리프트를 받치고 모터 전원을 꺼주세요.',flush=True)
    if error:raise RuntimeError(error)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    p.add_argument('--file',type=Path)
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--prepare',action='store_true',help='reuse EEPROM backup and torque-off speed-mode setup with exact v5 identity')
    mode.add_argument('--set-bottom',action='store_true')
    mode.add_argument('--link-check',action='store_true')
    mode.add_argument('--check-up',action='store_true')
    mode.add_argument('--run',action='store_true')
    p.add_argument('--bottom-confirmed',action='store_true')
    p.add_argument('--up-direction-confirmed',action='store_true')
    a=p.parse_args()
    if (a.prepare or a.set_bottom or a.link_check or a.check_up or a.run) and not a.file:p.error('--file required')
    if a.set_bottom and not a.bottom_confirmed:p.error('현재 실제 최저점 확인이 필요합니다.')
    if a.run and not a.up_direction_confirmed:p.error('짧은 상승 검사 후 실제 상승 확인이 필요합니다.')
    if a.file and a.file.exists():p.error('기존 기록을 덮어쓰지 않습니다.')
    import serial
    with base.stopping_signals(),base.acquire_backend_lease('stm32',0):
        with base.open_exclusive_serial(serial,a.port,921600,.01) as port:
            if a.prepare:base.prepare(port,a.file,expected_info=INFO)
            elif a.set_bottom:print(json.dumps(set_bottom(port,a.file),ensure_ascii=False))
            elif a.link_check:base.link_check(port,a.file,INFO)
            elif a.check_up or a.run:run(port,sys.stdin.fileno(),a.file,check_up=a.check_up)
            else:print(json.dumps(base.inspect(port,INFO)))


if __name__=='__main__':
    try:main()
    except (RuntimeError,OSError,ValueError) as exc:
        print('실행 중단: '+str(exc),file=sys.stderr);raise SystemExit(2)
