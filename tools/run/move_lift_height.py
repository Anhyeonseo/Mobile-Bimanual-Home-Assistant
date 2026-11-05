#!/usr/bin/env python3
"""Bounded operator-authorized lift commissioning, using v5 MCU reference/guards.

Default INFO only. --execute is required for torque-on or movement. The initial
up check is MCU-limited to two seconds. The visible check stops after 1cm
or a 15-second host deadline; the MCU watchdog remains 400ms. Target moves
use continuous feedback and the same MCU watchdog. No automatic origin acquisition or error recovery.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import secrets
import sys
import time
import teleop_lift as base
from teleop_lift_bidirectional import INFO


def target_counts(height_cm):
    if not math.isfinite(height_cm) or not 1 <= height_cm <= 61:
        raise ValueError('목표 높이는 최저점 대비 1..61cm 범위여야 합니다.')
    counts = -round(height_cm/100/.084*4096)
    # Keep at least ~5mm before the unchanged MCU upper guard.
    if -counts > INFO['top_guard_counts'] - 250:
        raise ValueError('목표가 제어기 상단 정지 구간에 너무 가깝습니다.')
    return counts


def run(port,path,*,check_up=False,visible_check=False,height_cm=None,maximum_seconds=600):
    if sum((bool(check_up),bool(visible_check),height_cm is not None))!=1:
        raise ValueError('select one movement mode')
    target=None if check_up or visible_check else target_counts(height_cm)
    if not 1 <= maximum_seconds <= 600:raise ValueError('invalid duration')
    if visible_check:maximum_seconds=min(maximum_seconds,25)
    path=Path(path);trace=Path(str(path)+'.jsonl')
    if path.exists() or trace.exists():raise ValueError('기존 기록을 덮어쓰지 않습니다.')
    base.inspect(port,INFO)
    base.send(port,b'STATUS\n');initial=base.receive(port,2)
    if (initial.get('armed') is not False or initial.get('fault') is not False or
            type(initial.get('reference_id')) is not int or initial['reference_id']==0):
        raise RuntimeError('유효한 최저점 기준과 오류 없는 대기 상태가 필요합니다.')
    reference=initial['reference_id']
    report=dict(status='incomplete',started_utc=datetime.now(timezone.utc).isoformat(),
                initial=initial,reference_id=reference,check_up=check_up,visible_check=visible_check,
                visible_rise_cm=1.0 if visible_check else None,
                maximum_movement_seconds=15 if visible_check else None,target_height_cm=height_cm,
                scale_m_per_turn=.084,scale_validated=False,s2_complete=False)
    path.parent.mkdir(parents=True,exist_ok=True)
    capture=base.Capture();nonce=secrets.randbelow(0xffffffff)+1
    attempted=False;stopped=None;error=None
    with trace.open('x') as log:
        def record(row):
            log.write(json.dumps(dict(host_monotonic=time.monotonic(),reply=row))+'\n');log.flush()
        try:
            attempted=True
            armed=base.arm_when_stationary(port,nonce,check_up,record,allow_holding_retry=True)
            if armed.get('reference_id')!=reference:raise RuntimeError('원점이 변경되었습니다.')
            seq=0;direction=None;settling=False;settle_start=None;ready=False
            started=last_print=time.monotonic();pulse_start=None
            print('토크 ON · 정지 확인 중. 아직 이동하지 않습니다.',flush=True)
            while True:
                now=time.monotonic()
                if now-started>maximum_seconds:raise RuntimeError('제한 시간 초과')
                current='Z' if not ready or settling else direction
                if check_up and pulse_start is not None and now-pulse_start>2.5:
                    raise RuntimeError('MCU 상승 검사 종료 응답 시간 초과')
                if visible_check and pulse_start is not None and now-pulse_start>=15 and not settling:
                    raise RuntimeError('가시 상승 검사 15초 제한: 이동을 중단합니다.')
                if current=='U' and (check_up or visible_check) and pulse_start is None:pulse_start=now
                seq+=1
                base.send(port,base.frame(f'D {nonce:08X} {seq:08X} {current}'))
                row=base.receive(port,.3);record(row)
                if row.get('reference_id')!=reference:raise RuntimeError('원점 무효화 또는 변경')
                if row.get('type')=='stopped':
                    if (check_up and row.get('reason')=='TEST_PULSE_COMPLETE' and
                            row.get('stop_confirmed') is True and row.get('fault') is False):
                        report.update(status='up_check_stopped',last=row,
                                      observed_upward_counts=report['start_counts']-row['relative_counts'])
                        break
                    raise RuntimeError('목표 전 제어기 정지: '+str(row.get('reason')))
                base.validate(row,'ack',nonce,seq);capture.add(row)
                if row.get('telemetry_valid') is not True or row.get('motor_status_raw')!=0:
                    raise RuntimeError('유효한 모터 상태가 아닙니다.')
                quiet=capture.stationary()
                if not ready and quiet is not None:
                    report['start_counts']=quiet
                    if check_up or visible_check:
                        direction='U'
                        if visible_check:
                            target=quiet+target_counts(1.0)
                            if target < -INFO['top_guard_counts']:
                                raise RuntimeError('상단 정지 구간에 가까워 1cm 상승 검사를 시작할 수 없습니다.')
                    else:
                        direction='U' if target<quiet else 'D'
                        if abs(target-quiet)<=8:settling=True;settle_start=now
                    ready=True
                    print('시작: '+('약 1cm 상승 검사 · 이동 제한 15초' if visible_check else '최대 2초 상승 검사' if check_up else f'최저점 대비 {height_cm:.1f}cm로 저속 이동'),flush=True)
                if ready and not check_up and not settling:
                    reached=(direction=='U' and row['relative_counts']<=target) or (direction=='D' and row['relative_counts']>=target)
                    if reached:settling=True;settle_start=now
                if settling and quiet is not None:
                    report.update(status='visible_up_check_stopped' if visible_check else 'target_reached_and_stopped',
                                  observed_upward_counts=report['start_counts']-quiet,relative_counts=quiet,
                                  height_above_bottom_m=-quiet*.084/4096,sample_tick_ms=row['sample_tick'])
                    break
                if settling and now-settle_start>5:raise RuntimeError('목표 정지 확인 시간 초과')
                if ready and now-last_print>=5:
                    print(f"최저점 대비 {-row['relative_counts']*.084/4096*100:.2f}cm · {current}",flush=True);last_print=now
                time.sleep(max(0,base.PERIOD-(time.monotonic()-now)))
        except base.StartupNotStationary as exc:
            attempted=False;error=str(exc)
        except BaseException as exc:error=str(exc) or type(exc).__name__
        finally:
            if attempted:
                try:
                    stopped=base.stop(port);record(stopped)
                    if stopped.get('fault') is not False:error=error or '제어기 오류 잠금'
                    if stopped.get('reference_id')!=reference:error=error or '정지 후 원점 무효화'
                except Exception as exc:error=(error or '종료')+'; STOP 확인 실패: '+str(exc)
            report.update(error=error,stop=stopped,finished_utc=datetime.now(timezone.utc).isoformat())
            if error:report['status']='failed'
            log.flush()
            report['trace_sha256']=hashlib.sha256(trace.read_bytes()).hexdigest()
            base.save_new(path,report)
            print('기록: '+str(path),flush=True)
            if stopped and stopped.get('stop_confirmed'):print('정지 응답 확인 · 토크 유지',flush=True)
            elif attempted:print('정지 미확인: 리프트를 받치고 모터 전원을 꺼주세요.',flush=True)
    if error:raise RuntimeError(error)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    p.add_argument('--execute',action='store_true')
    modes=p.add_mutually_exclusive_group()
    modes.add_argument('--check-up',action='store_true')
    modes.add_argument('--check-up-visible',action='store_true')
    modes.add_argument('--target-rise-cm',type=float)
    p.add_argument('--up-direction-confirmed',action='store_true')
    p.add_argument('--file',type=Path)
    a=p.parse_args()
    moving=a.check_up or a.check_up_visible or a.target_rise_cm is not None
    if moving and (not a.execute or a.file is None):p.error('movement requires --execute and --file')
    if a.execute and not moving:p.error('select movement mode')
    if a.target_rise_cm is not None:
        target_counts(a.target_rise_cm)
        if not a.up_direction_confirmed:p.error('actual up/stop confirmation required')
    import serial
    with base.stopping_signals(),base.acquire_backend_lease('stm32',0):
        with base.open_exclusive_serial(serial,a.port,921600,.01) as port:
            if moving:run(port,a.file,check_up=a.check_up,visible_check=a.check_up_visible,height_cm=a.target_rise_cm)
            else:print(json.dumps(base.inspect(port,INFO)))


if __name__=='__main__':
    try:main()
    except (RuntimeError,OSError,ValueError) as exc:
        print('실행 중단: '+str(exc),file=sys.stderr);raise SystemExit(2)
