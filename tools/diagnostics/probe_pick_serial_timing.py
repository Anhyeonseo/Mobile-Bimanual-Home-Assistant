"""Query-only timing probe on a STOP-latched or verified unprepared controller.

Negotiates binary-v2 mode, but does not heartbeat, renew evidence, clear STOP,
enable torque or send a trajectory. Inactive timing cannot certify active timing.
"""
import argparse
from contextlib import ExitStack
from dataclasses import asdict
from pathlib import Path
import json
import os
import subprocess
import sys
import time


def initialize_probe(transport, *, require_stopped=True):
    from so101_arm_bridge.unified_commissioning import PICK_BOARDS
    # Same parser-only negotiation as cached_controller(). In installed
    # firmware BinaryControl_EnterMode only initializes the parser/mode bit.
    # Never send unframed binary packets to an unconfirmed ASCII endpoint.
    hello=transport.enter_binary_mode()
    if (hello.protocol_version!=2 or hello.joint_count!=12
            or hello.firmware_version not in PICK_BOARDS):
        raise RuntimeError('unsupported controller identity; query probe stopped')
    if require_stopped and not hello.stop_latched:
        raise RuntimeError('STOP 잠금이 없습니다. 상태를 바꾸지 않고 검사를 종료합니다.')
    return hello


def read_unprepared_sample(transport, commission):
    """No session creation to make inactive hardware eligible for a probe."""
    state=transport.get_state()
    if state.stop_latched or state.status_code:
        raise RuntimeError('controller state changed during unprepared probe')
    setup=commission.query()
    if setup.flags!=0 or setup.boot_id!=0 or setup.reason:
        raise RuntimeError('작업 준비된 제어기입니다. 상태를 바꾸지 않고 검사를 종료합니다.')
    runtime=transport.stationary_runtime()
    if any(runtime.get(key) is not False for key in ('executor_active','settling')):
        raise RuntimeError('inactive controller not verified; query probe stopped')
    return dict(state=asdict(state),commission=asdict(setup),runtime=runtime)


def read_sample(transport, mobile, lift):
    state=transport.get_state()
    if not state.stop_latched:
        raise RuntimeError('STOP 잠금 상태에서만 조회합니다. 구동 중에는 검사하지 않습니다.')
    base=mobile.status()
    elevator=lift.status()
    if base.boot_id!=elevator.boot_id:
        raise RuntimeError('controller boot changed')
    return dict(state=asdict(state),mobile=asdict(base),lift=asdict(elevator))


def summarize(rows):
    groups={}
    for row in rows:
        key=row.get('inner_request_hex') or row['operation']
        groups.setdefault(key,[]).append(row)
    return {key:dict(count=len(group),maximum_ms=max(r['duration_ms'] for r in group),
                     over_50ms=sum(r['duration_ms']>=50 for r in group),
                     slowest=sorted(group,key=lambda r:r['duration_ms'],reverse=True)[:5])
            for key,group in groups.items()}


def run_probe(args, report):
    import serial
    from so101_arm_bridge.backend_lease import acquire_backend_lease
    from so101_arm_bridge.serial_port import open_exclusive_serial
    from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2, MobileV2Exchange
    from so101_arm_bridge.mobile_client import MobileClient
    from so101_arm_bridge.lift_wire import LiftClient
    port=args.port
    if not port:
        matches=list(Path('/dev/serial/by-id').glob('*STLINK*if02'))
        if len(matches)!=1:raise ValueError('STLINK serial port is ambiguous; specify --port')
        port=str(matches[0])
    report['port']=port
    with ExitStack() as stack:
        stack.enter_context(acquire_backend_lease('stm32',189))
        device=stack.enter_context(open_exclusive_serial(serial,port,921600,.02))
        transport=StreamValidationTransportV2(device,response_timeout_s=.4)
        try:
            report['hello']=asdict(initialize_probe(transport,require_stopped=False))
            state=transport.get_state()
            if state.stop_latched:
                mobile=MobileClient(MobileV2Exchange(transport),lambda:int(time.monotonic()*1000))
                mobile.synchronize()  # Hello/clock queries only, no activation.
                lift=LiftClient(mobile)
                sample_reader=lambda:read_sample(transport,mobile,lift)
            else:
                from so101_arm_bridge.unified_commissioning import CommissionClient
                commission=CommissionClient(transport)
                report['mode']='unprepared_controller_query_only'
                report['lift_session_queries']='skipped_no_prepared_session'
                sample_reader=lambda:read_unprepared_sample(transport,commission)
            report['initial']=sample_reader()
            available=len(os.sched_getaffinity(0)) if hasattr(os,'sched_getaffinity') else os.cpu_count() or 1
            count=min(args.workers,max(0,available-1))
            report['synthetic_cpu_workers']=count
            for name in ('idle','synthetic_cpu_load'):
                processes=[];rows=[]
                try:
                    if name=='synthetic_cpu_load':
                        # Independent processes; never inherit the serial fd.
                        code='import hashlib\np=b"x"*1048576\nwhile True: hashlib.sha256(p).digest()'
                        for _ in range(count):
                            processes.append(subprocess.Popen([sys.executable,'-c',code],close_fds=True,
                                stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL))
                    end=time.monotonic()+args.seconds
                    last_sequence=transport.timing_diagnostics()['transactions'][-1]['sequence']
                    while time.monotonic()<end:
                        report['last']=sample_reader()
                        fresh=transport.timing_diagnostics(after_sequence=last_sequence)['transactions']
                        rows.extend(fresh)
                        if fresh:last_sequence=fresh[-1]['sequence']
                        time.sleep(.01)
                finally:
                    for process in processes:process.terminate()
                    for process in processes:
                        try:process.wait(timeout=2)
                        except subprocess.TimeoutExpired:process.kill();process.wait()
                    report['phases'][name]=summarize(rows)
            report['status']='READ_ONLY_PROBE_COMPLETE'
        finally:
            report['last_transport_transactions']=transport.timing_diagnostics()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port')
    parser.add_argument('--seconds',type=float,default=5.)
    parser.add_argument('--workers',type=int,default=4)
    args=parser.parse_args()
    if not .5<=args.seconds<=15 or not 0<=args.workers<=4:parser.error('seconds .5..15; workers 0..4')
    report=dict(status='FAILED',mode='stopped_controller_read_only',motion_commands=0,
                stop_clear_commands=0,heartbeat_commands=0,phases={},
                limitation='Synthetic CPU load is not GraspNet; inactive firmware timing cannot certify active operation.')
    try:run_probe(args,report)
    except Exception as error:report['error']=type(error).__name__+': '+str(error)
    print(json.dumps(report,indent=2,ensure_ascii=False))
    return 0 if report['status']=='READ_ONLY_PROBE_COMPLETE' else 1


if __name__=='__main__':raise SystemExit(main())
