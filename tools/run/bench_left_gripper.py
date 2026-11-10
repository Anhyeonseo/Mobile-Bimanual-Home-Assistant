#!/usr/bin/env python3
"""Measure a supported left jaw only. Each command allows one bounded closing increment."""
import argparse
import json
from pathlib import Path
import secrets
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/so101_arm_bridge'))
from so101_arm_bridge.backend_lease import acquire_backend_lease
from so101_arm_bridge.serial_port import open_exclusive_serial
from so101_arm_bridge.gripper_bench_protocol import (
    FIRMWARE, SUPPORTED_FIRMWARE, MEASURED_ENDINGS, check_hold_feedback, target_outcome, trial_limits,
)

PORT = '/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_005100393235511438363730-if02'



def exchange(port, command):
    packet = (command+'\n').encode('ascii')
    if port.write(packet) != len(packet):
        raise ValueError('incomplete host packet')
    raw = port.readline(1024)
    if not raw.endswith(b'\n') or len(raw) >= 1024:
        raise ValueError('missing or incomplete firmware reply')
    row = json.loads(raw)
    if not isinstance(row, dict) or row.get('firmware') not in SUPPORTED_FIRMWARE:
        raise ValueError('unrecognized firmware')
    return row


def trial(port, mode, report, *, expected_start=None, target_plan=None, hold_ms=0,
          sleep=time.sleep, clock=time.monotonic):
    if type(hold_ms) is not int or (hold_ms != 0 and (mode != 'target' or not 500 <= hold_ms <= 3000)):
        raise ValueError('only a supported target trial permits a 500–3000ms hold')
    first = exchange(port, 'INFO')
    report['identity'] = first
    if first['active'] or first['locked']:
        raise ValueError('controller busy or stop unconfirmed')
    if hold_ms and first['firmware'] != FIRMWARE:
        raise ValueError('finite hold requires left-grip-bench-v5')
    if mode == 'identity':
        return
    ticket = secrets.randbelow(2**31-1)+1
    ready = exchange(port, f'BIND {ticket}')
    report['before'] = ready
    if (mode == 'inspect' and ready['ticket'] == ticket and ready['sequence'] == 0
            and ready['active'] == 0 and ready['sample_valid'] and ready['off_verified'] and ready['torque'] == 0):
        # A hand-set endpoint may lie outside the commanded stroke. It is still a valid passive reading.
        return
    if (ready['ticket'] != ticket or ready['bound'] != 1 or ready['active'] or ready['sequence'] != 0
            or not ready['off_verified'] or not ready['sample_valid'] or ready['torque'] != 0):
        raise ValueError('inspect rejected: '+ready['reason'])
    if mode == 'inspect':
        return
    if expected_start is None or abs(ready['position'] - expected_start) > 25:
        raise ValueError('opening has changed since the selected reference')
    floor,entry=trial_limits(mode,target_plan)
    if not floor+6 <= ready['position'] <= entry:
        raise ValueError('start outside the allowed closing range')
    if mode == 'target':
        if not hold_ms and ready['position'] <= target_plan['upper_contact_raw']+6:
            raise ValueError('insufficient free approach before the expected contact band')
        report['target_plan'] = target_plan
    try:
        command = (f'BEGIN_RIGID {ticket} {floor} {entry} {hold_ms}' if hold_ms
                   else f'BEGIN {ticket} {floor} {entry}')
        report['hold_requested_ms'] = hold_ms
        row = exchange(port, command)
        report['start_reply'] = row
        if row['ticket'] != ticket or row['sequence'] != 0 or row['active'] != 1 or row['reason'] != 'ANCHORED':
            raise ValueError('start rejected: '+row['reason'])
        if row.get('floor_raw') != floor or row.get('entry_raw') != entry or not floor+6<=row['goal']<=entry:
            raise ValueError('controller did not bind requested jaw limits')
        if row['firmware'] != first['firmware']:
            raise ValueError('controller changed during activation')
        if hold_ms:
            check_hold_feedback(dict(row,hold_goal=-1,hold_elapsed_ms=0),row,hold_ms)
        deadline = clock() + 25.
        for seq in range(1, 650):
            if clock() > deadline:
                break
            sleep(.055)
            previous = row
            previous_goal = previous['goal']
            row = exchange(port, f'STEP {ticket} {seq}')
            report['samples'].append(row)
            if row['ticket'] != ticket or row['sequence'] != seq:
                raise ValueError('lost step or expired lease: '+row['reason'])
            if row['firmware'] != first['firmware']:
                raise ValueError('controller changed during trial')
            if (row.get('floor_raw') != floor or row.get('entry_raw') != entry
                    or not max(floor,previous_goal-6) <= row['goal'] <= previous_goal
                    or not floor <= row['position'] <= entry):
                raise ValueError('measured jaw or goal outside bound')
            if hold_ms:
                check_hold_feedback(previous,row,hold_ms)
            if not row['active']:
                if (row['reason'] not in MEASURED_ENDINGS or not row['off_verified']
                        or row['locked'] or not row['sample_valid']):
                    raise ValueError('test stopped: '+row['reason'])
                report['endpoint'] = row
                if mode == 'target':
                    report['contact_outcome'] = target_outcome(row,target_plan)
                return
        raise ValueError('trial deadline')
    finally:
        stop = exchange(port, 'STOP')
        report['stop'] = stop
        if stop['ticket'] != ticket or stop['active'] or stop['locked'] or stop['off_verified'] != 1:
            raise ValueError('torque-off unconfirmed; support jaw and cut motor power')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=('identity','inspect','empty','target'))
    p.add_argument('--port', default=PORT)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--start-raw', type=int)
    p.add_argument('--target-plan', type=Path, help='calibrated bounds for a manually supported object')
    p.add_argument('--hold-ms',type=int,default=0,
                   help='supported rigid target only: preload to the planned floor, hold 500–3000ms, then torque OFF')
    args = p.parse_args()
    if (args.mode == 'target') != (args.target_plan is not None):
        p.error('target mode requires --target-plan')
    moving = args.mode in ('empty','target')
    if moving != args.execute or moving != (args.start_raw is not None):
        p.error('only a closing trial requires --execute and the observed --start-raw')
    if args.hold_ms and (args.mode != 'target' or not 500 <= args.hold_ms <= 3000):
        p.error('--hold-ms requires target mode and 500–3000ms')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(operator_label=args.mode, grasp_verified=False, samples=[], status='failed')
    with args.output.open('x') as dest:
        try:
            import serial
            plan=json.loads(args.target_plan.read_text()) if args.target_plan else None
            with acquire_backend_lease('stm32',0), open_exclusive_serial(serial,args.port,921600,.25) as port:
                port.reset_input_buffer()
                trial(port,args.mode,report,expected_start=args.start_raw,target_plan=plan,hold_ms=args.hold_ms)
                report['status'] = 'trial_recorded' if moving else 'observed'
        except (ValueError,OSError,RuntimeError,ImportError,KeyboardInterrupt) as exc:
            report['error'] = str(exc) or type(exc).__name__
        finally:
            json.dump(report,dest,indent=2,allow_nan=False)
            dest.write('\n')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'}))
    return 2 if report['status']=='failed' else 0


if __name__ == '__main__':
    raise SystemExit(main())
