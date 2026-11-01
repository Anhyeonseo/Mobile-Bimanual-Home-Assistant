#!/usr/bin/env python3
"""Read-only lift communication diagnosis; never a height calibration.

Preserve bad replies and recovery instead of terminating at a motor READ error.
Compare reference ID 1 and mobile IDs 7..10 after the first error of an episode.
Host disconnects and actual identity/mode/torque changes still end the session.
No encoder unwrapping, no bridging missing positions, and no motor writes.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import time

from calibrate_left_home import save
from measure_lift_range import identify, exchange, stdin_control
from probe_mobile_bus import probe


def check_row(row):
    if type(row.get('id')) is not int or row['id'] != 10:
        raise ValueError('unexpected motor ID')
    if row.get('identity_ok') is False:
        return False
    if row.get('identity_ok') is not True:
        raise ValueError('malformed identity result')
    if any(type(row.get(k)) is not int or row[k] != v for k, v in
           dict(reported_id=10, model_raw=777, baud_code=0).items()):
        raise ValueError('motor identity changed')
    if row.get('mode_raw') not in (-1, 0) or row.get('torque_raw') not in (-1, 0):
        raise ValueError('mode or torque state changed; keep lift supported')
    if any(type(row.get(k)) is not int for k in ('mode_raw', 'torque_raw', 'position_raw')):
        raise ValueError('malformed register result')
    if not -1 <= row['position_raw'] < 4096:
        raise ValueError('unexpected encoder format')
    return all(row[k] != -1 for k in ('mode_raw', 'torque_raw', 'position_raw'))


def run(port, path, seconds=600, control=stdin_control, clock=time.monotonic, sleep=time.sleep):
    if type(seconds) is not int or not 1 <= seconds <= 600:
        raise ValueError('duration must be 1..600 seconds')
    trace=path.with_suffix('.jsonl')
    if path.exists() or trace.exists():
        raise FileExistsError('use new diagnostic paths')
    info=identify(port)
    record=dict(schema_version=1, record_kind='lift_link_diagnostics', status='recording',
                firmware=info, motor_write_commands=0, calibration_valid=False,
                sample_count=0, bad_reply_count=0, recovered_episodes=0,
                comparisons=0, created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    path.parent.mkdir(parents=True,exist_ok=True);save(path,record)
    started=clock();failed_at=None;previous_sequence=None
    try:
        with trace.open('x') as log:
            def write(event):
                log.write(json.dumps(event)+'\n');log.flush()
            while clock()-started < seconds:
                row=exchange(port,'LIFT');stamp=clock()
                write(dict(event='lift_read',time_monotonic=stamp,motor=row))
                sequence=row.get('read_sequence')
                if type(sequence) is not int or not 0 < sequence < 2**32 or (previous_sequence is not None and sequence<=previous_sequence):
                    raise ValueError('controller restarted or sequence invalid')
                previous_sequence=sequence
                good=check_row(row)
                record['sample_count']+=1
                record['last_row']=row
                if not good:
                    record['bad_reply_count']+=1
                    if failed_at is None:
                        failed_at=stamp
                        print('FAULT: '+json.dumps(row),flush=True)
                        # A comparison is separately logged; it does not fill any encoder gap.
                        comparison=probe(port,scan=True)
                        write(dict(event='bus_comparison',time_monotonic=clock(),report=comparison))
                        record['comparisons']+=1
                        record['last_comparison']=comparison
                        save(path,record)
                elif failed_at is not None:
                    recovery=dict(event='recovery',time_monotonic=stamp,
                                  seconds_since_first_bad_reply=stamp-failed_at,
                                  includes_comparison_query_time=True)
                    write(recovery);record['recovered_episodes']+=1
                    failed_at=None
                    print('RECOVERED: '+json.dumps(recovery),flush=True);save(path,record)
                if record['sample_count']==1:
                    if not good:raise ValueError('initial motor read failed; do not begin hand test')
                    print('READY: torque OFF verified; diagnostic logging only, no height conversion',flush=True)
                action=control()
                if action in ('finish','abort'):
                    record['stop_reason']='operator_finished' if action=='finish' else 'operator_cancelled'
                    break
                if action:
                    raise ValueError('expected finish or abort')
                sleep(.05)
        record.update(status='diagnostic_complete',elapsed_seconds=clock()-started,
                      pending_read_failure=failed_at is not None,
                      stop_reason=record.get('stop_reason','duration_elapsed'))
    except BaseException as error:
        record.update(status='diagnostic_interrupted',error=str(error))
        raise
    finally:
        if trace.exists():record['trace_sha256']=hashlib.sha256(trace.read_bytes()).hexdigest()
        save(path,record)
    return record


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=int,default=600)
    a=p.parse_args()
    import serial
    from so101_arm_bridge.backend_lease import acquire_backend_lease
    from so101_arm_bridge.serial_port import open_exclusive_serial
    with acquire_backend_lease('stm32',0),open_exclusive_serial(serial,a.port,921600,.1) as port:
        port.reset_input_buffer();result=run(port,a.output,a.seconds)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
