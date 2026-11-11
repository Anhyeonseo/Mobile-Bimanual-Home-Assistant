#!/usr/bin/env python3
"""Read-only INFO/SCAN for mobile-bus-inspection-v2 or mobile-wheel-setup-v1.

INFO is the default; --scan explicitly reads reference ID 1 and mobile IDs 7..10. No motor writes.
"""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/so101_arm_bridge"))
from so101_arm_bridge.backend_lease import acquire_backend_lease
from so101_arm_bridge.serial_port import open_exclusive_serial


def receive(port, deadline):
    while time.monotonic() < deadline:
        line = port.readline()
        if line:
            result = json.loads(line.decode("ascii"))
            if not isinstance(result, dict):
                raise ValueError("invalid inspection response")
            return result
    raise RuntimeError("inspection response timeout")


def probe(port, scan=False):
    port.reset_input_buffer()
    if port.write(b"INFO\n") != 5:
        raise RuntimeError("partial INFO transmission")
    info = receive(port, time.monotonic() + 2)
    if (info.get("firmware") not in ("mobile-bus-inspection-v2", "mobile-wheel-setup-v1") or
            info.get("motion_enabled") is not False or
            info.get("host_baud") != 921600 or info.get("servo_baud") != 1000000 or
            info.get("ids") != [7, 8, 9, 10] or info.get("reference_id") != 1):
        raise RuntimeError("unexpected firmware/configuration; SCAN not sent")
    report = {"info": info, "scanned": False, "motor_write_commands": 0}
    if not scan:
        return report
    if port.write(b"SCAN\n") != 5:
        raise RuntimeError("partial SCAN transmission")
    deadline = time.monotonic() + 4
    if receive(port, deadline) != {"scan": "begin"}:
        raise RuntimeError("missing scan begin")
    reference = receive(port, deadline)
    if reference.get("id") != 1:
        raise RuntimeError("reference ID mismatch")
    motors = [receive(port, deadline) for _ in range(4)]
    if [m.get("id") for m in motors] != info["ids"]:
        raise RuntimeError("scan ID sequence mismatch")
    if receive(port, deadline) != {"scan": "end", "motor_write_commands": 0}:
        raise RuntimeError("incomplete scan")
    reference_ok = reference.get("identity_ok") is True and reference.get("reported_id") == 1
    report.update(scanned=True, reference=reference, reference_read_ok=reference_ok,
                  motors=motors, all_reads_ok=reference_ok and all(
        m.get("identity_ok") is True and m.get("reported_id") == m["id"] and
        all(type(m.get(k)) is int and m[k] >= 0 for k in
            ("model_raw", "baud_code", "mode_raw", "torque_raw", "position_raw"))
        for m in motors
    ))
    return report


def wheel_sample(port, info):
    """Read three timestamped position/speed blocks; no torque or speed writes."""
    if (info.get('firmware') != 'mobile-bus-inspection-v2'
            or info.get('wheel_samples_supported') is not True
            or info.get('motion_enabled') is not False or info.get('ids') != [7,8,9,10]):
        raise RuntimeError('read-only wheel sampling capability required')
    started=time.monotonic()
    if port.write(b'WHEELS\n')!=7:raise RuntimeError('partial WHEELS transmission; no retry')
    deadline=started+1
    begin=receive(port,deadline)
    if (begin.get('wheels')!='begin' or type(begin.get('sequence')) is not int
            or not 0<=begin['sequence']<2**32):raise ValueError('invalid wheel sequence')
    rows=[receive(port,deadline) for _ in range(3)]
    end=receive(port,deadline)
    if end!={'wheels':'end','motor_write_commands':0}:raise ValueError('incomplete wheel sample')
    if [r.get('id') for r in rows]!=[7,8,9]:raise ValueError('wheel ID mismatch')
    for row in rows:
        if (row.get('ok') is not True or row.get('hal_status')!=0
                or any(type(row.get(k)) is not int or not 0<=row[k]<2**32
                       for k in ('started_tick_ms','ended_tick_ms'))
                or (row['ended_tick_ms']-row['started_tick_ms'])%2**32>50
                or type(row.get('position_raw')) is not int or not 0<=row['position_raw']<4096
                or type(row.get('speed_raw')) is not int or not 0<=row['speed_raw']<65536):
            raise ValueError('invalid wheel feedback: '+json.dumps(row))
    return dict(sequence=begin['sequence'],host_started_s=started,host_received_s=time.monotonic(),wheels=rows)


def capture_wheels(port, seconds, output):
    if not 1<=seconds<=60:raise ValueError('wheel capture must last 1..60 seconds')
    report=probe(port,True)
    output.write(json.dumps({'inventory':report})+'\n');output.flush()
    if not report['all_reads_ok'] or any(m['mode_raw']!=1 or m['torque_raw']!=0 for m in report['motors'][:3]):
        raise ValueError('identified speed-mode wheels with torque OFF required; no mode/torque changes attempted')
    deadline=time.monotonic()+seconds
    previous=None;count=0
    while time.monotonic()<deadline:
        row=wheel_sample(port,report['info'])
        if previous is not None and (row['sequence']-previous)%2**32!=1:
            raise ValueError('wheel sequence changed; capture cannot resume across reset')
        output.write(json.dumps(row)+'\n');output.flush()
        previous=row['sequence'];count+=1
        time.sleep(.02)
    return dict(samples=count,motor_write_commands=0,motion_authorized=False,
                speed_scale_calibrated=False,output='timestamped raw samples; no automatic calibration')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--scan", action="store_true", help="Explicit servo READs; support lift before motor power-on")
    parser.add_argument('--sample-wheels-seconds',type=float,help='read-only torque-OFF wheel capture, 1..60 seconds')
    parser.add_argument('--output',type=Path,help='new JSONL file required for wheel capture')
    args = parser.parse_args()
    if args.sample_wheels_seconds is not None and (not 1<=args.sample_wheels_seconds<=60 or args.output is None):
        parser.error('wheel capture requires 1..60 seconds and a new --output file')
    if args.output is not None and args.sample_wheels_seconds is None:
        parser.error('--output requires --sample-wheels-seconds')
    try:
        import serial
        from contextlib import nullcontext
        capture_output = args.output.open('x') if args.output else nullcontext()
        with capture_output as output:
            with acquire_backend_lease("stm32", 0):
                with open_exclusive_serial(serial, args.port, 921600, .2) as port:
                    result = (capture_wheels(port,args.sample_wheels_seconds,output)
                              if output else probe(port, args.scan))
        print(json.dumps(result, indent=2))
        return 0 if result.get("all_reads_ok", True) else 2
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        print(json.dumps({"error": str(error), "motor_write_commands": 0}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
