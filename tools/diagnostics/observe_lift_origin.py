#!/usr/bin/env python3
"""Read fresh ID10 samples without changing torque, mode, goal or firmware.

Requires the operator-confirmed mechanical bottom at reader start. Legacy SNAP
is idle-only. This reader NEVER clears faults or sends STOP (STOP can write).
The captured origin expires when this process closes its serial connection.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks.lift_observation import LiftObservation


class ReadOnlyPort:
    def __init__(self, raw):
        self.raw = raw

    def __getattr__(self, name):
        return getattr(self.raw, name)

    def write(self, data):
        if data not in (b'INFO\n', b'STATUS\n', b'SNAP\n'):
            raise RuntimeError('read-only lift observer blocked command')
        return self.raw.write(data)


def idle_status(m, port, baseline=None):
    m.send(port, b'STATUS\n')
    row = m.receive(port, 2)
    if row.get('type') != 'status' or row.get('armed') is not False:
        raise RuntimeError('controller must be idle before SNAP')
    if row.get('fault') is not False:
        raise RuntimeError('controller fault: ' + str(row.get('reason')))
    counts = (row.get('motor_write_count'), row.get('nonzero_speed_writes'))
    if any(type(v) is not int or v < 0 for v in counts):
        raise RuntimeError('missing write counters')
    if baseline is not None and counts != baseline:
        raise RuntimeError('motor writes or controller restart detected')
    return row, counts


def observe(m, port, *, samples, bottom_confirmed):
    tracker = LiftObservation(bottom_confirmed=bottom_confirmed)
    report = dict(started_utc=datetime.now(timezone.utc).isoformat(), status='failed',
                  samples=[], motion_authorized=False, coordinate_use_allowed=False,
                  runtime_origin_valid=False, s2_complete=False,
                  session_id=tracker.session_id, scale_m_per_turn=tracker.m_per_turn,
                  origin_persists_after_close=False, firmware_boot_identity_available=False)
    port = ReadOnlyPort(port)
    try:
        report['info'] = m.inspect(port)
        report['before'], baseline = idle_status(m, port)
        for _ in range(samples):
            began = time.monotonic()
            _, _ = idle_status(m, port, baseline)
            raw = m.snapshot(port)
            received = time.monotonic()
            item = dict(raw=raw, received_monotonic_s=received)
            report['samples'].append(item)
            item['observation'] = tracker.add(raw, received)
            time.sleep(max(0, .05 - (time.monotonic() - began)))
        report['after'], _ = idle_status(m, port, baseline)
        report['motor_writes_during_check'] = 0
        report['origin_raw_unwrapped'] = tracker.origin
        report['origin_id'] = tracker.origin_id
        report['status'] = 'bottom_observation_passed' if tracker.origin is not None else 'quiet_bottom_not_verified'
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc) or type(exc).__name__
    finally:
        tracker.invalidate('reader session closed; confirm bottom again on reconnect')
        report['session_closed'] = True
        report['finished_utc'] = datetime.now(timezone.utc).isoformat()
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime', required=True, type=Path, help='existing teleop-lift.pyz')
    p.add_argument('--port', required=True)
    p.add_argument('--bottom-confirmed', action='store_true')
    p.add_argument('--samples', type=int, default=100)
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    if not args.bottom_confirmed or not 5 <= args.samples <= 200:
        p.error('current bottom confirmation and 5..200 samples required')
    if args.output.exists():
        p.error('output already exists')
    sys.path.insert(0, str(args.runtime))
    m = importlib.import_module('teleop_lift')
    serial = importlib.import_module('serial')
    with m.acquire_backend_lease('stm32', 0):
        with m.open_exclusive_serial(serial, args.port, 921600, .01) as port:
            report = observe(m, port, samples=args.samples, bottom_confirmed=True)
    report['runtime_sha256'] = hashlib.sha256(args.runtime.read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
        f.write('\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('samples', 'before', 'after')}, ensure_ascii=False))
    return 0 if report['status'] == 'bottom_observation_passed' else 2


if __name__ == '__main__':
    raise SystemExit(main())
