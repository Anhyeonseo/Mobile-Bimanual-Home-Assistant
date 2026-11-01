#!/usr/bin/env python3
"""Operator-authorized torque-on stationary check; nonzero drive is blocked."""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib
import json
import os
from pathlib import Path
import re
import secrets
import sys
import time


class ZeroOnlyPort:
    def __init__(self, port, frame):
        self.port, self.frame = port, frame

    def __getattr__(self, name):
        return getattr(self.port, name)

    def write(self, data):
        allowed = data in (b"INFO\n", b"STATUS\n", b"SNAP\n", b"STOP\n")
        if re.fullmatch(rb"ARM [0-9A-F]{8} [0-9A-F]{4}\n", data) or re.fullmatch(
                rb"D [0-9A-F]{8} [0-9A-F]{8} Z [0-9A-F]{4}\n", data):
            allowed = data == self.frame(data.decode('ascii').rsplit(' ', 1)[0])
        if not allowed:
            raise RuntimeError("Stationary check blocked a nonzero or unexpected command")
        return self.port.write(data)


def check(m, serial, path, duration=180, port_path='/dev/ttyACM0'):
    if duration not in (5, 180):
        raise ValueError("Unsupported duration")
    trace = Path(str(path)+'.jsonl')
    report = dict(status='incomplete', started_utc=datetime.now(timezone.utc).isoformat(),
                  requested_duration_s=duration, calibration_valid=False, replies=0,
                  nonzero_commands_attempted=0)
    error = None
    stopped = None
    counts = []
    arm_attempted = False
    with trace.open('x') as log:
        def record(row):
            log.write(json.dumps(dict(host_monotonic=time.monotonic(), reply=row))+'\n')
            log.flush()
        with m.stopping_signals(), m.acquire_backend_lease('stm32', 0):
            with m.open_exclusive_serial(serial, port_path, 921600, .01) as raw:
                port = ZeroOnlyPort(raw, m.frame)
                try:
                    report['info'] = m.inspect(port)
                    before = m.snapshot(port); record(before); report['before'] = before
                    if before['eeprom'][33] != 1 or before['goal'] != 0:
                        raise RuntimeError('Expected velocity mode and zero goal')
                    nonce = secrets.randbelow(0xffffffff)+1
                    arm_attempted = True
                    armed = m.arm_when_stationary(port, nonce, False, record)
                    baseline_nonzero = armed['nonzero_speed_writes']
                    capture = m.Capture()
                    began = time.monotonic(); progress = began
                    print('Torque ON, zero speed only; stationary check started', flush=True)
                    for seq in range(1, duration*20+1):
                        tick = time.monotonic()
                        m.send(port, m.frame(f'D {nonce:08X} {seq:08X} Z'))
                        row = m.receive(port, .3); record(row)
                        m.validate(row, 'ack', nonce, seq); capture.add(row)
                        report['replies'] += 1; counts.append(row['relative_counts'])
                        if (row['nonzero_speed_writes'] != baseline_nonzero
                                or row['goal_raw'] != 0 or abs(row['relative_counts']) > 8):
                            raise RuntimeError('Unexpected motion or speed command during hold check')
                        if tick-progress >= 30:
                            print(f"Progress: {round(tick-began)} s, replies={report['replies']}", flush=True)
                            progress = tick
                        time.sleep(max(0, .05-(time.monotonic()-tick)))
                    report['elapsed_s'] = time.monotonic()-began
                    if capture.stationary() is None:
                        raise RuntimeError('Final stationary window not verified')
                except m.StartupNotStationary as exc:
                    arm_attempted = False
                    report['startup_rejected_without_motor_writes'] = True
                    error = str(exc)
                except BaseException as exc:
                    error = str(exc) or type(exc).__name__
                finally:
                    if arm_attempted:
                        try:
                            stopped = m.stop(port); record(stopped)
                            if stopped.get('fault') is not False:
                                error = error or 'Controller fault latched; stop confirmed'
                        except Exception as exc:
                            error = f'{error or "Exit"}; STOP unconfirmed: {exc}'
        log.flush(); os.fsync(log.fileno())
    report.update(status='stationary_hold_passed' if error is None else 'stationary_hold_failed',
                  error=error, stop=stopped, relative_min=min(counts) if counts else None,
                  relative_max=max(counts) if counts else None,
                  trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
                  finished_utc=datetime.now(timezone.utc).isoformat())
    m.save_new(path, report)
    print(json.dumps(dict(report_on_jetson=str(path), **report), indent=2), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', required=True, type=Path)
    parser.add_argument('--duration', type=int, choices=(5, 180), default=180)
    parser.add_argument('--port', default='/dev/ttyACM0')
    args = parser.parse_args()
    sys.path.insert(0, str(args.runtime))
    m = importlib.import_module('teleop_lift')
    serial = importlib.import_module('serial')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+str(os.getpid())
    path = Path.home() / 'robot/shared/diagnostics/s1' / ('lift-stationary-hold-'+stamp+'.json')
    path.parent.mkdir(parents=True, exist_ok=True)
    return 0 if check(m, serial, path, args.duration, args.port)['error'] is None else 2


if __name__ == '__main__':
    raise SystemExit(main())
