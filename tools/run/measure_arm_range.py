#!/usr/bin/env python3
"""Record one hand-guided joint sweep. No motor writes or firmware changes.

Start at the current pose and slowly visit both comfortable endpoints. Other
joints may be repositioned; returning Home is not required. Send 'finish' on
stdin after the operator confirms completion. The initial encoder sample uses
turn zero by convention; its physical branch relative to Home is not verified.
The captured interval is review evidence, not an automatic runtime permission.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import select
import sys
import time

from calibrate_arm_home import SelectedArmLink
from calibrate_left_home import Link, save, validate

JOINTS = ('base', 'shoulder', 'elbow', 'wrist_flex', 'wrist_roll', 'gripper')


class Sweep:
    def __init__(self, reference, joint):
        self.reference, self.joint = reference, joint
        self.previous = None
        self.value = None
        self.minimum = None
        self.maximum = None
        self.last_time = None
        self.count = 0
        self.maximum_gap = 0
        self.maximum_step = 0

    def add(self, rows, timestamp):
        validate(rows)
        if any(a['eeprom_hex'] != b['eeprom_hex'] for a, b in zip(rows, self.reference, strict=True)):
            raise ValueError('calibration/configuration changed')
        raw = rows[self.joint]['position']
        if self.previous is None:
            self.value = raw
            self.minimum = self.maximum = raw
        else:
            gap = timestamp - self.last_time
            if not 0 < gap <= 1.0:
                raise ValueError('sample gap too large; sweep continuity unknown')
            delta = (raw - self.previous + 2048) % 4096 - 2048
            if abs(delta) > 256:
                raise ValueError('joint moved too fast or encoder jumped; repeat slowly')
            self.value += delta
            self.maximum_gap = max(self.maximum_gap, gap)
            self.maximum_step = max(self.maximum_step, abs(delta))
            self.minimum = min(self.minimum, self.value)
            self.maximum = max(self.maximum, self.value)
            if self.maximum - self.minimum >= 4096:
                raise ValueError('full-turn interval is ambiguous for runtime binding')
        self.previous, self.last_time = raw, timestamp
        self.count += 1
        return dict(time_monotonic=timestamp, positions_raw=[r['position'] for r in rows],
                    selected_unwrapped_raw=self.value,
                    minimum_unwrapped_raw=self.minimum, maximum_unwrapped_raw=self.maximum)

    def finish(self):
        if self.count < 5 or self.maximum - self.minimum < 32:
            raise ValueError('insufficient movement samples')
        return dict(minimum_unwrapped_raw=self.minimum, maximum_unwrapped_raw=self.maximum,
                    sample_count=self.count, maximum_gap_seconds=self.maximum_gap,
                    maximum_sample_step_raw=self.maximum_step, final_unwrapped_raw=self.value)


def capture(link, reference, path, joint, control, *, calibration_sha256,
            clock=time.monotonic, sleep=time.sleep):
    info = link.exchange('INFO')
    if (info.get('firmware') != 'arm-calibration-v1' or info.get('arm') != reference['arm']
            or info.get('motion_enabled') is not False or info.get('ids') != list(range(1,7))):
        raise ValueError('wrong maintenance firmware or arm')
    if path.exists() or path.with_suffix('.jsonl').exists():
        raise FileExistsError('use a new capture path')
    sweep = Sweep(reference['after'], joint)
    record = dict(schema_version=1, record_kind='hand_guided_joint_range', arm=reference['arm'],
                  joint=JOINTS[joint], motor_id=joint+1, firmware=info,
                  status='recording', motor_write_commands_sent=False,
                  calibration_sha256=calibration_sha256,
                  firmware_limit_authorized=False,
                  coordinate_basis='first_sample_modulo_as_turn_zero',
                  home_branch_verified=False,
                  other_joint_repositioning_allowed=True,
                  collision_free_joint_combinations_verified=False,
                  created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    path.parent.mkdir(parents=True, exist_ok=True)
    save(path, record)
    started = clock()
    try:
        with path.with_suffix('.jsonl').open('x') as log:
            while True:
                sample = sweep.add(link.exchange('SCAN', scan=True), clock())
                log.write(json.dumps(sample)+'\n'); log.flush()
                if sweep.count == 1:
                    print('READY: read-only capture running; sweep selected joint slowly. Other joints may move; no Home return required.', flush=True)
                action = control()
                if action == 'finish':
                    record.update(sweep.finish(), status='captured_pending_operator_review')
                    break
                if action in ('abort', 'eof'):
                    raise RuntimeError('capture cancelled')
                if clock()-started > 600:
                    raise TimeoutError('capture time exceeded; not approved')
                sleep(.05)
        record['samples_sha256'] = hashlib.sha256(path.with_suffix('.jsonl').read_bytes()).hexdigest()
        save(path, record)
        print(json.dumps(record), flush=True)
        return record
    except BaseException as error:
        record.update(status='incomplete_do_not_use', error=str(error), sample_count=sweep.count)
        save(path, record)
        raise


def stdin_control():
    if not select.select([sys.stdin], [], [], 0)[0]:
        return None
    line = sys.stdin.readline()
    return line.strip() if line else 'eof'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', choices=('left','right'), required=True)
    parser.add_argument('--joint', choices=JOINTS, required=True)
    parser.add_argument('--port', required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    reference = json.loads(args.calibration.read_text())
    if reference.get('arm') != args.arm or reference.get('status') != 'centered_verified':
        parser.error('successful matching-arm Home calibration required')
    validate(reference['after'])
    import serial
    from so101_arm_bridge.backend_lease import acquire_backend_lease
    from so101_arm_bridge.serial_port import open_exclusive_serial
    with acquire_backend_lease('stm32', 0), open_exclusive_serial(serial, args.port, 921600, .1) as port:
        port.reset_input_buffer()
        capture(SelectedArmLink(Link(port), args.arm), reference, args.output,
                JOINTS.index(args.joint), stdin_control,
                calibration_sha256=hashlib.sha256(args.calibration.read_bytes()).hexdigest())


if __name__ == '__main__':
    main()
