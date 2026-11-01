#!/usr/bin/env python3
"""Read-only hand-guided lift calibration. Never enables torque or writes motors.

Keep one continuous capture between low and high (either order): the encoder can turn many times.
Mark stable points with `low`, `high`, and optionally `return`; then `finish`.
The finalize subcommand adds measured platform heights, without opening a port.
"""
import argparse
from collections import deque
import datetime
import hashlib
import json
import math
from pathlib import Path
import select
import sys
import time

from probe_mobile_bus import receive
from calibrate_left_home import save


def exchange(port, command):
    if command not in ('INFO', 'LIFT'):
        raise ValueError('read-only command required')
    data = (command+'\n').encode('ascii')
    if port.write(data) != len(data):
        raise RuntimeError('partial read request')
    result = receive(port, time.monotonic()+2)
    if 'error' in result:
        raise ValueError(result['error'])
    return result


def identify(port):
    info = exchange(port, 'INFO')
    if (info.get('firmware') != 'mobile-bus-inspection-v2' or
            info.get('motion_enabled') is not False or
            info.get('lift_read_supported') is not True or
            info.get('ids') != [7, 8, 9, 10] or
            info.get('host_baud') != 921600 or info.get('servo_baud') != 1000000):
        raise ValueError('lift read-only firmware required; no motor query sent')
    return info


class LiftSweep:
    def __init__(self):
        self.previous = None
        self.sequence = None
        self.timestamp = None
        self.value = 0
        self.count = 0
        self.maximum_gap = 0
        self.maximum_step = 0
        self.window = deque()
        self.marks = {}
        self.pending_read_failure = False
        self.read_failures = 0

    def add(self, row, timestamp):
        expected = dict(id=10, reported_id=10, model_raw=777, baud_code=0,
                        mode_raw=0, torque_raw=0)
        if row.get('identity_ok') is not True or any(
                type(row.get(k)) is not int or row[k] != v for k, v in expected.items()):
            raise ValueError('lift identity, mode or torque-OFF check failed')
        raw, sequence = row.get('position_raw'), row.get('read_sequence')
        if (type(raw) is not int or not 0 <= raw < 4096 or
                type(sequence) is not int or not 0 < sequence < 2**32 or
                not math.isfinite(timestamp)):
            raise ValueError('invalid lift feedback')
        if self.previous is not None:
            gap = timestamp-self.timestamp
            if not 0 < gap <= .5 or sequence <= self.sequence:
                raise ValueError('feedback gap or controller reset; repeat continuous capture')
            delta = (raw-self.previous+2048) % 4096-2048
            if abs(delta) > 256:
                raise ValueError('encoder jump or too-fast manual movement; repeat slowly')
            self.value += delta
            self.maximum_gap = max(self.maximum_gap, gap)
            self.maximum_step = max(self.maximum_step, abs(delta))
        if abs(self.value) > 10_000_000:
            raise ValueError('capture travel exceeds supported range')
        self.pending_read_failure = False
        self.previous, self.sequence, self.timestamp = raw, sequence, timestamp
        self.count += 1
        self.window.append((timestamp, self.value))
        while self.window and timestamp-self.window[0][0] > .7:
            self.window.popleft()
        return dict(time_monotonic=timestamp, relative_encoder_raw=self.value, motor=row)

    def note_read_failure(self, row, timestamp):
        """Allow one immediate re-read, only for explicitly missing READ values.

        Keep the last good position/time: the next good sample must still pass
        the original 0.5 second / 256 count continuity limits. Never bypass a
        reported torque/mode change or a servo status error.
        """
        if self.previous is None or self.pending_read_failure or self.read_failures >= 32:
            raise ValueError('read failure retry budget exhausted')
        seq = row.get('read_sequence')
        if (row.get('id') != 10 or type(seq) is not int or not self.sequence < seq < 2**32 or
                not math.isfinite(timestamp) or not 0 < timestamp-self.timestamp <= .5):
            raise ValueError('read failure has no continuous sequence/time')
        if row.get('identity_ok') is True:
            if any(type(row.get(k)) is not int or row[k] != v for k, v in
                   dict(reported_id=10, model_raw=777, baud_code=0).items()):
                raise ValueError('identity changed')
            if (row.get('mode_raw') not in (-1, 0) or row.get('torque_raw') not in (-1, 0) or
                    type(row.get('position_raw')) is not int or not -1 <= row['position_raw'] < 4096 or
                    not any(row.get(k) == -1 for k in ('mode_raw', 'torque_raw', 'position_raw'))):
                raise ValueError('not a missing register read')
        elif not (row.get('identity_ok') is False and
                  row.get('identity_failure') in ('RX_TIMEOUT', 'CHECKSUM', 'HEADER', 'LENGTH') and
                  row.get('identity_hal_status') in (1, 3) and  # HAL_ERROR / HAL_TIMEOUT; 2 is HAL_BUSY
                  row.get('identity_servo_status') == 0 and
                  all(row.get(k) == -1 for k in ('reported_id', 'model_raw', 'baud_code',
                                                'mode_raw', 'torque_raw', 'position_raw'))):
            raise ValueError('unclassified motor response failure')
        self.sequence = seq
        self.pending_read_failure = True
        self.read_failures += 1
        self.window.clear()  # Endpoints require fresh stable samples after recovery.

    def mark(self, label):
        if label not in ('low', 'high', 'return'):
            raise ValueError('unknown mark')
        if label in self.marks:
            raise ValueError('mark already recorded; start a new capture to replace it')
        if label == 'return' and not {'low', 'high'} <= self.marks.keys():
            raise ValueError('mark both endpoints first')
        values = [v for _, v in self.window]
        if (len(values) < 5 or self.timestamp-self.window[0][0] < .4 or
                max(values)-min(values) > 8):
            raise ValueError('hold lift still for at least half a second before marking')
        value = round(sum(values)/len(values))
        other = 'low' if label == 'high' else 'high'
        if label != 'return' and other in self.marks and abs(value-self.marks[other]['encoder_raw']) < 32:
            raise ValueError('insufficient encoder travel')
        self.marks[label] = dict(encoder_raw=value, position_raw=self.previous,
                                sample_number=self.count, stability_span_raw=max(values)-min(values))
        return self.marks[label]

    def finish(self):
        if not {'low', 'high'} <= self.marks.keys():
            raise ValueError('low and high marks required')
        delta = self.marks['high']['encoder_raw']-self.marks['low']['encoder_raw']
        return dict(marks=self.marks, encoder_delta_raw=delta,
                    encoder_up_direction=1 if delta > 0 else -1,
                    sample_count=self.count, recovered_read_failures=self.read_failures,
                    maximum_gap_seconds=self.maximum_gap,
                    maximum_sample_step_raw=self.maximum_step,
                    return_error_raw=(self.marks['return']['encoder_raw']-self.marks['low']['encoder_raw']
                                      if 'return' in self.marks else None))


def stdin_control():
    if not select.select([sys.stdin], [], [], 0)[0]:
        return None
    line = sys.stdin.readline()
    return line.strip() if line else 'abort'


def capture(port, path, control=stdin_control, clock=time.monotonic, sleep=time.sleep):
    trace = path.with_suffix('.jsonl')
    if path.exists() or trace.exists():
        raise FileExistsError('use a new capture path')
    info = identify(port)
    sweep = LiftSweep()
    record = dict(schema_version=1, record_kind='hand_guided_lift_range',
                  firmware=info, status='recording', motor_write_commands=0,
                  hardware_enabled=False, runtime_profile_authorized=False,
                  physical_motion_test_verified=False,
                  created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    path.parent.mkdir(parents=True, exist_ok=True)
    save(path, record)
    started = clock()
    try:
        with trace.open('x') as log:
            while True:
                row = exchange(port, 'LIFT')
                timestamp = clock()
                try:
                    sample = sweep.add(row, timestamp)
                except ValueError as error:
                    record['last_rejected_sample'] = dict(time_monotonic=timestamp, motor=row, error=str(error))
                    save(path, record)
                    sweep.note_read_failure(row, timestamp)
                    log.write(json.dumps(dict(event='read_failure', time_monotonic=timestamp, motor=row))+'\n')
                    log.flush()
                    print('READ_RETRY: missing reply; checking again within original continuity limits', flush=True)
                    continue
                log.write(json.dumps(sample)+'\n'); log.flush()
                if sweep.count == 1:
                    print('READY: lift torque OFF verified; read-only capture running. low / high / return / finish / abort', flush=True)
                action = control()
                if action == 'abort':
                    raise RuntimeError('capture cancelled')
                if action == 'finish':
                    try:
                        summary = sweep.finish()
                    except ValueError as error:
                        print('MARK_REJECTED: '+str(error), flush=True)
                    else:
                        record.update(summary, status='captured_pending_height_measurement')
                        break
                elif action:
                    try:
                        mark = sweep.mark(action)
                    except ValueError as error:
                        print('MARK_REJECTED: '+str(error), flush=True)
                    else:
                        record['marks'] = dict(sweep.marks)
                        save(path, record)
                        print('MARKED '+action+': '+json.dumps(mark), flush=True)
                if clock()-started > 1200:
                    raise TimeoutError('capture exceeded 20 minutes')
                sleep(.05)
        record['samples_sha256'] = hashlib.sha256(trace.read_bytes()).hexdigest()
        save(path, record)
        return record
    except BaseException as error:
        record.update(status='incomplete_do_not_use', error=str(error), sample_count=sweep.count)
        save(path, record)
        raise


def finalize(source, output, low_mm, high_mm, reference):
    """Recheck every trace row and marked mean before deriving a measured scale."""
    if output.exists():
        raise FileExistsError('use a new calibration path')
    if not all(math.isfinite(v) for v in (low_mm, high_mm)) or not 0 <= low_mm < high_mm <= 3000:
        raise ValueError('finite platform heights required: 0 <= low < high <= 3000 mm')
    if high_mm-low_mm < 10 or not reference.strip():
        raise ValueError('record a fixed height reference and at least 10 mm of measured travel')
    record = json.loads(source.read_text())
    trace = source.with_suffix('.jsonl').read_bytes()
    if (record.get('record_kind') != 'hand_guided_lift_range' or
            record.get('status') != 'captured_pending_height_measurement' or
            record.get('motor_write_commands') != 0 or
            record.get('samples_sha256') != hashlib.sha256(trace).hexdigest()):
        raise ValueError('completed capture and matching trace required')
    sweep = LiftSweep()
    marks = record['marks']
    for line in trace.splitlines():
        row = json.loads(line)
        if row.get('event') == 'read_failure':
            sweep.note_read_failure(row['motor'], row['time_monotonic'])
            continue
        sample = sweep.add(row['motor'], row['time_monotonic'])
        if sample['relative_encoder_raw'] != row['relative_encoder_raw']:
            raise ValueError('trace encoder mismatch')
        for label in ('low', 'high', 'return'):
            if label in marks and marks[label]['sample_number'] == sweep.count:
                if sweep.mark(label) != marks[label]:
                    raise ValueError('mark does not match stable trace')
    summary = sweep.finish()
    if any(summary[k] != record[k] for k in summary):
        raise ValueError('capture summary mismatch')
    travel_um = round((high_mm-low_mm)*1000)
    scale = travel_um*4096/abs(summary['encoder_delta_raw'])
    if not 0 < scale <= 10_000_000:
        raise ValueError('height scale outside controller range')
    result = dict(schema_version=1, record_kind='measured_lift_geometry',
                  status='measured_pending_runtime_commissioning', hardware_enabled=False,
                  runtime_profile_authorized=False, physical_motion_test_verified=False,
                  source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  samples_sha256=record['samples_sha256'], motor_id=10, bus='left',
                  encoder_up_direction=summary['encoder_up_direction'],
                  um_per_turn=round(scale), measured_um_per_turn=scale,
                  maximum_height_um=travel_um, minimum_height_um=0,
                  low_platform_height_mm=low_mm, high_platform_height_mm=high_mm,
                  height_reference=reference, origin='operator_marked_low_position',
                  origin_persists_after_restart=False, automatic_homing_verified=False,
                  encoder_delta_raw=summary['encoder_delta_raw'],
                  return_error_um=(summary['return_error_raw']*scale/4096
                                   if summary['return_error_raw'] is not None else None))
    output.parent.mkdir(parents=True, exist_ok=True)
    save(output, result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    scan = commands.add_parser('capture')
    scan.add_argument('--port', required=True)
    scan.add_argument('--output', type=Path, required=True)
    convert = commands.add_parser('finalize')
    convert.add_argument('--capture', type=Path, required=True)
    convert.add_argument('--output', type=Path, required=True)
    convert.add_argument('--low-height-mm', type=float, required=True)
    convert.add_argument('--high-height-mm', type=float, required=True)
    convert.add_argument('--reference', required=True)
    args = parser.parse_args(argv)
    if args.command == 'finalize':
        result = finalize(args.capture, args.output, args.low_height_mm, args.high_height_mm, args.reference)
    else:
        import serial
        from so101_arm_bridge.backend_lease import acquire_backend_lease
        from so101_arm_bridge.serial_port import open_exclusive_serial
        with acquire_backend_lease('stm32', 0), open_exclusive_serial(serial, args.port, 921600, .1) as port:
            port.reset_input_buffer()
            result = capture(port, args.output)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
