#!/usr/bin/env python3
"""Bounded LDS-01 acquisition, reusing the commissioning packet inspection.

Only --start sends sensor commands b/e. No robot controller is opened.
Do not call Serial.flush(): write_timeout does not bound POSIX tcdrain.
References: ROBOTIS LDS_Basic_Specification.pdf, data information page 5.
"""
import argparse
import bisect
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import stat
import statistics
import struct
import subprocess
import time


def collect(stream, seconds, *, start_sensor=False, clock=time.monotonic):
    data = bytearray()
    chunks = []
    result = {'start_command_written': False, 'stop_command_written': False,
              'error': None, 'stop_error': None, 'robot_motion_commands': 0}
    start = clock()
    try:
        if start_sensor:
            if stream.write(b'b') != 1:
                raise OSError('incomplete sensor start write')
            result['start_command_written'] = True
        while clock() - start < seconds:
            block = stream.read(max(1, min(stream.in_waiting, 8192)))
            if block:
                data.extend(block)
                chunks.append([clock() - start, len(data)])
    except Exception as error:
        result['error'] = str(error)
    finally:
        if start_sensor:
            try:
                if stream.write(b'e') != 1:
                    raise OSError('incomplete sensor stop write')
                result['stop_command_written'] = True
            except Exception as error:
                result['stop_error'] = str(error)
    result['duration_s'] = clock() - start
    # A successful write is not a physical confirmation of spindle stopping.
    result['physical_stop_verified'] = False
    return bytes(data), chunks, result


def inspect_capture(data, chunks):
    sectors, scans, current = [], [], []
    i, discard = 0, 0
    while i + 42 <= len(data):
        if data[i] != 250 or not 160 <= data[i + 1] <= 219:
            i += 1
            discard += 1
            continue
        packet = bytes(data[i:i + 42])
        checksum = (255 - sum(packet[:40])) & 255
        sectors.append((i, packet, checksum in packet[40:42]))
        i += 42
    seq = [p[1] - 160 for _, p, _ in sectors]
    errors = [dict(packet=n, previous=a, next=b)
              for n, (a, b) in enumerate(zip(seq, seq[1:]), 1) if b != (a + 1) % 60]
    ends = [c[1] for c in chunks]
    starts = [chunks[bisect.bisect_right(ends, offset)][0]
              for offset, p, ok in sectors if p[1] == 160 and ok]
    gaps = [b - a for a, b in zip(starts, starts[1:])]
    for _, packet, ok in sectors:
        n = packet[1] - 160
        if n == 0:
            current = []
        if ok and n == len(current):
            current.append(packet)
        else:
            current = []
        if len(current) == 60:
            ranges = [struct.unpack_from('<H', p, j + 2)[0] / 1000
                      for p in current for j in range(4, 40, 6)]
            scans.append(ranges)
            current = []
    report = dict(bytes=len(data), raw_sha256=hashlib.sha256(data).hexdigest(),
                  sectors=len(sectors), discarded_bytes=discard,
                  checksum_failed_sectors=sum(not ok for _, _, ok in sectors),
                  sequence_errors=errors, complete_scans=len(scans),
                  scan_starts=len(starts), scan_gaps_over_035s=sum(g > .35 for g in gaps),
                  range_accuracy_validated=False, mount_calibrated=False,
                  motion_authorized=False)
    first_start = next((n for n, sector in enumerate(seq) if sector == 0), len(seq))
    report['steady_sequence_error_count'] = sum(e['packet'] > first_start for e in errors)
    if gaps:
        report['scan_start_interval_s'] = dict(min=min(gaps), median=statistics.median(gaps), max=max(gaps))
    if sectors:
        report['rpm_median'] = statistics.median(struct.unpack_from('<H', p, 2)[0] / 10 for _, p, _ in sectors)
    if scans:
        counts = [sum(.12 <= v <= 3.5 for v in scan) for scan in scans]
        report['valid_points'] = dict(min=min(counts), median=statistics.median(counts), max=max(counts))
    return report, scans[-1] if scans else None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=20)
    p.add_argument('--start', action='store_true', help='start sensor, then send pause on exit')
    a = p.parse_args()
    if not 1 <= a.seconds <= 60:
        p.error('seconds must be within 1..60')
    import serial
    from serial.tools import list_ports
    port = Path(a.port).resolve(strict=True)
    if not stat.S_ISCHR(port.stat().st_mode):
        p.error('character device required')
    matches = [item for item in list_ports.comports() if Path(item.device).resolve() == port]
    if len(matches) != 1 or (matches[0].vid, matches[0].pid) != (0x10c4, 0xea60):
        p.error('expected the identified CP210x adapter; no other device will be opened')
    busy = subprocess.run(['fuser', str(port)], capture_output=True, text=True, timeout=3)
    if busy.returncode != 1 or busy.stderr.strip():
        p.error('could not exclude another port owner: ' + busy.stdout + busy.stderr)
    a.output.mkdir(parents=True, exist_ok=False)
    with serial.Serial(str(port), 230400, timeout=.05, write_timeout=1,
                       xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True) as stream:
        settings = stream.get_settings()
        data, chunks, execution = collect(stream, a.seconds, start_sensor=a.start)
        # Allow the one-byte stop write to progress without an unbounded drain.
        if execution['stop_command_written']:
            time.sleep(.1)
    report, last = inspect_capture(data, chunks)
    report.update(execution, captured_at_utc=datetime.now(timezone.utc).isoformat(),
                  port=str(port), settings=settings, mode='active_sensor' if a.start else 'passive')
    (a.output/'raw.bin').write_bytes(data)
    (a.output/'capture.json').write_text(json.dumps(dict(chunks=chunks, duration_s=execution['duration_s']))+'\n')
    (a.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    if last is not None:
        (a.output/'last_scan.json').write_text(json.dumps(dict(ranges_sensor_order_m=last, robot_transform=None))+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['complete_scans'] and not execution['error'] and not execution['stop_error'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
