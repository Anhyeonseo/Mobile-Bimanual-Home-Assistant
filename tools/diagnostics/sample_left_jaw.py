#!/usr/bin/env python3
"""Run from the installed left-grip bench release, including over SSH stdin. Prints JSON."""
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time


def main():
    release=Path.cwd()
    if not release.name.startswith('s3-left-grip-') or not (release/'SHA256SUMS').is_file():
        raise SystemExit('expected the installed left gripper bench release')
    sys.path.insert(0,str(release))
    import serial
    from tools.run.bench_left_gripper import PORT, acquire_backend_lease, open_exclusive_serial, trial
    report=dict(release=release.name,observed_at=datetime.now(timezone.utc).isoformat(),
                mode='inspect',inspections=[])
    try:
        with acquire_backend_lease('stm32',0),open_exclusive_serial(serial,PORT,921600,.25) as port:
            port.reset_input_buffer()
            for _ in range(4):
                record={}
                trial(port,'inspect',record)
                report['inspections'].append(record)
                time.sleep(.08)
    except (ImportError,OSError,ValueError,RuntimeError) as exc:
        report['error']=str(exc)
    print(json.dumps(report,allow_nan=False))
    return 2 if 'error' in report else 0


if __name__=='__main__':
    raise SystemExit(main())
