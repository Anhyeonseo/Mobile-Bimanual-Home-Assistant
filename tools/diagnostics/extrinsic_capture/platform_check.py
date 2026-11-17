"""Read-only: show every row that verify_inventory checks, and which one fails.

Arms (both buses, ID 1-6): complete read, calibration offset as recorded, operating mode 0, torque off.
Platform (ID 7-9 wheels, 10 lift): complete read, zero goal speed.
verify_inventory stops at the first failing row with a short message; this prints all of them.
No torque, no motion. Run on the Jetson from inside the installed S3 release folder (source helpers.sh first).
"""
import importlib.util, json
from pathlib import Path
spec = importlib.util.spec_from_file_location('sp', Path.cwd()/'tools/run/stationary_pick.py')
sp = importlib.util.module_from_spec(spec); spec.loader.exec_module(sp)
import serial
from so101_arm_bridge.limit_manifest import JOINTS
RANGES = sp.read('arm_ranges.development.json')
NAMES = {7: 'wheel 7', 8: 'wheel 8', 9: 'wheel 9', 10: 'lift 10'}
with sp.acquire_backend_lease('stm32', 87), sp.open_exclusive_serial(serial, sp.DEVICE, 921600, .02) as port:
    t = sp.StreamValidationTransportV2(port, response_timeout_s=.12)
    try:
        sp.identity(t)
    except ValueError as exc:
        if 'stop_latched=True' not in str(exc): raise
        print('note: controller stop is latched (normal after a tour); reading anyway\n')
    for bus, side in enumerate(('left', 'right')):
        for i, name in enumerate(JOINTS):
            row = t.read_device_inventory(bus, i + 1)
            if row is None or not row.get('complete'):
                print(f'{side:5s} {name:11s} BAD  no complete reply  ' + json.dumps(row, default=str)[:200]); continue
            om, cmd = row['offset_and_mode'], row['command']
            want = RANGES['calibration'][side]['offsets_raw'][i]
            bad = [f'offset {om["offset_raw"]}!={want}'] if om['offset_raw'] != want else []
            if om['operating_mode'] != 0: bad.append(f'mode {om["operating_mode"]}')
            if cmd['torque_enabled_raw'] != 0: bad.append('TORQUE ON')
            print(f'{side:5s} {name:11s} {"BAD  " + ", ".join(bad) if bad else "OK"}  position_raw={row["feedback"]["position_raw"]}')
    for motor in range(7, 11):
        row = t.read_device_inventory(0, motor)
        if row is None:
            print(f'{NAMES[motor]:17s} NO REPLY'); continue
        cmd = row.get('command') or {}; fb = row.get('feedback') or {}
        ok = row.get('complete') and cmd.get('goal_speed_raw') == 0
        print(f'{NAMES[motor]:17s} {"OK " if ok else "BAD"} complete={row.get("complete")} goal_speed_raw={cmd.get("goal_speed_raw")} '
              f'torque={cmd.get("torque_enabled_raw")} position_raw={fb.get("position_raw")}')
