"""Read-only: show the wheel (ID 7-9) and lift (ID 10) rows that verify_inventory checks.

verify_inventory rejects with 'stationary platform must have no velocity goal' when one of these
rows is incomplete (no reply) or has a non-zero goal speed. This prints which one. No torque, no motion.
Run on the Jetson from inside the installed S3 release folder (source helpers.sh first).
"""
import importlib.util, json
from pathlib import Path
spec = importlib.util.spec_from_file_location('sp', Path.cwd()/'tools/run/stationary_pick.py')
sp = importlib.util.module_from_spec(spec); spec.loader.exec_module(sp)
import serial
NAMES = {7: 'wheel 7', 8: 'wheel 8', 9: 'wheel 9', 10: 'lift 10'}
with sp.acquire_backend_lease('stm32', 87), sp.open_exclusive_serial(serial, sp.DEVICE, 921600, .02) as port:
    t = sp.StreamValidationTransportV2(port, response_timeout_s=.12)
    sp.identity(t)
    for motor in range(7, 11):
        row = t.read_device_inventory(0, motor)
        if row is None:
            print(f'{NAMES[motor]:8s} NO REPLY'); continue
        cmd = row.get('command') or {}; fb = row.get('feedback') or {}
        ok = row.get('complete') and cmd.get('goal_speed_raw') == 0
        print(f'{NAMES[motor]:8s} {"OK " if ok else "BAD"} complete={row.get("complete")} goal_speed_raw={cmd.get("goal_speed_raw")} '
              f'torque={cmd.get("torque_enabled_raw")} position_raw={fb.get("position_raw")}')
        if not ok:
            print('         ' + json.dumps(row, default=str)[:400])
