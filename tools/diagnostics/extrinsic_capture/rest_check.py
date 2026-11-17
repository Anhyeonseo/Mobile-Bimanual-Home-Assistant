"""Read-only live guide: how far each arm joint is from the tour's rest pose. No torque, no motion.

calib_tour.py refuses to start when any joint is more than 5 deg from tour['rest_raw'].
Move the arms by hand (torque off) until every line says OK (within 3 deg), then support/tape them.
Run on the Jetson from inside the installed S3 release folder:  python rest_check.py TOUR.json   (Ctrl+C to stop)
"""
import importlib.util, json, sys, time
from pathlib import Path
spec = importlib.util.spec_from_file_location('sp', Path.cwd()/'tools/run/stationary_pick.py')
sp = importlib.util.module_from_spec(spec); spec.loader.exec_module(sp)
import serial
from so101_arm_bridge.limit_manifest import JOINTS
rest = json.load(open(sys.argv[1]))['rest_raw']
with sp.acquire_backend_lease('stm32', 87), sp.open_exclusive_serial(serial, sp.DEVICE, 921600, .02) as port:
    t = sp.StreamValidationTransportV2(port, response_timeout_s=.12)
    sp.identity(t)
    try:
        while True:
            lines, worst = [], 0.0
            for bus, side in enumerate(('left', 'right')):
                for i, name in enumerate(JOINTS):
                    row = t.read_device_inventory(bus, i + 1)
                    if row is None or not row.get('complete'):
                        lines.append(f'{side:5s} {name:11s} no reply'); worst = 99; continue
                    raw = row['feedback']['position_raw']; d = (raw - rest[side][name] + 2048) % 4096 - 2048
                    deg = d * 360 / 4096; worst = max(worst, abs(deg))
                    hint = '' if abs(deg) <= 3 else ('  move so the number goes DOWN' if d > 0 else '  move so the number goes UP')
                    lines.append(f'{side:5s} {name:11s} now {raw:4d}  target {rest[side][name]:4d}  off {deg:+6.1f} deg  '
                                 f'{"OK" if abs(deg) <= 3 else "--"}{hint}')
            print('\033[2J\033[H' + '\n'.join(lines) + f'\n\nworst {worst:.1f} deg  ->  '
                  + ('READY: all within 3 deg (tour needs <= 5)' if worst <= 3 else 'not yet') + '   (Ctrl+C to stop)', flush=True)
            time.sleep(.3)
    except KeyboardInterrupt:
        pass
