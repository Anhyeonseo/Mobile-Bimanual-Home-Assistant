"""Read-only: show which arm joints are inside the recorded range. No torque, no motion.

Uses the same inventory reads the stationary capture performs before it starts.
Run on the Jetson from inside the installed release folder.
"""
import importlib.util
from pathlib import Path
spec = importlib.util.spec_from_file_location('sp', Path.cwd()/'tools/run/stationary_pick.py')
sp = importlib.util.module_from_spec(spec); spec.loader.exec_module(sp)
import serial
from so101_arm_bridge.limit_manifest import JOINTS
ranges = sp.read('arm_ranges.development.json')
bad = 0
with sp.acquire_backend_lease('stm32', 87), sp.open_exclusive_serial(serial, sp.DEVICE, 921600, .02) as port:
    t = sp.StreamValidationTransportV2(port, response_timeout_s=.12)
    sp.identity(t)
    for bus, side in enumerate(('left', 'right')):
        for i, name in enumerate(JOINTS):
            raw = t.read_device_inventory(bus, i + 1)['feedback']['position_raw']
            e = ranges['arms'][side][name]
            lo, hi = e['minimum_unwrapped_raw'], e['maximum_unwrapped_raw']
            vals = [raw + k * 4096 for k in (-1, 0, 1)]
            if sum(lo <= v <= hi for v in vals) == 1:
                v = next(v for v in vals if lo <= v <= hi)
                margin = min(v - lo, hi - v) * 360 / 4096
                print(f'OK    {side:5s} {name:12s} raw={raw:4d}  edge margin {margin:5.1f} deg')
            else:
                off = min(min(abs(v - lo), abs(v - hi)) for v in vals) * 360 / 4096
                bad += 1
                print(f'OUT   {side:5s} {name:12s} raw={raw:4d}  allowed {lo}..{hi}  about {off:.0f} deg outside')
print('ALL IN RANGE - shot will work' if not bad else f'{bad} joint(s) out of range')
