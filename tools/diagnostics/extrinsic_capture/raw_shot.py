"""Read-only calibration shot: raw joint encoders + one RGB-D frame. No torque, no motion.

Skips the recorded-range gate of stationary_pick.py capture. It uses only the
inventory reads that capture itself performs before that gate, and never
starts the idle reader (the part that can latch the controller).
Joint raw values are read before and after the camera frame; the shot is
rejected if any arm joint moved in between.

Run on the Jetson from inside the installed release folder:
    python raw_shot.py <new output folder> <lift height m>
"""
import importlib.util, json, sys, time
from pathlib import Path
spec = importlib.util.spec_from_file_location('sp', Path.cwd()/'tools/run/stationary_pick.py')
sp = importlib.util.module_from_spec(spec); spec.loader.exec_module(sp)
import serial
from PIL import Image
from so101_arm_bridge.limit_manifest import JOINTS

out, lift_m = Path(sys.argv[1]), float(sys.argv[2])
if out.exists():
    sys.exit('already exists: ' + str(out))
MAX_MOVE_RAW = 3  # about 0.26 deg


def read_all(t):
    rows = {}
    for bus, side in enumerate(('left', 'right')):
        for i, name in enumerate(JOINTS):
            row = t.read_device_inventory(bus, i + 1)
            if row['command']['torque_enabled_raw'] != 0:
                raise SystemExit(f'REJECTED: torque is on at {side}/{name}; this tool only reads torque-off arms')
            rows[f'{side}/{name}'] = row
    rows['lift/id10'] = t.read_device_inventory(0, 10)
    return rows


raw = lambda rows: {k: r['feedback']['position_raw'] for k, r in rows.items()}
with sp.acquire_backend_lease('stm32', 87), sp.open_exclusive_serial(serial, sp.DEVICE, 921600, .02) as port:
    t = sp.StreamValidationTransportV2(port, response_timeout_s=.12)
    hello = sp.identity(t)
    before_t = time.time(); before = read_all(t)
    with sp.Capture(sp.camera_profile(), sp.read('camera_device.near.observed.json'), retain_frame=True) as camera:
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            record, images = camera.next_frame()
    after = read_all(t); after_t = time.time()
moved = {k: (raw(before)[k], raw(after)[k]) for k in raw(before) if not k.startswith('lift')
         and min(abs(raw(before)[k] - raw(after)[k]), 4096 - abs(raw(before)[k] - raw(after)[k])) > MAX_MOVE_RAW}
if moved:
    sys.exit(f'REJECTED: arm moved during shot {moved} - hold still and retry')
out.mkdir(parents=True)
Image.fromarray(images[0][:, :, ::-1]).save(out/'color.png')
Image.fromarray(images[1]).save(out/'depth_aligned.png')
(out/'metadata.json').write_text(json.dumps(images[2], indent=2))
(out/'joints.json').write_text(json.dumps(dict(
    kind='raw_calibration_shot', lift_m_operator=lift_m, raw_before=raw(before), raw_after=raw(after),
    read_window_utc_s=[before_t, after_t], camera_record=record, controller=hello,
    inventory_before=before, inventory_after=after, motor_write_commands=0), indent=2, default=str))
print('saved', out, '| left raw', [raw(after)['left/' + j] for j in JOINTS], '| lift raw', raw(after)['lift/id10'])
