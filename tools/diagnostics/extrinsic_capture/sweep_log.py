"""Hand-guided free-space sweep for the automatic calibration tour. Read-only: torque stays off.

Both arms are torque-off. A person moves ONE arm slowly through every place the automatic tour may
later use (start from its folded rest pose, reach out in front of the camera, sweep left/right,
up/down, bend and roll the wrist, come back to rest), while this tool logs all 12 joint encoders
and the lift encoder at ~25 Hz. The other arm stays at its rest pose. The PC planner
(analysis/plan_tour.py) later moves the arm only along configurations recorded here.

With --camera it also saves a synchronized photo whenever the swept arm has been still for 0.7 s
and has moved at least 5 deg since the previous photo (same folder format as raw_shot.py), so the
sweep itself yields calibration shots.

The idle reader latches the controller if an arm leaves the recorded joint ranges. Watch the
margin line; at 5 deg or less it prints WARN. A latch is harmless here (torque is already off) but
needs an STM32 reset.

Run on the Jetson from inside the installed release folder:
    python sweep_log.py OUT_DIR --arm left --lift-m 0.20 [--camera --attachment TL1]
Stop with Ctrl+C. Writes OUT_DIR/sweep.jsonl and, with --camera, OUT_DIR/<shot name>/.
"""
import argparse, importlib.util, json, sys, time
from collections import deque
from pathlib import Path
spec = importlib.util.spec_from_file_location('sp', Path.cwd()/'tools/run/stationary_pick.py')
sp = importlib.util.module_from_spec(spec); spec.loader.exec_module(sp)
import serial
from so101_arm_bridge.limit_manifest import JOINTS
from so101_arm_bridge.servo_position import raw_from_wire

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('out', type=Path)
ap.add_argument('--arm', choices=('left', 'right'), required=True, help='the arm being swept')
ap.add_argument('--lift-m', type=float, required=True, help='operator-established lift height')
ap.add_argument('--camera', action='store_true', help='also save still photos')
ap.add_argument('--attachment', default='', help='marker attachment name for --camera photos')
args = ap.parse_args()
if args.camera and not args.attachment:
    sys.exit('--camera needs --attachment (a new name each time the marker is re-taped)')
out = args.out
out.mkdir(parents=True, exist_ok=False)
RANGES = sp.read('arm_ranges.development.json')
DIR = {(a, j): RANGES['arms'][a][j]['positive_raw_direction'] for a in ('left', 'right') for j in JOINTS}
KEYS = [f'{a}/{j}' for a in ('left', 'right') for j in JOINTS]
STILL_S, STILL_RAW, MOVE_DEG, MAX_MOVE_RAW = 0.7, 4, 5.0, 3


def raw_of(exposure):
    return {k: raw_from_wire(int(round(r * 1e6)), DIR[tuple(k.split('/'))]) for k, r in zip(KEYS, exposure.positions_rad)}


def margin_deg(raw):
    worst = (999.0, '')
    for j in JOINTS[:5]:
        e = RANGES['arms'][args.arm][j]; v = raw[f'{args.arm}/{j}']
        m = min(v - e['minimum_unwrapped_raw'], e['maximum_unwrapped_raw'] - v) * 360 / 4096
        worst = min(worst, (m, j))
    return worst


def read_joints(t):
    for _ in range(20):
        begin = time.monotonic(); packet = t.get_feedback_snapshot(); end = time.monotonic()
        try:
            return sp.joint_exposure(packet, begin, end)
        except ValueError:
            time.sleep(.005)
    raise RuntimeError('no fresh joint feedback in 20 tries')


def save_shot(name, record, images, before, after, lift_raw):
    from PIL import Image
    folder = out/name; folder.mkdir()
    Image.fromarray(images[0][:, :, ::-1]).save(folder/'color.png')
    Image.fromarray(images[1]).save(folder/'depth_aligned.png')
    (folder/'metadata.json').write_text(json.dumps(images[2], indent=2))
    b, a = dict(before, **{'lift/id10': lift_raw}), dict(after, **{'lift/id10': lift_raw})
    (folder/'joints.json').write_text(json.dumps(dict(
        kind='sweep_still_shot', lift_m_operator=args.lift_m, raw_before=b, raw_after=a, camera_record=record,
        calibration_group=dict(arm=args.arm, attachment=args.attachment, level=f'{args.lift_m:.2f}', lift_m=args.lift_m,
                               session=out.resolve().parent.name),
        motor_write_commands=0), indent=2, default=str))


log = (out/'sweep.jsonl').open('x')
count = shots = 0; lift_raw = None; started = time.monotonic(); last_print = 0.0
still = deque(); last_shot_raw = None
with sp.acquire_backend_lease('stm32', 87), sp.open_exclusive_serial(serial, sp.DEVICE, 921600, .02) as port:
    t = sp.StreamValidationTransportV2(port, response_timeout_s=.12)
    hello = sp.identity(t)
    inventory = sp.verify_inventory(t, RANGES)      # rejects torque-on arms and poses outside the ranges
    log.write(json.dumps(dict(kind='header', arm=args.arm, lift_m_operator=args.lift_m, controller=hello,
                              started_utc=time.time(), motor_write_commands=0), default=str) + '\n')
    t.stationary_platform(1)
    camera = sp.Capture(sp.camera_profile(), sp.read('camera_device.near.observed.json'), retain_frame=True) if args.camera else None
    try:
        if camera: camera.__enter__()
        print(f'logging {args.arm} arm at lift {args.lift_m:.2f} m - move it slowly; Ctrl+C to stop', flush=True)
        while True:
            frame = camera.next_frame() if camera else None
            j = read_joints(t); raw = raw_of(j); now = time.monotonic()
            if count % 5 == 0:
                lift_raw = t.stationary_platform()['positions_raw'][3]
            log.write(json.dumps(dict(t=round(now - started, 3), raw=raw, lift_raw=lift_raw)) + '\n'); count += 1
            m, joint = margin_deg(raw)
            if now - last_print > .5:
                print(f'\r{count:6d} samples  {shots:3d} photos  range margin {m:5.1f} deg ({joint}){"  WARN" if m <= 5 else "     "}', end='', flush=True)
                last_print = now
            if camera:
                arm_raw = [raw[f'{args.arm}/{q}'] for q in JOINTS[:5]]
                still.append((now, arm_raw))
                while still and now - still[0][0] > STILL_S: still.popleft()
                steady = (still and now - still[0][0] > STILL_S * .9 and
                          max(max(v) - min(v) for v in zip(*[s[1] for s in still])) <= STILL_RAW)
                moved = last_shot_raw is None or max(abs(a - b) for a, b in zip(arm_raw, last_shot_raw)) * 360 / 4096 >= MOVE_DEG
                if steady and moved:
                    record, images = camera.next_frame()
                    after = raw_of(read_joints(t))
                    if max(abs(after[k] - raw[k]) for k in KEYS) <= MAX_MOVE_RAW:
                        shots += 1
                        save_shot(f'S{round(args.lift_m * 100):02d}-{args.arm[0].upper()}-{shots:03d}', record, images, raw, after, lift_raw)
                        last_shot_raw = arm_raw
    except KeyboardInterrupt:
        pass
    finally:
        if camera: camera.__exit__(None, None, None)
        sp.stop_idle_reads(t)
        log.close()
print(f'\nsaved {count} samples, {shots} photos to {out}')
