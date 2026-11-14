"""Automatic arm-marker calibration tour on the Jetson. THIS TOOL MOVES THE ARMS.

It executes one lift height of a tour planned on the PC (analysis/plan_tour.py) from hand-guided
sweeps (sweep_log.py): the arms only travel through space a person already swept them through,
at <= the planned peak joint speed, one arm at a time while the other holds its rest pose.
At each target it settles, checks the arm is still, and saves a photo + measured joints in the
same folder format as raw_shot.py (plus the commanded pose and the approach direction).

Safety model, read before running:
- No full-robot collision model exists. Safety rests on the sweep: same lift height, same
  surroundings, reachable area clear. The tour refuses to start unless the arms are at rest.
- The firmware safety stop DISABLES ARM TORQUE. Ctrl+C (or any fault) stops immediately and both
  arms fall. Keep a hand near the moving arm; support both arms before any intended stop.
- 'q' at a prompt returns along a pre-checked path to rest first, then asks before turning
  torque off. While armed the tool keeps sending the controller heartbeat during prompts.

Run on the Jetson from inside the installed release folder, lift already at the tour height:
    python calib_tour.py TOUR.json OUT_DIR --lift-m 0.20 --left-attachment TL1 --right-attachment TR1
Rehearse without hardware (Jetson: from the release folder; PC: from the repository root):
    python tools/diagnostics/extrinsic_capture/calib_tour.py TOUR.json OUT_DIR --lift-m 0.20 \
        --left-attachment X --right-attachment Y --simulate [--max-steps 6]
"""
import argparse, hashlib, json, select, signal, sys, time, types
from pathlib import Path
import numpy as np

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('tour', type=Path); ap.add_argument('out', type=Path)
ap.add_argument('--lift-m', type=float, required=True, help='lift height the operator established')
ap.add_argument('--left-attachment', required=True); ap.add_argument('--right-attachment', required=True)
ap.add_argument('--auto', action='store_true', help='do not pause before each move (still pauses at start/end)')
ap.add_argument('--simulate', action='store_true', help='in-memory controller, no camera, no hardware')
ap.add_argument('--max-steps', type=int, default=0, help='simulate: stop after this many tour steps')
args = ap.parse_args()

tour_bytes = args.tour.read_bytes(); TOUR_SHA = hashlib.sha256(tour_bytes).hexdigest()
tour = json.loads(tour_bytes)
if tour.get('kind') != 'extrinsic_calibration_tour' or tour.get('schema_version') != 1:
    sys.exit('not a calibration tour file')
level = next((l for l in tour['levels'] if abs(l['lift_m'] - args.lift_m) < 1e-6), None)
if level is None:
    sys.exit(f'tour has no lift {args.lift_m}; available: {[l["lift_m"] for l in tour["levels"]]}')

if args.simulate:
    # the installed release folder (cwd on the Jetson) or the repository this file lives in
    ROOT = Path.cwd() if (Path.cwd() / 'ros2_ws/src/so101_arm_bridge').is_dir() else Path(__file__).resolve().parents[3]
    sys.path[:0] = [str(ROOT / 'ros2_ws/src/so101_arm_bridge'), str(ROOT / 'ros2_ws/src/home_robot_tasks')]
    try:
        import serial  # noqa: F401
    except ModuleNotFoundError:
        sys.modules['serial'] = types.SimpleNamespace(SerialTimeoutException=type('SerialTimeoutException', (Exception,), {}))
    CONFIG = ROOT / 'config/home_assistant'; sp = None
else:
    import importlib.util
    spec = importlib.util.spec_from_file_location('sp', Path.cwd() / 'tools/run/stationary_pick.py')
    sp = importlib.util.module_from_spec(spec); spec.loader.exec_module(sp)
    import serial
    CONFIG = sp.CONFIG
from so101_arm_bridge.stationary_pick_adapter import StationaryPickAdapter
from so101_arm_bridge.bimanual_stream_adapter import (OperationalLimits, CANONICAL_JOINT_NAMES, TimedJointPoint,
                                                       BimanualTransientFeedbackError)
from so101_arm_bridge.development_ranges import read_development_ranges, calibration_hash
from so101_arm_bridge.stationary_runtime import ResidentStop
from so101_arm_bridge.servo_position import raw_from_wire
from so101_arm_bridge.limit_manifest import JOINTS

RANGES_PATH = CONFIG / 'arm_ranges.development.json'
if hashlib.sha256(RANGES_PATH.read_bytes()).hexdigest() != tour['arm_ranges_sha256']:
    sys.exit('arm_ranges.development.json differs from the one the tour was planned with; re-plan')
RANGES_DOC, ENTRIES, CHECKSUM = read_development_ranges(RANGES_PATH)
DIRS = tuple(e['positive_raw_direction'] for e in ENTRIES)
KEYS = [f'{a}/{j}' for a in ('left', 'right') for j in JOINTS]
OWNER = 'calib-tour'
ATT = {'left': args.left_attachment, 'right': args.right_attachment}
MAX_MOVE_RAW, SETTLE_S, START_TOLERANCE_DEG = 3, 1.0, 5.0


# ---------------------------------------------------------------- simulated controller
class SimTransport:
    """In-memory stand-in for the S3 resident: follows appended samples in real time. Rehearsal only."""
    simulated = True

    def __init__(self, start_urad):
        from so101_arm_bridge.stream_protocol_v2 import (StreamExecutorStateV2, StreamStatusCodeV2,
                                                         StreamContractResultV2, ARM_MASK_BOTH)
        self.E, self.S, self.C, self.mask = StreamExecutorStateV2, StreamStatusCodeV2, StreamContractResultV2, ARM_MASK_BOTH
        self.position = tuple(int(x) for x in start_urad); self.samples = []; self.horizon = None
        self.stopped = False; self.torque = False; self.seq = 0

    tick = staticmethod(lambda: int(time.monotonic() * 1000) & 0xFFFFFFFF)
    ns = staticmethod(lambda **k: types.SimpleNamespace(**k))

    def _advance(self):
        now = self.tick()
        while self.samples and ((now - self.samples[0][0]) & 0xFFFFFFFF) < 0x80000000:
            self.position = self.samples.pop(0)[1]

    def enter_binary_mode(self):
        h = calibration_hash(CHECKSUM)
        return self.ns(protocol_version=2, joint_count=12, stop_latched=self.stopped, firmware_version=StationaryPickAdapter.firmware_version,
                       left_calibration_hash=h, right_calibration_hash=h, capabilities=StationaryPickAdapter.capabilities)

    def heartbeat(self): return self.ns(stop_latched=self.stopped, status_code=0, last_heartbeat_ms=self.tick())
    get_state = heartbeat
    def get_dispatch_diagnostics(self): return self.ns(status_code=0, active=False, faulted=False, ready=True, failure_count=0)
    def get_tracking_diagnostics(self): return self.ns(status_code=0, active=False, pending=False, failed_pairs=0)

    def prepare_shadow(self):
        raw = tuple(raw_from_wire(p, d) for p, d in zip(self.position, DIRS))
        return self.ns(status_code=0, joint_count=12, left_present_mask=0x3F, right_present_mask=0x3F,
                       positions_raw=raw, unwrapped_positions_raw=raw, anchor_positions_urad=self.position)

    def arm(self, _hash): pass
    def enable(self): self.torque = True; return self.ns(stop_latched=False, status_code=0)
    def _ok(self, epoch=0): return self.ns(status_code=self.S.OK, contract_result=self.C.OK, arm_mask=self.mask, arbiter_epoch=epoch)
    def open_stream(self, policy): self.horizon = policy.horizon_end_tick or None; return self._ok()

    def append(self, batch):
        self._advance(); self.samples += [(s.apply_tick, tuple(s.positions_urad)) for s in batch.samples]
        return self._ok(batch.arbiter_epoch)

    def get_executor_diagnostics(self):
        self._advance()
        done = self.horizon is not None and not self.samples and ((self.tick() - self.horizon) & 0xFFFFFFFF) < 0x80000000
        return self.ns(state=self.E.SUCCEEDED if done else self.E.RUNNING, terminal_reason=0, safe_stop_required=False)

    def get_feedback_snapshot(self):
        self._advance()
        return self.ns(status_code=0, joint_count=12, present_mask=0x0FFF, firmware_tick_ms=self.tick(), completed_pairs=12,
                       positions_urad=self.position, sample_age_ms=(5,) * 12)

    def safe_stop(self): self.stopped = True; self.torque = False; self.samples = []; return self.ns(stop_latched=True, status_code=0)
    def stationary_runtime(self): return dict(stop_latched=self.stopped, ready_for_stream=True)
    def stationary_feedback_failures(self): return {}
    def write_diagnostics(self): return {}

    def stationary_platform(self, operation=0):
        self.seq += 1
        return dict(active=True, present_mask=15, firmware_tick_ms=self.tick(), sequence=self.seq,
                    positions_raw=(2048, 2048, 2048, 3000), sample_age_ms=(5, 5, 5, 5))

    def read_device_inventory(self, bus, motor):
        index = bus * 6 + motor - 1
        position = raw_from_wire(self.position[index], DIRS[index]) % 4096 if motor <= 6 else 2048
        return dict(complete=True, command=dict(torque_enabled_raw=int(self.torque), goal_speed_raw=0),
                    feedback=dict(position_raw=position))


# ---------------------------------------------------------------- helpers
emergency = []
def on_signal(*_): emergency.append('operator interrupt (Ctrl+C / signal)')
for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP): signal.signal(s, on_signal)


class Abort(Exception): pass


def check_emergency():
    if emergency: raise Abort(emergency[0])


def raw_of(positions_urad):
    return {k: raw_from_wire(int(p), d) for k, p, d in zip(KEYS, positions_urad, DIRS)}


class Keys:
    """Unbuffered line reader on fd 0. Python's buffered stdin can swallow typed lines that select()
    then never reports, which would stall a prompt; every prompt here reads through this one queue."""

    def __init__(self): self.buffer, self.lines = b'', []

    def poll(self, timeout):
        import os
        if not self.lines and select.select([0], [], [], timeout)[0]:
            chunk = os.read(0, 1024)
            if chunk == b'': raise Abort('stdin closed')
            self.buffer += chunk
            *done, self.buffer = self.buffer.split(b'\n')
            self.lines += [d.decode(errors='replace').strip().lower() for d in done]
        return self.lines.pop(0) if self.lines else None


keys = Keys()


def wait_key(prompt, adapter=None, camera=None):
    """Prompt and wait for a line WITHOUT blocking the controller heartbeat or the camera stream."""
    print(prompt, flush=True)
    while True:
        check_emergency()
        if adapter is not None: adapter.keepalive()
        if camera is not None: camera.next_frame()
        line = keys.poll(0.02)
        if line is not None: return line


def hold(adapter, seconds, camera=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        check_emergency(); adapter.keepalive()
        if camera is not None: camera.next_frame()
        else: time.sleep(0.01)


def execute(adapter, points, label):
    route = tuple(TimedJointPoint(int(t), tuple(float(x) for x in q)) for t, q in points)
    adapter.start(OWNER, route, finite=True)
    deadline = time.monotonic() + route[-1].offset_ms / 1000 + 5.0
    while adapter.state.value != 'ready':
        if emergency: raise Abort(emergency[0] + ' during ' + label)
        if time.monotonic() > deadline: raise Abort('move did not finish in time: ' + label)
        adapter.poll(OWNER)
        time.sleep(0.005)


def measured(adapter):
    for _ in range(10):
        try:
            return raw_of(adapter.feedback_snapshot().positions_urad)
        except BimanualTransientFeedbackError:
            time.sleep(0.01)
    raise Abort('no joint feedback')


def lift_raw(t):
    try:
        return t.stationary_platform()['positions_raw'][3]
    except Exception:
        return None


def capture(step, adapter, t, camera, log):
    hold(adapter, SETTLE_S, camera)
    for attempt in range(3):
        before = measured(adapter)
        record, images = camera.next_frame() if camera is not None else ({'simulated': True}, None)
        after = measured(adapter)
        if max(abs(after[k] - before[k]) for k in KEYS) <= MAX_MOVE_RAW: break
        hold(adapter, 0.5, camera)
    else:
        log.append(dict(step=step['name'], status='NOT_STILL')); print('  ! 팔이 멈추지 않아 건너뜀', flush=True); return
    lift = lift_raw(t); arm = step['arm']
    folder = args.out / step['name']; folder.mkdir()
    if images is not None:
        from PIL import Image
        Image.fromarray(images[0][:, :, ::-1]).save(folder / 'color.png')
        Image.fromarray(images[1]).save(folder / 'depth_aligned.png')
        (folder / 'metadata.json').write_text(json.dumps(images[2], indent=2))
    err = {j: after[f'{arm}/{j}'] - step['target_raw'][j] for j in JOINTS[:5]}
    (folder / 'joints.json').write_text(json.dumps(dict(
        kind='tour_shot', simulated=args.simulate, lift_m_operator=args.lift_m,
        raw_before={**before, 'lift/id10': lift}, raw_after={**after, 'lift/id10': lift},
        commanded_raw=step['target_raw'], settle_error_raw=err, approach_from=step['approach_from'],
        repeat_of=step['base_name'] if step['name'] != step['base_name'] else None,
        calibration_group=dict(arm=arm, attachment=ATT[arm], level=f'{args.lift_m:.2f}', lift_m=args.lift_m,
                               session=args.out.resolve().parent.name),
        tour_sha256=TOUR_SHA, camera_record=record, motor_write_commands='tour'), indent=2, default=str))
    log.append(dict(step=step['name'], status='SAVED', settle_error_raw=err))
    print(f'  saved {step["name"]}  settle error (raw) {err}', flush=True)


def interpolate(a, b, speed_deg_s=10.0, point_ms=40):
    T = max(1.0, 1.875 * float(np.abs(b - a).max()) / np.radians(speed_deg_s))
    n = int(np.ceil(T * 1000 / point_ms))
    return [[point_ms * (i + 1), (a + (b - a) * (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5)).tolist()]
            for i, u in enumerate(np.linspace(0, 1, n + 1))]


# ---------------------------------------------------------------- main
steps = level['steps'][:args.max_steps] if args.max_steps else level['steps']
moves = [s for s in steps if s['kind'] == 'move']
print(f'tour {args.tour.name} sha256 {TOUR_SHA[:12]}  lift {args.lift_m:.2f} m: '
      f'{sum(s["kind"] == "shot" for s in steps)} shots, {sum(m["duration_s"] for m in moves) / 60:.1f} min of motion'
      f'{"  [SIMULATION]" if args.simulate else ""}')
print('안전 확인: 손으로 훑을 때와 같은 리프트 높이 / 주변 물건 치움 / 사람이 팔 옆에 대기 / Ctrl+C = 즉시 토크 끔(팔이 떨어짐)')
if wait_key('준비됐으면 yes 입력:') != 'yes': sys.exit('cancelled')
args.out.mkdir(parents=True, exist_ok=False)

report = dict(kind='calibration_tour_run', tour_sha256=TOUR_SHA, lift_m=args.lift_m, simulated=args.simulate,
              attachments=ATT, started_utc=time.time(), steps=[])
rest = np.array(tour['rest_wire_rad'])
if args.simulate:
    from contextlib import nullcontext
    link = nullcontext(SimTransport(np.round((rest + np.radians(1.5)) * 1e6)))
else:
    from contextlib import ExitStack
    link = ExitStack()
camera = None; armed = False; stop = None; adapter = None
with link as handle:
    if args.simulate:
        t = handle
    else:
        handle.enter_context(sp.acquire_backend_lease('stm32', 87))
        port = handle.enter_context(sp.open_exclusive_serial(serial, sp.DEVICE, 921600, .02))
        t = sp.StreamValidationTransportV2(port, response_timeout_s=.12)
    try:
        if not args.simulate:
            report['controller'] = sp.identity(t)
            report['initial_inventory'] = sp.verify_inventory(t, RANGES_DOC)     # torque off, inside ranges
        limits = OperationalLimits(CANONICAL_JOINT_NAMES, tuple(e['minimum_urad'] for e in ENTRIES),
                                   tuple(e['maximum_urad'] for e in ENTRIES), CHECKSUM, hardware_authorized=True)
        adapter = StationaryPickAdapter(t, RANGES_PATH, authorized_limits=limits, motion_authorized=True)
        stop = ResidentStop(t)
        anchor = np.array(adapter.prepare().positions_urad) / 1e6
        off = np.degrees(np.abs(anchor - rest)).max()
        if off > START_TOLERANCE_DEG:
            raise Abort(f'arms are {off:.1f} deg away from the tour rest pose; place them at rest by hand first')
        if not args.simulate:
            camera = sp.Capture(sp.camera_profile(), sp.read('camera_device.near.observed.json'), retain_frame=True)
            camera.__enter__()
        if wait_key('Enter = 양팔 토크 켜고 휴식 자세 유지 시작 / q = 취소', adapter, camera) == 'q': raise Abort('cancelled before torque')
        adapter.activate_stationary_hold(); armed = True
        execute(adapter, interpolate(np.array(adapter.prepared_state.positions_urad) / 1e6, rest), 'settle to rest')
        auto, at_home, last_shot = args.auto, True, None
        for step in steps:
            if step['kind'] == 'move':
                if not auto:
                    key = wait_key(f'다음: {step["arm"]} → {step["to_name"]} ({step["duration_s"]:.1f}s)  '
                                   f'Enter=이동  a=이후 자동  q=휴식 자세로 돌아가 종료', adapter, camera)
                    if key == 'a': auto = True
                    if key == 'q':
                        if not at_home and last_shot is not None:
                            print('  휴식 자세로 복귀 중', flush=True)
                            execute(adapter, last_shot['return_points'], 'return to rest')
                        report['ended_early'] = True; break
                print(f'  이동 {step["arm"]} → {step["to_name"]}', flush=True)
                execute(adapter, step['points'], step['to_name'])
                at_home = step['to_name'] == 'REST'
                report['steps'].append(dict(step=step['to_name'], status='MOVED'))
            else:
                capture(step, adapter, t, camera, report['steps']); last_shot = step
        else:
            if not at_home and last_shot is not None:     # a truncated rehearsal (--max-steps) still goes home
                execute(adapter, last_shot['return_points'], 'return to rest')
        wait_key('휴식 자세 도착. 양팔을 손으로 받친 뒤 Enter = 토크 끄기', adapter, camera)
        report['status'] = 'COMPLETED'
    except Abort as exc:
        report.update(status='ABORTED', reason=str(exc)); print('\n중단:', exc, flush=True)
    except Exception as exc:
        report.update(status='FAULTED', reason=f'{type(exc).__name__}: {exc}'); print('\n오류:', exc, flush=True)
    finally:
        if adapter is not None and (armed or adapter.heartbeat_required or adapter.fault_diagnostic):
            stop.request('cleanup', True, time.monotonic())
            until = time.monotonic() + 3
            while time.monotonic() < until and stop.poll('cleanup', time.monotonic()).status != 'STOPPED': time.sleep(.05)
            report['stop'] = stop.diagnostics()
            print('토크 꺼짐 확인' if stop.status == 'STOPPED' else '정지 미확인: 양팔·리프트를 받치고 모터 전원을 꺼주세요.', flush=True)
        elif not args.simulate:
            try: t.stationary_platform(2)
            except Exception: pass
        if camera is not None: camera.__exit__(None, None, None)
        report['adapter_fault'] = None if adapter is None else adapter.fault_diagnostic
        report['finished_utc'] = time.time()
        (args.out / 'tour_run.json').write_text(json.dumps(report, indent=2, default=str))
saved = sum(s.get('status') == 'SAVED' for s in report['steps'])
print(f'{report["status"]}: {saved} shots saved in {args.out}')
sys.exit(0 if report['status'] == 'COMPLETED' else 2)
