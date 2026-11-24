#!/usr/bin/env python3
"""Supervised empty-arm A/B trial at the confirmed mechanical lift top.

Default is an offline description. --execute reuses the deployed pick station's
exclusive owner, commissioning, tracking limits, lift guard and STOP.
No reset, PID change, grasp, base motion or lift move is implemented here.
The operator must check the swept space; this is NOT collision planning.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import statistics
import sys
import time

JOINTS = tuple(f'{side}_{name}' for side in ('left', 'right')
               for name in ('base', 'shoulder', 'elbow', 'wrist_flex', 'wrist_roll', 'gripper'))
MASK = 0xffffffff
TRACKING_MARGIN_URAD = 49088  # Existing 32-encoder-count host guard, unchanged.


def tick_delta(tick, origin):
    return ((tick-origin+2**31) & MASK)-2**31


def leg_points(start, target, seconds, *, hold_s=3):
    """Fixed 50 ms stream, quintic movement and three-second endpoint dwell."""
    if (len(start) != 12 or len(target) != 12
            or not all(math.isfinite(v) for v in (*start, *target))
            or type(seconds) is not int or seconds not in (2, 4, 8) or hold_s != 3):
        raise ValueError('twelve finite joints, 2/4/8 second motion and 3 second dwell required')
    points = [(100, tuple(start))]
    for n in range(1, seconds*20+1):
        u = n/(seconds*20)
        blend = u**3*(10+u*(-15+6*u))
        q = tuple(a+(b-a)*blend for a, b in zip(start, target))
        points.append((100+50*n, tuple(target) if n == seconds*20 else q))
    for n in range(1, hold_s*20+1):
        points.append((100+seconds*1000+50*n, tuple(target)))
    return points


def targets(anchor, index, degrees, minimum, maximum):
    if (type(index) is not int or index not in tuple(i for i in range(12) if i % 6 != 5)
            or not math.isfinite(degrees) or not .5 <= abs(degrees) <= 5
            or any(len(v) != 12 for v in (anchor, minimum, maximum))):
        raise ValueError('arm joint and signed 0.5..5 degree excursion required')
    end = list(anchor)
    end[index] += math.radians(degrees)
    for q in (anchor, end):
        if any(not math.isfinite(v) or not lo <= v <= hi
               for v, lo, hi in zip(q, minimum, maximum)):
            raise ValueError('trial endpoint outside installed joint limits; no clipping')
    return tuple(anchor), tuple(end)


def measurement_reference(packet, accepted, initial):
    """Expected commanded angle at EACH joint's original measurement time."""
    if not accepted or len(initial) != 12 or len(packet.sample_age_ms) != 12:
        raise ValueError('accepted trajectory and twelve-axis source times required')
    origin = accepted[0]['apply_tick']
    wire = sorted(accepted, key=lambda p: tick_delta(p['apply_tick'], origin))
    times = [tick_delta(p['apply_tick'], origin) for p in wire]
    expected = []
    for i, age in enumerate(packet.sample_age_ms):
        stamp = tick_delta((packet.firmware_tick_ms-age) & MASK, origin)
        if stamp < 0:
            value = initial[i]
        elif stamp >= times[-1]:
            value = wire[-1]['positions_urad'][i]
        else:
            k = bisect_right(times, stamp)-1
            fraction = (stamp-times[k])/(times[k+1]-times[k])
            value = wire[k]['positions_urad'][i]+fraction*(
                wire[k+1]['positions_urad'][i]-wire[k]['positions_urad'][i])
        expected.append(round(value))
    return tuple(expected)


def trial_envelope(anchor, end):
    return tuple((round(min(a,b)*1e6)-TRACKING_MARGIN_URAD,
                  round(max(a,b)*1e6)+TRACKING_MARGIN_URAD) for a,b in zip(anchor,end))


def require_envelope(packet, envelope):
    if len(packet.positions_urad) != 12 or len(envelope) != 12:
        raise ValueError('twelve-axis trial envelope required')
    for i,(position,(low,high)) in enumerate(zip(packet.positions_urad,envelope)):
        if not low <= position <= high:
            raise ValueError('measured position left trial envelope: '+JOINTS[i])


def trial_hardware_factory(options):
    from home_robot_tasks.pick_hardware import UnifiedPickHardware
    from home_robot_tasks.platform_preparation import measured_hold_source

    class TrialHardware(UnifiedPickHardware):
        """Only this bounded empty-arm trial has a moving observation reference."""
        motion_envelope = None

        def _refresh_empty_stop_evidence(self):
            if self.motion_envelope is None:
                return super()._refresh_empty_stop_evidence()
            if not self.prepared or self.session is not None or self.inputs is not None:
                return
            sent=time.monotonic()
            packet=self.transport.get_feedback_snapshot()
            received=time.monotonic()
            # As in the production moving-route STOP, the MCU independently
            # checks its captured stop anchor and stationarity. Here the host
            # additionally bounds all axes to the declared trial's sweep.
            measured_hold_source(packet,packet.positions_urad,self.adapter.motion_limits,sent,received)
            require_envelope(packet,self.motion_envelope)
            self.evidence.publish_mcu_snapshot(packet,sent,received,
                arms_safe=True,lift_hold=False,payload_safe=True)

    return TrialHardware(options)


def summarize(leg, index):
    """Use this joint's MCU measurement times, never snapshot receive times.

    Targets are accepted scheduled samples, not a claim of measured actuator
    output. Firmware dispatch lateness and servo quantization remain separate.
    """
    wire = leg['accepted_samples']
    if not wire:
        return {'available': False, 'reason': 'no accepted trajectory'}
    origin = wire[0]['apply_tick']
    wire = sorted(wire, key=lambda p: tick_delta(p['apply_tick'], origin))
    times = [tick_delta(p['apply_tick'], origin) for p in wire]
    rows = {}
    for read in leg['feedback']:
        p = read['packet']
        if p['status_code'] or p['present_mask'] != 4095 or p['sample_age_ms'][index] >= 250:
            continue
        stamp = tick_delta((p['firmware_tick_ms']-p['sample_age_ms'][index]) & MASK, origin)
        if not 0 <= stamp <= times[-1]:
            continue
        k = min(bisect_right(times, stamp)-1, len(times)-2)
        fraction = (stamp-times[k])/(times[k+1]-times[k])
        command = (wire[k]['positions_urad'][index]+fraction*(
            wire[k+1]['positions_urad'][index]-wire[k]['positions_urad'][index]))/1e6
        measured = p['positions_urad'][index]/1e6
        if stamp in rows and rows[stamp][1] != measured:
            raise ValueError('conflicting measurements at the same MCU timestamp')
        rows[stamp] = (command, measured)
    stamps = sorted(rows)
    if len(stamps) < 4:
        return {'available': False, 'reason': 'too few distinct source samples'}
    errors = [rows[t][1]-rows[t][0] for t in stamps]
    gap = max(b-a for a, b in zip(stamps, stamps[1:]))/1000
    motion = [rows[t][1]-rows[t][0] for t in stamps if t <= leg['seconds']*1000]
    tail_stamps = [t for t in stamps if t >= times[-1]-1000]
    dense_tail = (len(tail_stamps) >= 4 and tail_stamps[-1]-tail_stamps[0] >= 500
                  and times[-1]-tail_stamps[-1] <= 100
                  and max(b-a for a, b in zip(tail_stamps, tail_stamps[1:])) <= 100)
    tail = [rows[t][1] for t in tail_stamps] if dense_tail else []
    return dict(available=True, joint=JOINTS[index], unique_samples=len(stamps),
                maximum_sample_gap_s=gap, median_sample_gap_s=statistics.median(
                    b-a for a, b in zip(stamps, stamps[1:]))/1000,
                tracking_rms_rad=math.sqrt(statistics.mean(e*e for e in errors)),
                motion_tracking_rms_rad=math.sqrt(statistics.mean(e*e for e in motion)) if motion else None,
                maximum_error_rad=max(map(abs, errors)),
                final_hold_peak_to_peak_rad=max(tail)-min(tail) if tail else None,
                target_basis='accepted scheduled MCU position samples',
                high_frequency_vibration_verified=False)


class TrialCancelled(Exception):
    pass


class TrialContext:
    def __init__(self, hardware, folder, report, *, clock=time.monotonic, sleep=time.sleep):
        self.hardware, self.folder, self.report = hardware, folder, report
        self.clock, self.sleep = clock, sleep
        self.cancelled = False
        self.leg = None
        self.accepted = {}

    def check(self):
        if self.cancelled:
            raise TrialCancelled('operator interrupt')

    def remember_commands(self):
        # Read-only diagnostics of accepted batches; never edit adapter state.
        for sample in self.hardware.adapter._route_samples:
            self.accepted[sample.apply_tick] = asdict(sample)
        self.leg['accepted_samples'] = list(self.accepted.values())

    def service(self):
        self.check()
        h = self.hardware
        if self.leg is not None and h.adapter.state.value == 'active':
            self.remember_commands()
            h.adapter.poll('pick-station')
            self.remember_commands()
        if self.leg is None:
            h.tick()  # Unchanged fixed-pose checks during preparation only.
        else:
            h.camera.check()
            h.adapter.keepalive()
            sent = self.clock()
            packet = h.adapter.feedback_snapshot()
            received = self.clock()
            self.leg['feedback'].append(dict(sent_s=sent, received_s=received, packet=asdict(packet)))
            from home_robot_tasks.platform_preparation import measured_hold_source
            expected = measurement_reference(packet,self.leg['accepted_samples'],self.leg['initial_urad'])
            measured_hold_source(packet,expected,h.adapter.motion_limits,sent,received)
            require_envelope(packet,h.motion_envelope)
            h.evidence.publish_mcu_snapshot(packet,sent,received,
                arms_safe=True,lift_hold=False,payload_safe=True)
            sent=self.clock()
            mobile=h.mobile.status()
            lift=h.lift.status()
            h.guard.update(mobile,lift,sent,self.clock())
        if h.prepared:
            h.guard.require_fresh(self.clock())
        self.check()

    def pause(self, seconds):
        deadline = self.clock()+seconds
        while self.clock() < deadline:
            self.service()
            self.sleep(min(.02, max(0, deadline-self.clock())))

    def wait(self, pending):
        while not pending.done():
            self.pause(.01)
        self.service()
        return pending.result()

    def note(self, message, **values):
        self.report['events'].append(dict(message=message, **values))
        print(message, flush=True)

    def artifact(self, path, label):
        self.report['artifacts'].append(dict(path=str(path), label=label))


def run_leg(ctx, points, leg):
    from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
    ctx.check()
    ctx.accepted = {}
    leg['initial_urad'] = tuple(round(q*1e6) for q in points[0][1])
    ctx.leg = leg
    try:
        ctx.hardware.adapter.start('pick-station', tuple(TimedJointPoint(t, q) for t, q in points), finite=True)
        ctx.remember_commands()
        deadline = ctx.clock()+points[-1][0]/1000+5
        while ctx.hardware.adapter.state.value == 'active':
            if ctx.clock() >= deadline:
                raise TimeoutError('trial trajectory completion timeout')
            ctx.pause(.01)
        if ctx.hardware.adapter.state.value != 'ready':
            raise RuntimeError('trial did not return to ready')
    finally:
        ctx.leg = None


def execute(args, *, hardware_factory=None):
    if args.seconds not in (2,4) or (abs(args.degrees)>1 and args.seconds != 4):
        raise ValueError('excursions above 1 degree require 4/8 second comparison')
    if not (args.supported_empty_top and args.clearance_confirmed):
        raise ValueError('confirm supported empty arms/lift at actual top and swept clearance')
    if not sys.stdin.isatty():
        raise ValueError('use an interactive terminal (SSH -t); Ctrl+C requests whole-system STOP')
    root = args.runtime_root.resolve()
    sys.path[:0] = [str(root), str(root/'ros2_ws/src/so101_arm_bridge'),
                   str(root/'ros2_ws/src/home_robot_tasks')]
    if hardware_factory is None:
        hardware_factory = trial_hardware_factory
    if not args.baseline.is_file():
        raise ValueError('existing top-reference calibration baseline required')
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(status='PREPARING', joint=args.joint, excursion_degrees=args.degrees,
                  events=[], artifacts=[], legs=[], base_move_commands=0, lift_move_commands=0,
                  automatic_collision_check=False, operator_swept_clearance_confirmed=True,
                  existing_pick_speed_reproduced=False,
                  durations_s=[args.seconds,args.seconds*2],
                  tracking_margin_urad=TRACKING_MARGIN_URAD,
                  note='Diagnostic time-scaled motion; not the speed of a previous pick.')
    def save():
        pending = args.output/'trial.tmp'
        pending.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        pending.replace(args.output/'trial.json')
    save()  # Reject unwritable output before any device access.
    h = hardware_factory(dict(execute=True, supported_empty_top=True, supported_empty_bottom=False,
        supported_startup_release=True, baseline=str(args.baseline), serial_port=args.port,
        camera_mode='wide', lift_target_um=0, fixed_base_station=False))
    ctx = TrialContext(h, args.output, report)
    def cancel(*_):
        ctx.cancelled = True
    old = {s:signal.signal(s, cancel) for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        h.prepare(ctx)
        ctx.service()
        index = JOINTS.index(args.joint)
        limits = h.adapter.motion_limits
        anchor, end = targets(tuple(q/1e6 for q in h.anchor.positions_urad), index, args.degrees,
                              tuple(q/1e6 for q in limits.minimum_urad),
                              tuple(q/1e6 for q in limits.maximum_urad))
        report.update(anchor_rad=anchor, endpoint_rad=end, status='RUNNING')
        h.motion_envelope = trial_envelope(anchor,end)
        report['motion_envelope_urad'] = h.motion_envelope
        report['tracking_reference'] = 'accepted command at each joint measurement timestamp'
        # Interleave A and B to reduce warm-up drift; dwell and endpoints are
        # identical. Every leg preserves all five other axes and the other arm.
        for repeat in range(1, args.repeats+1):
            for seconds in (args.seconds, args.seconds*2):
                for direction, start, finish in (('out', anchor, end), ('back', end, anchor)):
                    leg = dict(repeat=repeat, seconds=seconds, direction=direction,
                               accepted_samples=[], feedback=[], completed=False)
                    report['legs'].append(leg)
                    ctx.note(f'{repeat}/{args.repeats} · {args.joint} · {direction} · {seconds}초 이동 + 3초 정지')
                    run_leg(ctx, leg_points(start, finish, seconds), leg)
                    leg['completed'] = True
        report['status'] = 'COMPLETED'
    except (Exception, KeyboardInterrupt) as exc:
        report.update(status='CANCELLED' if isinstance(exc, (TrialCancelled, KeyboardInterrupt)) else 'FAILED',
                      error=str(exc) or type(exc).__name__)
    finally:
        report['stop_session_created'] = getattr(h, 'stop', None) is not None
        try:
            h.close()  # Existing integrated STOP; never releases torque or resets.
        except Exception as exc:
            report.update(status='STOP_UNCONFIRMED', stop_error=str(exc))
        report['stop_confirmed'] = bool(h.stop_confirmed)
        if report['status'] == 'COMPLETED' and not h.stop_confirmed:
            report['status'] = 'STOP_UNCONFIRMED'
        for leg in report['legs']:
            try:
                leg['summary'] = summarize(leg, JOINTS.index(args.joint))
            except Exception as exc:
                leg['summary'] = dict(available=False, reason=str(exc))
        save()
        for s, handler in old.items():
            signal.signal(s, handler)
    outcome = {key: report[key] for key in
               ('status', 'error', 'stop_error', 'stop_confirmed', 'stop_session_created') if key in report}
    outcome['report'] = str(args.output/'trial.json')
    print(json.dumps(outcome, ensure_ascii=False), flush=True)
    if not report['stop_session_created']:
        print('시험용 STOP 세션 생성 전에 종료됐습니다. stop_confirmed=false는 이번 시험의 정지 확인 실패를 뜻하지 않습니다.', flush=True)
    else:
        print('종료 후 토크 유지 또는 미확인 상태입니다. 자동 복귀·재실행·토크 해제는 하지 않습니다.', flush=True)
    return 0 if report['status'] == 'COMPLETED' else 2


def check_runtime(root):
    root = root.resolve()
    sys.path[:0] = [str(root), str(root/'ros2_ws/src/so101_arm_bridge'),
                   str(root/'ros2_ws/src/home_robot_tasks')]
    from home_robot_tasks.pick_hardware import UnifiedPickHardware
    from home_robot_tasks.pick_calibration import verify_installed
    from home_robot_tasks.s2_rgbd_acquisition import Capture
    from so101_arm_bridge.unified_commissioning import PICK_BOARDS
    import serial
    verify_installed(root)
    if 0x24930 not in PICK_BOARDS:
        raise ValueError('runtime does not support installed 0x24930 board')
    print(json.dumps(dict(status='RUNTIME_IMPORTS_CHECKED', root=str(root),
                          hardware_commands=0, devices_opened=0)))
    return 0


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime-root', type=Path, default=Path(__file__).resolve().parents[2])
    p.add_argument('--joint', choices=tuple(n for n in JOINTS if not n.endswith('gripper')), default='left_shoulder')
    p.add_argument('--degrees', type=float, default=1, help='controller-angle direction; signed 0.5..5 degrees')
    p.add_argument('--seconds', type=int, choices=(2,4), default=2,
                   help='one-way duration A; B is twice as long; >1 degree requires 4')
    p.add_argument('--repeats', type=int, choices=(1, 3), default=1)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true')
    mode.add_argument('--check-runtime', action='store_true', help='imports/calibration only; no devices')
    p.add_argument('--supported-empty-top', action='store_true',
                   help='actual top, empty grippers, supported mechanism; includes supported startup torque release if needed')
    p.add_argument('--clearance-confirmed', action='store_true',
                   help='operator checked entire excursion, static base, and supports clear of the moving arm')
    p.add_argument('--port', default='/dev/ttyACM0')
    p.add_argument('--baseline', type=Path, default=Path('/home/hyper/robot/diagnostics/tape-top-reference-20261117-preinstall-baseline.json'))
    p.add_argument('--output', type=Path)
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if not math.isfinite(args.degrees) or not .5 <= abs(args.degrees) <= 5:
        p.error('--degrees must have magnitude 0.5..5')
    if abs(args.degrees)>1 and args.seconds != 4:
        p.error('excursions above 1 degree require --seconds 4 (4/8 second comparison)')
    if args.check_runtime:
        return check_runtime(args.runtime_root)
    if args.execute:
        if args.output is None or not args.supported_empty_top or not args.clearance_confirmed:
            p.error('--execute requires new --output, --supported-empty-top and --clearance-confirmed')
        return execute(args)
    print(json.dumps(dict(mode='OFFLINE_PREVIEW', hardware_commands=0, joint=args.joint,
        excursion_degrees=args.degrees, durations_s=[args.seconds,args.seconds*2], dwell_s=3, repeats=args.repeats,
        sequence='A out/hold/back/hold, B out/hold/back/hold; same commanded endpoints',
        actual_anchor_checked=False, automatic_collision_check=False,
        existing_pick_speed_reproduced=False), ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print('시험 실행 거절: '+str(exc), file=sys.stderr)
        raise SystemExit(2)
