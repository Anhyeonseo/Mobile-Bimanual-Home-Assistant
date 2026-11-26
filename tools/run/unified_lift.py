#!/usr/bin/env python3
"""Supervised lift console for the existing 0x2492d integrated firmware.

Default: read-only inspection. Execution holds BOTH arms at their freshly read
pose, references an operator-confirmed bottom, and uses the existing <=10mm
test. Only a NEW g key at that held checkpoint starts a longer upward move.
The operator is responsible for the complete swept clearance of both arms,
supports, cables and lift. This is manual commissioning, not a grasp plan or
automatic collision certificate. No base command, reset, mode preparation,
torque release, downward homing or automatic retry is implemented.

At the target, stay connected and retain the original height reference. Space,
x, Ctrl-C, terminal EOF, SIGHUP or an error requests integrated STOP and retains
torque. STOP does not promise a reusable height reference for a new process.
"""
from dataclasses import asdict, replace
from datetime import datetime, timezone
import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import select
import signal
import sys
import termios
import time
import tty

DEFAULT_ROOT = Path('/home/hyper/robot/releases/s4-unified-pick-20261116')
DEVICE = '/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_005100393235511438363730-if02'
FIRST_RISE_UM = 10000
MAX_TARGET_UM = 550000
AGE_MS = 250
TOLERANCE_UM = 3000
EXPECTED_PROFILE_CRC32 = 2430209922


class OperatorStop(RuntimeError):
    pass


def runtime_sources(config_root):
    """Use a verified isolated bundle when present; do not mix release code."""
    candidate = Path(__file__).resolve().parents[2]
    manifest_path = candidate/'runtime_manifest.json'
    manifest = None
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text())
        if manifest.get('schema_version') != 1 or manifest.get('entrypoint') != 'tools/run/unified_lift.py':
            raise RuntimeError('invalid lift runtime manifest')
        for name, expected in manifest['files_sha256'].items():
            file = (candidate/name).resolve()
            if not file.is_relative_to(candidate) or not file.is_file():
                raise RuntimeError('missing bundled dependency: '+name)
            if hashlib.sha256(file.read_bytes()).hexdigest() != expected:
                raise RuntimeError('bundled dependency changed: '+name)
        code_root = candidate
    elif (candidate/'ros2_ws/src/so101_arm_bridge').is_dir() and (candidate/'ros2_ws/src/home_robot_tasks').is_dir():
        code_root = candidate
    else:
        code_root = config_root
    sys.path[:0] = [str(code_root/'tools/run'), str(code_root/'ros2_ws/src/so101_arm_bridge'),
                   str(code_root/'ros2_ws/src/home_robot_tasks')]
    if manifest is not None:
        # Include lazy dependencies BEFORE opening a serial port or arming.
        for name in manifest['modules']:
            if name == 'unified_lift':
                continue
            module = importlib.import_module(name)
            actual = Path(module.__file__).resolve()
            if actual != (code_root/manifest['modules'][name]).resolve():
                raise RuntimeError('runtime mixed with another installation: '+name)
    return code_root, manifest


def validate_target(target_mm):
    if type(target_mm) is not int or not 10 <= target_mm <= MAX_TARGET_UM // 1000:
        raise ValueError('target must be an integer 10..550mm above the physical bottom')
    return target_mm * 1000


def held(status):
    return (status.state == 5 and status.fault == 0 and status.flags & 31 == 29
            and status.command_raw == 0
            and abs(status.height_um - status.target_um) <= TOLERANCE_UM
            and (status.mcu_tick_ms - status.observed_ms) & 0xffffffff < AGE_MS)


def stop_and_confirm(stop, report, *, clock=time.monotonic, sleep=time.sleep):
    """One independent STOP request, then bounded proof queries. Never release."""
    report['stop_confirmed'] = False
    try:
        report['stop_ack'] = asdict(stop.request())
    except BaseException as exc:
        # A lost ACK does not prove STOP failed. Query its independent proof;
        # never resend a motion or depend on the lift client's local session.
        report['stop_request_error'] = str(exc) or type(exc).__name__
    try:
        until = clock()+4
        while clock() < until:
            proof, _ = stop.status()
            report['stop_proof'] = asdict(proof)
            if proof.confirmed:
                report['stop_confirmed'] = True
                return
            sleep(.03)
        report['stop_error'] = 'integrated stop confirmation timeout'
    except BaseException as exc:
        report['stop_error'] = str(exc) or type(exc).__name__


def rise_after_probe(lift, observe, target_um, probe, *,
                     clock=time.monotonic, sleep=time.sleep):
    """One upward move after a successful same-boot 1cm check. No auto retry.

    The enclosing console owns STOP on EVERY exception, including a lost move
    acknowledgement. Faults never trigger a second move or clear a reference.
    """
    if type(target_um) is not int or not FIRST_RISE_UM < target_um <= MAX_TARGET_UM:
        raise ValueError('long rise target must be >10mm and <=550mm')
    if (not isinstance(probe, dict) or probe.get('target_um') != FIRST_RISE_UM
            or not isinstance(probe.get('final'), dict)):
        raise ValueError('completed 1cm probe required')
    final = probe['final']
    initial = observe()
    boot = lift.mobile.boot_id
    if (not held(initial) or initial.boot_id != boot or not boot
            or final.get('boot_id') != boot or final.get('session') != initial.session
            or final.get('state') != 5 or final.get('fault') != 0
            or final.get('flags', 0) & 31 != 29 or final.get('command_raw') != 0
            or final.get('target_um') != FIRST_RISE_UM
            or type(final.get('height_um')) is not int
            or abs(final['height_um'] - FIRST_RISE_UM) > TOLERANCE_UM
            or initial.target_um != FIRST_RISE_UM
            or abs(initial.height_um - FIRST_RISE_UM) > TOLERANCE_UM
            or initial.height_um >= target_um
            or not 0 < initial.session < 0xffffffff):
        raise RuntimeError('fresh, unchanged same-boot held 1cm checkpoint required')
    session = initial.session + 1
    # Start once. A missing ACK is an error, never permission to resend.
    lift.move(session, target_um)
    started = clock()
    deadline = started + min(600., 30. + (target_um - initial.height_um) / 1000.)
    highest = initial.height_um
    last_time = started
    while True:
        state = observe()
        now = clock()
        if now < last_time or now - last_time >= .25:
            raise RuntimeError('lift observation gap or clock regression')
        last_time = now
        if now >= deadline:
            raise TimeoutError('lift rise deadline exceeded')
        if (state.boot_id != boot or state.session != session or state.fault
                or not state.flags & 1 or state.target_um != target_um
                or (state.mcu_tick_ms - state.observed_ms) & 0xffffffff >= AGE_MS
                or not -TOLERANCE_UM <= state.height_um <= target_um + TOLERANCE_UM):
            raise RuntimeError('lift reference, ownership, feedback or height invalid')
        if state.height_um < highest - TOLERANCE_UM:
            raise RuntimeError('lift moved downward during commanded rise')
        highest = max(highest, state.height_um)
        if held(state):
            if abs(state.height_um - target_um) > TOLERANCE_UM:
                raise RuntimeError('lift stopped at the wrong height')
            return dict(initial=asdict(initial), final=asdict(state), target_um=target_um,
                        elapsed_s=now-started)
        if state.state not in (3, 4) or state.flags & 7 != 7:
            raise RuntimeError('lift lost movement interlock or active ownership')
        lift.keepalive()
        sleep(.02)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    p.add_argument('--port', default=DEVICE)
    p.add_argument('--baseline', type=Path, default=Path('/home/hyper/robot/shared/diagnostics/s4-trial-20261114/baseline.json'))
    p.add_argument('--output', type=Path)
    p.add_argument('--target-mm', type=int, default=550)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--check-environment', action='store_true',
                   help='import all runtime dependencies and read configuration; never opens a device')
    p.add_argument('--supported-empty-bottom', action='store_true')
    p.add_argument('--swept-clearance-confirmed', action='store_true')
    a = p.parse_args(argv)
    target_um = validate_target(a.target_mm)
    if a.check_environment and a.execute:
        p.error('--check-environment cannot accompany --execute')
    if a.execute and not (a.supported_empty_bottom and a.swept_clearance_confirmed):
        p.error('execution requires supported empty robot at physical bottom and swept clearance confirmation')
    if a.execute and not sys.stdin.isatty():
        p.error('execution requires an interactive terminal: use ssh -t, not a stdin script pipe')
    root = a.root.resolve()
    code_root, manifest = runtime_sources(root)
    import serial
    import prepare_unified_controller as prep
    from home_robot_tasks.platform_preparation import (
        measured_hold_source, reference_supported_bottom, move_observed_lift,
        supported_lift_is_holding,
    )
    # These otherwise load only after activation in the first 1cm probe.
    from home_robot_tasks.device_ports import LiftPort
    from home_robot_tasks.system_stop_backend import SystemStopBackend
    from so101_arm_bridge.development_ranges import read_development_ranges, unwrap_in_range
    from so101_arm_bridge.limit_manifest import JOINTS
    from so101_arm_bridge.backend_lease import acquire_backend_lease
    from so101_arm_bridge.serial_port import open_exclusive_serial
    from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2, MobileV2Exchange
    from so101_arm_bridge.unified_arm_adapter import UnifiedArmAdapter
    from so101_arm_bridge.unified_commissioning import CommissionClient, FIRMWARE
    from so101_arm_bridge.mobile_client import MobileClient
    from so101_arm_bridge.lift_wire import LiftClient
    from so101_arm_bridge.robot_evidence import RobotEvidenceClient
    from so101_arm_bridge.system_stop import SystemStopClient

    if FIRMWARE != 0x2492d:
        raise RuntimeError('this console requires the 0x2492d host protocol')
    baseline = json.loads(a.baseline.read_text())
    ranges, _, digest = read_development_ranges(root/'config/home_assistant/arm_ranges.development.json')
    geometry = json.loads((root/'config/home_assistant/lift_geometry.measured.json').read_text())
    if not 0 < target_um <= geometry['maximum_height_um']:
        raise ValueError('target outside installed lift geometry')
    if a.check_environment:
        print(json.dumps(dict(status='ENVIRONMENT_OK', code_root=str(code_root), config_root=str(root),
            bundled_module_count=len(manifest['modules']) if manifest else 0,
            expected_firmware=hex(FIRMWARE), configuration_sha256=digest,
            serial_opened=False, motion_commands=0, torque_commands=0), ensure_ascii=False), flush=True)
        return 0
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    out = a.output or Path('/home/hyper/robot/shared/diagnostics')/('unified-lift-'+stamp)
    out.mkdir(parents=True, exist_ok=False)
    report = dict(status='INSPECTING', target_height_mm=a.target_mm, base_motion_commands=0,
                  arm_motion_scope='fresh measured constant-pose hold only',
                  automatic_retry=False, reset_requested=False, torque_release_requested=False,
                  automatic_collision_check=False, calibration_hash=digest,
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  runtime_code_root=str(code_root),
                  runtime_bundle_id=manifest['bundle_id'] if manifest else None,
                  events=[])
    cancelled = False
    def cancel(*_):
        nonlocal cancelled
        cancelled = True
    old_handlers = {s:signal.signal(s, cancel) for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    terminal = None
    stage = 'INSPECTING'
    last_save = last_display = float('-inf')
    def save():
        temp = out/'state.tmp'
        temp.write_text(json.dumps(report, indent=2)+'\n')
        temp.replace(out/'state.json')
    def event(name, text):
        nonlocal stage
        stage = name
        report['status'] = name
        report['events'].append(dict(state=name, monotonic_s=time.monotonic()))
        save()
        print(text, flush=True)
    def keys():
        if cancelled:
            raise OperatorStop('operator signal')
        if terminal is None or not select.select([sys.stdin.fileno()], [], [], 0)[0]:
            return ''
        raw = os.read(sys.stdin.fileno(), 4096)
        if not raw:
            raise OperatorStop('terminal EOF')
        value = raw.decode('ascii', errors='ignore').lower()
        if any(c in value for c in (' ', 'x', 'q', '\x03', '\x1b')):
            raise OperatorStop('operator stop key')
        return value

    try:
      with acquire_backend_lease('stm32', 189):
        with open_exclusive_serial(serial, a.port, 921600, .02) as port:
          transport = StreamValidationTransportV2(port, response_timeout_s=3)
          stop = None
          power_attempted = False
          try:
            inspected = {}
            report['inspection'] = inspected
            prep.inspect(transport, baseline=baseline, report=inspected)
            if inspected.get('status') != 'CONNECTED_NOT_ARMED' or inspected['commission_final']['flags'] != 17:
                raise RuntimeError('prepared torque-off unreferenced board required; no automatic preparation/reset')
            if inspected['commission_final']['profile_crc32'] != EXPECTED_PROFILE_CRC32:
                raise RuntimeError('commissioned mobile/lift profile changed')
            devices = inspected['inventory_before']['devices']
            if len(devices) != 16 or not all(d['complete'] and d['command']['torque_enabled_raw'] == 0 for d in devices):
                raise RuntimeError('sixteen complete torque-off devices required')
            for side in ('left', 'right'):
                for i, name in enumerate(JOINTS, 1):
                    matches = [d for d in devices if d['bus'] == side and d['requested_id'] == i]
                    if len(matches) != 1:
                        raise RuntimeError('missing or duplicate arm inventory')
                    unwrap_in_range(ranges['arms'][side][name], matches[0]['feedback']['position_raw'])
            if not a.execute:
                event('INSPECTED_NO_MOTION', '읽기 전용 검사 통과. --execute 없으므로 구동하지 않습니다.')
                return 0
            terminal = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
            keys()  # Stop keys received before activation are honored.
            event('PREPARING_HOLD', '양팔 현재 자세 유지 준비. Space / x / Ctrl-C: 전체 정지. 로그: '+str(out))
            preview = UnifiedArmAdapter(transport, root/'config/home_assistant/arm_ranges.development.json')
            adapter = UnifiedArmAdapter(transport, root/'config/home_assistant/arm_ranges.development.json',
                authorized_limits=replace(preview.motion_limits, hardware_authorized=True), motion_authorized=True)
            anchor = adapter.prepare()
            mobile = MobileClient(MobileV2Exchange(transport), lambda:int(time.monotonic()*1000))
            mobile.synchronize()
            boot = inspected['commission_final']['boot_id']
            if mobile.boot_id != boot:
                raise RuntimeError('controller boot changed before activation')
            lift = LiftClient(mobile)
            evidence = RobotEvidenceClient(transport, mobile, time.monotonic, maximum_age_ms=AGE_MS)
            stop = SystemStopClient(transport, boot, time.monotonic)
            keys()
            power_attempted = True
            report['activation'] = asdict(CommissionClient(transport).activate_zero(supported=True, empty_grippers=True))
            adapter.start_current_pose_hold('supervised-lift-console')
            anchor = adapter.prepared_state
            report['hold_anchor'] = asdict(anchor)
            deadline = time.monotonic()+4
            while adapter.state.value != 'ready':
                keys()
                if time.monotonic() >= deadline:
                    raise TimeoutError('current-pose arm hold timeout')
                adapter.poll('supervised-lift-console')
                time.sleep(.01)
            go = False
            def observe():
                nonlocal last_save, last_display, go
                pressed = keys()
                if stage == 'WAITING_UP_CHECK' and 'g' in pressed:
                    go = True
                heartbeat = transport.heartbeat()
                if heartbeat.stop_latched or heartbeat.status_code:
                    raise RuntimeError('controller stop or runtime fault')
                sent = time.monotonic()
                snapshot = adapter.feedback_snapshot()
                received = time.monotonic()
                measured_hold_source(snapshot, anchor.positions_urad, adapter.motion_limits, sent, received)
                evidence.publish_mcu_snapshot(snapshot, sent, received,
                    arms_safe=True, lift_hold=False, payload_safe=True)
                ls = lift.status()
                if ls.fault:
                    raise RuntimeError('lift feedback fault '+str(ls.fault))
                now = time.monotonic()
                report['latest'] = dict(joints=asdict(snapshot), lift=asdict(ls),
                                        host_monotonic_s=now, boot_id=boot)
                if now-last_save >= 1:
                    save();last_save=now
                if now-last_display >= 1:
                    print(f'{stage} | 최하단 기준 {ls.height_um/1000:.1f}mm | 목표 {ls.target_um/1000:.1f}mm', flush=True)
                    last_display=now
                return ls
            event('REFERENCING_BOTTOM', '확인된 최하단 위치를 원점으로 등록합니다. 자동 하강은 하지 않습니다.')
            reference_supported_bottom(observe, mobile, lift)
            event('PROBE_10MM', '1cm 상승·정지 검사 중입니다.')
            report['probe'] = move_observed_lift(lift, observe, stop, FIRST_RISE_UM)
            if target_um > FIRST_RISE_UM:
                keys()  # Discard an early g; the checkpoint needs a new key.
                event('WAITING_UP_CHECK', f'1cm 높이 유지 중. 실제 상승·정지와 전체 상승 공간을 확인한 뒤 g: {a.target_mm}mm까지 상승. Space/x: 정지.')
                waiting = time.monotonic()
                while not go:
                    ls = observe()
                    if (not supported_lift_is_holding(ls) or ls.target_um != FIRST_RISE_UM
                            or abs(ls.height_um-FIRST_RISE_UM) > TOLERANCE_UM):
                        raise RuntimeError('1cm checkpoint hold lost')
                    if time.monotonic()-waiting >= 300:
                        raise TimeoutError('checkpoint input timeout')
                    time.sleep(.02)
                event('RISING_TO_TARGET', f'최하단 기준 {a.target_mm}mm로 상승합니다. Space/x: 정지.')
                report['rise'] = rise_after_probe(lift, observe, target_um, report['probe'])
            event('TARGET_HOLDING', '목표 높이 도착·유지 중. 다음 작업 동안 이 터미널을 유지하세요. Space/x는 전체 STOP입니다.')
            while True:
                ls = observe()
                if (not supported_lift_is_holding(ls) or ls.target_um != target_um
                        or abs(ls.height_um-target_um) > TOLERANCE_UM):
                    raise RuntimeError('target height hold lost')
                time.sleep(.02)
          except BaseException as exc:
            report.update(status='STOP_REQUESTED' if isinstance(exc, OperatorStop) else 'FAILED',
                          error=str(exc) or type(exc).__name__, interrupted_stage=stage)
          finally:
            if power_attempted and stop is not None:
                # Independent whole-system STOP also handles a lost move ACK.
                stop_and_confirm(stop, report)
                report['final_power_state'] = 'torque retained or unconfirmed; no release requested'
                report['reference_reuse_authorized'] = False
            save()
    finally:
        if terminal is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, terminal)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    print(json.dumps({k:report[k] for k in ('status','error','stop_confirmed','stop_error') if k in report}, ensure_ascii=False), flush=True)
    print('기록: '+str(out/'state.json'), flush=True)
    if report.get('stop_confirmed'):
        print('전체 정지 확인. 토크는 유지합니다. 재실행하거나 원점을 다시 등록하지 마세요.', flush=True)
    elif report.get('final_power_state'):
        print('정지 미확인: 독립적인 현장 정지 절차와 기계적 지지가 필요합니다.', flush=True)
    return 0 if report.get('stop_confirmed') and report['status']=='STOP_REQUESTED' else 2


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print('실행 중단: '+str(exc), file=sys.stderr)
        raise SystemExit(2)
