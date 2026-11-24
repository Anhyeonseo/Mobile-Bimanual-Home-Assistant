#!/usr/bin/env python3
"""One explicitly supported recovery of the failed boot-87 lift trial.

Physical supports must carry both arms and the empty lift throughout reset and
torque removal. No movement, homing, reference, flash, prepare, torque enable,
automatic retry, or automatic second reset. Default is read-only preflight.
Uses the verified, already deployed unified-lift runtime bundle.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
import time

RUNTIME = Path('/home/hyper/robot/shared/unified-lift-b2e3b28efcd6')
ROOT = Path('/home/hyper/robot/releases/s4-unified-pick-20261116')
BASELINE = Path('/home/hyper/robot/shared/diagnostics/s4-trial-20261114/baseline.json')
DEVICE = '/dev/ttyACM0'
OPENOCD = Path('/home/hyper/robot-archive/20261029-cleanup/home-assistant-s0-debug-4_vjtnwn/root/usr')
EXPECTED_BOOT = 87
EXPECTED_PROFILE = 2430209922
EXPECTED_STOP_TICK = 7137307
EXPECTED_RAW = 3269


def verify_identity(hello, baseline, *, stopped):
    if (hello.firmware_version != 0x2492d or hello.joint_count != 12
            or bool(hello.stop_latched) != stopped):
        raise ValueError('unexpected firmware, joint count or STOP state; no recovery')
    for key in ('left_calibration_hash', 'right_calibration_hash'):
        if getattr(hello, key) != baseline['hello'][key]:
            raise ValueError('calibration identity changed; no recovery')


def preflight(transport, baseline, *, clock=time.monotonic, sleep=time.sleep):
    from so101_arm_bridge.unified_commissioning import CommissionClient
    from so101_arm_bridge.lift_wire import LiftCommand, LiftStatus
    from so101_arm_bridge.system_stop import SystemStopClient
    hello = transport.enter_binary_mode()
    verify_identity(hello, baseline, stopped=True)
    commission = CommissionClient(transport).query()
    if (commission.boot_id != EXPECTED_BOOT or commission.profile_crc32 != EXPECTED_PROFILE
            or commission.flags != 127 or commission.reason != 0):
        raise ValueError('not the recorded failed boot; do not rerun recovery')
    runtime = transport.stationary_runtime()
    first = runtime.get('mobile_board', {})
    if (not runtime.get('stop_latched') or runtime.get('executor_active')
            or first.get('source') != 2 or first.get('tick_ms') != EXPECTED_STOP_TICK
            or first.get('lift_fault') != 1 or first.get('position_raw') != EXPECTED_RAW
            or first.get('goal_velocity_raw') != 0):
        raise ValueError('first failure or runtime differs from the inspected trial')
    proof, _ = SystemStopClient(transport, EXPECTED_BOOT, clock).status()
    # This does not claim a complete STOP proof. The operator's independent
    # supports are essential. Require the recorded STOP actions to have run.
    if (proof.scope != 0 or proof.started_ms != EXPECTED_STOP_TICK
            or proof.actions_completed_ms != EXPECTED_STOP_TICK or not proof.flags & 8):
        raise ValueError('recorded STOP actions unavailable')
    rows = []
    start = clock()
    last_stamp = None
    low = high = 0
    while clock() - start < 2:
        sequence = len(rows) + 1
        sent = clock()
        reply = LiftStatus.decode(transport.exchange_mobile(
            LiftCommand(4, 1, sequence, 0, EXPECTED_BOOT).encode()))
        received = clock()
        if (not 0 <= received - sent <= .1 or reply.kind != 4
                or reply.request_sequence != sequence or reply.boot_id != EXPECTED_BOOT
                or reply.state != 6 or reply.fault != 1 or reply.flags & 3
                or not reply.flags & 8 or reply.command_raw != 0
                or (reply.mcu_tick_ms - reply.observed_ms) % 2**32 >= 200):
            raise ValueError('current lift state is stale, active or different; no reset')
        if last_stamp is not None and not 0 < (reply.observed_ms-last_stamp) % 2**32 <= 250:
            raise ValueError('lift observations did not advance; no reset')
        delta = (reply.position_raw - EXPECTED_RAW + 2048) % 4096 - 2048
        low, high = min(low, delta), max(high, delta)
        if abs(delta) > 2 or high-low > 2:
            raise ValueError('lift position changed since failure; no reset')
        rows.append(asdict(reply))
        last_stamp = reply.observed_ms
        span = (last_stamp - rows[0]['observed_ms']) % 2**32
        if len(rows) >= 3 and 400 <= span < 2**31:
            return dict(controller=asdict(hello), commission=asdict(commission),
                        runtime=runtime, stop_proof=asdict(proof), lift_observations=rows,
                        scope='read-only preflight with independent physical supports required')
        sleep(.08)
    raise TimeoutError('fresh stationary lift observations unavailable; no reset')


def release_once(transport, baseline, report, *, clock=time.monotonic, sleep=time.sleep):
    import prepare_unified_controller as prep
    from so101_arm_bridge.device_inventory import collect_inventory
    result = report.setdefault('release', {})
    try:
        prep.release_boot_supported(transport, baseline=baseline, supported_empty=True, report=result)
        return
    except Exception as error:
        result['release_error'] = str(error)
        # If the write was attempted but no valid ACK was returned, an error
        # may still have removed torque. Read back; never resend that command.
        # A later verification failure is not retried under a new anchor.
        if not result.get('release_attempted') or 'release' in result:
            raise
    after = collect_inventory(transport)
    result['independent_inventory_after_error'] = after
    prep.compare_calibration(result['inventory_before'], after,
                             allow_pending_wheel_reports=True, allow_pending_lift_reports=True)
    window = prep.ZeroPowerObservation(after, powered=False)
    samples = result['independent_off_observations'] = [after]
    window.observe(after)
    started = clock()
    while not window.proven:
        if clock()-started > 2:
            raise TimeoutError('torque-off stationary observation unconfirmed after release error')
        sleep(.05)
        sample = collect_inventory(transport)
        samples.append(sample)
        window.observe(sample)
    result.update(status='TORQUE_OFF_VERIFIED_AFTER_RELEASE_ERROR', final_all_torque_off=True,
                  calibration_preserved=True, unpowered_stationarity=window.report())


def run_recovery(open_transport, reset_once, baseline, report, save, *, sleep=time.sleep):
    # Both independently protected ports are opened under one outer backend
    # lease. Close the serial connection before the single MCU reset.
    with open_transport() as transport:
        report['preflight'] = preflight(transport, baseline)
    report.update(status='RESET_REQUESTED', reset_attempted=True)
    save()
    reset_once()
    report.update(status='RESET_COMPLETED', reset_completed=True)
    save()
    sleep(.5)
    with open_transport() as transport:
        hello = transport.enter_binary_mode()
        verify_identity(hello, baseline, stopped=False)
        from so101_arm_bridge.unified_commissioning import CommissionClient
        commission = CommissionClient(transport).query()
        report['after_reset'] = dict(controller=asdict(hello), commission=asdict(commission))
        if (commission.flags != 0 or commission.reason != 0 or commission.boot_id != 0
                or commission.profile_crc32 != EXPECTED_PROFILE):
            raise ValueError('reset did not produce a clean unconfigured boot; no release')
        report['status'] = 'SUPPORTED_RELEASE_REQUESTED'
        save()
        release_once(transport, baseline, report)
    result = report['release']
    if not result.get('final_all_torque_off') or not result.get('calibration_preserved'):
        raise RuntimeError('release did not prove sixteen torques OFF and preserved calibration')
    report.update(status='ALL_16_TORQUE_OFF_VERIFIED', final_all_torque_off=True,
                  manual_bottom_return_pending=True, lift_reference_valid=False)
    save()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime-root', type=Path, default=RUNTIME)
    parser.add_argument('--baseline', type=Path, default=BASELINE)
    parser.add_argument('--port', default=DEVICE)
    parser.add_argument('--stlink-serial', help='ST-Link serial for the explicit reset operation')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--supported-empty', action='store_true',
                        help='both arms and the empty lift are carried by independent mechanical supports')
    args = parser.parse_args(argv)
    if args.execute and (not args.stlink_serial or not args.stlink_serial.isalnum()):
        parser.error('--execute requires an alphanumeric --stlink-serial identifying this board')
    if args.execute and not args.supported_empty:
        parser.error('reset/release requires --supported-empty with current physical supports')
    sys.path.insert(0, str(args.runtime_root/'tools/run'))
    import unified_lift
    code_root, manifest = unified_lift.runtime_sources(ROOT)
    if manifest is None or code_root != args.runtime_root.resolve():
        raise ValueError('verified deployed lift runtime bundle required')
    baseline = json.loads(args.baseline.read_text())
    import serial
    from contextlib import contextmanager
    from so101_arm_bridge.backend_lease import acquire_backend_lease
    from so101_arm_bridge.serial_port import open_exclusive_serial
    from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    out = Path('/home/hyper/robot/shared/diagnostics')/('supported-lift-recovery-'+stamp)
    out.mkdir(parents=True, exist_ok=False)
    report = dict(status='PREFLIGHT', motion_commands=0, torque_enable_commands=0,
                  reference_commands=0, flash_requested=False, automatic_retry=False,
                  reset_attempted=False, reset_completed=False, final_all_torque_off=False,
                  operator_supported_empty=args.supported_empty, code_root=str(code_root))
    def save():
        temp = out/'state.tmp'
        temp.write_text(json.dumps(report, indent=2)+'\n')
        temp.replace(out/'state.json')
    @contextmanager
    def connection():
        with open_exclusive_serial(serial, args.port, 921600, .02) as port:
            # Existing firmware release performs six complete 16-device scans
            # and its own >=400ms quiet window before attempting torque OFF.
            yield StreamValidationTransportV2(port, response_timeout_s=3)
    reset_args = ['sudo', '-n', 'env', 'LD_LIBRARY_PATH='+str(OPENOCD/'lib/aarch64-linux-gnu'),
                  str(OPENOCD/'bin/openocd'), '-s', str(OPENOCD/'share/openocd/scripts'),
                  '-f', 'interface/stlink.cfg',
                  '-c', 'adapter serial '+str(args.stlink_serial)+'; transport select hla_swd',
                  '-f', 'target/stm32g4x.cfg',
                  '-c', 'adapter speed 1000; reset_config none; stm32g4x.cpu configure -event examine-end {}; stm32g4x.cpu configure -event reset-init {}; gdb_port disabled; tcl_port disabled; telnet_port disabled; init; reset run; shutdown']
    def reset():
        with (out/'reset.log').open('x') as log:
            subprocess.run(reset_args, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=30)
    print('기록: '+str(out/'state.json'), flush=True)
    try:
        if args.execute:
            for name in ('bin/openocd', 'share/openocd/scripts/interface/stlink.cfg',
                         'share/openocd/scripts/target/stm32g4x.cfg'):
                if not (OPENOCD/name).is_file():
                    raise FileNotFoundError('reset tool missing: '+str(OPENOCD/name))
            subprocess.run(['sudo', '-n', 'true'], check=True, timeout=5)
            print('양팔·리프트 받침을 유지하세요. 제어기 1회 재시작 후 토크를 해제합니다.', flush=True)
        with acquire_backend_lease('stm32', 189):
            if args.execute:
                run_recovery(connection, reset, baseline, report, save)
            else:
                with connection() as transport:
                    report['preflight'] = preflight(transport, baseline)
                report['status'] = 'READ_ONLY_PREFLIGHT_PASSED'
    except BaseException as error:
        report.update(status='FAILED', error=str(error) or type(error).__name__)
    finally:
        save()
    print(json.dumps({k:report.get(k) for k in ('status', 'error', 'reset_attempted',
          'reset_completed', 'final_all_torque_off', 'motion_commands', 'reference_commands')},
          ensure_ascii=False), flush=True)
    if report['status'] == 'ALL_16_TORQUE_OFF_VERIFIED':
        print('16개 모터 토크 OFF 확인. 받침을 유지하고 리프트를 실제 최하단으로 복귀한 뒤 알려주세요. 원점 등록·재상승은 하지 않습니다.', flush=True)
        if report['release'].get('release_error'):
            print('해제 응답 오류가 있었지만 독립 조회로 OFF를 확인했습니다. 해제 명령을 반복하지 마세요.', flush=True)
    elif report['status'] == 'FAILED':
        print('복구 완료를 확인하지 못했습니다. 받침을 유지하고 재실행하지 말고 결과를 전달해주세요.', flush=True)
    return int(report['status'] == 'FAILED')


if __name__ == '__main__':
    raise SystemExit(main())
