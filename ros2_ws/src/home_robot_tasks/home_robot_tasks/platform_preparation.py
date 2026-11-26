"""Shared measured platform preparation for navigation and stationary picking.

Clients are supplied by the caller; this module never opens or resets a device.
The bounds describe the supported preparation procedure, not calibrated physical
accuracy or permission for a different lift/transport operation.
"""
from dataclasses import asdict
import math
import time


def measured_hold_source(snapshot, anchor, limits, sent_s, received_s):
    """Return the oldest measured joint timestamp, including transit uncertainty.

    Applies to the supported, empty, constant-pose test. This does not establish
    a collision envelope for moving the lift or carrying an object.
    """
    if (snapshot.status_code or snapshot.joint_count != 12 or snapshot.present_mask != 4095
            or len(snapshot.positions_urad) != 12 or len(snapshot.sample_age_ms) != 12
            or len(anchor) != 12 or not all(math.isfinite(v) for v in (sent_s, received_s))
            or not 0 <= received_s - sent_s < .1
            or any(type(age) is not int or not 0 <= age < 250 for age in snapshot.sample_age_ms)):
        raise ValueError('fresh complete measured arm snapshot required')
    for i, position in enumerate(snapshot.positions_urad):
        if (not limits.minimum_urad[i] <= position <= limits.maximum_urad[i]
                or abs(position - anchor[i]) > 49088):  # Existing profile: 32 encoder counts.
            raise ValueError('measured arm left supported current-pose envelope')
    stamp = sent_s - max(snapshot.sample_age_ms) / 1000
    if received_s - stamp >= .25:
        raise ValueError('arm evidence expired in transit')
    return stamp


def supported_lift_is_holding(status):
    """Measured hold only, not permission to start a new lift movement.

    LS bit 4 is the movement interlock. Losing it inhibits movement, but is not
    loss of independent hold proof (LS bits 1/8/16). LiftStatus.holding() keeps
    the stronger admission checks needed by the runtime movement controller.
    """
    return (status.state == 5 and status.fault == 0 and status.flags & 25 == 25
            and not status.flags & 2 and status.command_raw == 0
            and abs(status.height_um - status.target_um) <= 2000
            and ((status.mcu_tick_ms - status.observed_ms) & 0xffffffff) < 250)


def stationary_platform_is_holding(mobile, lift, board, *, fixed_base_station=False):
    """Independent AS base-stop + LS hold proof; no lift motion admission.

    Only the installed 0x24930 firmware has the verified drive/encoder hold
    monitor used here. Require all AS feedback, mode, homing, stop and health
    bits, and the original 200 ms freshness bound on both observations.
    """
    if fixed_base_station:
        # Explicit operator-fixed tabletop station. Wheel telemetry is not
        # a grasp admission condition; this owner never enables base motion.
        return (tuple(board or ()) == (0x24930, 3399362976)
                and mobile.boot_id == lift.boot_id and mobile.boot_id > 0
                and mobile.state == 0 and mobile.session == 0 and mobile.reason == 0
                and not mobile.flags & 32
                and supported_lift_is_holding(lift)
                and ((lift.mcu_tick_ms-lift.observed_ms) & 0xffffffff) < 200)
    return (tuple(board or ()) == (0x24930, 3399362976)
            and mobile.boot_id == lift.boot_id and mobile.boot_id > 0
            and mobile.state == 0 and mobile.session == 0 and mobile.reason == 0
            and mobile.flags == 31
            and ((mobile.mcu_tick_ms-mobile.feedback_tick_ms) & 0xffffffff) < 200
            and supported_lift_is_holding(lift)
            and ((lift.mcu_tick_ms-lift.observed_ms) & 0xffffffff) < 200)


def move_observed_lift(lift, observe, stop, height_um, *, clock=time.monotonic, sleep=time.sleep):
    """Explicit <=1cm bottom-origin rise; LiftPort owns the lease and faults."""
    from .device_ports import LiftPort
    from .system_stop_backend import SystemStopBackend

    if type(height_um) is not int or not 0 < height_um <= 10000:
        raise ValueError('explicit bottom-origin lift target up to 1cm required')
    initial = observe()
    if not supported_lift_is_holding(initial) or abs(initial.height_um) > 3000:
        raise RuntimeError('fresh held bottom reference required before rise')
    port = LiftPort(lift, clock, SystemStopBackend(stop))
    port.start('supported-lift-rise', {'height_um': height_um})
    deadline = clock() + 30
    while True:
        observe()
        result = port.poll('supported-lift-rise')
        if result.state == 'SUCCEEDED':
            final = observe()
            if not supported_lift_is_holding(final) or abs(final.height_um - height_um) > 3000:
                raise RuntimeError('lift arrival evidence disagrees with target')
            return {'initial': asdict(initial), 'final': asdict(final), 'target_um': height_um}
        if result.state in ('FAILED', 'CANCELLED'):
            raise RuntimeError('lift rise failed: ' + result.reason)
        if clock() >= deadline:
            port.cancel('supported-lift-rise')
            raise TimeoutError('lift rise deadline exceeded')
        sleep(.02)


def reference_supported_bottom(observe, mobile, lift, *, clock=time.monotonic, sleep=time.sleep):
    return _reference_supported(observe, lift, top_um=None, clock=clock, sleep=sleep)


def reference_supported_top(observe, mobile, lift, height_um, *, operator_confirmed=False,
                            clock=time.monotonic, sleep=time.sleep):
    if operator_confirmed is not True or type(height_um) is not int or not 0 < height_um <= 10000000:
        raise ValueError("confirmed mechanical top and calibrated height required")
    return _reference_supported(observe, lift, top_um=height_um, clock=clock, sleep=sleep)


def _reference_supported(observe, lift, *, top_um, clock, sleep):
    """Observe rest, then renew the 200ms lease until endpoint hold settles.

    No homing or position move is sent; wheel interlocks are observed through
    lift telemetry. Registration alone never substitutes for measured hold.
    """
    deadline = clock() + 3
    window = []
    while True:
        status = observe()
        # AS is invalid before homing, so waiting on it would deadlock reference.
        # LS independently reports lift position and MCU interlocks. Registration checks
        # actual lift speed/current before accepting the zero-motion reference.
        valid = (status.state == 0 and status.fault == 0 and status.flags & 12 == 12
                 and status.command_raw == 0 and 0 <= status.position_raw < 4096
                 and (status.mcu_tick_ms - status.observed_ms) & 0xffffffff < 200)
        if not valid:
            window = []
        else:
            if window and (status.observed_ms - window[-1][0]) & 0xffffffff >= 200:
                window = []
            if not window or status.observed_ms != window[-1][0]:
                window.append((status.observed_ms, status.position_raw))
            window = [row for row in window if (status.observed_ms - row[0]) & 0xffffffff <= 650]
            delta = [(row[1] - window[0][1] + 2048) % 4096 - 2048 for row in window]
            if (len(window) >= 3 and (status.observed_ms - window[0][0]) & 0xffffffff >= 450
                    and max(delta) - min(delta) <= 2):
                try:
                    if top_um is None:
                        lift.reference_bottom(1, operator_confirmed=True)
                    else:
                        lift.reference_top(1, top_um, operator_confirmed=True)
                    break
                except RuntimeError as error:
                    if clock() >= deadline or not _reference_not_started(error, status, top_um):
                        raise
                    # A correlated rejection proves that no reference/hold was
                    # armed. Reobserve the stationary window before another
                    # zero-motion registration; never retry movement or an ACK loss.
                    window = []
        if clock() > deadline:
            raise TimeoutError('리프트 원점 등록 전 정지 관측 대기')
        sleep(.02)
    deadline = clock() + 3
    while True:
        status = observe()
        if supported_lift_is_holding(status):
            expected = 0 if top_um is None else top_um
            if abs(status.height_um - expected) > 2000 or status.target_um != expected:
                raise RuntimeError("registered lift height differs from confirmed endpoint")
            return status
        if clock() > deadline:
            raise TimeoutError('리프트 유지 확인 시간 초과')
        if status.flags & 2:
            lift.keepalive()
        sleep(.02)


def arm_observed_stationary(observe, mobile, *, clock=time.monotonic, sleep=time.sleep,
                            on_deferred=None, transport_only=False, session=1):
    """Wait parked, including a correlated rejection before a session exists.

    Only a healthy, same-boot rejection proving no session was armed can return
    to this wait. Ambiguous responses and accepted commands are never retried.
    """
    from so101_arm_bridge.mobile_client import MobileReplyError
    from so101_arm_bridge.mobile_wire import MobileStatus

    deadline = clock() + 3
    quiet_since = last_tick = None
    samples = 0
    while True:
        if clock() >= deadline:
            raise TimeoutError('출발 전 정지 관측 대기 시간 초과')
        observation = observe()
        if not transport_only and not supported_lift_is_holding(observation):
            raise RuntimeError('리프트 유지 상태 확인 실패')
        feedback = mobile.status()
        if feedback.feedback_is_fresh(100) and not any(feedback.velocity_raw):
            if quiet_since is None:
                quiet_since = clock()
            if feedback.feedback_tick_ms != last_tick:
                last_tick = feedback.feedback_tick_ms
                samples += 1
            if samples >= 3 and clock() - quiet_since >= .1:
                if clock() >= deadline:
                    raise TimeoutError('출발 전 정지 관측 대기 시간 초과')
                try:
                    mobile.arm(session)
                    return
                except MobileReplyError as exc:
                    diagnostics = exc.diagnostics
                    response = MobileStatus(**diagnostics['response'])
                    command = diagnostics.get('command', {})
                    remaining = (command.get('valid_until_ms', response.mcu_tick_ms)
                                 - response.mcu_tick_ms) & 0xffffffff
                    unarmed_speed_race = (
                        diagnostics['expected_kind'] == 1 and diagnostics['expected_sequence'] == 0
                        and response.kind == 129 and response.request_sequence == 0
                        and mobile.boot_id is not None and response.boot_id == mobile.boot_id
                        and response.state == 0 and response.session == 0
                        and response.sequence == 0 and response.reason == 0
                        and response.flags == (51 if transport_only else 23)
                        and response.capabilities & 7 == 7 and response.feedback_is_fresh(100)
                        and any(response.velocity_raw) and command.get('opcode') == 1
                        and command.get('session') == session and command.get('sequence') == 0
                        and tuple(command.get('velocity_raw', ())) == (0, 0, 0, 0)
                        and 0 < remaining <= 200
                    )
                    if not unarmed_speed_race:
                        raise
                    if on_deferred is not None:
                        on_deferred(diagnostics)
                    quiet_since = last_tick = None
                    samples = 0
        else:
            quiet_since = last_tick = None
            samples = 0
        sleep(.02)


def enter_base_transport(observe, evidence, *, clock=time.monotonic, sleep=time.sleep):
    """Busy read slots may defer mode admission while the platform is parked."""
    deadline = clock() + 2
    while clock() < deadline:
        if not supported_lift_is_holding(observe()):
            raise RuntimeError('출발 전 리프트 유지 확인 실패')
        if evidence.enter_transport():
            return
        sleep(.005)
    raise TimeoutError('이동 모드 전환 대기 시간 초과')


def _reference_not_started(error, observed, top_um):
    """Only a fresh explicit same-boot rejection of a never-started registration."""
    from so101_arm_bridge.lift_wire import LiftStatus
    try:
        command=error.diagnostics['command']
        reply=LiftStatus(**error.diagnostics['response'])
        kind=7 if top_um is None else 8
        return (command['kind']==kind and command['target_um']==(top_um or 0)
                and command['boot_id']==observed.boot_id==reply.boot_id
                and reply.kind==(kind|128) and reply.request_sequence==command['sequence']
                and reply.session==0 and reply.sequence==0 and reply.state==0 and reply.fault==0
                and reply.flags==12 and reply.command_raw==0
                and (reply.mcu_tick_ms-reply.observed_ms)&0xffffffff<200)
    except (AttributeError,KeyError,TypeError,ValueError):
        return False
