"""Stationary pick observations from the integrated AS/LS interfaces.

The base must never be armed in this session. LS movement admission includes
the MCU's independently measured wheel-stop window, including its established
encoder corroboration of noisy speed bytes. No legacy AP packet is fabricated.
"""
import math
import time
from copy import deepcopy
from dataclasses import asdict, is_dataclass

from .carry_live_inputs import ObservationNotFresh


class PlatformInterlockPending(ObservationNotFresh):
    """Fresh lift hold exists, but the aggregate movement admission is absent."""

    def __init__(self, message, mobile_tick=None):
        super().__init__(message)
        self.mobile_tick = mobile_tick


def read_stationary_sample(read, check, clock, budget_s, *, on_pending=None):
    """Allow the MCU to publish the next wheel sweep while servicing the hold."""
    deadline = clock() + budget_s
    rejected_sweeps = set()
    attempts = 0
    while True:
        check()
        try:
            return read()
        except ObservationNotFresh as error:
            if isinstance(error, PlatformInterlockPending) and error.mobile_tick is not None:
                rejected_sweeps.add(error.mobile_tick)
                if len(rejected_sweeps) >= 3 or clock() >= deadline:
                    raise
                if on_pending:
                    on_pending()
                # Fresh AS/LS reads also service the arm heartbeat. Polling one
                # cached AS sweep three times did not observe three failures.
                time.sleep(.005)
            else:
                attempts += 1
                if on_pending is not None or attempts >= 3 or clock() >= deadline:
                    raise


def controller_lift_height_m(height_um, profile=None, *, board=None):
    """Convert the existing LS controller distance to the selected model's travel.

    Legacy reports use 84 mm/rev. For a bound capture use the installed board's
    distance scale so the encoder calibration is applied exactly once.
    The lift zero offset already lives in the arm mounts.
    """
    from so101_arm_bridge.encoder_units import COUNTS_PER_TURN, LIFT_METERS_PER_TURN
    if type(height_um) is not int:
        raise ValueError('integer controller lift height required')
    controller_turn_m = LIFT_METERS_PER_TURN
    if board is not None:
        from so101_arm_bridge.unified_commissioning import pick_board_scale
        if not isinstance(board, (tuple, list)) or len(board) != 2:
            raise ValueError('controller firmware/profile pair required')
        controller_turn_m = pick_board_scale(*board) / 1e6
    scale = 1.0
    if profile is not None:
        meters_per_count = profile['lift']['meters_per_count']
        if not math.isfinite(meters_per_count) or meters_per_count >= 0:
            raise ValueError('negative up-direction lift calibration required')
        scale = -meters_per_count * COUNTS_PER_TURN / controller_turn_m
    return height_um / 1e6 * scale


class UnifiedPlatformGuard:
    maximum_age_s = .2

    def __init__(self, boot_id, height_m, *, profile=None, board=None, supported_top=False, top_reference=None):
        if type(boot_id) is not int or boot_id <= 0 or not math.isfinite(height_m) or height_m < 0:
            raise ValueError('actual boot and measured lift height required')
        self.boot_id = boot_id
        self.height = self.current_height = height_m
        self.profile = profile
        self.board = board
        self.supported_top = supported_top
        self.top_reference = deepcopy(top_reference)
        self.stamp = -1.
        self.base_stopped = self.lift_stopped = False
        self.last_observation = None

    def update(self, mobile, lift, sent_s, received_s):
        self.last_observation = dict(
            mobile=asdict(mobile) if is_dataclass(mobile) else dict(vars(mobile)),
            lift=asdict(lift) if is_dataclass(lift) else dict(vars(lift)),
            sent_s=sent_s, received_s=received_s, accepted=False)
        if self.board is not None:
            self.last_observation['controller_board'] = list(self.board)
        if self.supported_top:
            from .pick_calibration import TOP_BASIS
            self.last_observation['lift_reference_basis'] = self.top_reference['basis']
            self.last_observation['top_reference'] = deepcopy(self.top_reference)
        self.base_stopped = self.lift_stopped = False
        if (not math.isfinite(sent_s) or not math.isfinite(received_s) or
                received_s < sent_s or mobile.boot_id != self.boot_id or lift.boot_id != self.boot_id):
            raise ValueError('platform boot/clock changed')
        # Report the primary lift fault before the resulting base STOP latch.
        if lift.fault:
            raise ValueError(f'lift fault={lift.fault}, state={lift.state}, '
                             f'height_um={lift.height_um}, target_um={lift.target_um}, '
                             f'current_ma={lift.current_ma}')
        # IDLE with no mobile session is required, not merely a zero command.
        if mobile.state != 0 or mobile.session != 0 or mobile.reason or mobile.flags & 32:
            raise ValueError(f'pick requires an unarmed non-transport base: state={mobile.state}, '
                             f'session={mobile.session}, reason={mobile.reason}, flags={mobile.flags}')
        fixed_station = (self.supported_top and isinstance(self.top_reference,dict)
                         and self.top_reference.get('base_reference_basis')=='operator_fixed_station')
        lift_age = (lift.mcu_tick_ms-lift.observed_ms)&0xffffffff
        age = (lift_age if fixed_station else max(
            (mobile.mcu_tick_ms-mobile.feedback_tick_ms)&0xffffffff,lift_age))/1000
        stamp = sent_s-age
        if (not fixed_station and not mobile.flags & 1) or received_s-stamp > self.maximum_age_s:
            raise ObservationNotFresh('integrated platform measurement stale')
        from .platform_preparation import stationary_platform_is_holding
        independent_hold = (self.supported_top
                            and stationary_platform_is_holding(mobile, lift, self.board, fixed_base_station=fixed_station))
        if (not independent_hold and lift.flags == 25 and lift.state == 5 and lift.command_raw == 0
                and abs(lift.height_um-lift.target_um) <= 2000):
            # LS bit 4 combines asynchronously updated wheel/arm/mode proofs.
            # Never accept this packet or manufacture that bit. The reader may
            # acquire a new complete sample before deciding that proof is lost.
            raise PlatformInterlockPending(
                '리프트 높이 유지 관측은 있으나 이동 인터록 재확인이 필요합니다 '
                f'(flags={lift.flags}, encoder={lift.position_raw})', mobile.feedback_tick_ms)
        if self.supported_top:
            from .pick_calibration import top_model_height
            height = top_model_height(lift, self.profile, self.board, self.top_reference,
                                     stationary_mobile=mobile if independent_hold else None)
        else:
            height = controller_lift_height_m(lift.height_um, self.profile, board=self.board)
        if (not (lift.holding(200) or independent_hold) or lift.flags & 2 or lift.command_raw != 0 or
                abs(height-self.height) > .003):
            raise ValueError('measured fixed lift/base stop proof lost')
        self.current_height = height
        self.stamp = stamp
        self.base_stopped = self.lift_stopped = True
        self.last_observation['accepted'] = True
        self.last_observation['hold_proof'] = ('operator_fixed_base_and_measured_lift_hold' if fixed_station else 'independent_base_stop_and_lift_hold'
                                             if independent_hold else 'movement_interlock')

    def require_fresh(self, now):
        if not 0 <= now-self.stamp <= self.maximum_age_s:
            raise ObservationNotFresh('integrated platform sample stale')
        if not self.base_stopped or not self.lift_stopped:
            raise ValueError('stationary platform unconfirmed')


class UnifiedObservationReader:
    """Single serial owner; existing arm/profile/collision checks remain active."""
    def __init__(self, transport, adapter, mobile, lift, evidence, inputs, clock):
        self.transport, self.adapter = transport, adapter
        self.mobile, self.lift, self.evidence = mobile, lift, evidence
        self.inputs, self.clock = inputs, clock
        self.interlock_refreshes = 0

    def refresh(self):
        def pending():
            self.interlock_refreshes += 1
        return read_stationary_sample(self._refresh_once, lambda: None, self.clock, .1, on_pending=pending)

    def _refresh_once(self):
        self.adapter.keepalive()
        sent = self.clock()
        packet = self.adapter.feedback_snapshot()
        received = self.clock()
        # This is fixed-platform arm motion, never folded-arm driving admission.
        # The MCU base remains disabled, and the lift receives no move command.
        # All positions must be fresh and inside the installed arm envelope.
        limits = self.adapter.motion_limits
        if any(not lo <= q <= hi for q, lo, hi in
               zip(packet.positions_urad, limits.minimum_urad, limits.maximum_urad, strict=True)):
            raise ValueError('measured arm outside calibrated envelope')
        self.evidence.publish_mcu_snapshot(packet, sent, received,
            arms_safe=True, lift_hold=False, payload_safe=True)
        self.inputs.add_joints(packet, sent, received)
        sent = self.clock()
        mobile = self.mobile.status()
        lift = self.lift.status()
        received = self.clock()
        with self.inputs.lock:
            self.inputs.platform.update(mobile, lift, sent, received)
            self.inputs.generation += 1

    def read(self):
        self.refresh()
        return self.inputs.feedback()
