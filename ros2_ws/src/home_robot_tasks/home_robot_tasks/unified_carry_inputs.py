"""Stationary pick observations from the integrated AS/LS interfaces.

The base must never be armed in this session. LS movement admission includes
the MCU's independently measured wheel-stop window, including its established
encoder corroboration of noisy speed bytes. No legacy AP packet is fabricated.
"""
import math
from dataclasses import asdict, is_dataclass

from .carry_live_inputs import ObservationNotFresh


class UnifiedPlatformGuard:
    maximum_age_s = .2

    def __init__(self, boot_id, height_m):
        if type(boot_id) is not int or boot_id <= 0 or not math.isfinite(height_m) or height_m < 0:
            raise ValueError('actual boot and measured lift height required')
        self.boot_id = boot_id
        self.height = self.current_height = height_m
        self.stamp = -1.
        self.base_stopped = self.lift_stopped = False
        self.last_observation = None

    def update(self, mobile, lift, sent_s, received_s):
        self.last_observation = dict(
            mobile=asdict(mobile) if is_dataclass(mobile) else dict(vars(mobile)),
            lift=asdict(lift) if is_dataclass(lift) else dict(vars(lift)),
            sent_s=sent_s, received_s=received_s, accepted=False)
        self.base_stopped = self.lift_stopped = False
        if (not math.isfinite(sent_s) or not math.isfinite(received_s) or
                received_s < sent_s or mobile.boot_id != self.boot_id or lift.boot_id != self.boot_id):
            raise ValueError('platform boot/clock changed')
        # IDLE with no mobile session is required, not merely a zero command.
        if mobile.state != 0 or mobile.session != 0 or mobile.reason or mobile.flags & 32:
            raise ValueError('pick requires an unarmed non-transport base')
        age = max((mobile.mcu_tick_ms-mobile.feedback_tick_ms)&0xffffffff,
                  (lift.mcu_tick_ms-lift.observed_ms)&0xffffffff)/1000
        stamp = sent_s-age
        if not mobile.flags & 1 or received_s-stamp > self.maximum_age_s:
            raise ObservationNotFresh('integrated platform measurement stale')
        if (not lift.holding(200) or lift.flags & 2 or lift.command_raw != 0 or
                abs(lift.height_um/1e6-self.height) > .003):
            raise ValueError('measured fixed lift/base stop proof lost')
        self.current_height = lift.height_um/1e6
        self.stamp = stamp
        self.base_stopped = self.lift_stopped = True
        self.last_observation['accepted'] = True

    def require_fresh(self, now):
        if not 0 <= now-self.stamp <= self.maximum_age_s:
            raise ValueError('integrated platform sample stale')
        if not self.base_stopped or not self.lift_stopped:
            raise ValueError('stationary platform unconfirmed')


class UnifiedObservationReader:
    """Single serial owner; existing arm/profile/collision checks remain active."""
    def __init__(self, transport, adapter, mobile, lift, evidence, inputs, clock):
        self.transport, self.adapter = transport, adapter
        self.mobile, self.lift, self.evidence = mobile, lift, evidence
        self.inputs, self.clock = inputs, clock

    def refresh(self):
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
