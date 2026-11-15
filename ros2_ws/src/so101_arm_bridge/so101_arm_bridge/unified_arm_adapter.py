"""Calibrated resident arm routes on the explicitly prepared S4 board.

Reuses S3 range/geometry/gripper behavior. The integrated firmware owns its
observer handoff at first APPEND; sending the old AP suspension is forbidden.
Construction and commissioning queries do not authorize physical execution.
"""
from .stationary_pick_adapter import StationaryPickAdapter
from .bimanual_stream_adapter import (
    ResidentBimanualStreamAdapter, BimanualStreamContractError, TimedJointPoint,
)
from .unified_commissioning import CommissionClient, FIRMWARE


class UnifiedArmAdapter(StationaryPickAdapter):
    firmware_version = FIRMWARE

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._commission = CommissionClient(self._transport)
        self._commission_identity = None
        self._current_pose_only = False

    def _check_commission(self, *, initial=False):
        status = self._commission.query()
        if (not status.flags & 1 or status.flags & 8 or status.reason or
                not status.boot_id or (initial and status.flags not in (17,113))):
            raise BimanualStreamContractError('prepared, non-stopped unified board required')
        identity = (status.boot_id, status.profile_crc32)
        if initial:
            self._commission_identity = identity
        elif identity != self._commission_identity:
            raise BimanualStreamContractError('unified board boot/profile changed')
        if not initial and not status.flags&32:
            raise BimanualStreamContractError('explicit zero-power activation required before arm enable')
        return status

    def prepare(self):
        # Check before PREPARE_SHADOW, which can disable arm torque while
        # capturing an anchor. Never attach or prepare a board implicitly.
        self._check_commission(initial=True)
        return super().prepare()

    def start(self, owner, points, *, finite=False):
        if not self._motion_authorized:
            raise BimanualStreamContractError('motion_authorized is false')
        try:
            self._check_commission()
        except Exception as error:
            self._fault(error)
        return ResidentBimanualStreamAdapter.start(self, owner, points, finite=finite)

    def _initial_batch_base(self, previous_base):
        # OPEN can consume the original lead. APPEND performs the MCU's
        # observer handoff, so only refresh the shared clock here.
        base = self._healthy_heartbeat().last_heartbeat_ms
        self.last_start_timing = dict(previous_base_tick=previous_base,
            initial_batch_base_tick=base, setup_elapsed_ms=(base-previous_base)&0xffffffff,
            policy='unified_mcu_handoff_at_append')
        return base

    def current_pose_hold_points(self):
        """Prepare a finite, constant route for supervised first hold testing.

        Does not arm or transmit. start() refreshes the unpowered anchor again;
        finite completion retains torque and must not be treated as power OFF.
        """
        if self.state.value != 'ready' or self._armed or self._owner is not None:
            raise BimanualStreamContractError('unarmed, unowned READY anchor required')
        q = tuple(value/1e6 for value in self._prepared.positions_urad)
        return tuple(TimedJointPoint(t, q) for t in (100, 200, 300))

    def start_current_pose_hold(self, owner):
        """Bind the constant route to start()'s last torque-OFF measurement.

        Never command a return to the earlier planning snapshot if the
        supported arm settled while mobile power was being prepared.
        """
        points=self.current_pose_hold_points()
        self._current_pose_only=True
        try:return self.start(owner,points,finite=True)
        finally:self._current_pose_only=False

    def _validate_finite_plan_points(self, points):
        if self._current_pose_only:
            q=tuple(value/1e6 for value in self._prepared.positions_urad)
            points=tuple(TimedJointPoint(t,q) for t in (100,200,300))
        return super()._validate_finite_plan_points(points)

    def activate_stationary_hold(self):
        raise BimanualStreamContractError(
            'unified board uses a supervised finite current-pose route; legacy AP hold is unavailable')
