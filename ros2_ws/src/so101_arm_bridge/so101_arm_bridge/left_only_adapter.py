"""Six real left joints over a fixed-size v2 wire envelope.

Unused slots are reserved zero padding, never measurements. Their firmware
present bits are clear and their ages are UINT32_MAX. The public interface and
saved recipes expose only the six measured joints. Old bimanual tools reject
this profile's distinct firmware identity, capabilities and joint count.
"""
from dataclasses import replace
from .bimanual_stream_adapter import (
    ResidentBimanualStreamAdapter, OperationalLimits, TimedJointPoint,
    BimanualStreamError, CALIBRATION_HASH, CANONICAL_JOINT_NAMES,
)
from .fixed_pick_demo import vector, require_home, START_ANCHOR_TOLERANCE

LEFT_FIRMWARE_VERSION = 0x00024A01
LEFT_CAPABILITIES = 0xEF81FFFF


def compatible_left(hello):
    return (hello.protocol_version == 2 and hello.joint_count == 6
            and hello.firmware_version == LEFT_FIRMWARE_VERSION
            and hello.left_calibration_hash == hello.right_calibration_hash == CALIBRATION_HASH
            and hello.capabilities == LEFT_CAPABILITIES and not hello.stop_latched)


def left_limits(limits):
    if tuple(limits.joint_names[:6]) != CANONICAL_JOINT_NAMES[:6]:
        raise ValueError("왼팔 관절 순서가 다릅니다.")
    return replace(limits, joint_names=limits.joint_names[:6],
                   minimum_urad=limits.minimum_urad[:6], maximum_urad=limits.maximum_urad[:6])


class _LeftWireAdapter(ResidentBimanualStreamAdapter):
    home_tolerance = None
    firmware_version = LEFT_FIRMWARE_VERSION
    capabilities = LEFT_CAPABILITIES
    physical_joint_count = 6
    right_present_mask = 0
    feedback_present_mask = 0x3F
    arm_mask = 1

    def __init__(self, transport, limits, *, home_reference=None, motion_authorized=False):
        self.physical_limits = limits
        self.home_reference = None if home_reference is None else vector(home_reference, limits)
        self.planned_start = None
        wire_limits = OperationalLimits(CANONICAL_JOINT_NAMES,
            tuple(limits.minimum_urad)+(0,)*6, tuple(limits.maximum_urad)+(0,)*6,
            limits.source_sha256, hardware_authorized=limits.hardware_authorized)
        super().__init__(transport, wire_limits, motion_authorized=motion_authorized)

    def _prepared_from_snapshot(self, snapshot):
        if any(snapshot.positions_raw[6:]) or any(snapshot.unwrapped_positions_raw[6:]) or any(snapshot.anchor_positions_urad[6:]):
            raise BimanualStreamError("왼팔 전용 응답에 잘못된 오른팔 데이터가 있습니다.")
        return super()._prepared_from_snapshot(snapshot)

    def _validated_feedback_snapshot(self):
        value = super()._validated_feedback_snapshot()
        if any(value.positions_urad[6:]) or tuple(value.sample_age_ms[6:]) != (0xFFFFFFFF,)*6:
            raise BimanualStreamError("왼팔 전용 피드백의 비활성 관절 표시가 잘못됐습니다.")
        return value

    def refresh_unarmed_anchor(self):
        anchor = super().refresh_unarmed_anchor()
        actual = vector(tuple(q/1e6 for q in anchor.positions_urad[:6]), self.physical_limits)
        if self.home_reference is not None:
            if self.home_tolerance is None:
                require_home(actual, self.home_reference)
            elif max(abs(a-b) for a,b in zip(actual,self.home_reference,strict=True)) > self.home_tolerance:
                raise ValueError("현재 왼팔을 저장한 home 근처에 놓아주세요.")
        if self.planned_start is not None and max(abs(a-b) for a,b in zip(actual,self.planned_start,strict=True)) > START_ANCHOR_TOLERANCE:
            raise ValueError("출발 직전 왼팔 자세가 변했습니다. 안정적으로 받친 뒤 다시 실행하세요.")
        return anchor


class LeftOnlyAdapter:
    """Public adapter: exactly six physical positions, no synthetic arm."""
    def __init__(self, transport, limits, *, home_reference=None, motion_authorized=False):
        if len(limits.joint_names) != 6:
            raise ValueError("왼팔 단독 어댑터에는 관절 6개가 필요합니다.")
        self._limits = limits
        self._wire = _LeftWireAdapter(transport, limits, home_reference=home_reference,
                                     motion_authorized=motion_authorized)

    @property
    def state(self): return self._wire.state
    @property
    def heartbeat_required(self): return self._wire.heartbeat_required

    @staticmethod
    def _physical(anchor):
        return replace(anchor, positions_raw=anchor.positions_raw[:6],
            unwrapped_positions_raw=anchor.unwrapped_positions_raw[:6], positions_urad=anchor.positions_urad[:6])

    def prepare(self): return self._physical(self._wire.prepare())
    def refresh_unarmed_anchor(self): return self._physical(self._wire.refresh_unarmed_anchor())
    def keepalive(self, **kwargs): return self._wire.keepalive(**kwargs)
    def stop(self, owner): return self._wire.stop(owner)
    def poll(self, owner): return self._wire.poll(owner)
    def feedback_snapshot(self):
        value = self._wire.feedback_snapshot()
        return replace(value, positions_urad=value.positions_urad[:6], sample_age_ms=value.sample_age_ms[:6])
    def start(self, owner, points, *, finite):
        if not points: raise ValueError("빈 경로입니다.")
        physical = [TimedJointPoint(p.offset_ms,vector(p.positions_rad,self._limits)) for p in points]
        self._wire.planned_start = None if self.heartbeat_required else physical[0].positions_rad
        try:
            return self._wire.start(owner,[TimedJointPoint(p.offset_ms,p.positions_rad+(0.,)*6) for p in physical],finite=finite)
        finally:
            self._wire.planned_start = None
