"""Measured joint-pose teaching and a camera-free, single-arm filming sequence.

Uses the existing 12-axis resident adapter. The other arm holds its initial
measured pose; no wheel/lift commands or firmware changes are made here.
"""
from __future__ import annotations

import copy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time

from .bimanual_stream_adapter import (
    AdapterState, CALIBRATION_HASH, CANONICAL_JOINT_NAMES, F8_FIRMWARE_VERSION,
    ResidentBimanualStreamAdapter, TimedJointPoint,
)

STAGES = ("home", "approach", "grasp", "closed", "lift")
LABELS = dict(zip(STAGES, ("기본 자세·그리퍼 열림", "물체 위 접근·열림",
                         "집는 위치·열림", "같은 위치·그리퍼 닫힘", "들어 올린 자세·닫힘")))
OWNER = "fixed-pick-demo"
MAX_CAPTURE_SPREAD = 0.02
START_TOLERANCE = 0.05
POSE_TOLERANCE = 0.03
SPEED_RAD_S = 0.20
ACCEL_RAD_S2 = 0.40
SAMPLE_MS = 50
START_ANCHOR_TOLERANCE = 0.01


def load_json(path: Path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"중복 JSON 키: {key}")
            result[key] = value
        return result
    return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=unique)


def limits_digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vector(values, limits):
    count = len(limits.joint_names)
    if count not in (6, 12) or not isinstance(values, (list, tuple)) or len(values) != count:
        raise ValueError(f"{count}개 관절값이 필요합니다.")
    for i, value in enumerate(values):
        if type(value) not in (float, int) or not math.isfinite(value):
            raise ValueError("관절값은 유한한 숫자여야 합니다.")
        if not limits.minimum_urad[i] <= value * 1e6 <= limits.maximum_urad[i]:
            raise ValueError(f"관절 범위 초과: {limits.joint_names[i]}")
    return tuple(float(v) for v in values)


def arm_indices(arm):
    if arm not in ("left", "right"):
        raise ValueError("arm은 left 또는 right여야 합니다.")
    return tuple(range(0, 6)) if arm == "left" else tuple(range(6, 12))


def require_home(actual, home):
    if max(abs(a - b) for a, b in zip(actual, home, strict=True)) > START_TOLERANCE:
        raise ValueError("현재 팔 자세가 저장한 home과 다릅니다. 손으로 기본 자세를 맞춘 뒤 다시 실행하세요.")


def require_tracking(actual, target):
    if any(abs(a - b) > (.15 if i in (5, 11) else .09)
           for i, (a, b) in enumerate(zip(actual, target, strict=True))):
        raise ValueError("유지 중인 자세의 관절 오차가 제한을 넘었습니다.")


class FixedPickAdapter(ResidentBimanualStreamAdapter):
    """Check taught home again on the resident adapter's FINAL pre-ARM read."""

    def __init__(self, transport, limits, *, home_reference, motion_authorized=False):
        super().__init__(transport, limits, motion_authorized=motion_authorized)
        self._demo_limits = limits
        self._demo_home = vector(home_reference, limits)
        self._demo_start = None

    def refresh_unarmed_anchor(self):
        anchor = super().refresh_unarmed_anchor()
        actual = vector(tuple(q / 1e6 for q in anchor.positions_urad), self._demo_limits)
        require_home(actual, self._demo_home)
        if self._demo_start is not None and max(
            abs(a - b) for a, b in zip(actual, self._demo_start, strict=True)
        ) > START_ANCHOR_TOLERANCE:
            raise ValueError("출발 직전 팔 자세가 변했습니다. 팔을 안정적으로 지지하고 다시 실행하세요.")
        return anchor

    def start(self, owner, points, *, finite):
        if not points:
            raise ValueError("빈 이동 경로입니다.")
        if not self.heartbeat_required:
            self._demo_start = vector(points[0].positions_rad, self._demo_limits)
        try:
            # super().start refreshes once more immediately before ARM. The
            # override above checks that last sample, not just the early plan.
            return super().start(owner, points, finite=finite)
        finally:
            self._demo_start = None


def new_recipe(arm, digest, *, left_only=False):
    arm_indices(arm)
    if left_only:
        if arm != "left":
            raise ValueError("왼팔 단독 모드에서는 left만 사용할 수 있습니다.")
        from .left_only_adapter import LEFT_FIRMWARE_VERSION
        return dict(schema_version=2, record_kind="measured_left_only_fixed_pick_demo", arm="left",
                    firmware_version=LEFT_FIRMWARE_VERSION, calibration_hash=CALIBRATION_HASH,
                    limits_sha256=digest, joint_names=list(CANONICAL_JOINT_NAMES[:6]), poses={})
    return dict(schema_version=1, record_kind="measured_fixed_pick_demo", arm=arm,
                firmware_version=F8_FIRMWARE_VERSION, calibration_hash=CALIBRATION_HASH,
                limits_sha256=digest, joint_names=list(CANONICAL_JOINT_NAMES), poses={})


def validate_recipe(recipe, limits, digest, *, complete=True):
    if not isinstance(recipe, dict):
        raise ValueError("자세 파일은 JSON 객체여야 합니다.")
    expected = new_recipe(recipe.get("arm"), digest, left_only=len(limits.joint_names) == 6)
    for key in expected.keys() - {"poses"}:
        if type(recipe.get(key)) is not type(expected[key]) or recipe[key] != expected[key]:
            raise ValueError(f"자세 파일의 설정이 현재 제어기와 다릅니다: {key}")
    poses = recipe.get("poses")
    if not isinstance(poses, dict) or set(poses) - set(STAGES):
        raise ValueError("알 수 없는 자세 이름입니다.")
    if complete and set(poses) != set(STAGES):
        raise ValueError("저장하지 않은 자세: " + ", ".join(s for s in STAGES if s not in poses))
    values = {}
    for stage, record in poses.items():
        if not isinstance(record, dict) or record.get("source") != "resident_torque_off_samples":
            raise ValueError(f"실측 자세가 아닙니다: {stage}")
        if type(record.get("sample_count")) is not int or record["sample_count"] != 3:
            raise ValueError("자세 저장에는 새 측정 3회가 필요합니다.")
        spread = record.get("max_spread_rad")
        if type(spread) not in (int, float) or not math.isfinite(spread) or not 0 <= spread <= MAX_CAPTURE_SPREAD:
            raise ValueError(f"측정 중 자세가 움직였습니다: {stage}")
        if not isinstance(record.get("captured_at"), str) or not record["captured_at"]:
            raise ValueError("자세의 측정 시각이 없습니다.")
        values[stage] = vector(record.get("positions_rad"), limits)
    if not complete:
        return values
    indices = arm_indices(recipe["arm"])
    gripper = indices[-1]
    if max(abs(values["closed"][i] - values["grasp"][i]) for i in indices[:-1]) > POSE_TOLERANCE:
        raise ValueError("closed는 grasp와 같은 팔 자세에서 그리퍼만 닫아 저장하세요.")
    for stage, reference in (("approach", "home"), ("grasp", "home"), ("lift", "closed")):
        if abs(values[stage][gripper] - values[reference][gripper]) > POSE_TOLERANCE:
            raise ValueError(f"그리퍼 상태 불일치: {stage} / {reference}")
    # These motors use semantic Q0=open, more-negative=closed on both arms.
    if values["grasp"][gripper] - values["closed"][gripper] < 0.05:
        raise ValueError("closed의 그리퍼가 grasp보다 닫힌 실측값이어야 합니다.")
    return values


def capture_pose(adapter, limits, *, sleep=time.sleep):
    if adapter.heartbeat_required:
        raise ValueError("구동 중인 세션에서 수동 자세를 저장할 수 없습니다.")
    samples = []
    for index in range(3):
        if index:
            sleep(0.08)
        sample = adapter.refresh_unarmed_anchor()
        samples.append(vector(tuple(v / 1e6 for v in sample.positions_urad), limits))
    spread = max(max(row[i] for row in samples) - min(row[i] for row in samples) for i in range(len(limits.joint_names)))
    if spread > MAX_CAPTURE_SPREAD:
        raise ValueError("측정 중 팔이 움직였습니다. 팔을 지지하고 다시 저장하세요.")
    # Preserve an actual measured sample, not an averaged, unobserved pose.
    return dict(source="resident_torque_off_samples", sample_count=3,
                max_spread_rad=spread, positions_rad=list(samples[-1]),
                captured_at=datetime.now(timezone.utc).isoformat())


def compose_targets(recipe, measured_home, limits, digest):
    values = validate_recipe(recipe, limits, digest)
    start = vector(measured_home, limits)
    require_home(start, values["home"])
    indices = arm_indices(recipe["arm"])
    targets = {}
    for stage in STAGES:
        q = list(start)
        for i in indices:
            q[i] = values[stage][i]
        targets[stage] = tuple(q)
    # During close/open do not move the five arm joints. Maintain exact jaw
    # targets while approaching, lifting and returning, despite teaching noise.
    gi = indices[-1]
    for stage in ("approach", "grasp"):
        q = list(targets[stage]); q[gi] = targets["home"][gi]; targets[stage] = tuple(q)
    q = list(targets["grasp"]); q[gi] = values["closed"][gi]; targets["closed"] = tuple(q)
    q = list(targets["lift"]); q[gi] = targets["closed"][gi]; targets["lift"] = tuple(q)
    return targets


def smooth_leg(start, target, limits):
    start, target = vector(start, limits), vector(target, limits)
    delta = max(abs(b - a) for a, b in zip(start, target))
    # Quintic blend: peak first derivative 1.875, peak second < 5.774.
    seconds = max(1., 1.875 * delta / SPEED_RAD_S, math.sqrt(5.774 * delta / ACCEL_RAD_S2))
    steps = math.ceil(seconds * 1000 / SAMPLE_MS)
    if steps * SAMPLE_MS > 60000:
        raise ValueError("한 구간이 60초를 넘습니다. 저장한 자세를 확인하세요.")
    points = [TimedJointPoint(100, start)]
    for step in range(1, steps + 1):
        u = step / steps
        blend = u ** 3 * (10 + u * (-15 + 6 * u))
        q = tuple(a + (b - a) * blend for a, b in zip(start, target))
        points.append(TimedJointPoint(100 + step * SAMPLE_MS, target if step == steps else q))
    return tuple(points)


def check_recipe(recipe, limits, digest):
    values = validate_recipe(recipe, limits, digest)
    targets = compose_targets(recipe, values["home"], limits, digest)
    result, current = [], targets["home"]
    for stage in ("approach", "grasp", "closed", "lift", "closed", "grasp", "approach", "home"):
        points = smooth_leg(current, targets[stage], limits)
        result.append(dict(stage=stage, seconds=round(points[-1].offset_ms / 1000, 2)))
        current = targets[stage]
    return dict(status="JOINT_PATH_CHECKED", arm=recipe["arm"], legs=result,
                collision_checked=False, physical_pick_verified=False,
                max_joint_speed_rad_s=SPEED_RAD_S)


def measured_feedback(adapter, limits):
    snapshot = adapter.feedback_snapshot()
    ages = snapshot.sample_age_ms
    if len(ages) != len(limits.joint_names) or any(type(a) is not int or not 0 <= a <= 150 for a in ages):
        raise ValueError("관절 피드백이 누락되거나 오래되었습니다.")
    return vector(tuple(v / 1e6 for v in snapshot.positions_urad), limits)


def execute_leg(adapter, target, limits, *, commanded_start=None, interrupt=None,
                clock=time.monotonic, sleep=time.sleep):
    if interrupt:
        interrupt()
    if not adapter.heartbeat_required:
        anchor = adapter.refresh_unarmed_anchor()
        current = vector(tuple(v / 1e6 for v in anchor.positions_urad), limits)
    else:
        measured = measured_feedback(adapter, limits)
        if commanded_start is None:
            raise ValueError("유지 중인 이전 목표가 없어 다음 구간을 시작할 수 없습니다.")
        current = vector(commanded_start, limits)
        require_tracking(measured, current)
        # Keep the commanded jaw/other-arm targets continuous. Replacing them
        # with measured tracking error can open a loaded jaw or move the idle
        # arm at every segment boundary.
    points = smooth_leg(current, target, limits)
    if interrupt:
        interrupt()
    adapter.start(OWNER, points, finite=True)
    deadline = clock() + points[-1].offset_ms / 1000 + 5
    while adapter.state is AdapterState.ACTIVE:
        if interrupt:
            interrupt()
        if clock() > deadline:
            raise TimeoutError("자세 도착 확인 시간이 초과되었습니다.")
        adapter.poll(OWNER)
        if adapter.state is AdapterState.ACTIVE:
            sleep(.01)
    if adapter.state is not AdapterState.READY:
        raise RuntimeError("팔 제어기가 정상 대기 상태로 돌아오지 않았습니다.")
    if interrupt:
        interrupt()


def put_capture(recipe, stage, record, limits, digest):
    if stage not in STAGES:
        raise ValueError("알 수 없는 자세 이름입니다.")
    result = copy.deepcopy(recipe)
    result["poses"][stage] = record
    validate_recipe(result, limits, digest, complete=False)
    return result
