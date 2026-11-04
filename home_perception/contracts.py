"""Layer contracts from MOBILE_MANIPULATION_DESIGN section 7. No device access.

Coordinates are metres. Missing values stay None: a camera-frame observation
never gains a robot transform, and an unresolved request slot is never filled.
"""
from dataclasses import asdict, dataclass, field, fields
import json
import math

NOT_FOUND = 'NOT_FOUND'
AMBIGUOUS_TARGET = 'AMBIGUOUS_TARGET'
INVALID_DEPTH = 'INVALID_DEPTH'
STALE_OBSERVATION = 'STALE_OBSERVATION'
FAILURE_CODES = (NOT_FOUND, AMBIGUOUS_TARGET, INVALID_DEPTH, STALE_OBSERVATION)

ACCEPTED, HELD, REJECTED = 'accepted', 'held', 'rejected'
OBSERVATION_STATUSES = (ACCEPTED, HELD, REJECTED)
ENTITY_STATES = ('observed', 'unconfirmed', 'stale', 'occluded', 'carried', 'released_unconfirmed', 'placed')
ENTITY_SOURCES = ('observed', 'inferred', 'user_confirmed')
GEOMETRY_KINDS = ('none', 'surface_point', 'visible_surface_center')
SCAN_STATUSES = ('running', 'paused', 'stopped', 'saved', 'failed')


def _vector(value, name, size=3):
    if value is None:
        return
    if len(value) != size or any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError(f'{name} must be {size} finite numbers')


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{name} required')


def _fraction(value, name):
    if value is not None and (type(value) not in (int, float) or not 0 <= value <= 1):
        raise ValueError(f'{name} must be within 0..1')


class _Json:
    def to_dict(self):
        return asdict(self)

    def to_json(self):
        return json.dumps(self.to_dict(), ensure_ascii=False, allow_nan=False, sort_keys=True)

    @classmethod
    def from_dict(cls, data):
        known = {item.name for item in fields(cls)}
        if set(data) - known:
            raise ValueError(f'unknown {cls.__name__} fields: {sorted(set(data) - known)}')
        return cls(**data)

    @classmethod
    def from_json(cls, text):
        return cls.from_dict(json.loads(text))


@dataclass
class TaskRequest(_Json):
    """L5 output. Carries meaning only, never coordinates or joint values."""
    request_id: str
    user_id: str
    text: str
    action: str | None = None
    target: str | None = None
    source: str | None = None
    destination: str | None = None
    location: str | None = None
    excluded: list = field(default_factory=list)
    unresolved: list = field(default_factory=list)

    def __post_init__(self):
        for name in ('request_id', 'user_id', 'text'):
            _text(getattr(self, name), name)
        if self.action is None and 'action' not in self.unresolved:
            raise ValueError('a missing action must be listed as unresolved')


@dataclass
class ObjectObservation(_Json):
    """One detection in one capture. L3A/B evidence, not a persistent object."""
    capture_id: str
    captured_at_utc: str
    frame_id: str
    label: str
    box_xyxy: list
    detector: str
    status: str
    calibration_id: str = 'unmeasured'
    clock_domain: str = 'global_time'
    detector_revision: str | None = None
    detector_score: float | None = None
    source_sha256: dict | None = None
    mask_quality: float | None = None
    mask_pixels: int | None = None
    valid_depth_fraction: float | None = None
    point_count: int | None = None
    cluster_fraction: float | None = None
    surface_xyz_m: list | None = None
    center_xyz_m: list | None = None
    obb: dict | None = None
    support_height_m: float | None = None
    point_semantics: str = 'none'
    failure: str | None = None
    reason: str | None = None
    robot_transform: list | None = None

    def __post_init__(self):
        for name in ('capture_id', 'captured_at_utc', 'frame_id', 'label', 'detector'):
            _text(getattr(self, name), name)
        _vector(self.box_xyxy, 'box_xyxy', 4)
        _vector(self.surface_xyz_m, 'surface_xyz_m')
        _vector(self.center_xyz_m, 'center_xyz_m')
        _fraction(self.valid_depth_fraction, 'valid_depth_fraction')
        _fraction(self.cluster_fraction, 'cluster_fraction')
        if self.status not in OBSERVATION_STATUSES:
            raise ValueError('invalid observation status')
        if self.failure is not None and self.failure not in FAILURE_CODES:
            raise ValueError('invalid failure code')
        if self.status == ACCEPTED and (self.failure or self.reason):
            raise ValueError('accepted observation cannot carry a failure')
        if self.status != ACCEPTED and not self.reason:
            raise ValueError('held or rejected observation requires a reason')
        if self.status == REJECTED and (self.center_xyz_m or self.surface_xyz_m):
            raise ValueError('rejected observation cannot publish coordinates')


@dataclass
class WorldEntity(_Json):
    """L3B memory. Unseen entities become stale; they are not deleted."""
    entity_id: str
    kind: str
    state: str
    source: str
    frame_id: str
    position_m: list | None = None
    extent_m: list | None = None
    geometry_kind: str = 'none'
    position_uncertainty_m: float | None = None
    last_seen_utc: str | None = None
    room_id: str | None = None
    support_id: str | None = None
    aliases: list = field(default_factory=list)
    attributes: dict = field(default_factory=dict)
    relations: list = field(default_factory=list)
    evidence: list = field(default_factory=list)
    map_version: str | None = None
    revision: int = 1

    def __post_init__(self):
        for name in ('entity_id', 'kind', 'frame_id'):
            _text(getattr(self, name), name)
        _vector(self.position_m, 'position_m')
        _vector(self.extent_m, 'extent_m')
        if self.state not in ENTITY_STATES or self.source not in ENTITY_SOURCES:
            raise ValueError('invalid entity state or source')
        if self.geometry_kind not in GEOMETRY_KINDS:
            raise ValueError('invalid geometry kind')
        if (self.position_m is None) != (self.geometry_kind == 'none'):
            raise ValueError('geometry kind and position disagree')
        if self.extent_m is not None and any(value <= 0 for value in self.extent_m):
            raise ValueError('extent must be positive')
        if self.entity_id in (self.room_id, self.support_id):
            raise ValueError('entity cannot contain itself')
        if type(self.revision) is not int or self.revision < 1:
            raise ValueError('positive revision required')
        if self.source == 'observed' and not self.evidence:
            raise ValueError('observed entity requires evidence')


@dataclass
class ScanSession(_Json):
    session_id: str
    sensor_id: str
    calibration_id: str
    status: str
    coverage_fraction: float | None = None
    map_revision: str | None = None

    def __post_init__(self):
        for name in ('session_id', 'sensor_id', 'calibration_id'):
            _text(getattr(self, name), name)
        _fraction(self.coverage_fraction, 'coverage_fraction')
        if self.status not in SCAN_STATUSES:
            raise ValueError('invalid scan status')
