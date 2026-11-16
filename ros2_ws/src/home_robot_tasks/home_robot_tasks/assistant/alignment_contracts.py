"""Capture-bound host planning values. None of these values grants motion ownership."""
from dataclasses import dataclass
import hashlib
import numpy as np
from .contracts import Context, Stamp, finite, text, vector
from ..fetch import InvalidTask
from ..s2_geometry import rigid


def cloud(value, minimum=1):
    a = np.array(value, dtype=float, copy=True)
    if a.ndim != 2 or a.shape[1] != 3 or not minimum <= len(a) <= 20000 or not np.isfinite(a).all():
        raise InvalidTask('bounded finite observed cloud required')
    a.setflags(write=False)
    return a


@dataclass(frozen=True)
class TaskScene:
    object_id: str
    surface_id: str
    stamp: Stamp
    scene_revision: str
    model_revision: str
    pose_revision: str
    base_xy_yaw: tuple
    lift_m: float
    points_map_m: object
    support_z_m: float
    source_sha256: dict
    base_stopped: bool = True
    lift_stopped: bool = True

    def __post_init__(self):
        for name in ('object_id', 'surface_id', 'scene_revision', 'model_revision', 'pose_revision'):
            text(getattr(self, name), name)
        if not isinstance(self.stamp, Stamp): raise InvalidTask('capture stamp required')
        object.__setattr__(self, 'base_xy_yaw', vector(self.base_xy_yaw, 3, 'capture base'))
        finite(self.lift_m, 'capture lift', 0, 1)
        finite(self.support_z_m, 'support plane', 0, 3)
        object.__setattr__(self, 'points_map_m', cloud(self.points_map_m, 24))
        hashes = dict(self.source_sha256)
        if not hashes or any(not isinstance(v, str) or len(v) != 64 or set(v)-set('0123456789abcdef') for v in hashes.values()):
            raise InvalidTask('original capture hashes required')
        object.__setattr__(self, 'source_sha256', hashes)
        if self.base_stopped is not True or self.lift_stopped is not True:
            raise InvalidTask('stationary capture required')

    @property
    def identity(self):
        return (self.object_id, self.surface_id, self.stamp.context, self.scene_revision, self.model_revision)

    @property
    def cloud_sha256(self):
        return hashlib.sha256(self.points_map_m.tobytes()).hexdigest()

    def points_after_relative_motion(self, observed_base_from_current_base, *, ground_z_m):
        """Re-express remembered points using measured motion, not the goal.

        The transform must come from the uninterrupted localization interval
        since this stopped exposure. Height/tilt motion may be represented in
        the full rigid transform. This creates coordinates, not a fresh capture
        or a grasp permit; source stamp/hashes and unknown surfaces stay intact.
        """
        x,y,yaw=self.base_xy_yaw;c,s=np.cos(yaw),np.sin(yaw)
        floor=finite(ground_z_m,'base floor offset',-2,2)
        map_from_observed=np.array([[c,-s,0,x],[s,c,0,y],[0,0,1,-floor],[0,0,0,1.]])
        current_from_map=np.linalg.inv(rigid(observed_base_from_current_base))@np.linalg.inv(map_from_observed)
        return self.points_map_m@current_from_map[:3,:3].T+current_from_map[:3,3]

    def fresh(self, current_context, now, maximum_age_s=2):
        self.stamp.check(current_context, now, maximum_age_s)


@dataclass(frozen=True)
class AlignmentSelection:
    scene: TaskScene
    candidate: dict
    selected_s: float
    expires_s: float
    aligned_s: float | None = None

    def check_capture(self, fresh, *, now, context):
        fresh.fresh(context, now)
        if fresh.identity != self.scene.identity:
            raise InvalidTask('aligned scene identity changed')
        if (self.aligned_s is None or fresh.stamp.observed_s <= self.aligned_s
                or fresh.stamp.capture_id == self.scene.stamp.capture_id):
            raise InvalidTask('new exposure after base and lift arrival required')
        if now > self.expires_s: raise InvalidTask('alignment selection expired')
        x, y, yaw = fresh.base_xy_yaw
        tx, ty, tyaw = self.candidate['base_xy_yaw']
        if (np.hypot(x-tx, y-ty) > .03 or abs(np.arctan2(np.sin(yaw-tyaw), np.cos(yaw-tyaw))) > .1
                or abs(fresh.lift_m-self.candidate['lift_travel_m']) > .003):
            raise InvalidTask('new capture outside selected configuration')


@dataclass(frozen=True)
class HeldObject:
    object_id: str
    grasp_id: str
    context: Context
    model_revision: str
    observed_s: float
    tcp_from_object: object
    size_m: tuple
    translation_uncertainty_m: float
    payload_mesh: object = None

    def __post_init__(self):
        for n in ('object_id', 'grasp_id', 'model_revision'): text(getattr(self, n), n)
        if not isinstance(self.context, Context): raise InvalidTask('held object context required')
        matrix = rigid(self.tcp_from_object).copy(); matrix.setflags(write=False)
        object.__setattr__(self, 'tcp_from_object', matrix)
        size = vector(self.size_m, 3, 'held dimensions')
        if any(v <= 0 or v > 1 for v in size): raise InvalidTask('bounded held shape required')
        object.__setattr__(self, 'size_m', size)
        finite(self.observed_s, 'retention time', 0)
        finite(self.translation_uncertainty_m, 'attachment uncertainty', 0, .02)

    def check(self, context, object_id, now, model_revision):
        self.context.check(context)
        if (object_id != self.object_id or model_revision != self.model_revision
                or not 0 <= now-self.observed_s <= .5):
            raise InvalidTask('fresh measured attachment and retention required')
