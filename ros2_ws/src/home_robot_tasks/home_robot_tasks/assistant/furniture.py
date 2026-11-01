"""RGB-D furniture observations and persistent, estimated instance association.

This layer answers WHAT is WHERE. It never marks free space and its partial
bounds are neither a complete furniture mesh nor a verified support surface.
Navigation consumes all native depth independently of detection/segmentation.
"""
from copy import deepcopy
import hashlib
import numpy as np

from ..fetch import InvalidTask
from ..navigation import identifier
from .contracts import Stamp, document, finite
from .perception import TransformSample


FURNITURE_CATEGORIES = ('bed', 'sofa', 'desk', 'table', 'coffee_table', 'chair',
                        'wardrobe', 'bookshelf', 'nightstand', 'cabinet')
FURNITURE_PROMPT = ' . '.join(c.replace('_', ' ') for c in FURNITURE_CATEGORIES) + ' .'


def masked_furniture(*, category, detector_score, detection_id, mask, depth_m,
                     intrinsics, stamp, context, transform, now_s,
                     voxel_size_m=.05, minimum_depth_m=.3, maximum_depth_m=5.):
    """Deproject an instance mask with rectified, RGB-aligned metric depth.

    Any model may provide the mask; SAM is one existing adapter. Use the RGB
    intrinsics of that aligned image. Filled/colourized depth is not a
measurement source. The caller must pin the camera profile and model versions.
Native depth remains a separate input to the collision pipeline.
"""
    if not isinstance(stamp, Stamp):
        raise InvalidTask('furniture capture stamp required')
    stamp.check(context, now_s, 5.)
    if not isinstance(category, str) or category not in FURNITURE_CATEGORIES:
        raise InvalidTask('unsupported furniture category')
    identifier(detection_id, 'detection_id')
    score = finite(detector_score, 'detector score', 0, 1)
    size = finite(voxel_size_m, 'semantic voxel size', .02, .2)
    near = finite(minimum_depth_m, 'minimum depth', .01, 10)
    far = finite(maximum_depth_m, 'maximum depth', near, 10)
    if near >= far:
        raise InvalidTask('invalid depth interval')
    try:
        d, m = np.asarray(depth_m, dtype=float), np.asarray(mask)
    except (ValueError, TypeError, OverflowError) as error:
        raise InvalidTask('numeric depth and boolean mask required') from error
    if (d.ndim != 2 or m.shape != d.shape or m.dtype != np.bool_ or
            not 9 <= d.size <= 1280*720 or min(d.shape) < 3):
        raise InvalidTask('aligned depth and boolean mask required')
    if not isinstance(intrinsics, (tuple, list)) or len(intrinsics) != 4:
        raise InvalidTask('rectified RGB intrinsics required')
    fx, fy, cx, cy = [finite(v, 'intrinsics') for v in intrinsics]
    if fx <= 0 or fy <= 0 or not 0 <= cx < d.shape[1] or not 0 <= cy < d.shape[0]:
        raise InvalidTask('invalid RGB intrinsics')
    if not isinstance(transform, TransformSample) or transform.source_frame != 'color_camera_optical_frame':
        raise InvalidTask('capture-time RGB transform required')
    transform.transform((0, 0, 0), context=context, observation_stamp=stamp)
    # Exclude the mixed foreground/background edge. No bounding-box fill.
    core = np.zeros(m.shape, dtype=bool)
    core[1:-1, 1:-1] = np.logical_and.reduce([
        m[dy:dy+m.shape[0]-2, dx:dx+m.shape[1]-2]
        for dy in range(3) for dx in range(3)])
    valid = core & np.isfinite(d) & (d >= near) & (d <= far)
    if valid.sum() < 8 or valid.sum()/max(1, core.sum()) < .25:
        raise InvalidTask('insufficient_furniture_depth_reobserve')
    ys, xs = np.nonzero(valid)
    zs = d[ys, xs]
    camera = np.column_stack(((xs-cx)*zs/fx, (ys-cy)*zs/fy, zs))
    matrix = np.asarray(transform.matrix, dtype=float)
    points = camera @ matrix[:3, :3].T + matrix[:3, 3]
    if not np.isfinite(points).all() or np.abs(points).max() > 10000:
        raise InvalidTask('invalid mapped furniture geometry')
    cells = np.unique(np.floor(points/size).astype(np.int64), axis=0)
    if not 4 <= len(cells) <= 4096:
        raise InvalidTask('furniture_geometry_budget_or_extent_invalid')
    digest = hashlib.sha256(f'{document(context.to_dict())}:{stamp.capture_id}:{detection_id}'.encode()).hexdigest()[:24]
    return dict(observation_id='fobs-'+digest, category=category, detector_score=score,
        stamp=stamp.to_dict(), source=transform.source, voxel_size_m=size,
        observed_voxels=cells.tolist(), extent_complete=False,
        depth_valid_fraction=float(valid.sum()/max(1, core.sum())))


def _validated(value, context, now_s):
    fields = {'observation_id', 'category', 'detector_score', 'stamp', 'source',
              'voxel_size_m', 'observed_voxels', 'extent_complete', 'depth_valid_fraction'}
    if not isinstance(value, dict) or set(value) != fields:
        raise InvalidTask('invalid furniture observation')
    value = deepcopy(value)
    identifier(value['observation_id'], 'furniture observation')
    Stamp.parse(value['stamp']).check(context, now_s, 5.)
    if value['category'] not in FURNITURE_CATEGORIES or value['source'] not in ('measured', 'simulation') or value['extent_complete'] is not False:
        raise InvalidTask('invalid furniture provenance or extent')
    finite(value['detector_score'], 'score', 0, 1)
    finite(value['depth_valid_fraction'], 'valid depth fraction', .25, 1)
    finite(value['voxel_size_m'], 'voxel size', .02, .2)
    cells = value['observed_voxels']
    if (not isinstance(cells, list) or not 4 <= len(cells) <= 4096 or
            any(not isinstance(c, list) or len(c) != 3 or
                any(type(i) is not int or abs(i) > 500000 for i in c) for c in cells) or
            len({tuple(c) for c in cells}) != len(cells)):
        raise InvalidTask('bounded unique observed voxels required')
    document(value)
    return value


class FurnitureMemory:
    """Bounded replay mapper. Matches are estimates, never grasp identities.

Unique same-class voxel overlap can accumulate views without user labelling.
No overlap keeps distinct instances; several matches preserve ambiguity. This
simple association deliberately does not claim learned visual re-identification.
"""
    def __init__(self, store, context):
        self.store, self.context = store, context

    def instances(self):
        return [v for v in self.store.records('furniture_instance')
                if all(v['context'][key] == getattr(self.context, key)
                       for key in ('map_id', 'map_revision', 'frame_id', 'calibration_id'))]

    def observe(self, observation, *, now_s):
        obs = _validated(observation, self.context, now_s)
        key = obs['observation_id']
        with self.store.lock:
            old = self.store.record('furniture_observation', key)
            if old:
                if old['observation'] != obs:
                    raise InvalidTask('furniture_observation_id_conflict')
                return old['association']
            if len(self.store.records('furniture_observation')) >= 256:
                raise InvalidTask('furniture_observation_budget_exceeded')
            cells = {tuple(c) for c in obs['observed_voxels']}
            candidates = []
            for previous in self.instances():
                if (previous['category'], previous['source'], previous['voxel_size_m']) != (
                        obs['category'], obs['source'], obs['voxel_size_m']):
                    continue
                other = {tuple(c) for c in previous['observed_voxels']}
                overlap = len(cells & other)/min(len(cells), len(other))
                if overlap >= .35:
                    candidates.append(previous)
            instance = None
            if len(candidates) <= 1:
                previous = candidates[0] if candidates else None
                if (previous and previous['context']['clock_id'] == self.context.clock_id and
                        obs['stamp']['observed_s'] < previous['last_observed_s']):
                    raise InvalidTask('out_of_order_furniture_observation')
                if not previous and len(self.instances()) >= 128:
                    raise InvalidTask('furniture_instance_budget_exceeded')
                merged = cells | ({tuple(c) for c in previous['observed_voxels']} if previous else set())
                if len(merged) > 16384:
                    raise InvalidTask('furniture_instance_geometry_budget_exceeded')
                size = obs['voxel_size_m']
                lower = [min(c[i] for c in merged)*size for i in range(3)]
                upper = [(max(c[i] for c in merged)+1)*size for i in range(3)]
                instance = dict(instance_id=previous['instance_id'] if previous else key.replace('fobs-', 'furniture-'),
                    category=obs['category'], context=self.context.to_dict(), source=obs['source'],
                    voxel_size_m=size, observed_voxels=[list(c) for c in sorted(merged)],
                    observed_bounds_m={'min': lower, 'max': upper}, extent_complete=False,
                    last_observed_s=obs['stamp']['observed_s'], last_capture_id=obs['stamp']['capture_id'],
                    observation_count=(previous['observation_count'] if previous else 0)+1,
                    association_method='unique_voxel_overlap_estimate', identity_verified=False,
                    geometry_freshness='accumulated_history_not_current_occupancy',
                    placement_surface_verified=False)
            result = dict(status='ASSOCIATED_ESTIMATE' if candidates and instance else
                          ('NEW_ESTIMATE' if instance else 'AMBIGUOUS'),
                          instance_id=instance['instance_id'] if instance else None,
                          candidate_ids=sorted(c['instance_id'] for c in candidates),
                          motion_authorized=False)
            encoded = document(dict(observation=obs, association=result))
            encoded_instance = document(instance) if instance else None
            # Observation and associated geometry commit together.
            with self.store.db:
                self.store.db.execute('INSERT INTO assistant_records VALUES (?,?,?)',
                                      ('furniture_observation', key, encoded))
                if instance:
                    self.store.db.execute('INSERT INTO assistant_records VALUES (?,?,?) ON CONFLICT(kind,id) DO UPDATE SET body=excluded.body',
                                          ('furniture_instance', instance['instance_id'], encoded_instance))
            return result
