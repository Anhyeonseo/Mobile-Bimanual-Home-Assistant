"""L3A adapter for existing Grounding DINO/SAM reports and measured transforms."""
from dataclasses import dataclass
import hashlib
import math
import numpy as np

from ..fetch import InvalidTask
from .contracts import Context, Stamp, finite, make_entity, text, vector
from .language import NOUNS, COLORS, _matches


@dataclass(frozen=True)
class TransformSample:
    stamp: Stamp
    source_frame: str
    target_frame: str
    matrix: object
    source: str

    def transform(self, xyz, *, context, observation_stamp, maximum_skew_s=.05):
        self.stamp.context.check(context)
        observation_stamp.context.check(context)
        if abs(self.stamp.observed_s-observation_stamp.observed_s) > maximum_skew_s:
            raise InvalidTask('transform_capture_time_mismatch')
        if self.source not in {'measured', 'simulation'} or self.target_frame != context.frame_id:
            raise InvalidTask('transform_frame_or_provenance_mismatch')
        m = np.asarray(self.matrix, dtype=float)
        if m.shape != (4, 4) or not np.isfinite(m).all() or not np.allclose(m[3], [0, 0, 0, 1], atol=1e-8):
            raise InvalidTask('rigid transform required')
        r = m[:3, :3]
        if not np.allclose(r.T@r, np.eye(3), atol=1e-6) or not math.isclose(float(np.linalg.det(r)), 1, abs_tol=1e-6):
            raise InvalidTask('proper rotation required')
        point = np.array([*vector(xyz, 3, 'camera XYZ'), 1])
        return (m@point)[:3].tolist()


def report_entities(report, *, stamp, context, now_s, transform=None,
                    room_id=None, support_id=None, maximum_age_s=5):
    if not isinstance(stamp, Stamp):
        raise InvalidTask('capture stamp required')
    stamp.check(context, now_s, maximum_age_s)
    if not isinstance(report, dict) or type(report.get('schema_version')) is not int or report['schema_version'] not in {1, 2}:
        raise InvalidTask('unsupported grounding report')
    if report.get('coordinate_frame') != 'color_camera_optical_frame':
        raise InvalidTask('unexpected camera frame')
    detections = report.get('detections')
    if not isinstance(detections, list) or len(detections) > 64:
        raise InvalidTask('bounded detections required')
    if transform is not None and (not isinstance(transform, TransformSample) or transform.source_frame != report['coordinate_frame']):
        raise InvalidTask('camera transform required')
    results, seen = [], set()
    for item in detections:
        if not isinstance(item, dict) or type(item.get('id')) is not int or item['id'] < 0 or item['id'] in seen:
            raise InvalidTask('unique detection IDs required')
        seen.add(item['id'])
        label = text(item.get('label'), 'detector label')
        finite(item.get('score'), 'detector score', 0, 1)
        point = item.get('surface_point')
        if not isinstance(point, dict) or type(point.get('valid')) is not bool:
            raise InvalidTask('typed surface validity required')
        reason = point.get('reason') if not point['valid'] else None
        if point['valid']:
            camera = vector(point.get('xyz_m'), 3, 'camera surface point')
            if camera[2] <= 0:
                raise InvalidTask('positive optical depth required')
        if not point['valid'] or transform is None:
            results.append({'detection_id': item['id'], 'entity': None,
                            'reason': reason or 'camera_extrinsics_required', 'grasp_pose': None})
            continue
        categories = [key for key, names in NOUNS.items() if _matches(label, names)]
        colors = [key for key, names in COLORS.items() if _matches(label, names)]
        category = categories[0] if len(categories) == 1 else label.lower()
        digest = hashlib.sha256(f'{context.map_id}:{context.map_revision}:{stamp.capture_id}:{item["id"]}'.encode()).hexdigest()[:20]
        value = make_entity('observation-'+digest, 'object', label, category, stamp,
            attributes={'color': colors[0]} if len(colors) == 1 else {}, room_id=room_id,
            support_id=support_id, position_m=transform.transform(camera, context=context, observation_stamp=stamp),
            geometry_kind='surface_point', source='simulation' if transform.source == 'simulation' else 'sensor')
        results.append({'detection_id': item['id'], 'entity': value,
                        'reason': None, 'grasp_pose': None, 'association_required': True})
    return results


def associate_observation(observation, existing, *, maximum_distance_m):
    """Propose identity matches, keeping same-looking nearby objects ambiguous."""
    distance = finite(maximum_distance_m, 'association distance', .001, 1)
    matches = []
    for value in existing:
        if value['kind'] != 'object' or value['state'] == 'carried' or value['position_m'] is None:
            continue
        if any(value['stamp']['context'][k] != observation['stamp']['context'][k]
               for k in ('map_id', 'map_revision', 'frame_id', 'calibration_id')):
            continue
        if value['category'] != observation['category'] or value['attributes'] != observation['attributes']:
            continue
        if math.dist(value['position_m'], observation['position_m']) <= distance:
            matches.append(value['entity_id'])
    return {'candidate_ids': sorted(matches), 'status': 'NEW' if not matches else 'CONFIRM_ASSOCIATION',
            'automatic_identity_verified': False}
