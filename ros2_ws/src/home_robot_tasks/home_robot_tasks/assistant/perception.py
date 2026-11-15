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
        if self.source not in {'measured', 'simulation', 'recorded'} or self.target_frame != context.frame_id:
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
                    room_id=None, support_id=None, maximum_age_s=5, require_verification=False):
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
        if require_verification:
            checked = item.get('verification')
            categories = [key for key,names in NOUNS.items() if _matches(label,names)]
            if (not isinstance(checked,dict) or checked.get('accepted') is not True or
                    len(categories)!=1 or checked.get('category')!=categories[0]):
                results.append({'detection_id':item['id'],'entity':None,
                                'reason':'object_appearance_unconfirmed','grasp_pose':None})
                continue
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
            geometry_kind='surface_point', state='stale' if transform.source == 'recorded' else 'observed',
            source={'simulation':'simulation','recorded':'inferred','measured':'sensor'}[transform.source])
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


def ingest_object_capture(service, packet):
    """Accept existing detector output, preserving capture clock and provenance.

    Live inputs need a current matching transform. Recorded inputs keep their
    original clock and remain stale, even when uploaded today. A claimed support
    relationship is accepted only with explicit measured support evidence.
    """
    from .contracts import document, Context, entity
    from ..navigation import identifier
    expected = {'request_id','mode','stamp','transform','report','evidence',
                'support_id','support_verified'}
    if not isinstance(packet, dict) or set(packet) != expected:
        raise InvalidTask('invalid_object_capture_fields')
    rid = identifier(packet['request_id'], 'request_id')
    request = service.get(rid)
    if request.get('task_request_id') or request['status'] in {'CANCELLED','INTERRUPTED','SUCCEEDED','FAILED'}:
        raise InvalidTask('request_already_finalized')
    if service.app.lease.owner:
        raise InvalidTask('robot_busy')
    if not isinstance(packet['mode'],str) or packet['mode'] not in {'live','recorded'} or type(packet['support_verified']) is not bool:
        raise InvalidTask('invalid_object_capture_mode')
    stamp = Stamp.parse(packet['stamp'])
    c = service.context
    if any(getattr(stamp.context,k) != getattr(c,k) for k in ('map_id','map_revision','frame_id','calibration_id')):
        raise InvalidTask('object_capture_context_mismatch')
    recorded = packet['mode'] == 'recorded'
    if recorded and stamp.context.clock_id == c.clock_id:
        raise InvalidTask('recorded_capture_cannot_use_live_clock')
    if not recorded:
        stamp.check(c, service.clock(), 5)
    evidence = packet['evidence']
    if (not isinstance(evidence,dict) or set(evidence) != {'rgb_sha256','depth_sha256'} or
        any(not isinstance(v,str) or len(v)!=64 or any(x not in '0123456789abcdef' for x in v) for v in evidence.values())):
        raise InvalidTask('capture_hashes_required')
    tf = packet['transform']
    if not isinstance(tf,dict) or set(tf) != {'stamp','matrix','source_frame','target_frame','source'}:
        raise InvalidTask('complete_camera_transform_required')
    if tf['source'] != ('recorded' if recorded else 'measured'):
        raise InvalidTask('object_transform_provenance_mismatch')
    transform = TransformSample(Stamp.parse(tf['stamp']), tf['source_frame'], tf['target_frame'], tf['matrix'], tf['source'])
    # Validate transform even for an empty/invalid-depth detection report.
    transform.transform([0,0,1], context=stamp.context, observation_stamp=stamp)
    support_id = packet['support_id']
    support = service.store.get(identifier(support_id,'support_id')) if support_id else None
    if support_id and (not support or support['kind']!='surface' or support not in service.store.entities(c)):
        raise InvalidTask('selected_place_not_in_current_map')
    if packet['support_verified'] and (recorded or not support or support['source']!='sensor' or support['geometry_kind']!='support_surface'):
        raise InvalidTask('measured_support_surface_required')
    if packet['support_verified']:
        Stamp.parse(support['stamp']).check(c, service.clock(), 5)
    room_id = support['room_id'] if support and packet['support_verified'] else None
    entries = report_entities(packet['report'], stamp=stamp, context=stamp.context,
        now_s=stamp.observed_s if recorded else service.clock(), transform=transform,
        room_id=room_id, support_id=support_id if packet['support_verified'] else None, require_verification=True)
    key = 'capture-'+hashlib.sha256((c.map_id+':'+stamp.context.clock_id+':'+stamp.capture_id+':'+rid).encode()).hexdigest()[:24]
    fingerprint = hashlib.sha256(document(packet).encode()).hexdigest()
    previous = service.store.record('object_capture',key)
    if previous:
        if previous['input_sha256'] != fingerprint:
            raise InvalidTask('capture_identity_conflict')
        return previous
    prior = [p for p in service.store.records('object_capture') if p['request_id']==rid and p['stamp']['context']['clock_id']==stamp.context.clock_id]
    if any(p['stamp']['observed_s'] > stamp.observed_s for p in prior):
        raise InvalidTask('out_of_order_observation')
    values = []
    for entry in entries:
        value = entry['entity']
        if value:
            # Clock, query and capture form identity. Across captures, matching
            # is proposed separately; a model never silently merges objects.
            value['entity_id'] = 'object-'+hashlib.sha256((key+':'+str(entry['detection_id'])).encode()).hexdigest()[:24]
            value['attributes']['capture_mode'] = packet['mode']
            values.append(entity(value))
    record = dict(capture_key=key,request_id=rid,stamp=stamp.to_dict(),mode=packet['mode'],
        evidence=evidence,input_sha256=fingerprint,entity_ids=[v['entity_id'] for v in values],
        candidate_support_id=support_id,support_verified=packet['support_verified'],
        detections=len(entries),valid_depth_objects=len(values),
        result='NO_DETECTIONS' if not entries else 'NO_CONFIRMED_OBJECT' if not values else 'OBJECT_CANDIDATES',
        rejected=[{'detection_id':e['detection_id'],'reason':e['reason']} for e in entries if not e['entity']],
        physical_grasp_pose_available=False,hardware_commands=0)
    # Whole packet validation precedes one atomic write; malformed later items
    # cannot leave a partially imported scene.
    with service.store.lock, service.store.db:
        for value in values:
            service.store._relations(value)
            service.store.db.execute('INSERT INTO assistant_entities VALUES (?,?)',
                (value['entity_id'],document(value)))
        service.store.db.execute('INSERT INTO assistant_records VALUES (?,?,?)',
            ('object_capture',key,document(record)))
        # save_record commits on this same connection only after the request
        # has been grounded, so capture and selected scene survive together.
        service.refresh_request(rid, capture_key=key)
    return record
