"""Read-only S2 coordinates from nominal geometry and operator-selected references.

This is deliberately separate from measured commissioning and executable L3 entities.
No device I/O, motor targets, shortest-angle wrapping, or inferred live origins.
"""
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import json
import numpy as np
from . import s2_geometry as s2
from . import so101_joint_coordinates as arm_coordinates
from .s2_urdf import render_observation_urdf

KIND = 's2_nominal_development_coordinates'


def camera_transform(camera):
    """Apply an explicit base-Z registration correction once to the nominal pose."""
    transform=s2.rigid(camera['base_from_optical']).copy()
    adjustment=camera.get('operator_height_adjustment')
    if camera['status']=='nominal_model_retained_by_operator':
        if adjustment is not None:raise ValueError('height adjustment requires explicit operator-adjusted status')
    elif camera['status']=='nominal_with_operator_height_adjustment':
        if (not isinstance(adjustment,dict) or adjustment.get('operator_confirmed') is not True
                or adjustment.get('frame')!='base_link' or adjustment.get('axis')!='z'
                or adjustment.get('measured_extrinsic') is not False):
            raise ValueError('explicit operator vertical registration evidence required')
        transform[2,3]+=s2.number(adjustment['offset_m'],'camera height adjustment',.05)
    else:
        raise ValueError('nominal camera choice not confirmed')
    return transform


def build(root):
    root = Path(root)
    files = {}
    def read(name):
        raw = (root/name).read_bytes()
        files[name] = hashlib.sha256(raw).hexdigest()
        return json.loads(raw)
    g = s2.load_geometry(root/'config/home_assistant/official_geometry.json')
    installation = read('config/home_assistant/installation.development.json')
    alignment = read('config/home_assistant/arm_model_alignment.development.json')
    mounts = read('config/home_assistant/arm_mounts.nominal.json')
    contacts = read('tools/diagnostics/s2_viewer/lift-contacts.json')
    floor = read('tools/diagnostics/s2_viewer/lift-floor-reference.json')
    device = read('config/home_assistant/camera_device.observed.json')
    ranges = read(alignment['direction_source'])
    from so101_arm_bridge.development_ranges import validate_development_ranges
    validate_development_ranges(ranges)
    control_xml = (root/arm_coordinates.CONTROL_MODEL).read_bytes()
    files[arm_coordinates.CONTROL_MODEL] = hashlib.sha256(control_xml).hexdigest()
    axis_signs = arm_coordinates.arm_axis_signs(control_xml, g)
    if files[alignment['direction_source']] != alignment['direction_source_sha256']:
        raise ValueError('historical direction source changed')
    for value in (alignment, mounts, contacts):
        if value['geometry_sha256'] != s2.digest(g):
            raise ValueError('mixed geometry revisions')
    if floor['source_contact_report_sha256'] != s2.digest(contacts):
        raise ValueError('floor/contact report mismatch')
    if not floor['measurement_evidence']['operator_confirmed'] or installation['d415']['moves_with_lift']:
        raise ValueError('confirmed floor and fixed camera required')
    camera = installation['d415']['development_transform_choice']
    camera_pose=camera_transform(camera)
    base = s2.nominal_base_from_cad(g)
    js = {j['name']:j for j in g['mobile_reference_joints']}
    axis = np.array([float(v) for v in js['vertical_move']['axis']['xyz'].split()])
    axis /= np.linalg.norm(axis)
    shift = np.eye(4)
    shift[:3, 3] = axis * contacts['contacts']['low']['q_m']
    platform = base @ s2.origin(js['vertical_move']) @ shift
    axis_base = base[:3,:3] @ s2.origin(js['vertical_move'])[:3,:3] @ axis
    arms = {}
    for side in s2.ARMS:
        a = alignment['arms'][side]
        calibrated = alignment.get('home_recalibrated') is True
        evidence_key = 'eeprom_calibration_persistence_verified' if calibrated else 'eeprom_matches_pre_reference_backup'
        if not a['torque_off_verified'] or a.get(evidence_key) is not True:
            raise ValueError('arm reference capture not verified')
        if calibrated:
            if (a['reference_report_sha256'] != ranges['calibration'][side]['centered_sha256']
                    or a['persistence_report_sha256'] != ranges['calibration'][side]['persistence_sha256']):
                raise ValueError('Home reference and range EEPROM evidence differ')
            persistence = root/a['persistence_report']
            if persistence.exists() and hashlib.sha256(persistence.read_bytes()).hexdigest() != a['persistence_report_sha256']:
                raise ValueError('Home persistence record changed')
        # Archived capture bytes remain auditable; no local capture required on deployment.
        report = root/a['reference_report']
        if report.exists() and hashlib.sha256(report.read_bytes()).hexdigest() != a['reference_report_sha256']:
            raise ValueError('arm capture changed')
        mapping = arm_coordinates.model_alignment(a, ranges['arms'][side], axis_signs)
        q = s2.model_angles(a['reference_raw'], mapping)
        if any(abs(q[j]-a['reference_model_rad'][j]) > 1e-10 for j in s2.JOINTS):
            raise ValueError('reference offsets disagree')
        mount = platform @ s2.origin(js[side+'_base']) @ s2.rigid(mounts['arms'][side]['cad_arm_from_model_root'])
        arms[side] = {'base_from_arm_at_lift_zero':mount.tolist(),
                      'alignment':mapping, 'reference_raw':a['reference_raw'],
                      'reference_model_rad':a['reference_model_rad'],
                      'reference_from_tcp':np.eye(4).tolist(),
                      'reference_report_sha256':a['reference_report_sha256']}
    ground = float((base @ [0,0,floor['referenced_ground_cad_z_m'],1])[2])
    p = {'schema_version':2, 'arm_coordinate_convention':arm_coordinates.CONVENTION, 'record_kind':KIND, 'provenance':'nominal_with_operator_reference',
         'geometry_sha256':s2.digest(g), 'source_files_sha256':files,
         'assembly_revision':alignment.get('assembly_revision', 'mini1-fixed-d415-selected-home-20261105'),
         'source_frame':'color_camera_optical_frame', 'target_frame':'base_link',
         'base_definition':'official wheel axle centroid; +X forward, +Y left, +Z up',
         'base_from_cad':base.tolist(), 'ground_z_m':ground,
         'base_from_optical':camera_pose.tolist(),
         'camera_device':device['camera_device'], 'arms':arms,
         'lift':{'axis_base':axis_base.tolist(), 'platform_at_lift_zero':platform.tolist(),
                 'travel_m':contacts['travel_m'], 'meters_per_count':-g['lift_nominal_m_per_turn']/g['encoder_counts_per_turn'],
                 'floor_height_low_m':{side:floor['endpoints']['low'][side]['floor_height_m'] for side in s2.ARMS},
                 'plate_local_point_m':contacts['height_datum']['local_point_m'],
                 'live_origin':None, 'requires_new_origin_after_restart':True},
         'policy':{'maximum_skew_s':.050, 'maximum_age_s':.200, 'report_maximum_age_s':5.,
                   'maximum_sample_gap_s':.200, 'maximum_delta_counts':256},
         'tcp_definition':'official fixed gripper_frame_link; not object center or variable jaw contact center',
         'development_coordinates_available':True, 'coordinate_use_allowed':False,
         'validation_passed':False, 'strict_commissioning_passed':False, 'motion_authorized':False,
         'pending_physical_validation':['camera extrinsics during S3','multi-pose TCP and historical direction verification during S3'],
         'live_integration_verified':False}
    if camera.get('operator_height_adjustment') is not None:
        p.update(provenance='nominal_with_operator_height_adjustment',
            camera_nominal_base_from_optical=s2.rigid(camera['base_from_optical']).tolist(),
            camera_height_adjustment=deepcopy(camera['operator_height_adjustment']))
    p['calibration_id'] = 's2-dev-'+s2.digest(p)[:24]
    validate(p,g)
    return p


def released_profile(root, geometry):
    """Load the published mapping byte-for-byte; rebuilding on another BLAS can change its ID."""
    root = Path(root)
    profile = json.loads((root/'config/home_assistant/s2_coordinates.development.json').read_text())
    validate(profile, geometry)
    if profile.get('arm_coordinate_convention') != arm_coordinates.CONVENTION:
        raise ValueError('current model-coordinate profile required')
    for name, expected in profile['source_files_sha256'].items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != expected:
            raise ValueError(f'profile source changed: {name}')
    return profile


def validate(profile, geometry):
    unsigned = {k:v for k,v in profile.items() if k!='calibration_id'}
    if (profile.get('record_kind') != KIND or profile.get('calibration_id') != 's2-dev-'+s2.digest(unsigned)[:24]
            or profile['geometry_sha256'] != s2.digest(geometry)):
        raise ValueError('development profile/geometry mismatch')
    if any(profile[k] is not False for k in ('coordinate_use_allowed','validation_passed','strict_commissioning_passed','motion_authorized')):
        raise ValueError('nominal coordinates cannot assert measured acceptance or motion authorization')
    version = profile.get('schema_version')
    if version not in (1, 2) or (version == 2 and profile.get('arm_coordinate_convention') != arm_coordinates.CONVENTION):
        raise ValueError('unknown arm coordinate convention')
    s2.rigid(profile['base_from_optical'])
    if 'camera_height_adjustment' in profile:
        expected=camera_transform(dict(status='nominal_with_operator_height_adjustment',
            base_from_optical=profile['camera_nominal_base_from_optical'],
            operator_height_adjustment=profile['camera_height_adjustment']))
        if not np.allclose(expected,profile['base_from_optical'],atol=1e-12,rtol=0):
            raise ValueError('camera height adjustment differs from declared nominal pose')
    axis = s2.vector(profile['lift']['axis_base'],3,'lift axis')
    if not math.isclose(float(np.linalg.norm(axis)),1,abs_tol=1e-9):
        raise ValueError('unit lift axis required')
    for side in s2.ARMS:
        arm = profile['arms'][side]
        s2.rigid(arm['base_from_arm_at_lift_zero'])
        s2.rigid(arm['reference_from_tcp'])
        if any(arm['alignment'][j]['sign'] not in (-1,1) for j in s2.JOINTS):
            raise ValueError('joint directions must be signed')
        if version == 2:
            mapping = arm['alignment']
            if any(m.get('control_to_model_sign') not in (-1, 1) or m.get('raw_to_control_sign') not in (-1, 1)
                   or m['control_to_model_sign'] * m['raw_to_control_sign'] != m['sign'] for m in mapping.values()):
                raise ValueError('signed control/model conversion required')
            at_reference = s2.model_angles(arm['reference_raw'], mapping)
            if any(abs(at_reference[j]-arm['reference_model_rad'][j]) > 1e-10 for j in s2.JOINTS):
                raise ValueError('model conversion moved the reference pose')



def evaluate(profile, geometry, raw, lift_height_m, *, camera_xyz_m=None):
    """Offline calculation. Inputs are explicit continuous counts, never live by default."""
    validate(profile,geometry)
    h = s2.number(lift_height_m,'lift height')
    if not 0 <= h <= profile['lift']['travel_m']:
        raise ValueError('lift height outside CAD contact interval')
    if set(raw) != set(s2.ARMS): raise ValueError('both named arms required')
    frames, positions = {}, {'s2_lift_joint':h}
    for side in s2.ARMS:
        arm = profile['arms'][side]
        mount = s2.rigid(arm['base_from_arm_at_lift_zero']).copy()
        mount[:3,3] += np.array(profile['lift']['axis_base'])*h
        frames[side] = (mount @ s2.tool_pose(geometry,arm,raw[side])).tolist()
        positions.update({f'{side}_model_{j}':q for j,q in s2.model_angles(raw[side],arm['alignment']).items()})
    point = None
    if camera_xyz_m is not None:
        point = s2.optical_to_base(profile['base_from_optical'], camera_xyz_m)
    return {'calibration_id':profile['calibration_id'], 'coordinate_mode':'offline_development',
            'provenance':profile['provenance'], 'frame':'base_link',
            'object_surface_xyz_m':point, 'base_from_tcp':frames,
            'model_joint_positions_si':positions, 'grasp_pose':None,
            'motion_authorized':False, 'strict_commissioning_passed':False}


def export_urdf(profile,geometry):
    validate(profile,geometry)
    return render_observation_urdf(profile,geometry,
        lift_axis=' '.join(map(str,profile['lift']['axis_base'])),
        lift_travel=profile['lift']['travel_m'],ground_z=profile['ground_z_m'])


class ObservationSession:
    """Ephemeral read-only synchronized input boundary, NOT a hardware producer.

    Producer must unwrap each encoder with an established turn reference, convert
    all acquisition stamps into one host clock, and propagate reset/disconnect IDs.
    Opening a new session requires fresh producer evidence. Never reload an old
    session after a process/device restart. Any ingest failure invalidates this one.
    """
    KEYS = ('session_id','controller_boot_id','origin_id','arm_anchor_id','clock_id')

    def __init__(self,profile,geometry,evidence):
        validate(profile,geometry)
        if evidence.get('calibration_id') != profile['calibration_id']:
            raise ValueError('session profile mismatch')
        self.identity = {k:s2.text(evidence[k],k) for k in self.KEYS}
        if evidence.get('lift_bottom_confirmed') is not True or evidence.get('arm_turns_anchored') is not True:
            raise ValueError('fresh lift origin and continuous arm reference required')
        self.origin = s2.number(evidence['lift_origin_raw_unwrapped'],'lift origin',1e12)
        self.established_s = s2.number(evidence['established_s'],'reference time',1e12)
        self.anchor_counts = [self.origin]
        if set(evidence['arm_raw_unwrapped']) != set(s2.ARMS):
            raise ValueError('both arm turn anchors required')
        for side in s2.ARMS:
            s2.model_angles(evidence['arm_raw_unwrapped'][side],profile['arms'][side]['alignment'])
            self.anchor_counts.extend(evidence['arm_raw_unwrapped'][side][j] for j in s2.JOINTS)
        self.profile, self.geometry = deepcopy(profile),deepcopy(geometry)
        self.last = None
        self.valid = True

    def invalidate(self):
        self.valid = False

    def ingest(self,snapshot,*,now_s,clock_id):
        try:
            return self._ingest(snapshot,now_s=now_s,clock_id=clock_id)
        except (ValueError, KeyError, TypeError, OverflowError):
            self.invalidate()
            raise

    def _ingest(self,snapshot,*,now_s,clock_id):
        if not self.valid: raise ValueError('session invalid; fresh reference evidence required')
        p = self.profile
        if clock_id != self.identity['clock_id'] or any(snapshot[k]!=v for k,v in self.identity.items()):
            raise ValueError('clock/reset/origin/anchor changed')
        for key in ('calibration_id','assembly_revision','camera_device','source_frame'):
            if snapshot[key]!=p[key]: raise ValueError(key+' mismatch')
        if snapshot.get('raw_encoding') != 'continuous_unwrapped_counts':
            raise ValueError('explicit continuous encoder readings required')
        if type(snapshot['sequence']) is not int or snapshot['sequence']<0:
            raise ValueError('integer sequence required')
        s2.text(snapshot['capture_id'],'capture ID')
        now = s2.number(now_s,'now',1e12)
        receipt = s2.number(snapshot['received_s'],'receipt',1e12)
        if not 0<=now-receipt<=p['policy']['maximum_age_s']:
            raise ValueError('stale/future receipt')
        if set(snapshot['joint_times_s'])!=set(s2.ARMS): raise ValueError('both arm sample stamps required')
        stamps = [s2.number(snapshot[k],k,1e12) for k in ('camera_time_s','lift_time_s')]
        for side in s2.ARMS:
            times = snapshot['joint_times_s'][side]
            if set(times)!=set(s2.JOINTS): raise ValueError('per-joint acquisition timestamps required')
            stamps.extend(s2.number(times[j],j+' time',1e12) for j in s2.JOINTS)
        if any(t<self.established_s or not 0<=receipt-t<=p['policy']['maximum_age_s'] for t in stamps):
            raise ValueError('stale/future/acquired before reference')
        if max(stamps)-min(stamps)>p['policy']['maximum_skew_s']:
            raise ValueError('camera/arm/lift acquisition skew')
        lift_raw = s2.number(snapshot['lift_raw_unwrapped'],'lift count',1e12)
        # evaluate rejects absent/nonfinite joint counts and heights, without wrapping.
        h = (lift_raw-self.origin)*p['lift']['meters_per_count']
        result = evaluate(p,self.geometry,snapshot['raw'],h,camera_xyz_m=snapshot.get('camera_xyz_m'))
        counts = [lift_raw]+[snapshot['raw'][side][j] for side in s2.ARMS for j in s2.JOINTS]
        motor_stamps = [snapshot['lift_time_s']]+[snapshot['joint_times_s'][side][j] for side in s2.ARMS for j in s2.JOINTS]
        if self.last is None:
            if any(not 0<t-self.established_s<=p['policy']['maximum_sample_gap_s'] for t in motor_stamps):
                raise ValueError('first sample too far from turn anchor')
            if any(abs(v-u)>p['policy']['maximum_delta_counts'] for v,u in zip(counts,self.anchor_counts)):
                raise ValueError('first sample inconsistent with turn anchor')
        if self.last is not None:
            old = self.last
            if snapshot['sequence']<=old['sequence'] or snapshot['capture_id']==old['capture_id'] or snapshot['camera_time_s']<=old['camera_time_s']:
                raise ValueError('out-of-order/replayed capture')
            if any(not 0<t-u<=p['policy']['maximum_sample_gap_s'] for t,u in zip(motor_stamps,old['motor_stamps'])):
                raise ValueError('motor sample repeated, reset or gap')
            if any(abs(v-u)>p['policy']['maximum_delta_counts'] for v,u in zip(counts,old['counts'])):
                raise ValueError('encoder discontinuity; turn reference lost')
        self.last = {'sequence':snapshot['sequence'],'capture_id':snapshot['capture_id'],
                     'camera_time_s':snapshot['camera_time_s'],'motor_stamps':motor_stamps,'counts':counts}
        result.update(coordinate_mode='development_capture_time', capture_id=snapshot['capture_id'],
                      observed_s=snapshot['camera_time_s'], received_s=receipt,
                      clock_id=clock_id, controller_boot_id=self.identity['controller_boot_id'],
                      session_id=self.identity['session_id'], origin_id=self.identity['origin_id'])
        return result


def transform_report(profile,geometry,report,*,capture_snapshot,now_s,clock_id):
    """DINO/SAM diagnostic adapter. Output is NOT an executable semantic entity.

    capture_snapshot is a retained ObservationSession.ingest result. Delayed
    inference uses its acquisition identity, not the latest robot pose.
    """
    validate(profile,geometry)
    if (capture_snapshot.get('coordinate_mode')!='development_capture_time' or
        capture_snapshot.get('calibration_id')!=profile['calibration_id'] or
        capture_snapshot.get('clock_id')!=clock_id or report.get('capture_id')!=capture_snapshot['capture_id'] or
        report.get('camera_device')!=profile['camera_device'] or report.get('coordinate_frame')!=profile['source_frame']):
        raise ValueError('report/capture identity mismatch')
    now = s2.number(now_s,'now',1e12)
    age = now-capture_snapshot['observed_s']
    if now<capture_snapshot['received_s'] or not 0<=age<=profile['policy']['report_maximum_age_s']:
        raise ValueError('stale/future inference report')
    if type(report.get('schema_version')) is not int or report['schema_version'] not in (1,2):
        raise ValueError('unsupported report schema')
    detections = report.get('detections')
    if not isinstance(detections,list) or len(detections)>64: raise ValueError('bounded detections required')
    results, seen = [],set()
    for item in detections:
        identifier = item['id']
        if type(identifier) is not int or identifier<0 or identifier in seen: raise ValueError('unique detection IDs required')
        seen.add(identifier)
        label = s2.text(item['label'],'label')
        score = s2.number(item['score'],'score',1)
        if score<0: raise ValueError('negative score')
        surface = item['surface_point']
        if type(surface['valid']) is not bool: raise ValueError('typed surface validity required')
        xyz = None
        if surface['valid']:
            xyz = s2.optical_to_base(profile['base_from_optical'], surface['xyz_m'])
        results.append({'id':identifier,'label':label,'score':score,'base_surface_xyz_m':xyz,
                        'reason':surface.get('reason') if xyz is None else None,'grasp_pose':None})
    return {**deepcopy(capture_snapshot),'detections':results,'inference_age_s':age,
            'motion_authorized':False,'executable_entities':False}
