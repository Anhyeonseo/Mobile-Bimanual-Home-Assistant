"""Validate SDK-reported D415 capture metadata without connecting to a device.

Device identity and calibration fingerprints are evidence for S2, not an
extrinsic calibration, synchronized arm snapshot, or motion authorization.
"""
from copy import deepcopy
from . import s2_geometry as s2
import numpy as np


def match_observed_identity(observed, reference):
    """Bind a verified SDK observation to a reference across float32 roundoff.

    Only depth/color translation roundoff up to 2 nanometres is accepted.
    Intrinsics, rotation, scale, device and streams must still match exactly.
    Raw observed identity remains available in the acquisition evidence.
    """
    actual=observed['camera_device'];expected=reference['camera_device']
    for record in (observed,reference):
        if s2.digest(record['sdk_calibration'])!=record['camera_device']['factory_calibration_sha256']:
            raise ValueError('calibration evidence fingerprint mismatch')
    for key in ('device_id_hash','stream_profile_id'):
        if actual[key]!=expected[key]:raise ValueError('D415 device/stream changed')
    a=deepcopy(observed['sdk_calibration']);b=reference['sdk_calibration']
    maximum=0.
    for axis in range(3):
        av=a['color_optical_from_depth_optical'][axis][3]
        bv=b['color_optical_from_depth_optical'][axis][3]
        difference=abs(s2.number(av,'SDK translation',1.)-s2.number(bv,'reference translation',1.))
        maximum=max(maximum,difference)
        a['color_optical_from_depth_optical'][axis][3]=bv
    if maximum>2e-9 or a!=b:raise ValueError('D415 SDK calibration changed beyond translation roundoff')
    return {'method':'exact identity/intrinsics/rotation/scale; translation roundoff <= 2e-9 m',
            'maximum_translation_difference_m':maximum,'observed_camera_device':deepcopy(actual),
            'reference_camera_device':deepcopy(expected)}


def near_depth_profile(profile, baseline, measured):
    """Bind the *measured* 640x360 D415 mode to the same fixed color optical frame."""
    if profile['camera_device'] != baseline['camera_device'] or profile.get('motion_authorized') is not False:
        raise ValueError('read-only profile and its original D415 reference required')
    identity = profile['camera_device']
    if (identity['factory_calibration_sha256'] != s2.digest(baseline['sdk_calibration'])
            or identity['stream_profile_id'] != 'rgbd-v1-' + s2.digest(baseline['profiles'])):
        raise ValueError('original D415 reference fingerprint mismatch')
    expected = deepcopy(baseline)
    native = expected['sdk_calibration']['native_depth_intrinsics']
    stream = expected['profiles']['depth']
    if stream != dict(width=1280, height=720, fps=30, format='z16') or (native['width'], native['height']) != (1280,720):
        raise ValueError('expected original D415 depth mode')
    # On this device the SDK reports the exact half-scale native intrinsics in this mode.
    # Keep the color/registered intrinsics, scale and extrinsics; verify against a separate SDK observation.
    stream.update(width=640, height=360)
    native.update(width=640, height=360)
    for k in ('fx','fy','ppx','ppy'):
        native[k] /= 2
    expected['camera_device'].update(stream_profile_id='rgbd-v1-'+s2.digest(expected['profiles']),
                                     factory_calibration_sha256=s2.digest(expected['sdk_calibration']))
    if measured['profiles'] != expected['profiles']:
        raise ValueError('different RGB or native depth stream')
    match_observed_identity(measured, expected)
    near = deepcopy(profile)
    near['base_profile_id'] = near.pop('calibration_id')
    near['near_depth_reference_sha256'] = s2.digest(measured)
    near['camera_device'] = deepcopy(measured['camera_device'])
    near['calibration_id'] = 's2-dev-'+s2.digest(near)[:24]
    return near


def wide_color_profile(profile, baseline, measured):
    """Bind independently measured 1280x720 RGB without changing arm/extrinsics.

    Never infer intrinsics by stretching the old 4:3 image. The SDK evidence
    must come from an actual wide-mode exposure on the same physical camera.
    """
    if profile['camera_device']!=baseline['camera_device'] or profile.get('motion_authorized') is not False:
        raise ValueError('read-only baseline camera profile required')
    for evidence in (baseline,measured):
        if (evidence['camera_device']['factory_calibration_sha256']!=s2.digest(evidence['sdk_calibration'])
            or evidence['camera_device']['stream_profile_id']!='rgbd-v1-'+s2.digest(evidence['profiles'])):
            raise ValueError('camera evidence fingerprint mismatch')
    if baseline['camera_device']['device_id_hash']!=measured['camera_device']['device_id_hash']:
        raise ValueError('wide profile must use the same camera')
    if measured['profiles']['color']!=dict(width=1280,height=720,fps=30,format='bgr8'):
        raise ValueError('measured 1280x720 RGB at 30fps required')
    a,b=measured['sdk_calibration'],baseline['sdk_calibration']
    if (measured['profiles']['depth']!=baseline['profiles']['depth']
        or a['native_depth_intrinsics']!=b['native_depth_intrinsics']
        or a['depth_scale_m']!=b['depth_scale_m']
        or not np.allclose(a['color_optical_from_depth_optical'],b['color_optical_from_depth_optical'],rtol=0,atol=2e-9)):
        raise ValueError('wide color must preserve native depth and sensor extrinsics')
    if a['aligned_depth_intrinsics']!=a['color_intrinsics'] or (a['color_intrinsics']['width'],a['color_intrinsics']['height'])!=(1280,720):
        raise ValueError('measured wide aligned intrinsics required')
    wide=deepcopy(profile)
    wide['base_profile_id']=wide.pop('calibration_id')
    wide['wide_color_reference_sha256']=s2.digest(measured)
    wide['camera_device']=deepcopy(measured['camera_device'])
    wide['calibration_id']='s2-dev-'+s2.digest(wide)[:24]
    return wide


def intrinsics(value):
    result = {}
    for key in ('width', 'height'):
        n = value[key]
        if type(n) is not int or n <= 0:
            raise ValueError('intrinsics dimensions must be positive integers')
        result[key] = n
    for key in ('fx', 'fy', 'ppx', 'ppy'):
        result[key] = s2.number(value[key], 'intrinsics '+key, 1e6)
    if min(result['fx'], result['fy']) <= 0:
        raise ValueError('intrinsics focal lengths must be positive')
    result['coeffs'] = s2.vector(value['coeffs'], 5, 'distortion coefficients').tolist()
    result['model'] = s2.text(value['model'], 'distortion model')
    return result


def stream(value, intrinsic, required_format):
    if value['width'] != intrinsic['width'] or value['height'] != intrinsic['height']:
        raise ValueError('stream dimensions do not match intrinsics')
    if type(value['fps']) is not int or not 1 <= value['fps'] <= 120:
        raise ValueError('stream FPS must be a positive supported integer')
    if value['format'] != required_format:
        raise ValueError('unexpected stream format')
    return {k: value[k] for k in ('width', 'height', 'fps', 'format')}


def from_metadata(metadata):
    if 'color_capture' not in metadata or 'format' not in metadata.get('depth_capture', {}):
        raise ValueError('stream metadata missing; recapture with the current viewer')
    device = metadata['device']
    if 'D415' not in s2.text(device['name'], 'device name'):
        raise ValueError('D415 capture required')
    serial = s2.text(device['serial'], 'device serial')
    if metadata['coordinate_frame'] != 'color_camera_optical_frame':
        raise ValueError('color optical frame required')
    if metadata['measurement_depth_file'] != 'depth_aligned.png':
        raise ValueError('unfiltered aligned measurement depth required')
    if not s2.text(metadata['alignment'], 'alignment').startswith('depth to color via rs.align'):
        raise ValueError('depth-to-color SDK alignment required')
    color = intrinsics(metadata['color_intrinsics'])
    native = intrinsics(metadata['native_depth_intrinsics'])
    aligned = intrinsics(metadata['aligned_depth_intrinsics'])
    if aligned != color:
        raise ValueError('aligned depth and color intrinsics differ; frame mapping needs review')
    profiles = {'color': stream(metadata['color_capture'], color, 'bgr8'),
                'depth': stream(metadata['depth_capture'], native, 'z16'),
                'alignment': 'depth_to_color', 'source_frame': 'color_camera_optical_frame'}
    scale = s2.number(metadata['depth_scale_m'], 'depth scale', .1)
    if scale <= 0:
        raise ValueError('positive depth scale required')
    extr = metadata['depth_to_color']
    rotation = s2.vector(extr['rotation_column_major'], 9, 'SDK rotation').reshape(3, 3, order='F')
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = s2.vector(extr['translation_m'], 3, 'SDK translation')
    s2.rigid(transform)
    calibration = {'native_depth_intrinsics': native, 'color_intrinsics': color,
                   'aligned_depth_intrinsics': aligned, 'depth_scale_m': scale,
                   'color_optical_from_depth_optical': transform.tolist()}
    identity = {'device_id_hash': s2.digest({'scheme': 'realsense_serial_v1', 'serial': serial}),
                'stream_profile_id': 'rgbd-v1-'+s2.digest(profiles),
                'factory_calibration_sha256': s2.digest(calibration)}
    frame_evidence = {}
    for name in ('color', 'depth'):
        frame = metadata[name+'_frame']
        if type(frame['number']) is not int or frame['number'] < 0:
            raise ValueError('invalid camera frame number')
        timestamp = s2.number(frame['timestamp_ms'], 'frame timestamp', 1e15)
        if timestamp < 0:
            raise ValueError('negative frame timestamp')
        frame_evidence[name] = {'number': frame['number'], 'timestamp_ms': timestamp,
                               'timestamp_domain': s2.text(frame['timestamp_domain'], 'timestamp domain')}
    return {'schema_version': 1, 'status': 'camera_metadata_validated',
            'source_metadata_sha256': s2.digest(metadata), 'camera_device': identity,
            'device': {k: device[k] for k in ('name', 'firmware', 'usb') if k in device},
            'profiles': profiles, 'sdk_calibration': calibration,
            'calibration_hash_scope': 'SDK-reported intrinsics, depth scale and depth-to-color extrinsics; not full EEPROM',
            'captured_at_utc': s2.text(metadata['captured_at_utc'], 'capture wall time'),
            'frame_evidence': deepcopy(frame_evidence),
            'image_quality_validated': False, 'timestamps_synchronized_to_robot': False, 'camera_mount_calibrated': False,
            'coordinate_use_allowed': False, 's2_complete': False, 'motion_authorized': False}


def load_capture_profile(root, mode='wide'):
    """Select measured live profiles without rewriting historical camera evidence."""
    import json
    from pathlib import Path
    if mode not in ('wide', 'legacy'):
        raise ValueError('unknown camera mode')
    folder=Path(root)/'config/home_assistant'
    suffix='.wide' if mode=='wide' else ''
    profile=json.loads((folder/('s2_coordinates'+suffix+'.development.json')).read_text())
    reference=json.loads((folder/('camera_device'+suffix+'.observed.json')).read_text())
    if profile['camera_device']!=reference['camera_device']:
        raise ValueError('live camera profile and evidence mismatch')
    return profile, reference
