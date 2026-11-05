"""Validate SDK-reported D415 capture metadata without connecting to a device.

Device identity and calibration fingerprints are evidence for S2, not an
extrinsic calibration, synchronized arm snapshot, or motion authorization.
"""
from copy import deepcopy
from . import s2_commissioning as s2
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
