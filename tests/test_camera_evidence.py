"""SDK identities are stable across frames, sensitive to calibration, never approval."""
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest
from home_robot_tasks.camera_evidence import from_metadata, match_observed_identity

ROOT=Path(__file__).resolve().parents[1]


def test_float32_translation_roundoff_keeps_both_fingerprints():
    m=metadata();ref=from_metadata(m);m['depth_to_color']['translation_m'][0]+=9.313225746e-10
    observed=from_metadata(m);result=match_observed_identity(observed,ref)
    assert result['observed_camera_device']!=result['reference_camera_device']
    assert result['maximum_translation_difference_m']<2e-9


@pytest.mark.parametrize('field',['translation','intrinsics','scale','device','stream','tamper'])
def test_roundoff_equivalence_rejects_actual_changes(field):
    m=metadata();ref=from_metadata(m)
    if field=='translation':m['depth_to_color']['translation_m'][0]+=1e-7
    if field=='intrinsics':
        m['color_intrinsics']['fx']+=.001;m['aligned_depth_intrinsics']['fx']+=.001
    if field=='scale':m['depth_scale_m']+=1e-9
    if field=='device':m['device']['serial']='different'
    if field=='stream':m['color_capture']['fps']=15
    observed=from_metadata(m)
    if field=='tamper':observed['camera_device']['factory_calibration_sha256']='fake'
    with pytest.raises(ValueError):match_observed_identity(observed,ref)


def metadata():
    intr=dict(width=640,height=480,fx=600.,fy=605.,ppx=319.5,ppy=239.5,
              coeffs=[0.,0.,0.,0.,0.],model='distortion.none')
    # Non-symmetric rotation catches accidental row-major interpretation.
    rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    return dict(device=dict(name='Intel RealSense D415',serial='test-serial-only',firmware='fixture',usb='3.2'),
        coordinate_frame='color_camera_optical_frame',measurement_depth_file='depth_aligned.png',
        alignment='depth to color via rs.align; no hole filling or temporal filtering',
        color_intrinsics=deepcopy(intr),native_depth_intrinsics=deepcopy(intr),aligned_depth_intrinsics=deepcopy(intr),
        color_capture=dict(width=640,height=480,fps=30,format='bgr8'),
        depth_capture=dict(width=640,height=480,fps=30,format='z16'),depth_scale_m=.001,
        depth_to_color=dict(rotation_column_major=rotation.flatten(order='F').tolist(),translation_m=[.015,0,0]),
        color_frame=dict(number=1,timestamp_ms=1000.,timestamp_domain='hardware_clock'),
        depth_frame=dict(number=1,timestamp_ms=1000.,timestamp_domain='hardware_clock'),
        captured_at_utc='2026-11-05T00:00:00Z')


def test_identity_stable_across_time_without_claiming_mount_or_clock_calibration():
    m=metadata();a=from_metadata(m);m['color_frame']['number']=2;m['captured_at_utc']='2026-11-05T00:00:01Z'
    b=from_metadata(m)
    assert a['camera_device']==b['camera_device']
    assert a['source_metadata_sha256']!=b['source_metadata_sha256']
    assert 'test-serial-only' not in json.dumps(a)
    for k in ['coordinate_use_allowed','s2_complete','motion_authorized','timestamps_synchronized_to_robot','camera_mount_calibrated']:
        assert a[k] is False
    t=np.array(a['sdk_calibration']['color_optical_from_depth_optical'])
    np.testing.assert_allclose(t@[1,0,0,1],[.015,1,0,1])


def test_calibration_scale_intrinsics_and_profile_changes_invalidate_fingerprints():
    m=metadata();original=from_metadata(m)['camera_device']
    m['depth_scale_m']=.0005
    assert from_metadata(m)['camera_device']['factory_calibration_sha256']!=original['factory_calibration_sha256']
    m=metadata();m['color_capture']['fps']=15
    assert from_metadata(m)['camera_device']['stream_profile_id']!=original['stream_profile_id']
    m=metadata();m['device']['serial']='another-device'
    assert from_metadata(m)['camera_device']['device_id_hash']!=original['device_id_hash']
    m=metadata();m['color_intrinsics']['fx']+=1;m['aligned_depth_intrinsics']['fx']+=1
    assert from_metadata(m)['camera_device']['factory_calibration_sha256']!=original['factory_calibration_sha256']


@pytest.mark.parametrize('mutation',[
    lambda m:m.update(depth_scale_m=0),lambda m:m.update(depth_scale_m=float('nan')),
    lambda m:m['color_capture'].update(width=320),
    lambda m:m['aligned_depth_intrinsics'].update(fx=500),
    lambda m:m['depth_to_color'].update(rotation_column_major=[-1,0,0,0,1,0,0,0,1]),
    lambda m:m.update(measurement_depth_file='depth_preview_filtered.png'),
    lambda m:m.update(coordinate_frame='depth_camera_optical_frame'),
    lambda m:m['device'].update(name='D435'),lambda m:m['depth_frame'].update(timestamp_ms=-1),
])
def test_invalid_or_incompatible_metadata_is_rejected(mutation):
    m=metadata();mutation(m)
    with pytest.raises(ValueError):from_metadata(m)


def test_cli_exports_metadata_evidence_without_loading_camera_sdk(tmp_path):
    m=metadata();(tmp_path/'metadata.json').write_text(json.dumps(m))
    # This tool fingerprints image bytes, it does not claim image quality validation.
    for name in ['color.png','depth_aligned.png','depth_native.png']:(tmp_path/name).write_bytes(b'fixture-bytes')
    output=tmp_path/'evidence.json'
    cmd=[sys.executable,str(ROOT/'tools/diagnostics/s2_coordinates.py'),'camera-evidence','--snapshot',str(tmp_path),'--output',str(output)]
    r=subprocess.run(cmd,text=True,capture_output=True)
    assert r.returncode==0,r.stderr
    result=json.loads(output.read_text());assert len(result['source_files_sha256'])==4
    assert result['s2_complete'] is False
    assert subprocess.run(cmd,capture_output=True).returncode==2
    output.unlink();(tmp_path/'depth_native.png').unlink()
    assert subprocess.run(cmd,capture_output=True).returncode==2
    assert not output.exists()
