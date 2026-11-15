import json
from types import SimpleNamespace

import numpy as np
import pytest

from home_robot_tasks.stationary_rgbd import height_frame_issue
from tools.diagnostics.capture_height_window import capture_window


@pytest.mark.parametrize('settings,accepted',[
    ({'auto_exposure':0,'actual_exposure':16000},True),
    ({'auto_exposure':1,'actual_exposure':16000},False),
    ({'auto_exposure':0,'actual_exposure':33000},False),
    ({'auto_exposure':0,'exposure':16000},False),
    ({},False),
])
def test_height_uses_frame_exposure_not_readback_or_assumed_default(settings,accepted):
    assert (height_frame_issue({'depth_frame':{'settings':settings}}) is None)==accepted


def fake_camera(state,*,auto=False,fail=False):
    class Camera:
        def __init__(self,profile,reference,**kwargs):
            state.kwargs=kwargs;self.native_depth_images={};self.n=0
        def __enter__(self):return self
        def __exit__(self,*args):state.closed=True
        def next_frame(self):
            self.n+=1;state.now+=.5
            if fail and self.n==10:raise RuntimeError('camera disconnected')
            capture=f'frame-{self.n}';image=np.ones((2,2),dtype=np.uint16)
            self.native_depth_images[capture]=image
            row=dict(capture_id=capture,color_time_s=state.now,depth_frame={
                'settings':{'auto_exposure':int(auto),'actual_exposure':16000}})
            return row,(image,image,{'depth_scale_m':.001})
    return Camera


def test_height_window_archives_native_depth_and_closes_camera(tmp_path):
    state=SimpleNamespace(now=0.,closed=False)
    r=capture_window(tmp_path/'frames',5,capture_factory=fake_camera(state),clock=lambda:state.now)
    assert r['status']=='HEIGHT_WINDOW_RECORDED' and state.closed and r['settings_restored']
    assert state.kwargs['depth_exposure_us']==16000 and state.kwargs['retain_native_depth']
    assert not r['controller_access'] and not r['motion_authorized']
    assert np.load(tmp_path/'frames'/r['records'][0]['file'])['native_depth'].shape==(2,2)


def test_exposure_mismatch_requests_recapture_without_fault_or_usable_window(tmp_path):
    state=SimpleNamespace(now=0.,closed=False)
    r=capture_window(tmp_path/'frames',5,capture_factory=fake_camera(state,auto=True),clock=lambda:state.now)
    assert r['status']=='RETRY_CAPTURE' and not r['records'] and state.closed
    assert r['rejected_frames']>0 and not r['controller_access']


def test_interrupted_capture_preserves_previous_frames_and_closes_camera(tmp_path):
    state=SimpleNamespace(now=0.,closed=False)
    with pytest.raises(RuntimeError,match='disconnected'):
        capture_window(tmp_path/'frames',5,capture_factory=fake_camera(state,fail=True),clock=lambda:state.now)
    r=json.loads((tmp_path/'frames/report.json').read_text())
    assert r['status']=='CAPTURE_FAILED' and r['records'] and state.closed
    assert (tmp_path/'frames'/r['records'][0]['file']).exists()
    with pytest.raises(FileExistsError):
        capture_window(tmp_path/'frames',5,capture_factory=fake_camera(state),clock=lambda:state.now)


def test_floor_archive_review_preserves_exposure_binding_and_detects_corruption(tmp_path):
    from tools.diagnostics.capture_height_window import review_floor_window
    folder=tmp_path/'frames';state=SimpleNamespace(now=0.,closed=False)
    original=capture_window(folder,5,capture_factory=fake_camera(state),clock=lambda:state.now)
    metadata=dict(depth_scale_m=.001,native_depth_intrinsics=dict(fx=100.,fy=100.,ppx=0.,ppy=0.),
        depth_to_color=dict(rotation_column_major=np.eye(3).ravel().tolist(),translation_m=[0.,0.,0.]))
    (folder/'metadata.json').write_text(json.dumps(metadata))
    reviewed=review_floor_window(folder)
    assert reviewed['observed_floor_frames']==0 and not reviewed['controller_access']
    assert [r['sha256'] for r in reviewed['records']]==[r['sha256'] for r in original['records']]
    assert all(r['evidence']['source']=='calibrated_height_fallback' for r in reviewed['records'])
    (folder/original['records'][0]['file']).write_bytes(b'changed')
    with pytest.raises(ValueError,match='hash mismatch'):review_floor_window(folder)
