"""Explicit image-side requests resolve the reported rgbd-693 ambiguity.

Detection fields below are from the supplied report; the exposure harness is
synthetic. These tests do not reconstruct the point cloud or authorize motion.
"""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest

from home_robot_tasks.pick_target import parse_pick_request
from home_robot_tasks.pick_backend import PipelineBackend
from tools.lib.pick_compute import PickCompute
from tools.diagnostics.review_s3_capture import choose_observation


def reported_case():
    source={'color.png':'f59529199208311dd0e3f1ab2d3c1143c463872e5a97aa1478ef37ef0e2c4e7f'}
    common=dict(capture_id='rgbd-693',captured_at_utc='2026-11-23T11:30:00.903881+00:00',
        source_sha256=source,frame_id='color_camera_optical_frame',clock_domain='global_time',
        calibration_id='unmeasured',label='eraser')
    obs=[dict(common,box_xyxy=[186.391845703125,299.99163818359375,255.47433471679688,387.7234802246094],
        detection_score=.4381783902645111,status='accepted',failure=None,reason=None,
        valid_depth_fraction=.9961989230281914,support_height_m=.012087122083837713),
        dict(common,box_xyxy=[58.7929573059082,281.1040954589844,85.09736633300781,294.2044372558594],
        detection_score=.3056856691837311,status='rejected',failure='INVALID_DEPTH',
        reason='insufficient_mask_depth',valid_depth_fraction=1.,support_height_m=None)]
    meta=dict(captured_at_utc=common['captured_at_utc'],color_intrinsics=dict(width=640,height=480))
    profile=dict(source_frame=common['frame_id'],calibration_id='test')
    return obs,meta,profile,source


@pytest.mark.parametrize('text,prompt,side',[
    ('오른쪽 지우개를 집어줘','eraser','right'),
    ('화면 오른쪽 지우개를 집어줘','eraser','right'),
    ('화면의 왼쪽 파란 컵을 집어줘','blue cup','left'),
    ('pick up the right eraser','eraser','right'),
    ('leftmost blue cup','blue cup','left'),
    ('지우개를 집어줘','eraser',None),
])
def test_detector_caption_excludes_image_selector(text,prompt,side):
    assert parse_pick_request(text)[1:]==(prompt,side)


@pytest.mark.parametrize('text',['오른쪽 컵 말고 지우개','왼쪽 오른쪽 지우개',
                               '오른쪽 컵 옆에 있는 지우개','pick the left cup and bottle'])
def test_unsupported_remaining_constraints_are_not_dropped(text):
    with pytest.raises(ValueError):parse_pick_request(text)


def test_reported_case_preserves_bad_depth_identity_and_requires_explicit_choice():
    obs,meta,profile,source=reported_case();before=deepcopy(obs)
    choose=lambda side:choose_observation(obs,'rgbd-693',profile,meta,source,'eraser',side)
    assert choose(None)['status']=='AMBIGUOUS_TARGET'
    right=choose('right')
    assert right['status']=='SELECTED' and right['observation_index']==0
    assert right['target_selection']['method']=='image_side'
    assert choose('left')['status']=='REOBSERVE'
    assert choose('left')['target_selection']['selected_index']==1
    assert obs==before
    obs[1]['box_xyxy']=[180,281,220,320]
    assert choose('right')['status']=='AMBIGUOUS_TARGET'


def test_web_pipeline_forwards_side_to_capture_bound_grasp_runtime(tmp_path,monkeypatch):
    from tools.diagnostics import replay_s3_grasp
    obs,meta,profile,source=reported_case();seen=[]
    snapshot=tmp_path/'rgbd-693';snapshot.mkdir()
    (snapshot/'robot_capture.json').write_text('{}')
    def replay(snapshot,detections,prompt,*,side=None,**kwargs):
        seen.append((prompt,side,kwargs['prepare_carry']))
        choice=choose_observation(obs,'rgbd-693',profile,meta,source,prompt,side)
        return dict(status='CAPTURE_CANDIDATES_READY' if choice['status']=='SELECTED' else choice['status'],
                    review=choice,observations=obs)
    monkeypatch.setattr(replay_s3_grasp,'replay',replay)
    runtime=PickCompute(dict(candidate_source='graspnet'))
    runtime.grasp=NS(checkpoint=tmp_path/'weights')
    backend=PipelineBackend(dict(mode='replay'))
    backend._compute=lambda ctx,method,**kw:getattr(runtime,method)(**kw)
    target,prompt,side=parse_pick_request('오른쪽 지우개를 집어줘')
    request=dict(target=target,prompt=prompt,location=side,snapshot=str(snapshot),detections='detections.json')
    result=backend.candidates(NS(folder=tmp_path,artifact=lambda *a:None),request)
    saved=json.loads(open(result['prepared']).read())
    assert seen==[('eraser','right',True)]
    assert saved['review']['observation_index']==0
    assert saved['observations'][1]['status']=='rejected'


def test_station_keeps_selector_in_request_and_out_of_caption(tmp_path):
    from test_pick_service import RecordingBackend,start,wait_for
    class Record(RecordingBackend):
        def capture(self,ctx,value):
            self.request=deepcopy(value)
            return super().capture(ctx,value)
    backend=Record();station=start(tmp_path,backend)
    try:
        station.submit('오른쪽 지우개를 집어줘')
        wait_for(lambda:getattr(backend,'request',None))
        assert backend.request['location']=='right' and backend.request['prompt']=='eraser'
    finally:
        station.close()
