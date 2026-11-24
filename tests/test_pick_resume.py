"""Resume uses original pixels and a new measured controller start, never an old anchor."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from home_robot_tasks.pick_calibration import TOP_BASIS
from home_robot_tasks.pick_capture import make_evidence,save_capture
from home_robot_tasks.pick_lift_transition import bind_resume,planning_review
from home_robot_tasks.pick_resume import saved_request,ResumePickBackend
from home_robot_tasks.pick_service import PickStation,Rejected
from home_robot_tasks.carry_live_inputs import JointExposure
from test_pick_lift_transition import arrival_fixture


def resumed_fixture(tmp_path,monkeypatch):
    profile,data=arrival_fixture()
    old=data['start_revalidation']['arrival_evidence'];platform=deepcopy(old['platform'])
    platform.update(sent_s=21.,received_s=21.01,lift_reference_basis=TOP_BASIS)
    platform['mobile']['boot_id']=platform['lift']['boot_id']=5
    platform['top_reference'].update(basis=TOP_BASIS,boot_id=5,operator_confirmed=True)
    camera=deepcopy(old['camera']);camera.update(capture_id='new-frame',color_time_s=20.98,depth_time_s=20.99)
    camera.update(color_frame={'number':200},depth_frame={'number':200})
    history=[replace(JointExposure(**v),lower_s=v['lower_s']+1,upper_s=v['upper_s']+1)
             for v in old['stationary_window']]
    evidence=make_evidence(camera,history,platform,profile,now=21.02,requested_s=20.9,
                           session_id='new-session',anchor=(0.,)*12)
    arrival=tmp_path/'new-frame'
    save_capture(arrival,(np.zeros((8,8,3),np.uint8),np.ones((8,8),np.uint16),
                         dict(color_frame=camera['color_frame'],depth_frame=camera['depth_frame'])),evidence)
    source=tmp_path/'original.json';source.write_text(json.dumps(data))
    monkeypatch.setattr('tools.diagnostics.plan_capture_carry.load_models',lambda *a,**k:(profile,))
    output=tmp_path/'resumed.json'
    bind_resume(source,arrival,output,scene_unchanged=True)
    return profile,data,json.loads(output.read_text()),(source,arrival,output)


def test_resume_preserves_original_pixels_geometry_and_timestamp_with_new_controller_start(tmp_path,monkeypatch):
    profile,old,new,paths=resumed_fixture(tmp_path,monkeypatch)
    assert {k:v for k,v in new.items() if k!='start_revalidation'}=={k:v for k,v in old.items() if k!='start_revalidation'}
    record=new['start_revalidation']
    assert record['source_revalidation']==json.loads(json.dumps(old['start_revalidation']))
    assert record['session_id']=='new-session' and record['controller_boot_id']==5
    assert record['observation_time_s']==old['start_revalidation']['observation_time_s']
    review=planning_review(new,profile)
    assert review['lift_height_m']==pytest.approx(.630) and review['lift_reference_basis']==TOP_BASIS
    with pytest.raises(ValueError,match='confirmation'):bind_resume(*paths)


@pytest.mark.parametrize('fault',['boot','session','unconfirmed','old_source','old_time','anchor'])
def test_resume_does_not_hide_modified_or_unconfirmed_evidence(tmp_path,monkeypatch,fault):
    profile,old,new,_=resumed_fixture(tmp_path,monkeypatch);record=new['start_revalidation']
    if fault=='boot':record['controller_boot_id']=999
    if fault=='session':record['session_id']='old-session'
    if fault=='unconfirmed':record['operator_scene_unchanged']=False
    if fault=='old_source':record['source_revalidation']['source_sha256']={'changed':True}
    if fault=='old_time':record['observation_time_s']+=1
    if fault=='anchor':record['arrival_model_joint_positions_si']['s2_lift_joint']=.5
    with pytest.raises(ValueError):planning_review(new,profile)


def test_source_requires_preexecution_failure_and_unchanged_original_files(tmp_path):
    from home_robot_tasks.snapshot_io import hashes
    profile,_=arrival_fixture();capture=tmp_path/'bottom';capture.mkdir()
    for name in ('color.png','depth_aligned.png','metadata.json'):(capture/name).write_bytes(b'fixture')
    source=hashes(capture);det=capture/'detections';det.mkdir()
    (det/'detections.json').write_text(json.dumps({'source_sha256':source}))
    data=dict(capture_id='bottom',source_sha256=source,label='eraser',review={'calibration_id':profile['calibration_id']})
    (tmp_path/'prepared-at-top.json').write_text(json.dumps(data))
    request=dict(id='prior',text='eraser',prompt='eraser',operation='observe-rise-pick',
        stages=[dict(id=k,status=v) for k,v in [('LIFTING','done'),('PLANNING','failed'),('EXECUTING','skipped')]])
    (tmp_path/'result.json').write_text(json.dumps(request))
    assert saved_request(tmp_path)['camera_mode']=='wide'
    request['stages'][-1]['status']='failed'
    (tmp_path/'result.json').write_text(json.dumps(request))
    with pytest.raises(ValueError,match='팔 실행'):saved_request(tmp_path)
    request['stages'][-1]['status']='skipped';(tmp_path/'result.json').write_text(json.dumps(request))
    (capture/'color.png').write_bytes(b'changed')
    with pytest.raises(ValueError,match='변경'):saved_request(tmp_path)


def test_resume_station_skips_capture_inference_and_rise(tmp_path):
    calls=[]
    class Backend:
        mode='station';hardware=None;kit_id='fixture';options={}
        def ready(self):return True,''
        def check_operation(self,op,bottom,top):assert top and not bottom;return op
        def tick(self):pass
        def prepare_resume(self,ctx,value):calls.append('current-start');return value
        def plan(self,ctx,value):calls.append('plan');return value
        def execute(self,ctx,value):calls.append('execute');return value
        def verify(self,ctx,value):calls.append('verify');return dict(status='PLANNED_ONLY',physical_success=False)
        def on_failure(self,exc):raise AssertionError(str(exc))
    station=PickStation(Backend(),tmp_path);station.data['accepting']=True
    with pytest.raises(Rejected,match='이전 관측'):
        station.submit('eraser',operation='resume-top-pick',supported_empty_top=True)
    station.submit('eraser',operation='resume-top-pick',supported_empty_top=True,scene_unchanged=True)
    request=station.pending;station.pending=None;station._run_request(request)
    assert calls==['current-start','plan','execute','verify']
    assert [s['id'] for s in station.data['current']['stages']]==['PREPARING_PICK','PLANNING','EXECUTING','VERIFYING']


def test_resume_height_keeps_previous_measured_arrival_and_new_encoder_delta():
    from so101_arm_bridge.lift_wire import LiftStatus
    from home_robot_tasks.pick_calibration import make_resumed_height_reference,top_model_height,RISE_BOARD
    profile,data=arrival_fixture()
    source=deepcopy(data['start_revalidation']['arrival_evidence']['platform'])
    source['lift'].update(height_um=652841,position_raw=1907)
    source['top_reference'].update(height_um=652841,position_raw=1907)
    fresh=replace(LiftStatus(**source['lift']),boot_id=10,session=1,height_um=654588,position_raw=1908)
    ref=make_resumed_height_reference(fresh,profile,RISE_BOARD,source,operator_height_unchanged=True)
    expected=.628253-87287/4096e6
    assert top_model_height(fresh,profile,RISE_BOARD,ref)==pytest.approx(expected)
    assert top_model_height(replace(fresh,height_um=654567,position_raw=1909),profile,RISE_BOARD,ref)==pytest.approx(expected-.000021)
    with pytest.raises(ValueError):make_resumed_height_reference(fresh,profile,RISE_BOARD,source)
    with pytest.raises(ValueError):top_model_height(replace(fresh,boot_id=11),profile,RISE_BOARD,ref)
    with pytest.raises(ValueError):make_resumed_height_reference(replace(fresh,position_raw=2500),profile,RISE_BOARD,source,operator_height_unchanged=True)


def test_resume_height_source_must_match_original_arrival(tmp_path,monkeypatch):
    from so101_arm_bridge.lift_wire import LiftStatus
    from home_robot_tasks.pick_calibration import make_resumed_height_reference,RESUME_HEIGHT_BASIS
    profile,old,new,_=resumed_fixture(tmp_path,monkeypatch)
    record=new['start_revalidation'];platform=record['arrival_evidence']['platform']
    source=old['start_revalidation']['arrival_evidence']['platform']
    reference=make_resumed_height_reference(LiftStatus(**platform['lift']),profile,tuple(platform['controller_board']),source,operator_height_unchanged=True)
    platform.update(top_reference=reference,lift_reference_basis=RESUME_HEIGHT_BASIS)
    record['arrival_evidence']['lift_reference_basis']=record['lift_reference_basis']=RESUME_HEIGHT_BASIS
    assert planning_review(new,profile)['lift_height_m']==pytest.approx(.630)
    reference['source_platform']['lift']['position_raw']+=1
    with pytest.raises(ValueError,match='height source'):planning_review(new,profile)
