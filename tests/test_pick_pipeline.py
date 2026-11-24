"""Resident integration contracts, with model/device boundaries replaced only.

The real station and PipelineBackend run two requests, failure and cancellation.
No hardware authority is granted by these fixtures.
"""
from concurrent.futures import Future
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import asdict, fields
import json
from pathlib import Path
from types import SimpleNamespace as NS
import time

import numpy as np
import pytest

from home_robot_tasks.pick_backend import PipelineBackend
from home_robot_tasks.pick_capture import make_evidence, save_capture, review_capture
from home_robot_tasks.pick_rgbd_verification import compare_clouds, StableVerification
from home_robot_tasks.pick_service import PickStation
from home_robot_tasks.carry_live_inputs import JointExposure
from tools.diagnostics.carry_moveit_session import PersistentSession
from test_pick_service import wait_for

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('source', ['so101', 'graspnet'])
def test_camera_preview_honors_selected_candidate_model_without_fallback(tmp_path, monkeypatch, source):
    from tools.lib.pick_compute import PickCompute
    from tools.diagnostics import replay_s3_grasp

    runtime = PickCompute(dict(candidate_source=source))
    runtime.grasp = NS(checkpoint=tmp_path/'checkpoint-rs.tar') if source == 'graspnet' else None
    calls = []

    def no_candidates(*args, **kwargs):
        calls.append(kwargs)
        return dict(status='REOBSERVE', reason='no_observed_robot_width_proposal', observations=[])

    monkeypatch.setattr(replay_s3_grasp, 'replay', no_candidates)
    result = runtime.camera_candidates(str(tmp_path), str(tmp_path/'detections.json'),
                                       'remote control', 'wide', str(tmp_path/'candidates.json'))
    assert result['count'] == 0 and result['status'] == 'REOBSERVE'
    assert len(calls) == 1
    assert calls[0]['candidate_source'] == source
    assert calls[0]['grasp_inference'] is runtime.grasp
    assert calls[0]['grasp_checkpoint'] == (runtime.grasp.checkpoint if runtime.grasp else None)


def test_resident_planner_resets_each_request_reuses_process_and_retires_failed_session(tmp_path):
    starts=[];resets=[];closed=[]
    @contextmanager
    def factory(parameters,state,output):
        starts.append(parameters)
        try:yield NS(reset_scene=lambda state:resets.append(state))
        finally:closed.append(True)
    resident=PersistentSession(factory)
    for state in ('first','second'):
        with resident({'model':'same'},state,tmp_path):pass
    assert len(starts)==1 and resets==['first','second'] and not closed
    with pytest.raises(RuntimeError):
        with resident({'model':'same'},'third',tmp_path):raise RuntimeError('lost service')
    assert len(closed)==1
    with resident({'model':'same'},'fourth',tmp_path):pass
    with resident({'model':'changed'},'fifth',tmp_path):pass
    assert len(starts)==3 and resident.starts==3
    resident.close();assert len(closed)==3


def capture_fixture():
    from so101_arm_bridge.mobile_wire import MobileStatus
    from so101_arm_bridge.lift_wire import LiftStatus
    profile=json.loads((ROOT/'config/home_assistant/s2_coordinates.wide.development.json').read_text())
    m={f.name:0 for f in fields(MobileStatus)}
    m.update(boot_id=4,flags=17,mcu_tick_ms=1000,feedback_tick_ms=950)
    m['velocity_raw']=(0,0,0,0)
    l=LiftStatus(4,1,1000,4,1,1,5,0,29,960,8000,8000,0,40,3700,0,0)
    platform=dict(mobile=m,lift=asdict(l),sent_s=20.,received_s=20.01,accepted=True)
    history=[JointExposure(i,t,t+.01,(0.,)*12) for i,t in enumerate((19.67,19.78,19.89))]
    camera=dict(capture_id='rgbd-100',camera_device=profile['camera_device'],source_frame=profile['source_frame'],
                color_time_s=19.98,depth_time_s=19.99,color_frame={'number':100},depth_frame={'number':100})
    meta=dict(color_frame=camera['color_frame'],depth_frame=camera['depth_frame'])
    return profile,platform,history,camera,meta


def test_new_capture_records_real_hold_basis_and_detects_modified_source_or_pose(tmp_path):
    profile,platform,history,camera,meta=capture_fixture()
    evidence=make_evidence(camera,history,platform,profile,now=20.02,requested_s=19.9,session_id='session',anchor=(0.,)*12)
    folder=tmp_path/camera['capture_id']
    saved=save_capture(folder,(np.zeros((8,8,3),np.uint8),np.ones((8,8),np.uint16),meta),evidence)
    answer,_,_=review_capture(folder,saved,profile)
    expected = .008 * (-profile['lift']['meters_per_count'] * 4096 / .084)
    assert answer['lift_height_m']==pytest.approx(expected)
    assert saved['platform']['lift']['height_um']==8000  # Raw controller evidence unchanged.
    assert saved['timing_basis']=='measured_constant_hold_fixed_camera'
    assert 'motor_write_commands' not in saved
    changed=deepcopy(saved);changed['model_joint_positions_si']['s2_lift_joint']=.5
    with pytest.raises(ValueError,match='evidence changed'):review_capture(folder,changed,profile)
    (folder/'color.png').write_bytes(b'changed')
    with pytest.raises(ValueError,match='provenance'):review_capture(folder,saved,profile)


@pytest.mark.parametrize('defect',['old_capture','moving','stale','armed_base','lift_moved'])
def test_capture_requires_fresh_same_request_stationary_robot(defect):
    profile,platform,history,camera,meta=capture_fixture()
    if defect=='old_capture':camera['color_time_s']=19.8
    if defect=='moving':history[-1]=JointExposure(3,19.89,19.9,(.1,)*12)
    if defect=='stale':platform['sent_s']=19.
    if defect=='armed_base':platform['mobile']['state']=1
    if defect=='lift_moved':platform['lift']['flags']=31
    with pytest.raises(ValueError):
        make_evidence(camera,history,platform,profile,now=20.02,requested_s=19.9,session_id='session',anchor=(0.,)*12)


def test_depth_verifier_requires_object_follow_and_measured_contact_not_disappearance():
    rng=np.random.default_rng(2)
    reference=rng.uniform([-.02,-.04,.5],[.02,.04,.51],(600,3))
    before=np.eye(4);after=np.eye(4);after[2,3]=.06
    observed=reference+[0,0,.06]
    assert compare_clouds(reference,observed,before,after,contact=True)['state']=='SUCCESS'
    assert compare_clouds(reference,observed,before,after,contact=False)['state']=='UNKNOWN'
    assert compare_clouds(reference,reference,before,after,contact=False,empty=True)['state']=='FAILURE'
    assert compare_clouds(reference,np.empty((0,3)),before,after,contact=True)['state']=='UNKNOWN'
    assert compare_clouds(reference,observed[:30],before,after,contact=True)['state']=='UNKNOWN'
    assert compare_clouds(reference,reference,before,before,contact=True)['state']=='UNKNOWN'
    stable=StableVerification()
    for i in range(3):
        result=stable.add(dict(capture_id=str(i),observed_s=i*.2,state='SUCCESS'))
    assert result=='SUCCESS'
    with pytest.raises(ValueError):stable.add(dict(capture_id='2',observed_s=.4,state='SUCCESS'))


class FakeCompute:
    def __init__(self,*_):self.calls=[];self.prompts=[];self.closed=False;self.reject=False
    def submit(self,method,**values):
        self.calls.append(method);future=Future()
        if 'prompt' in values:self.prompts.append(values['prompt'])
        if method=='detect':
            if self.reject:
                future.set_exception(RuntimeError('target detection failed: NO_DETECTIONS'));return future
            folder=Path(values['snapshot'])/'detections';folder.mkdir()
            (folder/'detections.jpg').write_bytes(b'fixture')
            result=str(folder/'detections.json');Path(result).write_text('{}')
        elif method=='candidates':
            result=values['output'];Path(result).write_text('{}')
        elif method=='plan':
            output=Path(values['output']);output.mkdir()
            report=output/'report.json';report.write_text('{}')
            result=dict(bundle=str(output/'bundle'),report=str(report),moveit_starts=1)
        else:result={}
        future.set_result(result);return future
    def close(self):self.closed=True


def replay_options(tmp_path):
    source=tmp_path/'saved';source.mkdir()
    for name in ('color.png','depth_aligned.png','metadata.json','robot_capture.json'):
        (source/name).write_bytes(b'{}')
    return dict(mode='replay',snapshot=str(source),robot_report=str(source/'robot_capture.json'))


def test_real_backend_wiring_reuses_preparation_runs_all_stages_and_never_executes_replay(tmp_path):
    worker=FakeCompute()
    backend=PipelineBackend(replay_options(tmp_path),worker_factory=lambda *_:worker,
                            hardware_factory=lambda *_:pytest.fail('replay connected hardware'))
    station=PickStation(backend,tmp_path/'output');station.start()
    try:
        wait_for(lambda:station.snapshot()['accepting'])
        for count, text in enumerate(('노란 테이프 집어줘','pick up the stapler'),1):
            station.submit(text)
            wait_for(lambda:len(station.snapshot()['history'])==count and station.snapshot()['accepting'])
            assert station.snapshot()['last_result']['status']=='PLANNED_ONLY'
        assert worker.prompts==['yellow tape','yellow tape','stapler','stapler']
        assert worker.calls==['prepare']+['detect','candidates','plan']*2
        worker.reject=True;station.submit('리모컨 집기')
        wait_for(lambda:len(station.snapshot()['history'])==3 and station.snapshot()['accepting'])
        assert station.snapshot()['last_result']['status']=='FAILED'
        assert not worker.closed
    finally:station.close()
    assert worker.closed


def test_live_preparation_failure_does_not_open_hardware(tmp_path):
    worker=FakeCompute()
    def fail(*args,**kwargs):
        f=Future();f.set_exception(RuntimeError('model missing'));return f
    worker.submit=fail
    backend=PipelineBackend(dict(mode='live'),worker_factory=lambda *_:worker,
                            hardware_factory=lambda *_:pytest.fail('hardware before models'))
    station=PickStation(backend,tmp_path);station.start()
    try:
        wait_for(lambda:station.snapshot()['phase']=='FAULT')
        assert not station.snapshot()['accepting']
    finally:station.close()
    assert worker.closed


@pytest.mark.parametrize('mismatch',['capture','hashes','session'])
def test_live_execute_rejects_other_request_before_any_dispatch(tmp_path,monkeypatch,mismatch):
    snapshot=tmp_path/'frame';snapshot.mkdir()
    prepared=tmp_path/'prepared.json';prepared.write_text(json.dumps({'source_sha256':{'color.png':'a'}}))
    (snapshot/'robot_capture.json').write_text(json.dumps({'session_id':'session'}))
    program=NS(capture_id='frame',source_sha256={'color.png':'a'})
    if mismatch=='capture':program.capture_id='old'
    if mismatch=='hashes':program.source_sha256={'color.png':'b'}
    backend=PipelineBackend(dict(mode='live'))
    backend.hardware=NS(profile={},session_id='different' if mismatch=='session' else 'session',
                        execute=lambda *_:pytest.fail('dispatched mismatched request'))
    from tools.lib.pick_compute import PickCompute
    monkeypatch.setattr('home_robot_tasks.carry_bundle.load_bundle',lambda *a,**k:(program,))
    runtime=PickCompute({})
    backend._compute=lambda ctx,method,**kwargs:getattr(runtime,method)(**kwargs)
    value=dict(snapshot=str(snapshot),prepared=str(prepared),bundle='bundle')
    with pytest.raises(ValueError,match='binding mismatch'):backend.execute(NS(check=lambda:None),value)
    assert not backend.after_motion


def test_live_plan_only_cannot_dispatch_even_when_program_is_available():
    backend=PipelineBackend(dict(mode='live',plan_only=True))
    backend._compute=lambda *_args,**_kwargs:pytest.fail('decoding for execution in plan-only')
    assert backend.execute(NS(note=lambda *_args,**_kwargs:None),{'bundle':'checked'})=={'bundle':'checked'}
    assert backend.verify(None,{'bundle':'checked'})['physical_success'] is False


def test_integrated_stop_checks_proof_even_if_request_ack_is_lost():
    from home_robot_tasks.pick_hardware import UnifiedPickHardware
    hardware=UnifiedPickHardware({})
    def lost():raise RuntimeError('ACK lost')
    hardware.stop=NS(request=lost,status=lambda:(NS(confirmed=True),None))
    hardware.abort()
    assert hardware.stopped and hardware.stop_confirmed
    hardware.close()


def test_post_motion_state_cannot_accept_another_pick():
    backend=PipelineBackend(dict(mode='live'))
    backend.prepared=True;backend.blocked='';backend.after_motion=True
    assert backend.ready()[0] is False


@pytest.mark.parametrize('wrong', ['producer', 'weights', 'revision', None])
def test_graspnet_dispatch_requires_trained_model_for_this_request(tmp_path, monkeypatch, wrong):
    from home_perception.grasp.graspnet_assets import BASELINE_REVISION, CHECKPOINT_SHA256
    snapshot = tmp_path/'frame'
    snapshot.mkdir()
    (snapshot/'robot_capture.json').write_text(json.dumps({'session_id': 'boot'}))
    program = NS(capture_id='frame', source_sha256={'color.png': 'a'},
                 candidate_source='so101' if wrong == 'producer' else 'graspnet')
    group = {'baseline_revision': 'other' if wrong == 'revision' else BASELINE_REVISION,
             'checkpoint_sha256': 'other' if wrong == 'weights' else CHECKPOINT_SHA256}
    prepared = tmp_path/'prepared.json'
    prepared.write_text(json.dumps(dict(candidate_source='graspnet', source_sha256=program.source_sha256,
                                        group=group, label='eraser')))
    backend = PipelineBackend(dict(mode='station', candidate_source='graspnet'))
    sent = []
    backend.hardware = NS(profile={},session_id='boot', execute=lambda ctx, checked:sent.append(checked[0]))
    from tools.lib.pick_compute import PickCompute
    monkeypatch.setattr('home_robot_tasks.carry_bundle.load_bundle',lambda *a,**k:(program, {'target_label':'eraser'}))
    runtime=PickCompute({})
    backend._compute=lambda ctx,method,**kwargs:getattr(runtime,method)(**kwargs)
    value = dict(snapshot=str(snapshot), prepared=str(prepared), bundle='bundle', prompt='eraser')
    ctx = NS(check=lambda:None)
    if wrong:
        with pytest.raises(ValueError, match='GraspNet'):
            backend.execute(ctx, value)
        assert not sent
    else:
        backend.execute(ctx, value)
        assert sent == [program]
