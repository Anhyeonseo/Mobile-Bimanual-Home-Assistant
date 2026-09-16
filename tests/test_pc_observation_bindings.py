from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest

from home_robot_tasks.arm_routes import JawCalibration, ResidentRouteBuilder, SharedGripperPort
from home_robot_tasks.evidence_producer import EvidenceProducer, SourceSample, FIELDS, measured_gripper
from home_robot_tasks.manipulation_port import PlanningContext, ManipulationPort
from home_robot_tasks.manipulation_bindings import ManipulationBindings, MOTION_STAGES
from home_robot_tasks.skill_ports import PortResult
from home_robot_tasks.execution import Feedback
from home_robot_tasks.offline_stack import OfflineStack
from home_robot_tasks.surface_observer import SurfaceSampler
from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint


def calibration():return JawCalibration([(-.5,0.),(0.,20.),(.5,50.)],open_rad=.4,close_rad=-.3)


def route_builder(validate=lambda p,s:True):
    return ResidentRouteBuilder(minimum_rad=(-1.,)*12,maximum_rad=(1.,)*12,
        max_speed_rad_s=(.5,)*12,max_acceleration_rad_s2=(1.,)*12,max_jerk_rad_s3=(8.,)*12,
        validate_route=validate)


@pytest.mark.parametrize('arm,index',[('left',5),('right',11)])
def test_jaw_route_uses_measured_anchor_preserves_other_eleven_and_same_owner(arm,index):
    calls=[];anchor=[.1]*12;anchor[index]=0.
    snapshot=SimpleNamespace(positions_urad=tuple(round(v*1e6) for v in anchor))
    builder=route_builder(lambda points,s:calls.append((points,s)) or True)
    class Arm:
        owner=None
        def ready(self):return self.owner is None
        def start(self,g,p):
            if not self.ready():raise ValueError('owned')
            self.route=builder(p,snapshot);self.owner=g
        def poll(self,g):return PortResult('RUNNING')
        def cancel(self,g):calls.append(('cancel',g))
    a=Arm();grip=SharedGripperPort(a,dict(left=calibration(),right=calibration()))
    grip.start('close',dict(arm=arm,action='close'))
    assert not grip.ready() and a.owner=='close' and a.route[-1].positions_rad[index]==-.3
    for point in a.route:
        assert all(point.positions_rad[i]==anchor[i] for i in range(12) if i!=index)
    grip.cancel('close');assert calls[-1]==('cancel','close')
    assert calls[0][1] is snapshot


def test_generated_route_requires_current_scene_validation_and_rejects_out_of_range():
    snap=SimpleNamespace(positions_urad=(0,)*12)
    p={'arm':'left','steps':[{'kind':'gripper','target_position_rad':.3}]}
    with pytest.raises(ValueError):route_builder(lambda p,s:False)(p,snap)
    p['steps'][0]['target_position_rad']=2
    with pytest.raises(ValueError):route_builder()(p,snap)


def test_planned_route_rejects_moved_anchor_or_excess_speed():
    c=PlanningContext(1,'c','p','s','cal','base_link',(0.,)*12,True,True)
    points=(TimedJointPoint(50,(0.,)*12),TimedJointPoint(100,(.1,)*12))
    with pytest.raises(ValueError):route_builder()({'trajectory':points,'context':c},SimpleNamespace(positions_urad=(0,)*12))
    with pytest.raises(ValueError):route_builder()({'trajectory':points,'context':c},SimpleNamespace(positions_urad=(30000,)*12))


@pytest.mark.parametrize('samples',[[(0,0),(0,5)],[(0,5),(1,4),(2,6)],[(0,-1),(1,4)]])
def test_nonmonotonic_or_invalid_jaw_calibration_is_rejected(samples):
    with pytest.raises(ValueError):JawCalibration(samples,open_rad=.4,close_rad=0)


def test_gripper_measurement_preserves_time_and_cannot_extrapolate_or_use_stale_input():
    c=calibration();sample=SourceSample(1,.9,42,'a'*64,dict(position_rad=.25,effort_raw=40,healthy=True))
    grip=measured_gripper(sample,c,now_s=1.,maximum_age_s=.2)
    assert grip.gap_mm==35 and grip.observed_s==.9 and grip.sequence==1
    with pytest.raises(ValueError):measured_gripper(sample,c,now_s=2.,maximum_age_s=.2)
    with pytest.raises(ValueError):c.gap(.51)
    reverse=JawCalibration([(0,50),(1,0)],open_rad=.1,close_rad=.9)
    assert reverse.gap(.25)==37.5


def source_values():
    return dict(joints_rad=(0.,)*12,healthy=True,base_velocity=(0.,)*3,xy_yaw=(1.,2.,0.),
        localized=True,lift_um=1000,lift_velocity=0.,lift_homed=True,lift_supported=True,
        surface_valid=False,collision_scene_ready=True,arm_clear=True,reachable=False,collision_checked=False,
        checked_capture_id='',checked_scene_revision='')


def producer():
    now=[1.];p=EvidenceProducer(lambda:now[0],boot_id=42,profile_sha256='a'*64,maximum_age_s=.25,maximum_skew_s=.1)
    values=source_values()
    for i,(key,fields) in enumerate(FIELDS.items()):
        p.observe(key,SourceSample(1,.91+.01*i,42,'a'*64,{k:values[k] for k in fields}))
    return p,now


def test_source_fusion_preserves_oldest_time_and_does_not_refresh_on_poll():
    p,now=producer();r=p.produce()
    assert r.observed_s==.91 and r.sequence==1 and r.healthy and not r.reachable
    assert p.produce() is None
    p.observe('arm',replace(p.samples['arm'],sequence=2,observed_s=1.))
    assert p.produce().observed_s==pytest.approx(.92)
    now[0]=2
    with pytest.raises(ValueError):p.produce()


@pytest.mark.parametrize('mutate',[
    lambda s:replace(s,sequence=1),lambda s:replace(s,boot_id=43,sequence=2,observed_s=1.),
    lambda s:replace(s,profile_sha256='b'*64,sequence=2,observed_s=1.),
    lambda s:replace(s,observed_s=2.,sequence=2),
    lambda s:replace(s,values={**s.values,'healthy':1},sequence=2,observed_s=1.),
    lambda s:replace(s,values={**s.values,'joints_rad':(float('nan'),)*12},sequence=2,observed_s=1.),
])
def test_replayed_rebooted_or_invalid_sources_latch_fault(mutate):
    p,_=producer()
    with pytest.raises(ValueError):p.observe('arm',mutate(p.samples['arm']))
    assert p.fault
    with pytest.raises(ValueError):p.produce()


def test_missing_sources_and_scene_readiness_never_manufacture_evidence():
    p,_=producer();p.samples.pop('lift');assert p.produce() is None
    s=p.samples['scene']
    with pytest.raises(ValueError):p.observe('scene',replace(s,sequence=2,observed_s=1.,values={**s.values,'reachable':True}))


def test_long_execution_uses_fresh_state_but_admission_requires_fresh_target():
    now=[1.];ctx=[PlanningContext(1,'c','p','s','cal','base_link',(0.,)*12,True,True,target_observed_s=1.)]
    class Port:
        state='RUNNING'
        def ready(self):return True
        def start(self,g,p):pass
        def poll(self,g):return PortResult(self.state)
        def cancel(self,g):self.state='CANCELLED'
        def result(self,g):return ('trajectory',)
    planner=Port();executor=Port();p=ManipulationPort(planner,executor,lambda:ctx[0],lambda:now[0])
    p.start('run',{});planner.state='SUCCEEDED';p.poll('run')
    now[0]=4.;ctx[0]=replace(ctx[0],observed_s=4.)
    assert p.poll('run').state=='RUNNING'
    ctx[0]=replace(ctx[0],scene_revision='changed')
    p.poll('run');assert p.poll('run').state=='FAILED'
    q=ManipulationPort(Port(),Port(),lambda:ctx[0],lambda:now[0])
    with pytest.raises(ValueError):q.start('stale',{})


def test_surface_uses_frozen_depth_and_rejects_curvature_missing_depth_or_wrong_object():
    from pathlib import Path
    import json
    root=Path(__file__).resolve().parents[1]
    read=lambda n:json.loads((root/'config'/n).read_text())
    s=OfflineStack(read('fetch_stack.simulation.json'),read('home.example.json'),read('navigation_map.simulation.json'))
    for _ in range(3):s.tick()
    pair=s.source.latest;sampler=SurfaceSampler({'bedroom_drop_zone':(24,16,40,32)})
    params={'destination_place':'bedroom_drop_zone','object_id':'remote_control'}
    samples=sampler(pair,params);info={'surface_samples_uv_depth':samples,'detections':[]}
    result=s.surface('inspect_destination',params,(pair.frame,info))
    assert result['observed_s']==pair.frame.observed_s and result['capture_id']==pair.frame.capture_id
    assert result['position_m'][2]==1. and not result['object_at_destination']
    for samples in ((),tuple((u,v,z+.3 if i%2 else z) for i,(u,v,z) in enumerate(samples))):
        with pytest.raises(ValueError):s.surface('inspect_destination',params,(pair.frame,{**info,'surface_samples_uv_depth':samples}))
    with pytest.raises(ValueError):s.surface('verify_delivery',params,(pair.frame,info))
    with pytest.raises(ValueError):sampler(pair,{'destination_place':'unknown'})


def test_bindings_resolve_fresh_stage_and_share_arm_without_guessing_pose():
    class Child:
        starts=[]
        def ready(self):return True
        def start(self,g,p):self.starts.append((g,p))
        def poll(self,g):return PortResult('SUCCEEDED')
        def cancel(self,g):pass
    scene=Child();scene.links={'left':['left_hand'],'right':['right_hand']};scene.world='base_link';scene.age=.5
    catalog=SimpleNamespace(object_id='remote',motion=lambda:SimpleNamespace(pose_revision='p'),scene=lambda:'s')
    supplied=dict(stage='pick_approach',object_id='remote',arm='left',observed_s=1.,capture_id='c',
        pose_revision='p',scene_revision='s',calibration_id='cal',frame_id='base_link',position_m=[.1,.2,.3],quaternion_xyzw=[0,0,0,1])
    bindings=ManipulationBindings(calibrations=dict(left=calibration(),right=calibration()),scene_port=scene,
        stage_observation=lambda *a:supplied,scene_evidence=lambda *a:dict(object_id='remote',observed_s=1.,scene_revision='s',attached=False,detached=True),
        offsets_m={k:(0,0,.02) for k in MOTION_STAGES},maximum_age_s=.5)
    arm=Child();p=bindings.factory(motion=Child(),arm=arm,monitor=lambda g,p,t:Feedback(g,t,'READY',{}),clock=lambda:1.,catalog=catalog)
    params=dict(object_id='remote',arm='left',operation='pick',target=dict(calibration_id='cal',frame_id='base_link'))
    resolved=p.resolve('pick_approach',params,1.)
    assert resolved['position_m']==pytest.approx([.1,.2,.32]) and catalog.manipulation_target['observed_s']==1.
    assert p.gripper.arm is arm
    for bad in ({'observed_s':0},{'scene_revision':'other'},{'object_id':'other'},{'stage':'pick_retreat'}):
        original=deepcopy(supplied);supplied.update(bad)
        with pytest.raises(ValueError):p.resolve('pick_approach',params,1.)
        supplied.clear();supplied.update(original)
