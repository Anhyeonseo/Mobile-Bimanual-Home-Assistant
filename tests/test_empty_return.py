"""Empty recovery is one finite checked route; no grasp/contact claims."""
from dataclasses import asdict
from types import SimpleNamespace as NS
import json
import numpy as np
import pytest
from home_robot_tasks.empty_return import run_empty_return,compile_empty_return,load_empty_return
from home_robot_tasks.carry_trajectory import MotionBounds
from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.carry_program import _control
from tools.diagnostics.run_s3_stationary_lab import model
from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint


def fixture(tmp_path):
    arm=model(0);profile=arm.profile
    q=arm.anchor.copy();up=np.vstack([q,q+[0,.01,0,0,0]]);back=up[::-1]
    anchor=[]
    for side in ('left','right'):
        a=model(0,side=side)
        anchor.extend([_control(profile,side,n,float(v)) for n,v in zip(geo.JOINTS[:5],a.anchor)]+[0.])
    bounds=MotionBounds((-4.,)*12,(4.,)*12,(.3,)*12,(.6,)*12,(3.,)*12)
    p=compile_empty_return(anchor,up,back,-.4,profile,bounds);m={'profile':profile}
    d=dict(kind='checked_empty_return',schema_version=1,program=p,model=m,planned_lift_m=0.,
           proof=dict(program_sha256=geo.digest(p),model_sha256=geo.digest(m),observed_empty_required=True,
                      base_commands=0,lift_commands=0,native_geometry={'status':'PASSED'}))
    path=tmp_path/'return.json';path.write_text(json.dumps(d));return path,d


def test_return_preserves_right_arm_and_opens_before_any_arm_motion(tmp_path):
    path,d=fixture(tmp_path);_,points=load_empty_return(path)
    q=np.array([p.positions_rad for p in points]);first=d['program']['stages'][0]['samples']
    assert np.all(q[:first,:5]==q[0,:5]) and np.all(q[:,6:]==q[0,6:])
    assert np.all(q[first:,5]==-.4)


@pytest.mark.parametrize('defect',['binding','loaded','right_arm','opening_arm','closing_again','lift'])
def test_return_cannot_hide_changed_or_unchecked_motion(tmp_path,defect):
    path,d=fixture(tmp_path)
    if defect=='binding':d['proof']['program_sha256']='changed'
    elif defect=='loaded':d['proof']['observed_empty_required']=False
    elif defect=='lift':d['planned_lift_m']=.01
    else:
        i=1 if defect=='opening_arm' else -1
        axis=6 if defect=='right_arm' else 0 if defect=='opening_arm' else 5
        d['program']['points'][i]['positions_rad']=list(d['program']['points'][i]['positions_rad'])
        d['program']['points'][i]['positions_rad'][axis]+=.01
        d['proof']['program_sha256']=geo.digest(d['program'])
    path.write_text(json.dumps(d))
    with pytest.raises(ValueError):load_empty_return(path)


def test_resident_return_dispatches_once_without_stage_anchor_rechecks():
    points=(TimedJointPoint(50,(0.,)*12),TimedJointPoint(100,(.01,)*6+(0.,)*6))
    clock=[0.];calls=[];state=NS(value='ready');polls=[0]
    def start(*a,**kw):calls.append(('start',kw));state.value='active'
    def poll(*a):
        polls[0]+=1
        if polls[0]==3:state.value='ready'
    adapter=NS(state=state,feedback_snapshot=lambda:NS(positions_urad=(0,)*12),start=start,poll=poll)
    run_empty_return(adapter,points,owner='test',service=lambda:calls.append('service'),
        clock=lambda:clock[0],sleep=lambda dt:clock.__setitem__(0,clock[0]+dt),cancelled=lambda:False)
    assert sum(isinstance(c,tuple) and c[0]=='start' for c in calls)==1
    assert calls.count('service')==4 and state.value=='ready'


def test_changed_anchor_never_dispatches_return():
    adapter=NS(feedback_snapshot=lambda:NS(positions_urad=(100000,)*12),start=lambda *a,**kw:pytest.fail('dispatch'))
    with pytest.raises(ValueError,match='anchor changed'):
        run_empty_return(adapter,(TimedJointPoint(50,(0.,)*12),),owner='test',service=lambda:None,
            clock=lambda:0.,sleep=lambda _:None,cancelled=lambda:False)


def test_cancelled_return_does_not_dispatch_or_replay():
    calls=[]
    adapter=NS(feedback_snapshot=lambda:NS(positions_urad=(0,)*12),
               start=lambda *a,**kw:calls.append('start'))
    with pytest.raises(ValueError,match='cancelled before dispatch'):
        run_empty_return(adapter,(TimedJointPoint(50,(0.,)*12),),owner='test',service=lambda:None,
            clock=lambda:0.,sleep=lambda _:None,cancelled=lambda:True)
    assert not calls


def test_active_return_fault_propagates_without_restart():
    calls=[];state=NS(value='ready')
    def start(*a,**kw):calls.append('start');state.value='active'
    def poll(*a):state.value='fault'
    adapter=NS(state=state,feedback_snapshot=lambda:NS(positions_urad=(0,)*12),start=start,poll=poll)
    with pytest.raises(ValueError,match='controller not active'):
        run_empty_return(adapter,(TimedJointPoint(50,(0.,)*12),),owner='test',service=lambda:None,
            clock=lambda:0.,sleep=lambda _:None,cancelled=lambda:False)
    assert calls==['start']
