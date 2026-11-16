"""Stage sequencing and rejection tests; real meshes run in check_carry_moveit."""
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from home_robot_tasks.carry_planning import GraspRouteCandidate, plan_place_carry, PlanRejected
from home_robot_tasks.carry_moveit import StagePath


class CartesianFixture:
    profile={'calibration_id':'fixture'}
    lo=np.full(5,-2.);hi=np.full(5,2.)
    def fk(self,q):
        pose=np.eye(4);pose[:3,3]=q[:3]
        return pose,None
    def solve(self,pose,seed):return np.r_[pose[:3,3],0.,0.],0.,0.


class Planner:
    def __init__(self,blocked=None):self.calls=[];self.blocked=blocked
    def path(self,start,end,candidate,attached,label):
        self.calls.append((label,attached))
        if label==self.blocked:raise PlanRejected('blocked whole path')
        return StagePath(label,np.linspace(start,end,10),'fixture')
    def connect(self,start,end,candidate,*,attached,label):return self.path(start,end,candidate,attached,label)
    def cartesian(self,start,pose,candidate,*,attached,label):return self.path(start,np.r_[pose[:3,3],0.,0.],candidate,attached,label)
    def release(self,q,candidate):return self.path(q,q,candidate,False,'release_geometry')


def inputs():
    arm=CartesianFixture();pose=np.eye(4);pose[:3,3]=[.3,0,.511]
    held=GraspRouteCandidate('held','fixture','scene',np.zeros(5),pose,np.eye(4),(.04,.04,.02),.5,.3)
    kwargs=dict(scene_id='scene',base_from_object=pose,support_bounds_m=[[.2,-.1,.5],[.4,.1,.5]],
        support_observation_id='capture',support_observed_s=1.,now_s=1.1,
        empty_transport_model_rad=np.zeros(5))
    return arm,held,kwargs


def test_place_checks_attached_lowering_and_detached_open_retreat():
    arm,held,kw=inputs();planner=Planner()
    result=plan_place_carry(arm,planner,np.zeros(5),held,**kw)
    assert result.status=='OFFLINE_ROUTE_FOUND'
    assert planner.calls==[('place_preplace',True),('place_lower',True),('release_geometry',False),('place_retreat',False),('empty_transport',False)]
    assert result.runtime_release_verification_required and not result.motion_authorized
    assert not result.physical_task_completed


@pytest.mark.parametrize('bad',['old_support','scene','calibration','outside','hover','below','tilted_support'])
def test_invalid_support_or_attachment_cannot_start_planning(bad):
    arm,held,kw=inputs();planner=Planner()
    if bad=='old_support':kw['now_s']=4.
    elif bad=='scene':kw['scene_id']='other'
    elif bad=='calibration':held=replace(held,calibration_id='other')
    elif bad=='outside':kw['base_from_object'][0,3]=.5
    elif bad=='hover':kw['base_from_object'][2,3]+=.01
    elif bad=='below':kw['base_from_object'][2,3]-=.01
    else:kw['support_bounds_m'][1][2]+=.01
    result=plan_place_carry(arm,planner,np.zeros(5),held,**kw)
    assert result.status=='NO_COMPLETE_ROUTE' and not planner.calls


@pytest.mark.parametrize('stage',['place_preplace','place_lower','release_geometry','place_retreat','empty_transport'])
def test_incomplete_place_path_is_never_reported_complete(stage):
    arm,held,kw=inputs();planner=Planner(stage)
    result=plan_place_carry(arm,planner,np.zeros(5),held,**kw)
    assert result.status=='NO_COMPLETE_ROUTE' and not result.stages
    assert planner.calls[-1][0]==stage
