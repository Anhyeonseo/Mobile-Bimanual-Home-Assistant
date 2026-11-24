from dataclasses import dataclass

import numpy as np
import pytest

from home_perception.geometry.plane import Plane
from home_perception.grasp import graspnet
from tools.diagnostics.benchmark_grasp_feasibility import scan_candidates, bound_prepared


def test_disabling_legacy_audit_preserves_proposals(monkeypatch):
    import home_perception.geometry.workspace as workspace
    monkeypatch.setattr(workspace, 'workspace_mask', lambda depth, *args: np.ones_like(depth, dtype=bool))
    raw=np.zeros((1,17)); raw[0,15]=.5
    monkeypatch.setattr(graspnet, 'infer', lambda *args:(raw,object))
    monkeypatch.setattr(graspnet, 'associate', lambda rows,*args:rows)
    calls=[]
    def collision(rows,*args):calls.append('collision');return rows
    def rank(rows,*args):calls.append('rank');return rows,{}
    monkeypatch.setattr(graspnet,'collision_filter',collision)
    monkeypatch.setattr(graspnet,'rank_for_jaw',rank)
    monkeypatch.setattr(graspnet,'select_proposals',lambda *args,**kwargs:([{'source':'test'}],{}))
    depth=np.full((2,2),.5);intr=dict(fx=500.,fy=500.,ppx=0.,ppy=0.)
    plane=Plane(np.array([0.,0.,-1.]),np.array([0.,0.,.6]),0.,1.)
    args=(depth,intr,[depth>0],0,np.array([[0.,0.,.5]]),plane,.1,(.002,.06))
    enabled=graspnet.candidates(*args)
    assert calls==['collision','rank']
    calls.clear()
    disabled=graspnet.candidates(*args,audit_enabled=False)
    assert not calls
    assert disabled['refined_grasps']==enabled['refined_grasps']
    assert disabled['legacy_parallel_jaw_audit']==dict(enabled=False,authoritative=False)


@dataclass
class Route:
    status: str


def test_first_route_stop_does_not_claim_full_pool_rate():
    def bind(index,sign):return (index,sign),{}
    result=scan_candidates(3,bind,lambda c:Route('OFFLINE_ROUTE_FOUND'))
    assert result['tested_signed_candidates']==1
    assert result['first_feasible_validation_ms'] is not None
    assert result['full_pool_feasible_fraction'] is None
    assert not result['full_pool_evaluated']


def test_exhaustive_counts_binding_and_route_failures_separately():
    def bind(index,sign):
        if index==0:raise ValueError('IK rejected')
        return (index,sign),{}
    result=scan_candidates(2,bind,lambda c:Route('OFFLINE_ROUTE_FOUND' if c[1]==1 else 'NO_COMPLETE_ROUTE'),exhaustive=True)
    assert result['tested_signed_candidates']==4
    assert result['bound_signed_candidates']==2
    assert result['feasible_signed_candidates']==1
    assert result['full_pool_feasible_fraction']==.25


def test_infrastructure_error_is_not_an_infeasible_grasp():
    def plan(candidate):raise RuntimeError('MoveIt unavailable')
    with pytest.raises(RuntimeError,match='MoveIt unavailable'):
        scan_candidates(1,lambda i,s:(object(),{}),plan)


def test_no_route_has_no_time_to_success():
    result=scan_candidates(1,lambda i,s:(object(),{}),lambda c:Route('NO_COMPLETE_ROUTE'))
    assert result['status']=='NO_COMPLETE_ROUTE'
    assert result['first_feasible_validation_ms'] is None
    assert result['full_pool_feasible_fraction']==0.


def test_new_prepared_drops_old_candidate_diagnostics():
    keys=('capture_id','observation_index','label','source_sha256','coordinate_frame',
          'perception_status','session_id','calibration_id','mask_sha256')
    template=dict(group={k:k for k in keys},candidate_diagnostics=['old'],path='old')
    group=dict(benchmark_backend='so101',refined_grasps=[])
    result=bound_prepared(template,group)
    assert 'path' not in result and 'candidate_diagnostics' not in result
    assert template['candidate_diagnostics']==['old']
    assert result['group']['capture_id']=='capture_id'
