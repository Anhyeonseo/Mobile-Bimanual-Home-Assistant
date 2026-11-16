import math
from dataclasses import replace

import numpy as np
import pytest

from home_robot_tasks.assistant.contracts import Context, Stamp
from home_robot_tasks.assistant.object_alignment import (
    AlignmentPolicy, ObjectAlignmentPlanner, shape_regions,
)
from home_robot_tasks.assistant.placement import Check
from home_robot_tasks.fetch import InvalidTask
from tools.diagnostics.run_s3_stationary_lab import model


def patch(length=.16, width=.04, yaw=0, center=(1.,.5,.82)):
    x,y=np.meshgrid(np.linspace(-length/2,length/2,35),np.linspace(-width/2,width/2,15))
    c,s=math.cos(yaw),math.sin(yaw)
    xy=np.column_stack([x.ravel(),y.ravel()])@np.array([[c,s],[-s,c]])
    return np.column_stack([xy,np.zeros(len(xy))])+center


@pytest.fixture(scope='module')
def planner():
    return ObjectAlignmentPlanner({'left':model(0)})


def plan(planner, points=None, **extra):
    ctx=Context('map','revision',planner.arms['left'].profile['calibration_id'],'clock')
    args=dict(points_map_m=patch() if points is None else points,support_z_m=.8,
              stamp=Stamp(1.,ctx,'capture'),context=ctx,now_s=1.,scene_revision='scene',
              current_xy_yaw=[.5,.5,0.],current_lift_m=.4)
    args.update(extra)
    return planner.plan(**args)


@pytest.mark.parametrize('yaw',[0.,.4,math.pi/2,2.3])
def test_elongated_shape_prefers_short_width_without_name(yaw):
    regions,_=shape_regions(patch(yaw=yaw),.8,(.0024,.0633))
    best=min(regions,key=lambda r:r['cost'])
    # The preferred closing axis is transverse to the long axis after rotation.
    assert abs(np.dot(best['closing_xy'],[math.cos(yaw),math.sin(yaw)]))<.3
    assert best['observed_width_m']<.05


def test_round_shape_keeps_direction_options():
    a=np.linspace(0,2*np.pi,96,endpoint=False)
    r=np.linspace(.001,.023,12)
    xy=np.stack([np.cos(a),np.sin(a)],axis=1)[None,:,:]*r[:,None,None]
    points=np.column_stack([xy.reshape(-1,2),np.full(xy.size//2,.825)])
    regions,_=shape_regions(points,.8,(.0024,.0633))
    assert len({tuple(np.round(x['closing_xy'],3)) for x in regions})>=8


def test_wide_body_can_retain_a_narrow_observed_neck():
    body=patch(.14,.12,center=(0.,0.,.83))
    neck=patch(.08,.03,center=(.105,0.,.83))
    regions,_=shape_regions(np.vstack([body,neck]),.8,(.0024,.0633))
    assert regions
    assert any(r['anchor_map_m'][0]>.06 for r in regions)


def test_recovered_base_lift_is_a_true_five_joint_fk_witness(planner):
    result=plan(planner)
    assert result['status']=='MODEL_PROPOSALS' and result['candidates']
    arm=planner.arms['left']
    for c in result['candidates']:
        tcp=model(c['lift_travel_m']).fk(c['fk_witness_model_rad'])[0]
        x,y,yaw=c['base_xy_yaw'];co,si=math.cos(yaw),math.sin(yaw)
        stock=tcp@arm.tcp_from_stock
        fixed=stock[:3,3]-stock[:3,0]*arm.jaw.fixed_face_m
        rotation=np.array([[co,-si],[si,co]])
        xy=rotation@fixed[:2]+[x,y]
        z=fixed[2]-arm.profile['ground_z_m']
        region=c['observed_region'];axis=np.array(region['closing_xy'])
        sign=1 if (rotation@(-stock[:2,0]))@axis>0 else -1
        expected=np.array(region['anchor_map_m'])-np.r_[axis*sign,0.]*region['observed_width_m']/2
        np.testing.assert_allclose([*xy,z],expected,atol=1e-10)
        direction=np.array([[co,-si],[si,co]])@(-tcp[:2,0])
        direction/=np.linalg.norm(direction)
        assert abs(np.dot(direction,c['observed_region']['closing_xy']))>1-1e-10
        assert 0<=c['lift_travel_m']<=arm.profile['lift']['travel_m']
    assert not result['executable'] and not result['motion_authorized']
    assert result['requires_reobservation'] and not result['fine_grasp_generated']


def test_screening_failure_never_falls_back_to_unscreened(planner):
    def blocked(c):
        return dict(navigation=Check(False,'table_blocks_route',mode='simulation'),
                    view=Check(True,'visible',mode='simulation'),
                    configuration=Check(True,'fixture_only',mode='simulation'))
    r=plan(planner,screen=blocked)
    assert r['status']=='NO_SAMPLED_PROPOSAL' and not r['candidates']
    assert r['rejected']['navigation:table_blocks_route']==r['evaluated']


def test_unverified_or_missing_collision_screen_cannot_pass(planner):
    def incomplete(c):
        return dict(navigation=Check(True,'route',mode='simulation'),
                    view=Check(True,'view',mode='simulation'))
    with pytest.raises(InvalidTask,match='complete prealignment'):
        plan(planner,screen=incomplete)
    def unverified(c):
        return {**incomplete(c),'configuration':Check(True,'not_checked')}
    assert not plan(planner,screen=unverified)['candidates']


def test_stale_and_wrong_calibration_rejected(planner):
    with pytest.raises(InvalidTask,match='stale'):
        plan(planner,now_s=5.)
    ctx=Context('map','revision','other-cal','clock')
    with pytest.raises(InvalidTask,match='calibration'):
        plan(planner,context=ctx,stamp=Stamp(1.,ctx,'capture'))


def test_map_rotation_and_translation_preserve_relative_solution(planner):
    original=plan(planner)['candidates'][0]
    angle=.7;c,s=math.cos(angle),math.sin(angle)
    rotation=np.array([[c,-s],[s,c]]);translation=np.array([2.,-1.])
    moved=patch();moved[:,:2]=moved[:,:2]@rotation.T+translation
    current=rotation@np.array([.5,.5])+translation
    candidate=plan(planner,moved,current_xy_yaw=[*map(float,current),angle])['candidates'][0]
    np.testing.assert_allclose(candidate['base_xy_yaw'][:2],
                               rotation@original['base_xy_yaw'][:2]+translation,atol=1e-8)
    assert candidate['lift_travel_m']==pytest.approx(original['lift_travel_m'])
    assert candidate['cost']==pytest.approx(original['cost'])


def test_empty_search_is_not_a_claim_of_unreachability(planner):
    r=plan(planner,patch(center=(1.,.5,2.9)))
    assert r['status']=='NO_SAMPLED_PROPOSAL' and not r['candidates']
    assert r['rejected']['lift_outside_model']>0


def test_invalid_geometry_and_budgets_fail_explicitly():
    with pytest.raises(ValueError,match='segmented'):
        shape_regions(np.full((30,3),np.nan),.8,(.002,.063))
    with pytest.raises(ValueError,match='support'):
        shape_regions(patch(center=(1.,.5,.7)),.8,(.002,.063))
    with pytest.raises(ValueError,match='bounded'):
        replace(AlignmentPolicy(),joint_samples=100000)


def test_arrival_sensitivity_resolves_stock_contact_but_rejects_far_target(planner):
    from copy import deepcopy
    from home_robot_tasks.assistant.object_alignment import alignment_reach_sensitivity
    arm=planner.arms['left']
    sample=min(planner.samples['left'],key=lambda s:s['posture']['closing_line_from_horizontal_deg'])
    stock=sample['pose']@arm.tcp_from_stock; width=.04
    center=stock[:3,3]-stock[:3,0]*arm.jaw.center_from_fixed_m(width)
    center[2]-=arm.profile['ground_z_m']
    axis=-stock[:2,0];axis/=np.linalg.norm(axis)
    candidate=dict(base_xy_yaw=[0.,0.,0.],lift_travel_m=arm.lift_m,
        fk_witness_model_rad=sample['q'].tolist(),observed_region=dict(
            anchor_map_m=center.tolist(),closing_xy=axis.tolist(),observed_width_m=width))
    original=deepcopy(candidate)
    r=alignment_reach_sensitivity(arm,candidate,translation_m=.01,yaw_rad=.02)
    assert r['nominal_passed'] and r['passed']>0
    assert not r['motion_authorized'] and not r['physical_accuracy_verified']
    assert not r['collision_checked'] and r['requires_fresh_capture']
    assert candidate==original
    far=deepcopy(candidate);far['observed_region']['anchor_map_m'][0]+=2.
    r=alignment_reach_sensitivity(arm,far,translation_m=.01,yaw_rad=.02)
    assert not r['nominal_passed'] and r['passed']==0


def test_signed_finger_direction_rejects_oblique_elongated_but_not_round(planner):
    from copy import deepcopy
    from home_robot_tasks.assistant.object_alignment import contact_direction_review
    arm=planner.arms['left'];sample=planner.samples['left'][0];q=sample['q']
    tcp=arm.fk(q)[0];axis=tcp[:2,2]/np.linalg.norm(tcp[:2,2])
    c=dict(base_xy_yaw=[0.,0.,0.],lift_travel_m=0.,fk_witness_model_rad=q.tolist(),
           observed_region=dict(observed_width_m=.04,long_axis_preference_weight=1.,long_axis_xy=axis.tolist()))
    r=contact_direction_review(arm,c);assert r['accepted']
    assert r['approach_parallel_error_deg']<1e-5
    fixed=np.array(r['fixed_contact_map_m']);moving=np.array(r['moving_contact_map_m'])
    np.testing.assert_allclose(moving-fixed,.04*np.array(r['fixed_to_moving_map']),atol=1e-10)
    wider=deepcopy(c);wider['observed_region']['observed_width_m']=.05
    rw=contact_direction_review(arm,wider)
    np.testing.assert_allclose(rw['fixed_contact_map_m'],fixed,atol=1e-12)
    assert np.linalg.norm(np.array(rw['moving_contact_map_m'])-moving)==pytest.approx(.01)
    # A 20-degree oblique approach must not be rescued by low travel cost.
    yaw=math.radians(20);R=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    c['observed_region']['long_axis_xy']=(R@axis).tolist()
    assert not contact_direction_review(arm,c)['accepted']
    c['observed_region']['long_axis_preference_weight']=0.
    assert contact_direction_review(arm,c)['accepted']
    assert not r['collision_checked'] and not r['motion_authorized']


def test_shape_regions_preserve_unoriented_long_axis_under_rotation():
    for yaw in (0.,.5,1.3):
        regions,_=shape_regions(patch(yaw=yaw),.8,(.0024,.0633))
        for r in regions:
            assert abs(np.dot(r['long_axis_xy'],[math.cos(yaw),math.sin(yaw)]))>.999


def test_reach_comfort_uses_physical_contact_and_not_map_distance(planner):
    from home_robot_tasks.assistant.object_alignment import reach_comfort_review
    arm=planner.arms['left'];sample=planner.samples['left'][0]
    c=dict(fk_witness_model_rad=sample['q'].tolist(),observed_region=dict(observed_width_m=.04),base_xy_yaw=[0,0,0])
    r=reach_comfort_review(arm,c)
    assert 0<r['extension_ratio']<=1 and r['joint_margin_fraction']>=0
    assert not r['payload_verified']
    c['base_xy_yaw']=[10,-5,2]
    assert reach_comfort_review(arm,c)==r
