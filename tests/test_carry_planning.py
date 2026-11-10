"""Carry semantics and coordinator regression tests; no ROS/hardware required."""
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.carry_posture import CarryPolicy, ObjectAxisConstraint, carry_goals
from home_robot_tasks.carry_planning import GraspRouteCandidate, PlanRejected, plan_pick_carry
from home_robot_tasks.carry_moveit import StagePath
from home_robot_tasks.stationary_pick_sim import StationaryArm

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def arm():
    config = ROOT/'config/home_assistant'
    read = lambda n: json.loads((config/n).read_text())
    return StationaryArm(read('s2_coordinates.development.json'), geo.load_geometry(config/'official_geometry.json'),
                         read('arm_ranges.development.json'), 0., None)


def test_grasp_changes_wrist_without_replacing_common_posture(arm):
    outputs = []
    for angle in (-35, 35):
        lifted = arm.anchor+np.deg2rad([0,20,-30,-25,angle])
        goals = carry_goals(arm, lifted, arm.fk(lifted)[0])
        assert goals
        assert len(goals) <= 6
        q = goals[0].model_rad
        reference=np.deg2rad(CarryPolicy().reference_model_deg)
        assert np.max(abs(q[:3]-reference[:3])) < np.deg2rad(25)
        outputs.append(q)
    assert abs(outputs[1][-1]-outputs[0][-1]) > np.deg2rad(30)


def test_object_upright_is_in_object_frame_and_is_hard(arm):
    lifted = arm.anchor+np.deg2rad([0,20,-30,-25,10])
    pose = arm.fk(lifted)[0]
    attached = np.eye(4)
    attached[:3,:3] = pose[:3,:3].T  # upright object, tilted wrist
    constraint = ObjectAxisConstraint(attached, maximum_tilt_rad=np.deg2rad(5), evidence='fixture upright task')
    assert constraint.satisfied(pose)
    goals = carry_goals(arm, lifted, pose, object_constraint=constraint)
    assert goals
    assert all(g.object_tilt_rad <= np.deg2rad(5)+1e-7 for g in goals)
    bad = replace(constraint, base_axis=(0.,0.,-1.))
    with pytest.raises(ValueError, match='lifted state violates'):
        carry_goals(arm, lifted, pose, object_constraint=bad)


def test_carry_reference_does_not_change_calibrated_home(arm):
    home=arm.anchor.copy();reference=np.deg2rad(CarryPolicy().reference_model_deg)
    assert not np.allclose(home,reference)
    carry_goals(arm,home,arm.fk(home)[0])
    np.testing.assert_array_equal(arm.anchor,home)


def test_clearance_alternatives_explore_forward_without_new_axis_search(arm):
    reference=np.deg2rad(CarryPolicy().reference_model_deg)
    q=arm.anchor+np.deg2rad([0,30,-40,-30,0])
    goals=carry_goals(arm,q,arm.fk(q)[0])
    assert 2<=len(goals)<=6
    assert max(g.base_from_tcp[0,3] for g in goals)-min(g.base_from_tcp[0,3] for g in goals)>.02


@pytest.mark.parametrize('kwargs', [dict(maximum_results=0), dict(maximum_iterations=10000),
                                   dict(grasp_orientation_weight=float('nan')), dict(joint_weights=(1,1))])
def test_search_budget_and_weights_validated(kwargs):
    with pytest.raises(ValueError): CarryPolicy(**kwargs)


def test_orientation_requires_explicit_semantics():
    with pytest.raises(ValueError): ObjectAxisConstraint(np.eye(4))
    with pytest.raises(ValueError): ObjectAxisConstraint(np.eye(4), object_axis=(0,0,2), evidence='task')


def candidate(arm, name='one'):
    q = arm.anchor.copy()
    return GraspRouteCandidate(name, arm.profile['calibration_id'], 'scene-1', q, arm.fk(q)[0],
                               np.eye(4), (.02,.02,.03), .5, .3)


class FakePlanner:
    """Explicit coordinator fake; physical route assertions use the native lab."""
    def __init__(self, blocked=None): self.calls=[]; self.blocked=blocked
    def open(self,q,c):
        self.calls.append((c.candidate_id,'open_geometry',False))
        return StagePath('open_geometry',np.array([q]),'fake')
    def connect(self, start, goal, c, *, attached, label):
        self.calls.append((c.candidate_id,label,attached))
        if (c.candidate_id,label)==self.blocked: raise PlanRejected('fixture blocked')
        return StagePath(label,np.array([start,goal]),'fake')
    def cartesian(self, start, goal, c, *, attached, label):
        self.calls.append((c.candidate_id,label,attached))
        if (c.candidate_id,label)==self.blocked: raise PlanRejected('partial Cartesian')
        return StagePath(label,np.array([start,start]),'fake')
    def close(self,q,c):
        self.calls.append((c.candidate_id,'close_geometry',True))
        return StagePath('close_geometry',np.array([q]),'fake')


def test_complete_route_selection_tries_other_grasp_after_carry_failure(arm):
    backend = FakePlanner(blocked=('one','carry'))
    plan = plan_pick_carry(arm,backend,arm.anchor,[candidate(arm),candidate(arm,'two')],scene_id='scene-1')
    assert plan.status=='OFFLINE_ROUTE_FOUND' and plan.candidate_id=='two'
    assert [s.label for s in plan.stages]==['open_geometry','pregrasp','approach','close_geometry','lift','carry']
    assert not plan.motion_authorized and not plan.physical_task_completed
    assert plan.failures[0]['stage']=='carry'


def test_partial_approach_never_attaches_or_lifts(arm):
    backend=FakePlanner(blocked=('one','approach'))
    result=plan_pick_carry(arm,backend,arm.anchor,[candidate(arm)],scene_id='scene-1')
    assert result.status=='NO_COMPLETE_ROUTE'
    assert all(not attached for _,_,attached in backend.calls)


@pytest.mark.parametrize('center_blocked',[False,True])
def test_balance_precedes_carry_comfort_but_never_overrides_failed_path(arm,monkeypatch,center_blocked):
    from types import SimpleNamespace
    from home_robot_tasks import carry_planning
    center=candidate(arm,'center')
    q=arm.anchor+np.deg2rad([0,20,-10,0,0]);pose=arm.fk(q)[0]
    edge=replace(center,candidate_id='edge',pregrasp_model_rad=q,base_from_grasp_tcp=pose,
        tcp_from_object=np.linalg.inv(pose)@center.base_from_grasp_tcp)
    # Same object's world pose; edge contact has a cheaper carry pose on purpose.
    def goals(arm,q,*args,**kwargs):
        near=np.linalg.norm(kwargs['payload'].tcp_from_object[:3,3])<.001
        return [SimpleNamespace(model_rad=q,cost=10. if near else 0.)]
    monkeypatch.setattr(carry_planning,'carry_goals',goals)
    backend=FakePlanner(blocked=('center','approach') if center_blocked else None)
    result=plan_pick_carry(arm,backend,arm.anchor,[edge,center],scene_id='scene-1')
    assert result.status=='OFFLINE_ROUTE_FOUND'
    assert result.candidate_id==('edge' if center_blocked else 'center')
    assert result.selection['policy']=='near_best_feasible_balance_then_grasp_and_carry_cost'
    assert not result.selection['physical_balance_validated']
    if center_blocked:assert any(f['candidate']=='center' for f in result.failures)
    else:assert len(result.selection['alternatives'])==2


@pytest.mark.parametrize('field,value', [('scene_id','old'),('calibration_id','old')])
def test_stale_grasp_does_not_reach_planner(arm,field,value):
    backend=FakePlanner()
    result=plan_pick_carry(arm,backend,arm.anchor,[replace(candidate(arm),**{field:value})],scene_id='scene-1')
    assert not backend.calls and result.status=='NO_COMPLETE_ROUTE'


def test_control_encoder_wrap_not_accepted_as_model_angles(arm):
    bad=arm.anchor.copy(); bad[0]+=2*np.pi
    with pytest.raises(ValueError): carry_goals(arm,bad,arm.fk(arm.anchor)[0])


def test_payload_and_upright_frame_must_match(arm):
    transform=np.eye(4); transform[0,3]=.01
    with pytest.raises(ValueError,match='attachment differ'):
        replace(candidate(arm),object_constraint=ObjectAxisConstraint(transform,evidence='fixture'))


def bound_input(arm):
    from home_robot_tasks.so101_jaw import load_stock_jaw
    from home_robot_tasks.stationary_pick_sim import GRASP_FROM_TCP
    from home_robot_tasks.installed_jaw import InstalledLeftJaw
    from home_robot_tasks.carry_binding import BoundPayload
    arm.jaw=load_stock_jaw(ROOT/'config/home_assistant/official_sources/so101_new_calib.urdf',
                           ROOT/'ros2_ws/src/so101_description/meshes')
    q=arm.anchor+np.deg2rad([0,30,-40,-30,0])
    tcp=arm.fk(q)[0]
    center=tcp.copy(); center[:3,:3]=tcp[:3,:3]@GRASP_FROM_TCP.T
    center[:3,3]-=tcp[:3,0]*arm.jaw.center_from_fixed_m(.04)
    optical=np.linalg.inv(arm.profile['base_from_optical'])@center
    source=dict(source='graspnet',width_m=.04,depth_m=.02,rotation_matrix=optical[:3,:3].tolist(),
                translation_m=(optical[:3,3]-.02*optical[:3,0]).tolist(),
                closing_section=dict(status='VISIBLE_SPAN',low_m=-.016,high_m=.016))
    hashes={n:'a'*64 for n in ('color.png','depth_aligned.png','metadata.json')}
    review=dict(mode='offline_capture_review',status='TARGET_DISPLAY_ONLY',executable=False,motion_authorized=False,
                frame='base_link',calibration_id=arm.profile['calibration_id'],capture_id='fixture-capture',
                label='fixture-remote',observation_index=0,session_id='fixture-session',source_sha256=hashes,
                base_points_m={'center_xyz_m':center[:3,3].tolist()},lift_height_m=arm.lift_m,
                left_anchor_rad=dict(zip(geo.JOINTS[:5],q)))
    group={k:review[k] for k in ('capture_id','label','observation_index','session_id','source_sha256','calibration_id')}
    group.update(perception_status='accepted',coordinate_frame=arm.profile['source_frame'],
                 tcp_convention='graspnet_origin_plus_depth_to_pad_tip',refined_grasps=[source])
    payload=BoundPayload(review['capture_id'],hashes,arm.profile['calibration_id'],center,(.032,.015,.05))
    from test_installed_jaw import synthetic_jaw
    installed=synthetic_jaw(arm.profile)
    return review,group,payload,installed


def test_existing_graspnet_convention_and_measured_jaw_bind_to_carry(arm):
    from home_robot_tasks.carry_binding import bind_grasp
    review,group,payload,installed=bound_input(arm)
    bound,command=bind_grasp(arm,review,group,0,payload,installed,approach_distance_m=.006)
    assert bound.scene_id==review['capture_id']
    assert np.allclose(bound.base_from_grasp_tcp@bound.tcp_from_object,payload.base_from_object)
    assert command['held_geometry_raw']>command['measured_gap_preview']['lower_search_raw']
    assert not command['motion_authorized']


@pytest.mark.parametrize('jaw_sign',[1,-1])
def test_fixed_finger_contact_not_centered_in_open_aperture(arm,monkeypatch,jaw_sign):
    from home_robot_tasks import carry_binding
    from home_robot_tasks.stationary_pick_sim import pad_tip_optical
    review,group,payload,installed=bound_input(arm)
    source=group['refined_grasps'][0]
    source['closing_section']=dict(status='VISIBLE_SPAN',method='observed_contact_line',low_m=-.012,high_m=.025)
    # Translation contract is independent of whether a particular fixture pose
    # has five-DOF IK. Native scene tests separately check the complete route.
    monkeypatch.setattr(carry_binding,'fit_pregrasp',lambda arm,grip,seed,*a,**k:(True,seed,grip,{}))
    bound,command=carry_binding.bind_grasp(arm,review,group,0,payload,installed,jaw_sign=jaw_sign)
    tip=geo.rigid(arm.profile['base_from_optical'])@pad_tip_optical(source)
    expected=tip[:3,3]+tip[:3,1]*(-.012 if jaw_sign==1 else .025)
    fixed_face=bound.base_from_grasp_tcp[:3,3]-bound.base_from_grasp_tcp[:3,0]*arm.jaw.fixed_face_m
    np.testing.assert_allclose(fixed_face,expected,atol=1e-12)
    assert command['closure']['entry_gap_m']==pytest.approx(.041)
    assert command['closure']['target_gap_m']==pytest.approx(.032)
    np.testing.assert_allclose(bound.base_from_grasp_tcp@bound.tcp_from_object,payload.base_from_object,atol=1e-12)
    assert source['closing_section']['low_m']==-.012  # no mutation of evidence


@pytest.mark.parametrize('jaw_sign',[1,-1])
def test_wider_entry_retains_contact_attachment_and_close_target(arm,monkeypatch,jaw_sign):
    from home_robot_tasks import carry_binding
    review,group,payload,installed=bound_input(arm)
    source=group['refined_grasps'][0]
    source['closing_section']=dict(status='VISIBLE_SPAN',method='observed_contact_line',low_m=-.012,high_m=.025)
    seen=[]
    def fit(arm,grip,seed,*args,**kwargs):
        seen.append(kwargs['fixed_finger_clearance_m'])
        return True,seed,grip,{}
    monkeypatch.setattr(carry_binding,'fit_pregrasp',fit)
    original,old=carry_binding.bind_grasp(arm,review,group,0,payload,installed,jaw_sign=jaw_sign)
    revised,new=carry_binding.bind_grasp(arm,review,group,0,payload,installed,jaw_sign=jaw_sign,
                                        entry_clearance_m=.012,fixed_finger_clearance_m=.006)
    assert seen==[0.,.006]
    np.testing.assert_allclose(revised.base_from_grasp_tcp,original.base_from_grasp_tcp)
    np.testing.assert_allclose(revised.tcp_from_object,original.tcp_from_object)
    assert new['closure']['entry_gap_m']==pytest.approx(.049)
    for key in ('target_gap_m','upper_contact_gap_m','visible_span_m'):
        assert new['closure'][key]==old['closure'][key]
    assert new['measured_gap_preview']['entry_raw']>old['measured_gap_preview']['entry_raw']
    assert revised.held_jaw_model_rad==original.held_jaw_model_rad
    with pytest.raises(ValueError,match='moving finger'):
        carry_binding.bind_grasp(arm,review,group,0,payload,installed,
                                 entry_clearance_m=.012,fixed_finger_clearance_m=.0101)


@pytest.mark.parametrize('blocked',[False,True])
def test_preferred_wrist_is_soft_and_keeps_feasible_fallback(arm,monkeypatch,blocked):
    from types import SimpleNamespace
    from home_robot_tasks import carry_planning
    good=candidate(arm,'preferred')
    q=arm.anchor.copy();q[4]-=np.deg2rad(150)
    bad=replace(good,candidate_id='turned',pregrasp_model_rad=q)
    # Isolate ranking: equal valid route/geometry, not physical contact proof.
    monkeypatch.setattr(carry_planning,'at_actual_contact',lambda c,p:c)
    monkeypatch.setattr(carry_planning,'carry_goals',lambda arm,q,*a,**k:[SimpleNamespace(model_rad=q,cost=0.)])
    backend=FakePlanner(blocked=('preferred','approach') if blocked else None)
    result=plan_pick_carry(arm,backend,arm.anchor,[bad,good],scene_id='scene-1')
    assert result.candidate_id==('turned' if blocked else 'preferred')
    assert not result.carry_quality['grasp_preference']['hard_angle_filter']


@pytest.mark.parametrize('blocked',[False,True])
def test_tabletop_policy_preserves_moderate_fallback_but_excludes_extreme_tilt(arm,monkeypatch,blocked):
    from types import SimpleNamespace
    from scipy.spatial.transform import Rotation
    from home_robot_tasks import carry_planning
    from home_robot_tasks.grasp_preferences import TabletopGraspPolicy
    original=candidate(arm,'easy')
    def tilted(name,angle):
        pose=original.base_from_grasp_tcp.copy()
        pose[:3,:3]=np.diag([1.,-1.,-1.])@Rotation.from_euler('x',angle,degrees=True).as_matrix()
        return replace(original,candidate_id=name,base_from_grasp_tcp=pose)
    monkeypatch.setattr(carry_planning,'at_actual_contact',lambda c,p:c)
    monkeypatch.setattr(carry_planning,'carry_goals',lambda arm,q,*a,**k:[SimpleNamespace(model_rad=q,cost=0.)])
    backend=FakePlanner(blocked=('easy','approach') if blocked else None)
    result=plan_pick_carry(arm,backend,arm.anchor,[tilted('extreme',65),tilted('moderate',42),tilted('easy',15)],
        scene_id='scene-1',grasp_policy=TabletopGraspPolicy())
    assert result.candidate_id==('moderate' if blocked else 'easy')
    assert any(f['candidate']=='extreme' and f['stage']=='grasp_posture' for f in result.failures)
    assert result.carry_quality['grasp_preference']['hard_angle_filter']


def test_actual_contact_cannot_bypass_posture_filter(arm,monkeypatch):
    from home_robot_tasks import carry_planning
    from home_robot_tasks.grasp_preferences import TabletopGraspPolicy
    good=candidate(arm,'good');pose=good.base_from_grasp_tcp.copy();pose[:3,:3]=np.diag([1.,-1.,-1.])
    good=replace(good,base_from_grasp_tcp=pose)
    bad=pose.copy();bad[:3,:3]=np.eye(3)
    monkeypatch.setattr(carry_planning,'at_actual_contact',lambda c,p:replace(c,base_from_grasp_tcp=bad))
    result=plan_pick_carry(arm,FakePlanner(),arm.anchor,[good],scene_id='scene-1',grasp_policy=TabletopGraspPolicy())
    assert result.status=='NO_COMPLETE_ROUTE'
    assert any('actual contact exceeds' in f['reason'] for f in result.failures)


def test_tabletop_policy_rejects_end_grasp_and_prefers_centered_contact(arm,monkeypatch):
    from types import SimpleNamespace
    from home_robot_tasks import carry_planning
    from home_robot_tasks.grasp_preferences import TabletopGraspPolicy
    original=candidate(arm,'center');pose=original.base_from_grasp_tcp.copy();pose[:3,:3]=np.diag([1.,-1.,-1.])
    def shifted(name,offset):
        relative=np.eye(4);relative[1,3]=offset
        return replace(original,candidate_id=name,base_from_grasp_tcp=pose,tcp_from_object=relative)
    monkeypatch.setattr(carry_planning,'at_actual_contact',lambda c,p:c)
    monkeypatch.setattr(carry_planning,'carry_goals',lambda arm,q,*a,**k:[SimpleNamespace(model_rad=q,cost=0.)])
    backend=FakePlanner()
    result=plan_pick_carry(arm,backend,arm.anchor,[shifted('end',.03),shifted('near',.004),shifted('center',0.)],
        scene_id='scene-1',grasp_policy=TabletopGraspPolicy())
    assert result.candidate_id=='center'
    assert any(f['candidate']=='end' and f['stage']=='grasp_balance' for f in result.failures)
    assert not any(call[0]=='end' for call in backend.calls)


def test_bounded_budget_keeps_other_wrist_side_despite_preferred_duplicates(arm,monkeypatch):
    from types import SimpleNamespace
    from home_robot_tasks import carry_planning
    good=[candidate(arm,f'preferred-{i}') for i in range(5)]
    q=arm.anchor.copy();q[4]-=np.deg2rad(150)
    other=replace(good[0],candidate_id='other-side',pregrasp_model_rad=q)
    monkeypatch.setattr(carry_planning,'at_actual_contact',lambda c,p:c)
    monkeypatch.setattr(carry_planning,'carry_goals',lambda arm,q,*a,**k:[SimpleNamespace(model_rad=q,cost=0.)])
    class BlockPreferred(FakePlanner):
        def open(self,q,c):
            if c.candidate_id.startswith('preferred'):raise PlanRejected('fixture collision')
            return super().open(q,c)
    result=plan_pick_carry(arm,BlockPreferred(),arm.anchor,good+[other],scene_id='scene-1',maximum_grasps=3)
    assert result.candidate_id=='other-side' and len(result.failures)==2


@pytest.mark.parametrize('field,value',[('capture_id','old'),('calibration_id','old'),('source_sha256',{})])
def test_payload_from_other_capture_never_binds(arm,field,value):
    from home_robot_tasks.carry_binding import bind_grasp
    review,group,payload,installed=bound_input(arm)
    with pytest.raises(PlanRejected,match='share the capture'):
        bind_grasp(arm,review,group,0,replace(payload,**{field:value}),installed)


def test_hypothetical_lift_requires_explicit_override_preserving_capture(arm):
    from home_robot_tasks.carry_binding import bind_grasp
    review,group,payload,installed=bound_input(arm)
    review['lift_height_m']=.1
    with pytest.raises(PlanRejected,match='fixed lift'):
        bind_grasp(arm,review,group,0,payload,installed)
    _,preview=bind_grasp(arm,review,group,0,payload,installed,hypothetical_lift_m=0.,approach_distance_m=.006)
    assert review['lift_height_m']==.1 and preview['hypothetical_lift_m']==0.
    assert not preview['motion_authorized']


def test_bounded_cartesian_uses_small_same_branch_samples(arm):
    from home_robot_tasks.carry_cartesian import bounded_cartesian
    q=arm.anchor+np.deg2rad([0,30,-40,-30,0]);origin=arm.fk(q)[0];goal=origin.copy()
    goal[:3,3]+=.006*goal[:3,2]
    path=bounded_cartesian(arm,q,goal)
    assert np.max(abs(np.diff(path,axis=0)))<=np.deg2rad(1)+1e-10
    assert np.linalg.norm(arm.fk(path[-1])[0][:3,3]-goal[:3,3])<.001
    np.testing.assert_array_equal(path[0],q)


@pytest.mark.parametrize('change',['large_stroke','orientation'])
def test_bounded_cartesian_rejects_unspecified_relaxation(arm,change):
    from home_robot_tasks.carry_cartesian import bounded_cartesian
    from scipy.spatial.transform import Rotation
    q=arm.anchor;goal=arm.fk(q)[0].copy();goal[2,3]+=.005
    if change=='large_stroke':goal[2,3]+=.2
    else:goal[:3,:3]=goal[:3,:3]@Rotation.from_euler('x',10,degrees=True).as_matrix()
    with pytest.raises(PlanRejected):bounded_cartesian(arm,q,goal)


def test_attachment_preserves_observed_world_pose_despite_ik_residual(arm):
    from home_robot_tasks.carry_planning import at_actual_contact
    from scipy.spatial.transform import Rotation
    c=candidate(arm);before=c.base_from_grasp_tcp@c.tcp_from_object
    actual=c.base_from_grasp_tcp.copy();actual[2,3]+=.0005
    actual[:3,:3]=actual[:3,:3]@Rotation.from_euler('x',.6,degrees=True).as_matrix()
    resolved=at_actual_contact(c,actual)
    np.testing.assert_allclose(actual@resolved.tcp_from_object,before,atol=1e-12)
    np.testing.assert_allclose(c.base_from_grasp_tcp@c.tcp_from_object,before,atol=1e-12)
