"""Regression from a saved physical capture. No device or MoveIt access."""
import json
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.stationary_pick_sim import StationaryArm
from home_robot_tasks.so101_jaw import load_stock_jaw
from home_robot_tasks.carry_cartesian import bounded_cartesian
from home_robot_tasks.carry_planning import GraspRouteCandidate, PlanRejected, at_actual_contact, fixed_contact_review
from home_robot_tasks.grasp_preferences import TabletopGraspPolicy,contact_posture
from home_robot_tasks.carry_load import CarriedGeometry,grasp_balance

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def observed():
    data=json.loads((ROOT/'tests/fixtures/pick_vertical_ik_20261119.json').read_text())
    config=ROOT/'config/home_assistant'
    read=lambda name:json.loads((config/name).read_text())
    jaw=load_stock_jaw(config/'official_sources/so101_new_calib.urdf',ROOT/'ros2_ws/src/so101_description/meshes')
    arm=StationaryArm(read('s2_coordinates.wide.development.json'),geo.load_geometry(config/'official_geometry.json'),
        read('arm_ranges.development.json'),data['lift_m'],jaw)
    spec=data['candidate']
    for key in ('pregrasp_model_rad','base_from_grasp_tcp','tcp_from_object'):spec[key]=np.asarray(spec[key])
    return arm,GraspRouteCandidate(**spec)


def test_previous_centered_capture_fails_attitude_not_position(observed):
    arm,c=observed
    with pytest.raises(PlanRejected) as exc:bounded_cartesian(arm,c.pregrasp_model_rad,c.base_from_grasp_tcp)
    d=exc.value.diagnostics
    assert d['violations']==['orientation'] and d['solver_success']
    assert d['position_error_mm']<.1 and d['minimum_joint_limit_margin_deg']>5
    assert 2<d['goal_orientation_deg']<3


def test_explicit_tabletop_tolerance_retains_contact_balance_and_finger_slope(observed):
    arm,c=observed
    path=bounded_cartesian(arm,c.pregrasp_model_rad,c.base_from_grasp_tcp,orientation_tolerance_deg=5)
    actual=at_actual_contact(c,arm.fk(path[-1])[0])
    assert fixed_contact_review(arm,actual)['accepted']
    policy=TabletopGraspPolicy()
    assert all(contact_posture(arm.fk(q)[0],policy)['accepted'] for q in path)
    balance=grasp_balance(actual.base_from_grasp_tcp,CarriedGeometry(actual.tcp_from_object,actual.object_size_m,actual.payload_mesh))
    assert balance['normalized_offset']<policy.preferred_balance_offset
    assert np.max(abs(np.diff(path,axis=0)))<=np.deg2rad(1)+1e-10
    assert np.linalg.norm(actual.base_from_grasp_tcp[:3,3]-c.base_from_grasp_tcp[:3,3])<.001
    assert 4<np.rad2deg(Rotation.from_matrix(c.base_from_grasp_tcp[:3,:3].T@actual.base_from_grasp_tcp[:3,:3]).magnitude())<5


def test_geometrically_unreachable_position_is_not_called_attitude_failure():
    # Valid start, but this fixture has no translational degree of freedom.
    arm=NS(lo=-np.ones(5),hi=np.ones(5),fk=lambda q:(np.eye(4),None))
    goal=np.eye(4);goal[0,3]=.01
    with pytest.raises(PlanRejected) as exc:bounded_cartesian(arm,np.zeros(5),goal,orientation_tolerance_deg=5)
    assert exc.value.diagnostics['violations']==['position']


@pytest.mark.parametrize('limit',[0,6,float('nan')])
def test_no_unbounded_attitude_option(observed,limit):
    arm,c=observed
    with pytest.raises(ValueError):bounded_cartesian(arm,c.pregrasp_model_rad,c.base_from_grasp_tcp,orientation_tolerance_deg=limit)


def test_singular_region_subdivides_cartesian_distance_without_loosening_joint_limit():
    def fk(q):
        pose=np.eye(4);pose[0,3]=.02*q[0]**2;return pose,None
    arm=NS(lo=np.zeros(5),hi=np.ones(5),fk=fk)
    start=np.full(5,.01);goal=fk(start)[0];goal[0,3]+=.01
    path=bounded_cartesian(arm,start,goal)
    assert np.linalg.norm(fk(path[-1])[0][:3,3]-goal[:3,3])<.001
    assert np.max(abs(np.diff(path,axis=0)))<=np.deg2rad(1)+1e-10
    assert len(path)<200


def test_tabletop_fallback_is_still_passed_through_moveit_collision_checks(observed,monkeypatch):
    import sys
    from home_robot_tasks.carry_moveit import MoveItCarryPlanner
    arm,c=observed
    monkeypatch.setitem(sys.modules,'moveit_msgs.srv',NS(GetCartesianPath=NS(Request=lambda:NS(header=NS()))))
    planner=MoveItCarryPlanner.__new__(MoveItCarryPlanner)
    planner.arm=arm;planner.group='left_arm';planner.tcp='left_tcp'
    planner.vertical_grasp_policy=TabletopGraspPolicy()
    planner.prepare=lambda *a:None;planner.state=lambda *a:NS();planner.pose=lambda x:x
    planner.call=lambda *a:NS(error_code=NS(val=-31),fraction=0.)
    checked=[];planner.valid_many=lambda samples,*a:checked.append(samples)
    path=planner.cartesian(c.pregrasp_model_rad,c.base_from_grasp_tcp,c,attached=False,label='approach')
    assert len(checked)==1 and np.array_equal(path.joints,checked[0])
    def collision(*args):raise PlanRejected('collision fixture')
    planner.valid_many=collision
    with pytest.raises(PlanRejected,match='collision fixture'):
        planner.cartesian(c.pregrasp_model_rad,c.base_from_grasp_tcp,c,attached=False,label='approach')
    planner.vertical_grasp_policy=None
    with pytest.raises(PlanRejected,match='orientation'):
        planner.cartesian(c.pregrasp_model_rad,c.base_from_grasp_tcp,c,attached=False,label='approach')
