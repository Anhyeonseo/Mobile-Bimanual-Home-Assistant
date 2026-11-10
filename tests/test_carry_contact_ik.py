"""Five-DOF adaptation keeps the observed pinch line; it cannot certify a path."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from home_robot_tasks.carry_contact_ik import fit_pregrasp
from tools.diagnostics.run_s3_stationary_lab import model
from home_robot_tasks.grasp_preferences import TabletopGraspPolicy, contact_posture, contact_roll_interval


def tabletop_pose(roll, closing=0.):
    pose=np.eye(4)
    pose[:3,:3]=Rotation.from_euler('y',closing,degrees=True).as_matrix()@np.diag([1.,-1.,-1.])@Rotation.from_euler('x',roll,degrees=True).as_matrix()
    return pose


def test_overhead_entry_preserves_reachable_tilt_and_contact_xy():
    arm=model(.5)
    q=arm.anchor+np.deg2rad([0,20,-30,-25,10])
    pre=arm.fk(q)[0];grip=pre.copy();grip[2,3]-=.02
    ok,solution,fitted,evidence=fit_pregrasp(arm,grip,q,.02,approach_mode='vertical')
    assert ok
    np.testing.assert_allclose(fitted,grip,atol=1e-10)
    np.testing.assert_allclose(arm.fk(solution)[0],pre,atol=1e-8)
    np.testing.assert_allclose(np.asarray(evidence['desired_base_from_tcp'])[:3,3]-grip[:3,3],[0,0,.02])
    assert evidence['approach_mode']=='vertical'
    with pytest.raises(ValueError,match='lateral alignment'):
        fit_pregrasp(arm,grip,q,.02,approach_mode='vertical',fixed_finger_clearance_m=.006)


def test_vertical_route_checks_interior_samples_not_just_endpoints():
    from home_robot_tasks.carry_cartesian import validate_vertical_descent
    poses=[np.eye(4) for _ in range(3)]
    for p,z in zip(poses,(.02,.01,0)):p[2,3]=z
    assert validate_vertical_descent(poses)['drop_m']==pytest.approx(.02)
    poses[1][0,3]=.004
    with pytest.raises(ValueError,match='lateral drift'):validate_vertical_descent(poses)
    poses[1][0,3]=0;poses[1][2,3]=.021
    with pytest.raises(ValueError,match='rises'):validate_vertical_descent(poses)


def test_moderate_tilt_is_softly_discouraged_not_rejected():
    policy=TabletopGraspPolicy()
    easy=contact_posture(tabletop_pose(15),policy)
    moderate=contact_posture(tabletop_pose(42,12),policy)
    assert easy['accepted'] and moderate['accepted']
    assert easy['cost']<moderate['cost']
    assert not contact_posture(tabletop_pose(58,12),policy)['accepted']
    assert not contact_posture(tabletop_pose(0,25),policy)['accepted']


@pytest.mark.parametrize('roll,closing',[(0,0),(85,0),(-85,0),(40,12),(0,25)])
def test_roll_search_bounds_match_physical_tabletop_envelope(roll,closing):
    policy=TabletopGraspPolicy();source=tabletop_pose(roll,closing)
    interval=contact_roll_interval(source,policy)
    for angle in np.linspace(-np.pi/3,np.pi/3,241):
        pose=source.copy();pose[:3,:3]=source[:3,:3]@Rotation.from_rotvec([angle,0,0]).as_matrix()
        expected=contact_posture(pose,policy)['accepted']
        inside=interval is not None and interval[0]-1e-10<=angle<=interval[1]+1e-10
        assert inside==expected
        np.testing.assert_allclose(pose[:3,0],source[:3,0],atol=1e-12)


@pytest.mark.parametrize('values',[dict(maximum_approach_deg=float('nan')),dict(maximum_closing_deg=5),dict(preferred_approach_deg=0)])
def test_invalid_posture_policy_rejected(values):
    with pytest.raises(ValueError):TabletopGraspPolicy(**values)


def test_posture_policy_searches_moderate_roll_instead_of_accepting_tilted_exact_ik():
    from types import SimpleNamespace
    def fk(q):
        pose=tabletop_pose(np.rad2deg(q[0]));pose[:3,3]=q[1:4]
        return pose,None
    source=tabletop_pose(65)
    seed=np.zeros(5);seed[0]=np.deg2rad(65);seed[1:4]=-.02*source[:3,2]
    arm=SimpleNamespace(lo=np.full(5,-2.),hi=np.full(5,2.),anchor=seed.copy(),fk=fk)
    policy=TabletopGraspPolicy()
    ok,q,pose,evidence=fit_pregrasp(arm,source,seed,.02,contact_roll=True,grasp_policy=policy)
    assert ok and evidence['contact_roll_search']
    assert contact_posture(pose,policy)['accepted']
    assert contact_posture(fk(q)[0],policy)['accepted']
    np.testing.assert_allclose(pose[:3,3],source[:3,3],atol=1e-12)
    np.testing.assert_allclose(pose[:3,0],source[:3,0],atol=1e-12)


@pytest.mark.parametrize('roll',[-25,25])
def test_recover_reachable_pregrasp_by_only_changing_contact_roll(roll):
    arm=model(.5)
    q=arm.anchor+np.deg2rad([0,20,-30,-25,10])
    pre=arm.fk(q)[0];grip=pre.copy();grip[:3,3]+=.02*pre[:3,2]
    grip[:3,:3]=grip[:3,:3]@Rotation.from_euler('x',roll,degrees=True).as_matrix()
    exact=fit_pregrasp(arm,grip,q,.02,contact_roll=False)
    assert not exact[0]
    ok,solution,fitted,evidence=fit_pregrasp(arm,grip,q,.02,contact_roll=True)
    assert ok and evidence['contact_roll_search']
    np.testing.assert_allclose(fitted[:3,3],grip[:3,3],atol=1e-12)
    np.testing.assert_allclose(fitted[:3,0],grip[:3,0],atol=1e-12)
    assert abs(evidence['contact_roll_deg'])<=60
    assert evidence['position_error_m']<=.001 and evidence['orientation_error_deg']<=1
    assert np.all(solution>=arm.lo) and np.all(solution<=arm.hi)
    assert not evidence['full_path_collision_checked'] and not evidence['motion_authorized']


def test_adaptation_cannot_make_far_target_reachable_or_change_its_position():
    arm=model(.5);grip=arm.fk(arm.anchor)[0];grip[0,3]+=2.
    ok,_,fitted,_=fit_pregrasp(arm,grip,arm.anchor,.02,contact_roll=True)
    assert not ok
    np.testing.assert_allclose(fitted[:3,3],grip[:3,3])


def test_fixed_side_entry_clearance_preserves_final_contact_and_closing_axis():
    arm=model(.5)
    q=arm.anchor+np.deg2rad([0,20,-30,-25,10])
    pre=arm.fk(q)[0];grip=pre.copy()
    grip[:3,3]+=.02*pre[:3,2]-.006*pre[:3,0]
    ok,solution,fitted,evidence=fit_pregrasp(arm,grip,q,.02,
        contact_roll=True,fixed_finger_clearance_m=.006)
    assert ok
    np.testing.assert_allclose(fitted,grip,atol=1e-9)
    np.testing.assert_allclose(arm.fk(solution)[0],pre,atol=1e-8)
    entry=np.asarray(evidence['desired_base_from_tcp'])
    delta=fitted[:3,:3].T@(entry[:3,3]-fitted[:3,3])
    np.testing.assert_allclose(delta,[.006,0,-.02],atol=1e-10)
    assert evidence['pinch_center_displacement_m']==0


@pytest.mark.parametrize('clearance',[-.001,.011,float('nan'),float('inf')])
def test_invalid_fixed_side_entry_clearance_rejected(clearance):
    arm=model(.5)
    with pytest.raises(ValueError,match='clearance'):
        fit_pregrasp(arm,arm.fk(arm.anchor)[0],arm.anchor,.02,
                     fixed_finger_clearance_m=clearance)


def test_valid_seed_not_hidden_by_smaller_total_error_with_failed_orientation(monkeypatch):
    from types import SimpleNamespace
    from home_robot_tasks import carry_contact_ik
    def fk(q):
        position,angle=(.0009,.9) if q[0]==2 else (.0002,1.01)
        pose=np.eye(4);pose[0,3]=position;pose[2,3]=-.02
        pose[:3,:3]=Rotation.from_euler('x',angle,degrees=True).as_matrix()
        return pose,None
    arm=SimpleNamespace(lo=np.full(5,-3.),hi=np.full(5,3.),anchor=np.zeros(5),fk=fk)
    values=iter([1.,2.,1.,1.,1.])
    monkeypatch.setattr(carry_contact_ik,'least_squares',lambda *a,**k:
        SimpleNamespace(success=True,x=np.array([next(values),0.,0.,0.,0.])))
    ok,q,_,diagnostics=fit_pregrasp(arm,np.eye(4),np.zeros(5),.02)
    assert ok and q[0]==2 and diagnostics['orientation_error_deg']<1.
