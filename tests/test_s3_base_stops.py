import math

import numpy as np
import pytest

from home_robot_tasks.assistant.contracts import Context, Stamp
from home_robot_tasks.assistant.placement import Check, PlacementPlanner, PlacementProfile, PICK_LIFT_PHASES
from home_robot_tasks.stationary_pick_sim import Rejected
from tools.diagnostics import run_s3_base_stops as stops


def test_world_geometry_is_reobserved_in_the_new_base_pose():
    world = stops.world_from_base((.7,-.3,math.pi/2),-.07)
    local = np.array([[.01,0.,1.],[.03,0.,1.],[-.01,0.,1.]])
    patches = stops.points_in(world,local)
    intr = dict(width=40,height=40,ppx=20,ppy=20,fx=100,fy=100)
    reason, observed, margin = stops.capture_patches(world,intr,[patches],[])
    assert reason == 'VISIBLE' and margin > 0
    # These locations lie at exact pixel centres.
    np.testing.assert_allclose(observed[0].mean(axis=0),patches.mean(axis=0),atol=1e-12)
    other_stop = stops.world_from_base((1.7,-.3,math.pi/2),-.07)
    assert stops.capture_patches(other_stop,intr,[patches],[])[0] == 'PATCH_OUTSIDE_FRAME'
    lo = world[:3,3] + [-.1,-.1,.2]
    assert stops.capture_patches(world,intr,[patches],[(lo,lo+[.2,.2,.1])])[0] == 'PATCH_OCCLUDED'
    assert stops.capture_patches(world,intr,[patches,patches],[])[0] == 'PATCHES_OVERLAP_IN_IMAGE'


def test_rotated_scene_is_checked_in_its_own_frame():
    arm = stops.lab.model(.61)
    world = stops.world_from_base((2.,1.,math.pi/2), -.0702)
    point = stops.points_in(world, arm.fk(arm.anchor)[1][3])
    obstacle = [(point-.005,point+.005)]
    arm.segment(arm.anchor,arm.anchor,obstacle,False,.03,.02)
    with pytest.raises(Rejected,match='ARM_SCENE_INTERSECTION'):
        arm.segment(arm.anchor,arm.anchor,obstacle,False,.03,.02,scene_from_base=world)


@pytest.mark.parametrize('phases,expected', [(PICK_LIFT_PHASES,'CANDIDATES_READY'),
                                             (PICK_LIFT_PHASES-{'hold'},'UNREACHABLE')])
def test_stop_requires_complete_lift_but_does_not_ask_for_transport(phases, expected):
    class Oracle:
        def navigation(self,c): return Check(True,'route',mode='simulation')
        def view(self,c): return Check(True,'view',mode='simulation')
        def whole_motion(self,c): return Check(True,'path',phases=tuple(phases),mode='simulation')
    ctx = Context('map','revision','cal','clock')
    profile = PlacementProfile(0,.64,(.61,),(.30,),(math.pi,))
    r = PlacementPlanner(profile,Oracle()).plan(target_m=[0.,0.,.91],stamp=Stamp(1.,ctx,'fixture'),
        context=ctx,now_s=1.,scene_revision='scene',grasp_ids=['flat'],current_xy_yaw=(-.75,0.,0.),
        current_height_m=.61,operation='pick_lift')
    assert r['status'] == expected and r['evaluated'] == 1
    assert not r['motion_authorized'] and r['requires_reobservation']


def test_inset_target_has_no_base_clearance_before_ik():
    r = stops.run_layout(stops.LAYOUTS[1])
    assert r['lift_setting']['target_travel_from_bottom_m'] == .61
    assert r['evaluated'] == 210 and r['distinct_stops'] == 105
    assert r['reachable_stops'] == r['motion_evaluations'] == 0
    assert r['best'] is None and all(not s['arms'] for s in r['stops'])


def test_sampled_reach_uses_same_fk_as_individual_arm():
    arm = stops.lab.model(.61,'right')
    q = np.stack([arm.anchor, (arm.lo+arm.hi)/2, arm.lo, arm.hi])
    expected = np.stack([arm.fk(v)[0][:3,3] for v in q])
    np.testing.assert_allclose(stops.sampled_tcp_positions(arm,q), expected, atol=1e-12)


def test_stop_search_cannot_add_another_lift_dimension():
    from home_robot_tasks.fetch import InvalidTask
    profile = PlacementProfile(0,.64,(.60,.61),(.30,),(math.pi,))
    ctx = Context('map','revision','cal','clock')
    with pytest.raises(InvalidTask,match='single_pick_lift_setting_required'):
        PlacementPlanner(profile,None).plan(target_m=[0,0,.8], stamp=Stamp(1.,ctx,'fixture'), context=ctx,
            now_s=1.,scene_revision='scene',grasp_ids=['flat'],current_xy_yaw=(-.75,0.,0.),
            current_height_m=.61,operation='pick_lift')


def test_bounded_fk_distinguishes_proof_witness_and_search_budget():
    arm = stops.lab.model(.61)
    bound = stops.forward_bound(arm,.60)
    assert bound['status'] == 'ALL_TCP_X_BELOW_LIMIT' and not bound['physical_reach_verified']
    assert bound['frame'] == 'base_link' and bound['arm_side'] == 'left'
    np.testing.assert_allclose(bound['joint_intervals_deg']['wrist_flex'], np.degrees([arm.lo[3], arm.hi[3]]))
    assert stops.forward_bound(arm,.25)['status'] == 'FK_WITNESS'
    assert stops.forward_bound(arm,.60,max_cells=1)['status'] == 'INCONCLUSIVE'
