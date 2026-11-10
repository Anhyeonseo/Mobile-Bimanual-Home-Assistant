"""Generic gripper heuristics cannot decide reachability of the installed hand."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from home_perception.geometry.plane import Plane
from home_perception.grasp.proposals import select
from home_perception.grasp.refine import refine


def example():
    plane = Plane(np.array([0., 0., -1.]), np.array([0., 0., .6]), .001, 1.)
    # An oblique grasp with observed contact above the table. The generic
    # 20 mm finger corners intersect; the real hand must decide feasibility.
    rotation = Rotation.from_euler('y', 40, degrees=True).as_matrix() @ np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
    row = np.zeros(17); row[:4] = [.8, .04, .02, .02]
    row[4:13] = rotation.ravel(); row[13:16] = np.array([0., 0., .597])-rotation[:,0]*.02
    points = np.array([row[13:16]+rotation@np.array([.015, x, y])
                       for x in np.linspace(-.012, .012, 12) for y in np.linspace(-.002, .002, 4)])
    return row, points, plane


def test_generic_corner_failure_is_not_an_actual_robot_collision():
    row, points, plane = example()
    legacy, reasons = refine(row, points, plane, .015)
    assert not len(legacy) and reasons['below_table'] == 1
    candidates, evidence = select([row], points, plane, (.0024, .0633),surface_heights=False,straight_across=False)
    assert len(candidates) == 1
    assert candidates[0]['robot_feasibility_checked'] is False
    assert evidence['physical_collision_model'] == 'deferred_to_SO101_CAD_and_full_path'


@pytest.mark.parametrize('change,reason', [
    ('span','observed_span_outside_robot_range'), ('below','contact_point_below_support'),
    ('rotation','invalid_pose'), ('nan','invalid_pose'), ('depth','invalid_pose'),
    ('no_target','target_section_unresolved')])
def test_only_evidence_valid_proposals_reach_robot_checker(change, reason):
    row, points, plane = example()
    if change == 'span':
        axis=row[4:13].reshape(3,3)[:,1]
        offset=(points-row[13:16])@axis
        points+=3*offset[:,None]*axis
    elif change == 'below': row[15] += .01
    elif change == 'rotation': row[4:13] = np.eye(3).ravel()*2
    elif change == 'nan': row[0] = np.nan
    elif change == 'depth': row[3] = .06
    elif change == 'no_target': points = points[:2]
    candidates, evidence = select([row], points, plane, (.0024, .0633),surface_heights=False,straight_across=False)
    assert not candidates and evidence['rejected_by_evidence'][reason] == 1


def test_camera_frame_rotation_and_translation_do_not_change_evidence_decision():
    row, points, plane = example()
    for angles in ([0,0,0], [10,-20,35], [-20,15,-45]):
        r = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
        t = np.array([.03,-.04,.2])
        moved = row.copy(); moved[4:13] = (r@row[4:13].reshape(3,3)).ravel()
        moved[13:16] = r@row[13:16]+t
        proposals, _ = select([moved], points@r.T+t,
            Plane(r@plane.normal,r@plane.point+t,plane.rms_m,1.), (.0024,.0633),surface_heights=False,straight_across=False)
        assert len(proposals) == 1


def test_nearby_orientation_variants_survive_before_five_dof_ik():
    row, points, plane = example()
    other = row.copy(); other[0] = .7
    r = row[4:13].reshape(3,3)
    other[4:13] = (r@Rotation.from_euler('y',15,degrees=True).as_matrix()).ravel()
    proposals, _ = select([row, row, other], points, plane, (.0024,.0633),surface_heights=False,straight_across=False)
    assert len(proposals)==2
    assert [p['proposal_row_index'] for p in proposals]==[0,2]


def test_network_hand_width_does_not_override_measured_so101_opening():
    row,points,plane=example();row[1]=.09
    candidates,_=select([row],points,plane,(.0024,.0633),surface_heights=False,straight_across=False)
    assert len(candidates)==1
    assert candidates[0]['network_width_m']==.09
    assert .02<candidates[0]['width_m']<.03


def test_float32_roundoff_normalized_but_invalid_rotation_still_rejected():
    from home_robot_tasks.s2_geometry import rigid
    row,points,plane=example();row[4]+=2e-7
    candidates,_=select([row],points,plane,(.0024,.0633))
    pose=np.eye(4);pose[:3,:3]=candidates[0]['rotation_matrix']
    rigid(pose)


def test_surface_height_uses_measured_points_and_rechecks_the_section():
    row,points,plane=example()
    points[:,2]=.585  # an independently known, flat 15 mm top surface
    candidates,evidence=select([row],points,plane,(.0024,.0633))
    moved=[c for c in candidates if c['proposal_variant']=='observed_surface_height']
    assert moved and evidence['surface_heights_sampled']
    candidate=moved[0]
    rotation=np.asarray(candidate['rotation_matrix'])
    tip=np.asarray(candidate['translation_m'])+candidate['depth_m']*rotation[:,0]
    original=row[13:16]+row[3]*row[4:13].reshape(3,3)[:,0]
    displacement=tip-original
    np.testing.assert_allclose(displacement,np.dot(displacement,plane.normal)*plane.normal,atol=1e-12)
    assert plane.height(tip)==pytest.approx(np.median(plane.height(points)))
    assert candidate['closing_section']['status']=='VISIBLE_SPAN'


def test_long_object_contact_is_bound_to_body_not_center_sphere():
    from itertools import product
    from home_robot_tasks.stationary_pick_sim import check_contact_target,Rejected
    center=np.array([.4,.1,.8])
    vertices=np.array(list(product((-.07,.07),(-.02,.02),(-.01,.01))))+center
    contact=center+[.05,0.,0.]
    with pytest.raises(Rejected,match='CANDIDATE_OFF_TARGET'):
        check_contact_target(contact,center.tolist())  # old point-only evidence
    check_contact_target(contact,center,vertices)
    with pytest.raises(Rejected,match='CANDIDATE_OFF_TARGET'):
        check_contact_target(center+[0.,.03,0.],center,vertices)
    r=Rotation.from_euler('xyz',[25,-40,30],degrees=True).as_matrix()
    t=np.array([.1,.2,.3])
    check_contact_target(r@contact+t,r@center+t,vertices@r.T+t)


def test_observed_closing_span_is_invariant_to_approach_roll():
    from home_perception.grasp.handoff import candidate_from_row,observed_contact_line
    row,points,_=example()
    r=row[4:13].reshape(3,3);tip=row[13:16]+row[3]*r[:,0]
    reference=observed_contact_line(candidate_from_row(row,'graspnet'),points)
    for angle in (-60,-30,30,60):
        rotation=r@Rotation.from_euler('y',angle,degrees=True).as_matrix()
        moved=row.copy();moved[4:13]=rotation.ravel();moved[13:16]=tip-row[3]*rotation[:,0]
        actual=observed_contact_line(candidate_from_row(moved,'graspnet'),points)
        assert actual['status']=='VISIBLE_SPAN'
        assert actual['low_m']==pytest.approx(reference['low_m'])
        assert actual['high_m']==pytest.approx(reference['high_m'])


def test_balanced_contact_survives_bounded_pool_despite_lower_network_score():
    plane=Plane(np.array([0.,0.,-1.]),np.array([0.,0.,.6]),.001,1.)
    r=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
    points=np.array([[x,y,.59] for x in np.linspace(-.02,.02,21) for y in np.linspace(-.08,.08,81)])
    rows=[]
    for score,y in ((.9,.06),(.7,0.)):
        row=np.zeros(17);row[:4]=[score,.04,.02,.02];row[4:13]=r.ravel()
        row[13:16]=np.array([0.,y,.59])-.02*r[:,0];rows.append(row)
    kept,evidence=select(rows,points,plane,(.0024,.0633),limit=1,surface_heights=False,straight_across=False)
    assert len(kept)==1 and kept[0]['proposal_row_index']==1
    assert kept[0]['balance_proxy']['transverse_offset_m']<1e-10
    assert not kept[0]['balance_proxy']['measured_com']
    # Densely sampling one end must not shift the footprint's area centroid.
    biased=np.vstack((points,np.repeat(points[points[:,1]>.06],8,axis=0)))
    again,_=select(rows,biased,plane,(.0024,.0633),limit=1,surface_heights=False,straight_across=False)
    assert again[0]['proposal_row_index']==1
    assert again[0]['balance_proxy']['transverse_offset_m']<1e-10


def test_balance_reference_is_rigid_frame_invariant():
    from home_perception.grasp.balance import footprint_reference,contact_centrality
    p=np.array([[x,y,0.]for x in (-.02,.02)for y in (-.08,.08)])
    normal=np.array([0.,0.,1.]);tip=np.array([0.,.05,.01]);axis=np.array([1.,0.,0.])
    expected=contact_centrality(tip,axis,footprint_reference(p,normal))
    r=Rotation.from_euler('xyz',[25,-18,40],degrees=True).as_matrix();t=np.array([.2,-.1,.7])
    actual=contact_centrality(r@tip+t,r@axis,footprint_reference(p@r.T+t,r@normal))
    assert actual['normalized_offset']==pytest.approx(expected['normalized_offset'])
    assert actual['transverse_offset_m']==pytest.approx(.05)


@pytest.mark.parametrize('angles',[[0,0,0],[20,-30,45]])
def test_straight_across_alternative_remeasures_width_and_keeps_network_pose(angles):
    from home_perception.grasp.handoff import observed_contact_line
    from home_perception.grasp.contact_orientation import footprint_short_axis
    from home_perception.grasp.balance import footprint_reference
    points=np.array([[x,y,.59] for x in np.linspace(-.02,.02,41) for y in np.linspace(-.08,.08,81)])
    plane=Plane(np.array([0.,0.,-1.]),np.array([0.,0.,.6]),.001,1.)
    rotation=Rotation.from_euler('z',25,degrees=True).as_matrix()@np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
    row=np.zeros(17);row[:4]=[.8,.06,.02,.01];row[4:13]=rotation.ravel()
    tip=np.array([0.,0.,.59]);row[13:16]=tip-.01*rotation[:,0]
    r=Rotation.from_euler('xyz',angles,degrees=True).as_matrix();t=np.array([.1,-.1,.3])
    points=points@r.T+t;row[4:13]=(r@rotation).ravel();row[13:16]=r@row[13:16]+t
    plane=Plane(r@plane.normal,r@plane.point+t,.001,1.)
    kept,_=select([row],points,plane,(.0024,.0633),surface_heights=False)
    assert {c['orientation_variant'] for c in kept}=={'network_orientation','observed_straight_across'}
    straight=next(c for c in kept if c['orientation_variant']=='observed_straight_across')
    actual=np.array(straight['rotation_matrix'])
    assert abs(actual[:,1]@(r@np.array([1.,0.,0.])))==pytest.approx(1.)
    np.testing.assert_allclose(np.array(straight['translation_m'])+.01*actual[:,0],r@tip+t,atol=1e-12)
    assert straight['closing_section']==observed_contact_line(straight,points)
    assert straight['closing_section']['high_m']-straight['closing_section']['low_m']==pytest.approx(.04,abs=.002)
    reference=footprint_reference(points,plane.normal)
    biased=np.vstack([points,np.repeat(points[:100],10,axis=0)])
    assert abs(footprint_short_axis(reference)@footprint_short_axis(footprint_reference(biased,plane.normal)))==pytest.approx(1.)


def test_round_footprint_does_not_invent_a_preferred_grasp_axis():
    from home_perception.grasp.balance import footprint_reference
    from home_perception.grasp.contact_orientation import footprint_short_axis
    theta=np.linspace(0,2*np.pi,100,endpoint=False)
    points=np.c_[.02*np.cos(theta),.02*np.sin(theta),np.zeros(100)]
    assert footprint_short_axis(footprint_reference(points,np.array([0.,0.,1.]))) is None


@pytest.mark.parametrize('angles',[[0,0,0],[20,-30,45]])
def test_center_variant_remeasures_width_preserves_height_and_network_alternative(angles):
    points=np.array([[x,y,.59] for y in np.linspace(-.08,.08,81)
                     for x in np.linspace(-(.02+.1*y),.02+.1*y,41)])
    plane=Plane(np.array([0.,0.,-1.]),np.array([0.,0.,.6]),.001,1.)
    rotation=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
    row=np.zeros(17);row[:4]=[.8,.06,.02,.01];row[4:13]=rotation.ravel()
    tip=np.array([0.,.06,.59]);row[13:16]=tip-.01*rotation[:,0]
    r=Rotation.from_euler('xyz',angles,degrees=True).as_matrix();t=np.array([.1,-.1,.3])
    points=points@r.T+t;row[4:13]=(r@rotation).ravel();row[13:16]=r@row[13:16]+t
    plane=Plane(r@plane.normal,r@plane.point+t,.001,1.)
    kept,_=select([row],points,plane,(.0024,.0633),surface_heights=False,straight_across=False,centered_contacts=True)
    centered=next(c for c in kept if c['contact_position_variant']=='observed_longitudinal_center')
    original=next(c for c in kept if c['contact_position_variant']=='network_contact_position')
    assert centered['balance_proxy']['transverse_offset_m']<1e-10
    center_tip=np.array(centered['translation_m'])+.01*np.array(centered['rotation_matrix'])[:,0]
    assert plane.height(center_tip)==pytest.approx(plane.height(r@tip+t))
    assert centered['width_m']<original['width_m']-.005


def test_empty_center_is_not_fabricated_as_a_valid_grasp():
    points=np.array([[x,y,.59] for x in np.linspace(-.02,.02,21)
                     for y in np.r_[np.linspace(-.08,-.04,21),np.linspace(.04,.08,21)]])
    plane=Plane(np.array([0.,0.,-1.]),np.array([0.,0.,.6]),.001,1.)
    rotation=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
    row=np.zeros(17);row[:4]=[.8,.04,.02,.01];row[4:13]=rotation.ravel()
    row[13:16]=np.array([0.,.06,.59])-.01*rotation[:,0]
    kept,report=select([row],points,plane,(.0024,.0633),surface_heights=False,straight_across=False,centered_contacts=True)
    assert kept and all(c['contact_position_variant']=='network_contact_position' for c in kept)
    assert report['rejected_by_evidence']['target_section_unresolved']==1
