"""Independent geometry/load checks; no measured payload capacity is implied."""
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial import ConvexHull
from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.stationary_pick_sim import StationaryArm
from home_robot_tasks.carry_load import CarriedGeometry,payload_load,inflated_payload_mesh,grasp_balance

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def arm():
    c=ROOT/'config/home_assistant';read=lambda n:json.loads((c/n).read_text())
    return StationaryArm(read('s2_coordinates.development.json'),geo.load_geometry(c/'official_geometry.json'),
        read('arm_ranges.development.json'),.3,None)

def test_payload_gravity_torque_matches_potential_energy_derivative(arm):
    relative=np.eye(4);relative[:3,3]=[-.025,.008,.012]
    payload=CarriedGeometry(relative,(.14,.04,.02),com_object_m=(.018,-.002,.003))
    q=np.deg2rad([10,-60,30,65,80]);epsilon=1e-6
    def energy(value):
        p=arm.fk(value)[0]@relative
        center=p[:3,3]+p[:3,:3]@payload.com_object_m
        return 9.80665*center[2]
    numeric=[]
    for i in range(5):
        step=np.eye(5)[i]*epsilon
        numeric.append(abs((energy(q+step)-energy(q-step))/(2*epsilon)))
    quality=payload_load(arm,q,payload)
    np.testing.assert_allclose(quality['payload_gravity_torque_per_kg_nm'],numeric,atol=1e-8)
    assert quality['com_basis']=='supplied'
    assert quality['includes_arm_weight_or_dynamics'] is False

def test_default_center_is_explicit_proxy_and_offset_changes_load(arm):
    p=CarriedGeometry(np.eye(4),(.2,.04,.02))
    a=payload_load(arm,arm.anchor,p)
    b=payload_load(arm,arm.anchor,replace(p,com_object_m=(.07,0,0)))
    assert a['com_basis']=='geometric_center_proxy' and not a['physical_load_validated']
    assert not np.allclose(a['payload_gravity_torque_per_kg_nm'],b['payload_gravity_torque_per_kg_nm'])


def test_balance_uses_contact_line_and_gravity_moment_not_asymmetric_tcp():
    relative=np.eye(4);relative[:3,3]=[.025,.05,0.]
    payload=CarriedGeometry(relative,(.04,.16,.02))
    quality=grasp_balance(np.eye(4),payload)
    assert quality['center_to_closing_line_m']==pytest.approx(.05)
    assert quality['pinch_axis_gravity_torque_per_kg_nm']==pytest.approx(.05*9.80665)
    assert not quality['physical_balance_validated']
    # Move the TCP reference from fixed pad towards the midpoint along X.
    moved=np.eye(4);moved[0,3]=.02
    other=grasp_balance(moved,replace(payload,tcp_from_object=np.linalg.inv(moved)@relative))
    for key in ('center_to_closing_line_m','normalized_offset','pinch_axis_gravity_torque_per_kg_nm'):
        assert other[key]==pytest.approx(quality[key])
    np.testing.assert_allclose(other['center_base_m'],quality['center_base_m'])
    centered=grasp_balance(np.eye(4),replace(payload,com_object_m=(0.,-.05,0.)))
    assert centered['center_to_closing_line_m']==pytest.approx(0.) and centered['com_basis']=='supplied'

def test_convex_probe_contains_spherical_expansion_and_preserves_input():
    p=CarriedGeometry(np.eye(4),(.10,.04,.02));original=p.vertices.copy()
    probe=inflated_payload_mesh(p,.01);v=np.array(probe['vertices'])
    np.testing.assert_allclose(np.ptp(v,axis=0),(.12,.06,.04),atol=1e-10)
    hull=ConvexHull(v)
    directions=np.random.default_rng(42).normal(size=(100,3));directions/=np.linalg.norm(directions,axis=1)[:,None]
    expanded=(original[:,None,:]+.01*directions).reshape(-1,3)
    assert np.max(expanded@hull.equations[:,:3].T+hull.equations[:,3])<1e-10
    np.testing.assert_array_equal(original,p.vertices)

@pytest.mark.parametrize('margin',[0,-.01,.06,float('nan')])
def test_probe_margin_is_bounded(margin):
    with pytest.raises(ValueError):inflated_payload_mesh(CarriedGeometry(np.eye(4),(.1,.1,.1)),margin)
