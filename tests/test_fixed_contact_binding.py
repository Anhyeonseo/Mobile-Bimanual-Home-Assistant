import itertools
from types import SimpleNamespace as NS
import numpy as np
import pytest
from tools.diagnostics.run_s3_stationary_lab import stock_jaw
from home_robot_tasks.carry_binding import bind_observed_fixed_contact
from home_robot_tasks.carry_planning import GraspRouteCandidate, fixed_contact_review


def box(low,high):
    return np.asarray(list(itertools.product(*zip(low,high))))


def test_asymmetric_fixed_contact_is_independent_of_open_gap_and_world_rotation():
    from scipy.spatial.transform import Rotation
    jaw=stock_jaw();arm=NS(jaw=jaw,tcp_from_stock=np.eye(4))
    pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('xyz',[.2,.1,.7]).as_matrix();pose[:3,3]=[.4,.2,.3]
    # Fixed face at 0, object occupies -X from -6 to +50 mm: 6 mm penetration.
    local=box([-.05,-.01,-.01],[.006,.01,.01])
    vertices=local@pose[:3,:3].T+pose[:3,3]
    section=jaw.contact_section(pose,vertices)
    assert section['fixed_intrusion_m']==pytest.approx(.006,abs=1e-8)
    corrected,source,_=bind_observed_fixed_contact(arm,pose,vertices)
    assert source['width_m']==pytest.approx(.056)
    assert jaw.contact_section(corrected,vertices)['fixed_intrusion_m']==pytest.approx(0,abs=1e-9)
    corrected_wide,_,_=bind_observed_fixed_contact(arm,pose,vertices,entry_clearance_m=.006)
    np.testing.assert_allclose(corrected,corrected_wide)


def test_missing_section_or_overwide_directed_section_is_not_a_grasp():
    jaw=stock_jaw();arm=NS(jaw=jaw,tcp_from_stock=np.eye(4))
    with pytest.raises(ValueError,match='misses'):
        jaw.contact_section(np.eye(4),box([-.04,.02,-.01],[0,.04,.01]))
    with pytest.raises(ValueError,match='exceed'):
        bind_observed_fixed_contact(arm,np.eye(4),box([-.07,-.01,-.01],[0,.01,.01]))


def test_native_contact_review_detects_fixed_intrusion_and_wrong_moving_contact():
    from dataclasses import replace
    jaw=stock_jaw();arm=NS(jaw=jaw,tcp_from_stock=np.eye(4))
    pose=np.eye(4);obj=np.eye(4);obj[0,3]=-.025
    c=GraspRouteCandidate('shape','cal','scene',np.zeros(5),pose,obj,(.05,.02,.02),
        float(np.interp(.054,jaw.gaps,jaw.angles)),float(np.interp(.05,jaw.gaps,jaw.angles)))
    assert fixed_contact_review(arm,c)['accepted']
    shifted=obj.copy();shifted[0,3]+=.006
    assert not fixed_contact_review(arm,replace(c,tcp_from_object=shifted))['accepted']
    assert not fixed_contact_review(arm,replace(c,held_jaw_model_rad=float(np.interp(.04,jaw.gaps,jaw.angles))))['accepted']
