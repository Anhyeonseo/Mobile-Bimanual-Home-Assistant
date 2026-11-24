"""Two-sided entry room with a small, explicitly planned closing displacement."""
from dataclasses import replace
from types import SimpleNamespace as NS
import itertools
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
from tools.diagnostics.run_s3_stationary_lab import stock_jaw
from home_robot_tasks.carry_planning import GraspRouteCandidate,fixed_contact_review,closing_sweep_candidate,at_actual_contact

@pytest.mark.parametrize('yaw',[90,-90])
def test_fixed_six_moving_four_survive_finger_side_swap(yaw):
    jaw=stock_jaw();arm=NS(jaw=jaw,tcp_from_stock=np.eye(4));pose=np.eye(4)
    pose[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix();pose[:3,3]=[.4,.2,.7]
    held=np.eye(4);held[0,3]=-.0178/2
    observed=held.copy();observed[0,3]-=.006;observed=pose@observed
    c=GraspRouteCandidate('test','cal','scene',np.zeros(5),pose,held,(.0178,.04,.02),
        float(np.interp(.0278,jaw.gaps,jaw.angles)),float(np.interp(.0178,jaw.gaps,jaw.angles)),
        observed_base_from_object=observed,fixed_side_clearance_m=.006)
    review=fixed_contact_review(arm,c)
    assert review['accepted']
    assert review['entry_fixed_clearance_m']==pytest.approx(.006,abs=1e-8)
    assert review['open_moving_clearance_m']==pytest.approx(.004,abs=1e-8)
    assert np.linalg.norm(review['planned_object_translation_base_m'])==pytest.approx(.006)
    # IK residual must not move either the original observation or expected held pose.
    actual=pose.copy();actual[0,3]+=.0001;r=at_actual_contact(c,actual)
    np.testing.assert_allclose(r.observed_base_from_object,observed)
    np.testing.assert_allclose(r.base_from_grasp_tcp@r.tcp_from_object,pose@held)
    sweep=closing_sweep_candidate(c);hull=ConvexHull(sweep.payload_mesh['vertices'])
    local=np.asarray(list(itertools.product(*[(-s/2,s/2) for s in c.object_size_m])))
    for f in np.linspace(0,1,13):
        v=local+[-.006*f,0,0]
        assert np.max(v@hull.equations[:,:3].T+hull.equations[:,3])<1e-9
    # 6 mm must not be mistaken for an intrusion or silently accepted with only 4 mm total opening.
    narrow=replace(c,open_jaw_model_rad=float(np.interp(.0218,jaw.gaps,jaw.angles)))
    assert not fixed_contact_review(arm,narrow)['accepted']
    inside=observed.copy();inside[:3,3]+=pose[:3,0]*.009
    assert not fixed_contact_review(arm,replace(c,observed_base_from_object=inside))['accepted']


def test_clearance_cannot_be_declared_without_observation():
    with pytest.raises(ValueError,match='original observed'):
        GraspRouteCandidate('test','cal','scene',np.zeros(5),np.eye(4),np.eye(4),(.02,.02,.02),.2,.1,
                            fixed_side_clearance_m=.006)
