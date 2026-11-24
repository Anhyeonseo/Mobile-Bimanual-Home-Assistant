import copy
import numpy as np
import pytest

from tools.lib.pick_contact_review import contact_height_review


def scene():
    pose=np.eye(4);pose[2,3]=.710
    pre=pose.copy();pre[2,3]+=.020
    object_pose=np.eye(4);object_pose[2,3]=.700
    report=dict(status='OFFLINE_ROUTE_FOUND', capture_id='new-capture', calibration_id='cal',
        observed_support_plane_base=[0,0,1,-.700], initial_model_joint_positions_si={'s2_lift_joint':.63},
        candidates=[dict(candidate={'candidate_id':'selected'},jaw_preview={'contact_fit':{'desired_base_from_tcp':pre.tolist()}})],
        plan=dict(candidate_id='selected', resolved_grasp=dict(base_from_grasp_tcp=pose.tolist(),
            observed_base_from_object=object_pose.tolist(), payload_mesh={'vertices':[[0,0,0],[.01,.01,.012]]}),
            carry_quality={'top_calibration':{'within_recorded_ranges':False}}))
    prepared={'review':{'observation_index':1},'observations':[{'support_height_m':.050},{'support_height_m':.012}]}
    return report,prepared


def test_review_uses_selected_object_table_height_and_does_not_apply_a_correction():
    report,prepared=scene();before=copy.deepcopy(report)
    review=contact_height_review(report,prepared)
    assert review['object_observed_height_m']==.012
    assert review['planned_tcp_height_above_support_m']==pytest.approx(.010)
    assert review['pregrasp_tcp_height_above_support_m']==pytest.approx(.030)
    assert review['planned_vertical_drop_m']==pytest.approx(.020)
    assert review['observed_object_envelope_height_m']==pytest.approx([0,.012])
    assert not review['physical_contact_height_verified']
    assert review['correction_applied_m']==0
    assert not review['calibration_posture_review']['within_recorded_ranges']
    assert report==before


def test_scaled_plane_has_same_metric_height():
    report,prepared=scene()
    reference=contact_height_review(report,prepared)
    report['observed_support_plane_base']=[0,0,2,-1.4]
    assert contact_height_review(report,prepared)==reference


@pytest.mark.parametrize('patch',[{'status':'NO_COMPLETE_ROUTE'}, {'observed_support_plane_base':None}, {'plan':{}}])
def test_absent_evidence_never_invents_height(patch):
    report,prepared=scene();report.update(patch)
    assert contact_height_review(report,prepared) is None
