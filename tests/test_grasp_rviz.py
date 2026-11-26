import copy
import numpy as np
import pytest
from tools.diagnostics.replay_grasp_rviz import first_plan, playback_frames


def plan():
    labels=['open_geometry','pregrasp','approach','close_geometry','lift','carry']
    return dict(motion_authorized=False,physical_task_completed=False,
        resolved_grasp=dict(open_jaw_model_rad=.8,held_jaw_model_rad=.3),
        stages=[dict(label=label,joints=[[0.]*5,[.1]*5]) for label in labels])


def test_only_complete_successful_offline_route_is_selected():
    p=plan()
    assert first_plan(dict(attempts=[dict(status='NO_COMPLETE_ROUTE'),dict(status='OFFLINE_ROUTE_FOUND',plan=p)])) is p
    bad=copy.deepcopy(p);bad['stages'].pop()
    with pytest.raises(ValueError,match='six-stage'):
        first_plan(dict(attempts=[dict(status='OFFLINE_ROUTE_FOUND',plan=bad)]))
    with pytest.raises(ValueError,match='no successful'):
        first_plan(dict(attempts=[]))


def test_playback_jaw_and_attachment_follow_stages():
    frames=playback_frames(plan(),dict(left_model_gripper=.1),fps=10,stage_seconds=1.)
    assert len(frames)==60
    assert frames[0]['jaw']==pytest.approx(.1)
    assert frames[9]['jaw']==pytest.approx(.8)
    assert frames[39]['jaw']==pytest.approx(.3)
    assert all(f['attached']==(f['stage'] in ('lift','carry')) for f in frames)
    assert np.allclose(frames[-1]['arm'],[.1]*5)


def test_nonfinite_saved_trajectory_is_rejected():
    p=plan();p['stages'][1]['joints'][0][0]=float('nan')
    with pytest.raises(ValueError,match='finite five-joint'):
        playback_frames(p,dict(left_model_gripper=0.))
