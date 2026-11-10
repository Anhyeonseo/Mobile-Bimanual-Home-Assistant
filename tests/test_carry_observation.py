from dataclasses import replace
import numpy as np
import pytest

from home_robot_tasks.carry_observation import CarryVisualMonitor
from home_robot_tasks.probe_evidence import LiftSample, lift_relation


def sample(t, z, target_z=None, error=.0001):
    return LiftSample('scene','remote','visible-button',t,t,(0.,0.,z),
                      (.01,0.,z if target_z is None else target_z),error)


def pose(z):
    p=np.eye(4);p[2,3]=z
    return p


def test_slow_retimed_probe_uses_its_bounded_duration_not_one_second_reference_age():
    start=sample(1.,0.);end=sample(3.8,.004)
    assert lift_relation(start,end,3.8)[0]=='unknown'
    monitor=CarryVisualMonitor(probe_duration_s=2.8)
    monitor.begin_probe(start,pose(0.),1.)
    assert monitor.observe(end,pose(.004),3.8)==('held','TARGET_FOLLOWED')
    # A rotation changes the visible patch offset, even if the TCP stays still.
    p=pose(.004);p[:2,:2]=[[0,-1],[1,0]]
    rotated=replace(sample(3.9,.004),target_xyz_m=(0.,.01,.004))
    assert monitor.observe(rotated,p,3.9)==('held','MEASURED_PATCH_RETAINED')


@pytest.mark.parametrize('issue',['object_stays','uncertain','too_far','unsynchronized'])
def test_no_independent_probe_evidence_no_held_claim(issue):
    monitor=CarryVisualMonitor(probe_duration_s=3.)
    monitor.begin_probe(sample(1.,0.),pose(0.),1.)
    end=sample(3.,.004)
    if issue=='object_stays':end=replace(end,target_xyz_m=(.01,0.,0.))
    if issue=='uncertain':end=replace(end,position_error_bound_m=.002)
    if issue=='too_far':end=replace(end,tool_xyz_m=(0.,0.,.02))
    if issue=='unsynchronized':end=replace(end,vision_time_s=2.8)
    assert monitor.observe(end,pose(end.tool_xyz_m[2]),3.)[0]=='unknown'


def test_repeated_or_slipping_patch_cannot_keep_held_state_alive():
    monitor=CarryVisualMonitor(probe_duration_s=3.)
    monitor.begin_probe(sample(1.,0.),pose(0.),1.)
    good=sample(3.,.004)
    assert monitor.observe(good,pose(.004),3.)[0]=='held'
    slip=replace(sample(3.1,.004),target_xyz_m=(.03,0.,.004))
    assert monitor.observe(slip,pose(.004),3.1)[0]=='unknown'
    assert monitor.observe(sample(3.2,.004),pose(.004),3.2)[0]=='unknown'


def test_missing_frame_cannot_be_restamped_as_current_observation():
    monitor=CarryVisualMonitor(probe_duration_s=3.)
    monitor.begin_probe(sample(1.,0.),pose(0.),1.)
    assert monitor.observe(sample(3.,.004),pose(.004),3.3)[0]=='unknown'


@pytest.mark.parametrize('budget',[float('nan'),0.,20.,True])
def test_probe_duration_remains_bounded(budget):
    assert lift_relation(sample(1,0),sample(3,.004),3,maximum_duration_s=budget)==('unknown','INVALID_PROBE_DURATION')
