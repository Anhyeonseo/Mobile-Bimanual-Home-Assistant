"""Planner output is untrusted until joint/time/speed/anchor checks pass."""
from types import SimpleNamespace as N
from pathlib import Path
import json
import pytest
from home_robot_tasks.moveit_bridge import MoveItBridge

ROOT=Path(__file__).resolve().parents[1]
NAMES=json.loads((ROOT/'config/bimanual_operational_limits.json').read_text())['joint_order']
def fixture():
    bridge=MoveItBridge(NAMES,[-1.]*12,[1.]*12,lambda:0.)
    point=lambda q,t:N(positions=q,time_from_start=N(sec=t,nanosec=0))
    trajectory=N(joint_trajectory=N(joint_names=NAMES[:5],points=[point([0.]*5,0),point([.1,0.,0.,0.,0.],1)]))
    return bridge,trajectory,N(anchor_rad=(0.,)*6+(.2,)*6)

def test_mapping_preserves_other_arm_and_grippers_and_50ms_grid():
    b,t,c=fixture();points=b.map(t,c)
    assert len(points)==21 and points[-1].positions_rad[0]==pytest.approx(.1)
    assert all(p.positions_rad[6:]==c.anchor_rad[6:] and p.positions_rad[5]==0 for p in points)
    assert all(y.offset_ms-x.offset_ms==50 for x,y in zip(points,points[1:]))

@pytest.mark.parametrize('fault',('unknown','duplicate','missing','reverse_time','nan','joint_limit','speed','anchor','anchor_transition'))
def test_invalid_plans_never_become_resident_commands(fault):
    b,t,c=fixture();jt=t.joint_trajectory
    if fault=='unknown':jt.joint_names=['bad',*jt.joint_names[1:]]
    elif fault=='duplicate':jt.joint_names=[jt.joint_names[1],*jt.joint_names[1:]]
    elif fault=='missing':jt.joint_names=jt.joint_names[:-1]
    elif fault=='reverse_time':jt.points[1].time_from_start.sec=0
    elif fault=='nan':jt.points[1].positions[0]=float('nan')
    elif fault=='joint_limit':jt.points[1].positions[0]=1.1
    elif fault=='speed':jt.points[1].positions[0]=.6
    elif fault=='anchor':jt.points[0].positions[0]=.03
    elif fault=='anchor_transition':jt.points[0].positions[0]=.02;jt.points[1].positions[0]=.52
    with pytest.raises(ValueError):b.map(t,c)
