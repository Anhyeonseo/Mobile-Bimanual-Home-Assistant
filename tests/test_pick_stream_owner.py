"""Production hold -> carry-port transition on an in-memory unified controller."""
from dataclasses import replace
from types import SimpleNamespace as NS
import pytest
from home_robot_tasks.carry_execution import build_carry_session
from home_robot_tasks.carry_program import CarryStage
from home_robot_tasks.pick_hardware import OWNER
from test_unified_arm_adapter import make_adapter
from test_carry_execution import program

@pytest.mark.parametrize('owner',[OWNER,'stationary-pick'])
def test_prepared_station_retains_single_owner_through_carry_routes(monkeypatch,owner):
    adapter,transport,clock=make_adapter(monkeypatch)
    adapter.prepare();points=adapter.current_pose_hold_points()
    adapter.start_current_pose_hold(OWNER)
    for _ in range(40):
        clock.advance(.02);adapter.poll(OWNER)
        if adapter.state.value=='ready':break
    assert adapter.state.value=='ready' and transport.enabled
    transport.flags=115
    p=replace(program(),stages=tuple(CarryStage(n,points,{'duration_s':.3},False) for n in ('open','pregrasp')))
    stops=[]
    stop=NS(request=lambda *a:stops.append(a),poll=lambda *a:NS(status='STOPPED'))
    session=build_carry_session(p,adapter=adapter,coordinated_stop=stop,observe=lambda:None,
        clock=clock,validate_current_scene=lambda *a:True,transport_owner=owner)
    before=len(transport.goal_log)
    session.motion.start('open',{'carry_stage':0})
    if owner!=OWNER:
        assert stops and 'owned by' in session.motion.pending_stop_reason()
        assert len(transport.goal_log)==before
        return
    assert not stops and len(transport.goal_log)>before
    for _ in range(40):
        clock.advance(.02);result=session.motion.poll('open')
        if result.state=='SUCCEEDED':break
    assert result.state=='SUCCEEDED'
    session.motion.retire_completed('open')
    session.motion.start('pregrasp',{'carry_stage':1})
    assert not stops and adapter.state.value=='active'
    assert transport.calls.count('arm')==1 and transport.calls.count('enable')==1
