from dataclasses import replace
from threading import RLock
from types import SimpleNamespace as NS
import pytest
from so101_arm_bridge.lift_wire import LiftStatus
from home_robot_tasks.unified_carry_inputs import (
    UnifiedPlatformGuard, UnifiedObservationReader, PlatformInterlockPending)


def reader_fixture(flags, *, fault=0, elapsed=.01):
    now=[20.]; events=[]; index=[0]
    mobile=NS(boot_id=4,state=0,session=0,reason=0,flags=23,
              mcu_tick_ms=1000,feedback_tick_ms=980)
    sample=LiftStatus(4,1,1000,4,1,1,5,fault,29,980,8000,8000,0,0,1908,0,0)
    def lift_status():
        flag=flags[min(index[0],len(flags)-1)];index[0]+=1
        now[0]+=elapsed
        return replace(sample,flags=flag)
    packet=NS(positions_urad=(0,)*12)
    adapter=NS(keepalive=lambda:events.append('keepalive'),
        feedback_snapshot=lambda:packet,
        motion_limits=NS(minimum_urad=(-1,)*12,maximum_urad=(1,)*12))
    inputs=NS(lock=RLock(),platform=UnifiedPlatformGuard(4,.008),generation=0,
        add_joints=lambda *args:events.append('joints'))
    evidence=NS(publish_mcu_snapshot=lambda *args,**kwargs:events.append('evidence'))
    reader=UnifiedObservationReader(None,adapter,NS(status=lambda:mobile),
        NS(status=lift_status),evidence,inputs,lambda:now[0])
    return reader,inputs,events,index


def test_interlock_only_gap_requires_new_complete_accepted_sample():
    reader,inputs,events,index=reader_fixture([25,29])
    reader.refresh()
    assert index[0]==2 and reader.interlock_refreshes==1
    assert events.count('evidence')==2 and events.count('joints')==2
    assert inputs.generation==1
    assert inputs.platform.last_observation['accepted']
    assert inputs.platform.last_observation['lift']['flags']==29


def test_persistent_interlock_loss_still_fails_without_accepting_packet():
    reader,inputs,_,index=reader_fixture([25])
    with pytest.raises(PlatformInterlockPending):reader.refresh()
    assert index[0]==3 and inputs.generation==0
    assert not inputs.platform.last_observation['accepted']
    with pytest.raises(ValueError):inputs.platform.require_fresh(20.04)


def test_refresh_time_budget_does_not_extend_on_each_retry():
    reader,inputs,_,index=reader_fixture([25],elapsed=.11)
    with pytest.raises(PlatformInterlockPending):reader.refresh()
    assert index[0]==1 and inputs.generation==0


@pytest.mark.parametrize('flags,fault', [([25],1),([27],0),([9],0)])
def test_fault_motion_or_missing_hold_is_not_retried(flags,fault):
    reader,inputs,_,index=reader_fixture(flags,fault=fault)
    with pytest.raises(ValueError):reader.refresh()
    assert index[0]==1 and inputs.generation==0
