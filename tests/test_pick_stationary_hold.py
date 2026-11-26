from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS
import pytest
from so101_arm_bridge.lift_wire import LiftStatus
from home_robot_tasks.pick_calibration import (
    make_resumed_height_reference, top_model_height, RISE_BOARD)
from home_robot_tasks.platform_preparation import stationary_platform_is_holding
from home_robot_tasks.unified_carry_inputs import UnifiedPlatformGuard
from test_pick_lift_transition import arrival_fixture


def fixture():
    profile,data=arrival_fixture()
    source=deepcopy(data['start_revalidation']['arrival_evidence']['platform'])
    source['lift'].update(height_um=652841,position_raw=1907)
    source['top_reference'].update(height_um=652841,position_raw=1907)
    lift=replace(LiftStatus(**source['lift']),boot_id=137,session=1,
                 mcu_tick_ms=142633,observed_ms=142616,height_um=654588,
                 target_um=654588,position_raw=1908,flags=29)
    ref=make_resumed_height_reference(lift,profile,RISE_BOARD,source,
        operator_height_unchanged=True,height_model='encoder_no_sag')
    height=top_model_height(lift,profile,RISE_BOARD,ref)
    mobile=NS(boot_id=137,state=0,session=0,reason=0,flags=31,
              mcu_tick_ms=142629,feedback_tick_ms=142531)
    guard=UnifiedPlatformGuard(137,height,profile=profile,board=RISE_BOARD,
                               supported_top=True,top_reference=ref)
    return profile,ref,mobile,replace(lift,flags=25),guard


def test_recorded_stationary_hold_does_not_require_new_lift_motion_permission():
    profile,ref,m,l,g=fixture()
    # Movement admission is unchanged. No status flags are synthesized.
    assert not l.holding()
    with pytest.raises(ValueError):top_model_height(l,profile,RISE_BOARD,ref)
    g.update(m,l,20.,20.008)
    assert g.last_observation['accepted']
    assert g.last_observation['lift']['flags']==25
    assert g.last_observation['hold_proof']=='independent_base_stop_and_lift_hold'
    assert g.current_height==pytest.approx(.652820584665)
    g.require_fresh(20.02)


@pytest.mark.parametrize('changes',[
    dict(flags=23),dict(flags=29),dict(flags=15),dict(flags=63),
    dict(state=1),dict(session=1),dict(reason=1),dict(boot_id=138),
    dict(feedback_tick_ms=142429)])
def test_base_stop_health_mode_session_and_freshness_remain_required(changes):
    _,_,m,l,g=fixture();m.__dict__.update(changes)
    assert not stationary_platform_is_holding(m,l,RISE_BOARD)
    with pytest.raises(ValueError):g.update(m,l,20.,20.008)
    assert not g.last_observation['accepted']


@pytest.mark.parametrize('changes',[
    dict(fault=1),dict(flags=27),dict(flags=9),dict(state=4),
    dict(command_raw=1),dict(observed_ms=142433),dict(height_um=650000),
    dict(boot_id=138),dict(session=2),dict(position_raw=1700),
    dict(target_um=653588)])
def test_lift_fault_motion_staleness_drift_and_reference_still_fail(changes):
    _,_,m,l,g=fixture()
    with pytest.raises(ValueError):g.update(m,replace(l,**changes),20.,20.008)
    assert not g.last_observation['accepted']


def test_unknown_board_and_delayed_host_packet_do_not_pass():
    _,_,m,l,g=fixture()
    assert not stationary_platform_is_holding(m,l,(0x2492F,RISE_BOARD[1]))
    with pytest.raises(ValueError):g.update(m,l,20.,20.3)


def test_capture_replay_preserves_independent_hold_evidence(tmp_path):
    import numpy as np
    from home_robot_tasks.pick_calibration import TOP_BASIS,make_top_reference
    from home_robot_tasks.pick_capture import make_evidence,save_capture,review_capture
    from test_pick_pipeline import capture_fixture
    profile,platform,history,camera,meta=capture_fixture()
    platform.update(lift_reference_basis=TOP_BASIS,controller_board=list(RISE_BOARD))
    platform['mobile']['flags']=31
    platform['lift'].update(height_um=654588,target_um=654588,position_raw=1908)
    platform['top_reference']=make_top_reference(LiftStatus(**platform['lift']),profile,
        RISE_BOARD,operator_confirmed=True,height_model='encoder_no_sag')
    platform['lift']['flags']=25
    evidence=make_evidence(camera,history,platform,profile,now=20.02,requested_s=19.9,
        session_id='independent-hold',anchor=(0.,)*12)
    folder=tmp_path/camera['capture_id']
    saved=save_capture(folder,(np.zeros((8,8,3),np.uint8),np.ones((8,8),np.uint16),meta),evidence)
    review,_,_=review_capture(folder,saved,profile)
    assert saved['platform']['lift']['flags']==25
    assert review['lift_height_m']==pytest.approx(evidence['fixed_lift_m'])
