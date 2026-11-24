"""Top-reference admission and web operations on the normal station request path."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from home_robot_tasks.pick_calibration import (TOP_BASIS, TOP_BOARD, audit_top_program, check_top_surface,
                                              make_top_reference, straight_top_contact, top_model_height, verify_installed)
from home_robot_tasks.pick_capture import make_evidence, review_capture, save_capture
from home_robot_tasks.pick_service import PickStation, Rejected
from home_robot_tasks.pick_web_backend import WebPickBackend
from home_robot_tasks.s2_geometry import JOINTS, control_from_model
from so101_arm_bridge.lift_wire import LiftCommand, LiftStatus
from test_pick_pipeline import FakeCompute, capture_fixture
from test_pick_service import wait_for

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('position_raw', [1743, 1979, 0, 4095])
def test_top_evidence_retains_registered_height_and_replays_with_the_same_model(tmp_path, position_raw):
    profile, platform, history, camera, meta = capture_fixture()
    raw_um = round(profile['lift']['travel_m'] * 1e6)
    platform.update(lift_reference_basis=TOP_BASIS, controller_board=list(TOP_BOARD))
    platform['lift'].update(target_um=raw_um, height_um=raw_um, position_raw=position_raw)
    platform['top_reference'] = make_top_reference(LiftStatus(**platform['lift']), profile, TOP_BOARD,
                                                  operator_confirmed=True)
    evidence = make_evidence(camera, history, platform, profile, now=20.02, requested_s=19.9,
                             session_id='real-time-fixture', anchor=(0.,)*12)
    folder = tmp_path/camera['capture_id']
    saved = save_capture(folder, (np.zeros((8, 8, 3), np.uint8), np.ones((8, 8), np.uint16), meta), evidence)
    actual, _, _ = review_capture(folder, saved, profile)
    assert actual['lift_height_m'] == .630 and actual['lift_reference_basis'] == TOP_BASIS
    assert saved['platform']['lift']['height_um'] == raw_um
    assert saved['platform']['top_reference']['position_raw'] == position_raw
    changed = deepcopy(saved)
    changed['platform']['lift']['position_raw'] = 2650
    with pytest.raises(ValueError, match='상단'):
        review_capture(folder, changed, profile)
    with pytest.raises(ValueError):
        top_model_height(NS(**platform['lift']), profile, (0x2492D,2430209922))
    packet = LiftCommand(8, 1, 1, 100, 114, raw_um)
    assert LiftCommand.decode(packet.encode()) == packet


def test_surface_and_top_binding_are_checked_but_historical_postures_are_advisory():
    profile, *_ = capture_fixture()
    pose = dict(base_from_optical=np.eye(4))
    # Different tabletops reach geometric planning instead of matching an old cabinet.
    for z in (.7234170847, .7338, .7369, .64):
        assert check_top_surface(dict(point_m=[0,0,z],normal=[0,0,1]), [.44,.15,.77], pose) == z
    with pytest.raises(ValueError, match='수평 상판'):
        check_top_surface(dict(point_m=[0,0,.735],normal=[1,0,0]), [.44,.15,.77], pose)
    for normal in ([0,0,0], [0,float('nan'),1]):
        with pytest.raises(ValueError, match='유효한'):
            check_top_surface(dict(point_m=[0,0,.735],normal=normal), [.44,.15,.77], pose)
    with pytest.raises(ValueError, match='유효한'):
        check_top_surface(dict(point_m=[0,0,float('nan')],normal=[0,0,1]), [.44,.15,.77], pose)
    extended = [0.,.8,-.08,.45,1.]
    assert straight_top_contact([0.,.758657,-.017911,.451664,0.])
    assert straight_top_contact([0.,.447914,.0164,.701182,0.], vertical_path=True)
    folded = [0.,1.42,-1.29,1.13,1.]
    assert straight_top_contact(extended) and not straight_top_contact(folded)
    def point(q):
        return NS(positions_rad=tuple(np.float64(control_from_model(v,profile['arms']['left']['alignment'][n]))
                                      for n,v in zip(JOINTS[:5],q))+(0.,)*7)
    stages = [NS(name=name, points=[point(extended)]) for name in ('approach','close','probe','lift')]
    program = NS(planned_lift_m=.630, hypothetical_lift=False, stages=stages)
    assert audit_top_program(program, profile)['within_recorded_ranges']
    stages[-1].points.append(point(folded))
    review=audit_top_program(program, profile)
    assert review['outside_recorded_stages']==['lift'] and review['hard_limit'] is False
    assert review['physical_accuracy_verified'] is False
    program.planned_lift_m=.60
    with pytest.raises(ValueError, match='높이'):
        audit_top_program(program, profile)
    program.planned_lift_m=.630
    program.stages=stages[:-1]
    with pytest.raises(ValueError, match='경로가 필요'):
        audit_top_program(program, profile)


def test_extended_posture_near_historical_box_is_preferred_without_excluding_folded_arm():
    from home_robot_tasks.pick_calibration import top_posture_preference,review_top_path
    extended=[-.02158,.98766,-.07212,-.00510,1.60595]  # saved rgbd-687:3:reversed
    folded=[-.02158,1.47721,-1.28849,1.09430,1.60415]  # saved rgbd-687:6
    assert not straight_top_contact(extended)
    assert top_posture_preference(extended)['cost']<top_posture_preference(folded)['cost']
    for q in (extended,folded):
        review=review_top_path([('close_geometry',[q])])
        assert not review['within_recorded_ranges'] and not review['hard_limit']


def test_profile_pin_detects_mixed_installation(tmp_path):
    from home_robot_tasks.pick_calibration import PROFILE_FILES, REVISION
    assert verify_installed(ROOT) == REVISION
    folder = tmp_path/'config/home_assistant'
    folder.mkdir(parents=True)
    for name in PROFILE_FILES:
        (folder/name).write_bytes((ROOT/'config/home_assistant'/name).read_bytes())
    (folder/'camera_height.calibrated.json').write_text('{}')
    with pytest.raises(ValueError, match='camera_height'):
        verify_installed(tmp_path)


def test_bottom_preview_checks_target_before_activation_and_allows_retry(tmp_path):
    baseline = tmp_path/'baseline.json'
    baseline.write_text('{}')
    captures = []
    worker = FakeCompute()
    class CameraFixture(WebPickBackend):
        def _camera_capture(self, ctx, mode):
            captures.append(mode)
            return dict(snapshot='camera', camera_mode=mode)
        def _detect_camera(self, ctx, value):
            assert value['prompt'] in ('remote control','tape')
            return value
        def _camera_result(self, ctx, value):
            return dict(status='REOBSERVE', reason='partial_mask_depth', count=0,
                        depth_fraction=.40, surface_error=None)
    backend = CameraFixture(dict(mode='station', top_baseline=str(baseline), baseline=str(baseline), camera_mode='wide'),
                            worker_factory=lambda *_:worker,
                            hardware_factory=lambda *_:pytest.fail('actuation on insufficient depth'))
    station = PickStation(backend, tmp_path/'web')
    station.start()
    try:
        wait_for(lambda:station.snapshot()['accepting'])
        for bottom, top in ((True,False), (False,False), (True,True), (False,1)):
            with pytest.raises(Rejected):
                station.submit('리모컨을 집어줘',operation='top-pick',supported_empty_bottom=bottom,supported_empty_top=top)
        for index, text in enumerate(('리모컨을 집어줘','테이프를 집어줘'), 1):
            station.submit(text,operation='bottom-pick',supported_empty_bottom=True)
            wait_for(lambda:len(station.snapshot()['history'])==index and station.snapshot()['accepting'])
            assert station.snapshot()['last_result']['physical_success'] is False
            assert station.snapshot()['current']['stages'][3]['status']=='skipped'
        assert captures == ['wide','near']*2
    finally:
        station.close()


def test_confirmed_top_request_uses_one_held_capture_before_dispatch(tmp_path, monkeypatch):
    from home_robot_tasks.pick_backend import PipelineBackend
    baseline = tmp_path/'baseline.json'
    baseline.write_text('{}')
    calls = []
    backend = WebPickBackend(dict(mode='station',camera_mode='wide',top_baseline=str(baseline)),
                             worker_factory=lambda *_:FakeCompute())
    def pre_camera(ctx, mode):
        calls.append(('preview',mode))
        return dict(snapshot='preview-only',camera_mode=mode)
    def prepare(ctx, options):
        calls.append(('prepare',options['supported_empty_top'],options['supported_empty_bottom'],options['baseline']))
        backend.hardware = NS(options=options, guard=NS(supported_top=True), tick=lambda:None,
                              close=lambda:None, stopped=False)
    def capture(me, ctx, value):
        calls.append(('held-capture',value['camera_mode']))
        return dict(value,snapshot='fresh-held-frame')
    def run(stage):
        def execute(me, ctx, value):
            assert value['snapshot']=='fresh-held-frame'
            calls.append((stage,value['prompt']))
            if stage=='execute':
                backend.hardware.session=NS(state='HOLDING')
                return dict(value,bundle='checked')
            if stage=='verify':
                return dict(status='UNVERIFIED', physical_success=False, reason='fixture verification inconclusive')
            return value
        return execute
    monkeypatch.setattr(backend,'_camera_capture',pre_camera)
    monkeypatch.setattr(backend,'_detect_camera',lambda ctx,value:value)
    monkeypatch.setattr(backend,'_camera_result',lambda ctx,value:dict(
        status='CAMERA_CANDIDATES_ONLY' if value['camera_mode']=='near' else 'REOBSERVE',
        count=4 if value['camera_mode']=='near' else 0, surface_error=None,
        reason=None if value['camera_mode']=='near' else 'partial_mask_depth'))
    monkeypatch.setattr(backend,'_prepare_hardware',prepare)
    monkeypatch.setattr(PipelineBackend,'capture',capture)
    for stage in ('detect','candidates','plan','execute','verify'):
        monkeypatch.setattr(PipelineBackend,stage,run(stage))
    station = PickStation(backend,tmp_path/'station')
    station.start()
    try:
        wait_for(lambda:station.snapshot()['accepting'])
        station.submit('테이프를 집어줘',operation='top-pick',supported_empty_top=True)
        wait_for(lambda:station.snapshot()['history'])
        assert station.snapshot()['last_result']['status']=='UNVERIFIED'
        assert calls == [('prepare',True,False,str(baseline)),
                         ('held-capture','near')]+[(stage,'tape') for stage in ('detect','candidates','plan','execute')]
    finally:
        station.close()


def test_top_position_uses_lift_servo_readback_before_hold():
    from home_robot_tasks.pick_calibration import check_top_inventory
    other = dict(bus='right', requested_id=1, feedback={'position_raw': 1800})
    def check(position):
        lift = dict(bus='left', requested_id=10, feedback={'position_raw': position})
        return check_top_inventory({'devices': [other, lift]})
    for pos in (0, 1727, 1882, 1979, 4095):
        assert check(pos) == pos  # A valid raw reading alone does not authorize reference.
    for pos in (-1, 4096, True, 1979.0, None):
        with pytest.raises(ValueError, match=str(pos)):
            check(pos)
    with pytest.raises(ValueError, match='측정값'):
        check_top_inventory({'devices': [other]})


def test_top_reference_checks_only_measured_fresh_ls_position():
    from home_robot_tasks.pick_calibration import check_top_sample
    for flags, observed in ((4, 950), (12, 800)):
        check_top_sample(NS(flags=flags, mcu_tick_ms=1000, observed_ms=observed, position_raw=0))
    check_top_sample(NS(flags=12, mcu_tick_ms=30, observed_ms=0xfffffff0, position_raw=1979))
    with pytest.raises(ValueError, match='4096'):
        check_top_sample(NS(flags=12, mcu_tick_ms=30, observed_ms=0xfffffff0, position_raw=4096))


def registered_top(position_raw=1979):
    profile, platform, *_ = capture_fixture()
    target = round(profile['lift']['travel_m'] * 1e6)
    status = replace(LiftStatus(**platform['lift']), height_um=target, target_um=target,
                     position_raw=position_raw)
    reference = make_top_reference(status, profile, TOP_BOARD, operator_confirmed=True)
    return profile, status, reference


def test_top_requires_current_explicit_reference_even_for_historical_encoder_values():
    for raw in (1743, 1979):
        profile, status, reference = registered_top(raw)
        for confirmed in (False, None, 1):
            with pytest.raises(ValueError, match='최상단 확인'):
                make_top_reference(status, profile, TOP_BOARD, operator_confirmed=confirmed)
        for ref in (None, {}, dict(reference, boot_id=3), dict(reference, lift_session=2),
                    dict(reference, operator_confirmed=False), dict(reference, calibration_id='old'),
                    dict(reference, basis='supported_top_straight_arm_20261118')):
            with pytest.raises(ValueError, match='등록 기록'):
                top_model_height(status, profile, TOP_BOARD, ref)


@pytest.mark.parametrize('changes', [dict(fault=1), dict(state=0), dict(flags=31),
    dict(command_raw=1), dict(observed_ms=700), dict(height_um=640000), dict(target_um=0),
    dict(boot_id=5), dict(session=2), dict(position_raw=2650)])
def test_new_top_reference_preserves_fault_freshness_height_and_drift_checks(changes):
    profile, status, reference = registered_top()
    with pytest.raises(ValueError):
        top_model_height(replace(status, **changes), profile, TOP_BOARD, reference)


def test_top_relative_encoder_wrap_and_registered_drift():
    profile, status, reference = registered_top(4095)
    assert top_model_height(replace(status, position_raw=0, height_um=status.height_um-21),
                            profile, TOP_BOARD, reference) == pytest.approx(.630-.000021)
    with pytest.raises(ValueError, match='2 mm'):
        top_model_height(replace(status, position_raw=94), profile, TOP_BOARD, reference)


def test_top_cannot_be_registered_after_lift_slips_during_initial_hold():
    from home_robot_tasks.pick_calibration import check_top_sample
    _, status, _ = registered_top()
    check_top_sample(status, anchor_raw=1979, board=TOP_BOARD)
    with pytest.raises(ValueError, match='2 mm'):
        check_top_sample(replace(status, position_raw=2650), anchor_raw=1979, board=TOP_BOARD)


@pytest.mark.parametrize('moving', [False, True])
def test_operator_top_reference_still_requires_fresh_stationary_window(moving):
    from home_robot_tasks.platform_preparation import reference_supported_top
    profile, held, _ = registered_top()
    clock = [0.]
    calls = []
    def observe():
        tick = round(clock[0]*1000)
        return replace(held, mcu_tick_ms=tick, observed_ms=tick, state=5 if calls else 0,
                       flags=29 if calls else 12, position_raw=1979+(tick//20 if moving else 0))
    def reference(*args, **kwargs):
        calls.append((clock[0], args, kwargs))
    lift = NS(reference_top=reference, keepalive=lambda:pytest.fail('unexpected lease renewal'))
    kwargs = dict(operator_confirmed=True, clock=lambda:clock[0],
                  sleep=lambda dt:clock.__setitem__(0, clock[0]+dt))
    if moving:
        with pytest.raises(TimeoutError, match='정지 관측'):
            reference_supported_top(observe, None, lift, held.target_um, **kwargs)
        assert not calls
    else:
        status = reference_supported_top(observe, None, lift, held.target_um, **kwargs)
        assert len(calls) == 1 and calls[0][0] >= .45
        assert calls[0][1:] == ((1, held.target_um), dict(operator_confirmed=True))
        reference = make_top_reference(status, profile, TOP_BOARD, operator_confirmed=True)
        assert top_model_height(status, profile, TOP_BOARD, reference) == .630


@pytest.mark.parametrize('failure',['transient','boot','timeout','active','fault','sequence','stale','persistent'])
def test_reference_waits_only_after_proven_nonstarting_rejection(failure):
    from dataclasses import asdict
    from home_robot_tasks.platform_preparation import reference_supported_top
    _,held,_=registered_top();clock=[0.];attempts=[];accepted=[False]
    def observe():
        tick=round(clock[0]*1000)
        return replace(held,mcu_tick_ms=tick,observed_ms=tick,state=5 if accepted[0] else 0,
                       session=1 if accepted[0] else 0,flags=29 if accepted[0] else 12)
    def reference(session,height,**kwargs):
        attempts.append(clock[0])
        if failure=='transient' and len(attempts)==2:accepted[0]=True;return
        if failure=='timeout':raise TimeoutError('lost response')
        reply=replace(observe(),kind=136,request_sequence=len(attempts),session=0,sequence=0)
        if failure=='boot':reply=replace(reply,boot_id=reply.boot_id+1)
        if failure=='active':reply=replace(reply,flags=14,session=1)
        if failure=='fault':reply=replace(reply,fault=1)
        if failure=='sequence':reply=replace(reply,request_sequence=99)
        if failure=='stale':reply=replace(reply,observed_ms=reply.observed_ms-250)
        error=RuntimeError('rejected')
        error.diagnostics=dict(command=dict(kind=8,target_um=height,boot_id=held.boot_id,sequence=len(attempts)),response=asdict(reply))
        raise error
    lift=NS(reference_top=reference,keepalive=lambda:pytest.fail('unexpected renewal'))
    kwargs=dict(operator_confirmed=True,clock=lambda:clock[0],sleep=lambda dt:clock.__setitem__(0,clock[0]+dt))
    if failure=='transient':
        assert reference_supported_top(observe,None,lift,held.target_um,**kwargs).state==5
        assert len(attempts)==2 and attempts[1]-attempts[0]>=.45
    else:
        with pytest.raises((RuntimeError,TimeoutError)):reference_supported_top(observe,None,lift,held.target_um,**kwargs)
        assert len(attempts)>1 if failure=='persistent' else len(attempts)==1
        assert clock[0]<3.6
