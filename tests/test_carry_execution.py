from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from home_robot_tasks.carry_execution import CarryFeedback, CarryRouteBuilder, CarrySession
from home_robot_tasks.carry_program import CarryProgram, CarryStage, compile_program, manifest
from home_robot_tasks.carry_trajectory import MotionBounds, polyline_corners, retime
from home_robot_tasks.installed_jaw import InstalledLeftJaw
from home_robot_tasks.s2_geometry import JOINTS
from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
from so101_arm_bridge.stationary_pick_adapter import StationaryPickAdapter
from tools.diagnostics.carry_stream_sim import SimClock, SimTransport, simulate

ROOT = Path(__file__).resolve().parents[1]


def bounds():
    return MotionBounds((-4.,)*12, (4.,)*12, (.3,)*12, (.6,)*12, (3.,)*12)


def program():
    q = np.zeros(12)
    q[5] = -.5
    stages = []
    for name, attached in [('open',False), ('pregrasp',False), ('approach',False), ('close',False),
                           ('probe',True), ('lift',True), ('carry',True), ('hold',True)]:
        target = q.copy()
        if name == 'open': target[5] = -.55
        if name == 'pregrasp': target[0] = .1
        if name == 'approach': target[1] = .08
        if name == 'close': target[5] = -.2
        if name == 'probe': target[2] = .04
        if name == 'lift': target[2] = .2
        if name == 'carry': target[0] = -.1
        points, timing = retime([q, target], bounds(), dwell_s=2. if name in ('close','hold') else 0.)
        stages.append(CarryStage(name, points, timing, attached))
        q = target
    return CarryProgram('capture', 'calibration', {'color':'a'}, 'report', 'prepared', 'grasp',
                        'synthetic_fixture', .3, .3, False, tuple(stages), bounds(), -.2,
                        (-.28,-.2), (2178,2230), 2178, .035)


def test_retime_preserves_corners_reversals_and_bounded_rates():
    path = np.zeros((5,12))
    path[:,0] = [0,.1,.2,.1,.1]
    path[-1,1] = .2
    assert len(polyline_corners(path)) == 4
    points, peaks = retime(path, bounds())
    q = np.asarray([p.positions_rad for p in points])
    assert np.all(q[:,2:] == 0)
    assert np.all((q[:,1] == 0) | np.isclose(q[:,0], .1))
    for name, limit in [('velocity',.3), ('acceleration',.6), ('jerk',3.)]:
        assert max(peaks[name]) <= limit+1e-10
    assert all(p.offset_ms % 5 == 0 for p in points)
    # Endpoints of each segment occur explicitly, never spline-rounding bends.
    for corner in polyline_corners(path):
        assert np.min(np.max(abs(q-corner),axis=1)) < 1e-12


@pytest.mark.parametrize('value', [float('nan'),float('inf'),4.1,-4.1])
def test_retime_rejects_nonfinite_and_outside_limits(value):
    p = np.zeros((2,12)); p[1,0] = value
    with pytest.raises(ValueError): retime(p,bounds())


def test_continuous_encoder_coordinates_do_not_take_modulo_shortcut():
    p = np.zeros((2,12)); p[:,0] = [3.,-3.]
    with pytest.raises(ValueError,match='branch'): retime(p,bounds())


def test_empty_or_bad_timing_is_not_admitted():
    with pytest.raises(ValueError): retime([],bounds())
    with pytest.raises(ValueError): retime(np.zeros((2,12)),bounds(),interval_ms=7)


@pytest.mark.parametrize('continuous',[False,True])
def test_full_resident_execution_retains_pressure_and_single_writer(continuous):
    p = program(); r = simulate(p,continuous_observation=continuous)
    assert r['state'] == 'HOLDING'
    assert r['simulated_arm_calls'] == r['simulated_enable_calls'] == 1
    assert r['hardware_commands'] == r['simulated_stop_calls'] == 0
    assert [s['stage'] for s in r['stages']] == [s.name for s in p.stages]
    assert r['final_command_rad'][5] == -.2
    assert r['final_measured_rad'][5] == -.24
    assert r['final_command_rad'][6:] == [0.]*6
    assert not r['physical_task_completed']


@pytest.mark.parametrize('action,state',[('wait','AWAITING_DESCENT_CONFIRMATION'),
    ('resume','HOLDING'),('cancel','CANCELLED'),('stale','FAILED')])
def test_pregrasp_height_checkpoint_holds_without_descent_until_explicit_release(action,state):
    p=program()
    r=simulate(p,pause_before_descent=True,checkpoint_action=action,
               observation_policy='snapshot_then_verify',continuous_observation=True)
    assert r['state']==state
    assert r['checkpoint_ticks']>=200 and r['checkpoint_hold_cycles']>=3
    assert r['simulated_arm_calls']==r['simulated_enable_calls']==1
    if action!='resume':
        assert [s['stage'] for s in r['stages']]==['open','pregrasp']
        np.testing.assert_allclose(r['final_command_rad'],p.stages[1].points[-1].positions_rad,atol=1e-6)
        assert not r['descent_released']
    else:
        assert r['descent_released'] and r['stages'][-1]['stage']=='hold'
    if action in ('cancel','stale'):assert r['simulated_stop_calls']>0


def test_checkpoint_cannot_skip_stages_accept_early_release_or_change_anchor():
    p=program();builder=CarryRouteBuilder(p,lambda *a:True,pause_before_descent=True)
    snap=lambda q:NS(positions_urad=tuple(round(v*1e6) for v in q))
    with pytest.raises(ValueError,match='only after'):builder({'pregrasp_hold':True},snap(p.stages[0].points[0].positions_rad))
    for i in (0,1):builder({'carry_stage':i},snap(p.stages[i].points[0].positions_rad))
    q=list(p.stages[1].points[-1].positions_rad)
    points=builder({'pregrasp_hold':True},snap(q))
    assert all(x.positions_rad==tuple(q) for x in points)
    q[0]+=.03
    with pytest.raises(ValueError,match='anchor changed'):builder({'pregrasp_hold':True},snap(q))
    session=CarrySession(p,None,None,None,SimClock(),pause_before_descent=True)
    with pytest.raises(ValueError,match='active pregrasp'):session.resume_descent()


@pytest.mark.parametrize('fault,state,forbidden', [
    ('no_follow','FAILED','lift'), ('empty_close','FAILED','lift'),
    ('slip','FAILED','carry'), ('timeout','FAILED','lift'),
    ('dispatch','FAILED','lift'), ('stale','FAILED','lift'),
    ('cancel','CANCELLED','probe'), ('base_moved','FAILED','carry'),
    ('stop_unconfirmed','STOPPING','carry'),
])
@pytest.mark.parametrize('continuous',[False,True])
def test_resident_failure_never_advances_or_claims_unconfirmed_stop(fault,state,forbidden,continuous):
    r = simulate(program(),fault=fault,continuous_observation=continuous)
    assert r['state'] == state
    assert forbidden not in [s['stage'] for s in r['stages']]
    assert r['simulated_stop_calls'] > 0
    assert r['simulated_enable_calls'] == 1


def feedback(p,clock,**changes):
    f = CarryFeedback(1,clock(),clock(),p.capture_id,p.calibration_id,dict(p.source_sha256),
                      p.stages[0].points[0].positions_rad,p.planned_lift_m,
                      True,True,True,True,resident_profile_verified=True,grip_cap_verified=True)
    return replace(f,**changes)


@pytest.mark.parametrize('changes', [
    {'capture_id':'old'}, {'calibration_id':'old'}, {'source_sha256':{'color':'wrong'}},
    {'observed_s':0}, {'capture_observed_s':0}, {'lift_m':0.},
    {'base_stopped':False}, {'lift_stopped':False}, {'controller_ok':False},
    {'scene_current':False}, {'resident_profile_verified':False}, {'grip_cap_verified':False},
])
def test_live_admission_fails_before_dispatch(changes):
    p=program(); clock=SimClock(); calls=[]
    motion=NS(ready=lambda: True, start=lambda *args: calls.append(args))
    session=CarrySession(p,motion,None,lambda:feedback(p,clock,**changes),clock)
    with pytest.raises(ValueError): session.start('test')
    assert not calls


@pytest.mark.parametrize('stop_confirmed', [False, True])
def test_first_motion_fault_survives_stale_observations_during_stop(stop_confirmed):
    from home_robot_tasks.device_ports import ResidentArmPort
    from so101_arm_bridge.bimanual_stream_adapter import ResidentBimanualStreamAdapter, OperationalLimits, CANONICAL_JOINT_NAMES
    p=program();clock=SimClock()
    transport=SimTransport(clock,p.stages[0].points[0].positions_rad)
    adapter=ResidentBimanualStreamAdapter(transport,OperationalLimits(
        CANONICAL_JOINT_NAMES,(-4000000,)*12,(4000000,)*12),motion_authorized=True)
    adapter.prepare()
    stop=NS(request=lambda *a:None,poll=lambda *a:NS(status='STOPPED' if stop_confirmed else 'STOPPING'))
    port=ResidentArmPort(adapter,CarryRouteBuilder(p,lambda *a:True),stop,clock)
    calls=[]
    def observe():
        calls.append(clock())
        if transport.stop_latched:raise ValueError('platform sample stale')
        return feedback(p,clock)
    session=CarrySession(p,port,stop,observe,clock)
    session.start('first-output-fault')
    transport.fault='dispatch'
    session.poll()
    assert session.state==('FAILED' if stop_confirmed else 'STOPPING')
    assert 'dispatch fault' in session.reason
    assert 'platform sample stale' not in session.reason
    observed=len(calls)
    clock.advance(1.)
    session.poll()
    assert len(calls)==observed
    assert session.history==[] and port.owner is not None


def test_hypothetical_height_never_authorizes_live_execution():
    p=replace(program(),hypothetical_lift=True);clock=SimClock()
    session=CarrySession(p,NS(ready=lambda:True),None,lambda:feedback(p,clock),clock)
    with pytest.raises(ValueError,match='hypothetical'): session.start('live')
    assert manifest(p)['motion_authorized'] is False


def test_planning_delay_uses_separate_scene_revalidation_without_restamping_exposure():
    p=program();clock=SimClock()
    f=feedback(p,clock,capture_observed_s=1.,scene_validated_s=clock())
    session=CarrySession(p,NS(ready=lambda:True),None,lambda:f,clock)
    session.start('revalidated')
    assert f.capture_observed_s==1.
    assert session.state=='RUNNING'


def test_simulation_flag_cannot_bypass_live_transport_gates():
    p=program();clock=SimClock()
    with pytest.raises(ValueError,match='in-memory'):
        CarrySession(p,NS(adapter=NS(simulation_transport=False)),None,
                     lambda:feedback(p,clock),clock,simulation=True)


def test_route_rechecks_anchor_and_does_not_dispatch_reordered_stage():
    p=program(); calls=[]
    builder=CarryRouteBuilder(p,lambda stage,snapshot:calls.append(stage) or True)
    snap=NS(positions_urad=tuple(round(v*1e6) for v in p.stages[0].points[0].positions_rad))
    with pytest.raises(ValueError,match='reordered'): builder({'carry_stage':1},snap)
    snap.positions_urad=(100000,)+snap.positions_urad[1:]
    with pytest.raises(ValueError,match='anchor'): builder({'carry_stage':0},snap)
    assert not calls


def test_current_scene_validator_can_reject_even_a_compiled_route():
    p=program(); builder=CarryRouteBuilder(p,lambda stage,snapshot:False)
    snap=NS(positions_urad=tuple(round(v*1e6) for v in p.stages[0].points[0].positions_rad))
    with pytest.raises(ValueError,match='scene'): builder({'carry_stage':0},snap)


def test_stationary_adapter_defaults_to_no_motion_and_binds_exact_firmware():
    clock=SimClock(); transport=SimTransport(clock,(0.,)*12)
    path=ROOT/'config/home_assistant/arm_ranges.development.json'
    with pytest.raises(ValueError,match='development'):
        StationaryPickAdapter(transport,path,motion_authorized=True)
    adapter=StationaryPickAdapter(transport,path)
    with pytest.raises(Exception,match='identity'): adapter.prepare()
    assert 'enable' not in transport.calls


def compiler_fixture():
    profile=json.loads((ROOT/'config/home_assistant/s2_coordinates.development.json').read_text())
    from test_installed_jaw import synthetic_jaw
    jaw=synthetic_jaw(profile)
    closure=dict(status='BOUNDED_CLOSE',entry_gap_m=.05,visible_span_m=.035,
                 target_gap_m=.033,min_search_gap_m=.033,upper_contact_gap_m=.037)
    initial={side+'_model_'+n:profile['arms'][side]['alignment'][n]['home_model_rad']
             for side in ('left','right') for n in JOINTS}
    initial['s2_lift_joint']=.3
    q=np.array([initial['left_model_'+n] for n in JOINTS[:5]])
    lifted=q.copy();lifted[2]+=.2
    fk=lambda row:(np.array([[1,0,0,0],[0,1,0,0],[0,0,1,float(row[2])*.1],[0,0,0,1]]),None)
    stages=[dict(label=name,joints=np.asarray(path).tolist(),planner='fixture',duration_s=0)
            for name,path in [('open_geometry',[q]),('pregrasp',[q]),('approach',[q]),
                              ('close_geometry',[q]),('lift',[q,lifted]),('carry',[lifted])]]
    report=dict(status='OFFLINE_ROUTE_FOUND',initial_state_valid=True,
                capture_id='fixture',source_sha256={'depth':'d'},prepared_sha256='p',
                calibration_id=profile['calibration_id'],initial_model_joint_positions_si=initial,
                captured_model_joint_positions_si=dict(initial),hypothetical_lift_m=None,
                plan=dict(candidate_id='g',stages=stages),
                candidates=[dict(candidate={'candidate_id':'g'},source='synthetic_fixture',
                                 jaw_preview=dict(closure=closure,measured_gap_preview=jaw.preview(closure)))])
    return report,profile,jaw,NS(side='left',lift_m=.3,fk=fk,lo=np.full(5,-4.),hi=np.full(5,4.))


def test_exact_stationary_identity_can_prepare_without_authorizing_motion():
    clock=SimClock();transport=SimTransport(clock,(0.,)*12)
    path=ROOT/'config/home_assistant/arm_ranges.development.json'
    adapter=StationaryPickAdapter(transport,path)
    original=transport.enter_binary_mode
    def hello():
        result=original()
        result.firmware_version=adapter.firmware_version
        result.left_calibration_hash=result.right_calibration_hash=adapter.calibration_hash
        return result
    transport.enter_binary_mode=hello
    adapter.prepare()
    with pytest.raises(Exception,match='motion_authorized'):
        adapter.start('test',program().stages[0].points,finite=True)
    assert adapter.state.value=='ready'
    assert 'arm' not in transport.calls and 'enable' not in transport.calls


@pytest.mark.parametrize('fault',[None,'write_pending','stale','settle','dispatch_fault'])
def test_stationary_terminal_with_continuous_reads_preserves_freshness_and_fault_checks(fault):
    from dataclasses import replace
    from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
    clock=SimClock();transport=SimTransport(clock,(0.,)*12)
    path=ROOT/'config/home_assistant/arm_ranges.development.json'
    unarmed=StationaryPickAdapter(transport,path)
    adapter=StationaryPickAdapter(transport,path,authorized_limits=replace(unarmed._limits,hardware_authorized=True),motion_authorized=True)
    hello=transport.enter_binary_mode
    def identity():
        result=hello();result.firmware_version=adapter.firmware_version
        result.left_calibration_hash=result.right_calibration_hash=adapter.calibration_hash
        return result
    transport.enter_binary_mode=identity
    transport.stationary_platform=lambda op:None
    adapter.prepare();route=tuple(TimedJointPoint(t,(0.,)*12) for t in (80,130))
    adapter.start('test',route,finite=True);clock.advance(.2)
    transport.get_dispatch_diagnostics=lambda:NS(status_code=0,active=fault=='write_pending',ready=False,
                                                faulted=fault=='dispatch_fault',failure_count=0)
    transport.get_tracking_diagnostics=lambda:NS(status_code=0,active=True,pending=True,failed_pairs=0)
    if fault!='dispatch_fault':
        adapter.poll('test')
        assert adapter.state.value=='active'  # MCU terminal settling is still allowed to finish.
        # The firmware's readiness is authoritative even after a long wait.
        actual_runtime=transport.stationary_runtime
        transport.stationary_runtime=lambda:dict(stop_latched=False,ready_for_stream=False)
        clock.advance(2.)
        adapter.poll('test');assert adapter.state.value=='active'
        transport.stationary_runtime=actual_runtime
    if fault=='stale':transport.fault='stale'
    if fault=='settle':
        original=transport.get_feedback_snapshot
        def outside():
            result=original();result.positions_urad=(100000,)+(0,)*11;return result
        transport.get_feedback_snapshot=outside
    if fault in ('stale','settle','dispatch_fault'):
        with pytest.raises(Exception):adapter.poll('test')
        assert adapter.state.value=='faulted' and 'safe_stop' in transport.calls
    else:
        adapter.poll('test')
        assert adapter.state.value==('active' if fault=='write_pending' else 'ready')
        assert 'safe_stop' not in transport.calls
        if fault is None:
            adapter.start('test',route,finite=True)
            assert transport.calls.count('enable')==1


def test_compiler_converts_model_signs_preserves_other_arm_and_splits_probe():
    args=compiler_fixture();p=compile_program(*args,bounds=bounds())
    assert [s.name for s in p.stages] == ['open','pregrasp','approach','close','probe','lift','carry','hold']
    assert p.close_raw < args[2].raw_at_least(35.)
    for s in p.stages:
        assert np.max(abs(np.array([x.positions_rad[6:] for x in s.points]))) < 1e-12
    # Installed elbow sign is inverted: increasing model angle decreases controller angle.
    assert p.stages[4].points[-1].positions_rad[2] == pytest.approx(-.04)
    assert p.stages[5].points[-1].positions_rad[2] == pytest.approx(-.2)
    assert all(x.positions_rad[5] == p.close_control_rad for s in p.stages[4:] for x in s.points)


def test_compiler_keeps_explicit_stock_width_basis_and_rejects_mixed_preview():
    from test_installed_jaw import stock_conversion
    report,profile,measured,arm=compiler_fixture()
    stock,_=stock_conversion()
    preview=report['candidates'][0]['jaw_preview']
    old=preview.pop('measured_gap_preview')
    preview['nominal_gap_preview']=stock.preview(preview['closure'])
    p=compile_program(report,profile,stock,arm,bounds=bounds())
    assert p.close_raw==stock.raw_at_least(33.)
    assert p.contact_raw_interval==(p.close_raw,stock.contact_envelope(preview['closure'])['upper_contact_raw'])
    assert p.close_raw!=measured.raw_at_least(33.)
    preview['nominal_gap_preview']=old
    with pytest.raises(ValueError,match='jaw calibration changed'):
        compile_program(report,profile,stock,arm,bounds=bounds())


@pytest.mark.parametrize('fault',['reorder','gap','jaw','lift'])
def test_compiler_rejects_stale_or_incomplete_plan(fault):
    report,profile,jaw,arm=compiler_fixture()
    if fault=='reorder':report['plan']['stages'].reverse()
    if fault=='gap':report['plan']['stages'][1]['joints']=[np.ones(5)]
    if fault=='jaw':report['candidates'][0]['jaw_preview']['measured_gap_preview']['entry_raw']+=1
    if fault=='lift':arm.lift_m=.31
    with pytest.raises(ValueError):compile_program(report,profile,jaw,arm,bounds=bounds())
