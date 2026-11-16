from dataclasses import replace
from types import SimpleNamespace as NS
import pytest
from home_robot_tasks.unified_carry_inputs import UnifiedPlatformGuard
from home_robot_tasks.carry_live_inputs import ObservationNotFresh
from so101_arm_bridge.lift_wire import LiftStatus


def samples():
    return (NS(boot_id=4,state=0,session=0,reason=0,flags=17,mcu_tick_ms=1000,feedback_tick_ms=950),
            LiftStatus(4,1,1000,4,1,1,5,0,29,960,8059,8000,0,40,3700,0,0))


def test_integrated_hold_uses_real_disabled_base_and_lift_interlock():
    m,l=samples();g=UnifiedPlatformGuard(4,.008059)
    g.update(m,l,20.,20.01);g.require_fresh(20.02)
    assert g.current_height==.008059 and g.stamp==19.95 and g.base_stopped


@pytest.mark.parametrize('changes',[{'state':1},{'session':1},{'flags':49},{'boot_id':5},{'reason':1}])
def test_active_base_transport_reset_or_fault_cannot_pass(changes):
    m,l=samples();m.__dict__.update(changes)
    with pytest.raises(ValueError):UnifiedPlatformGuard(4,.008059).update(m,l,20,20.01)


@pytest.mark.parametrize('changes',[{'flags':25},{'flags':31},{'command_raw':1},
    {'height_um':15000},{'state':4},{'fault':1},{'boot_id':5}])
def test_missing_stop_proof_moving_lift_or_changed_height_rejected(changes):
    m,l=samples()
    with pytest.raises(ValueError):UnifiedPlatformGuard(4,.008059).update(m,replace(l,**changes),20,20.01)


def test_rejected_lift_packet_is_preserved_instead_of_previous_healthy_sample():
    m,l=samples();g=UnifiedPlatformGuard(4,.008059)
    g.update(m,l,20,20.01)
    assert g.last_observation['accepted']
    with pytest.raises(ValueError):g.update(m,replace(l,flags=25),20.1,20.11)
    assert not g.last_observation['accepted']
    assert g.last_observation['lift']['flags']==25
    assert g.last_observation['mobile']['state']==0
    assert g.last_observation['sent_s']==20.1


def test_delay_not_relabelled_as_fresh_and_failed_query_invalidates_guard():
    m,l=samples();g=UnifiedPlatformGuard(4,.008059);g.update(m,l,20,20.01)
    with pytest.raises(ObservationNotFresh):g.update(m,l,20.1,20.4)
    with pytest.raises(ValueError):g.require_fresh(20.04)


def test_wraparound_age_is_preserved():
    m,l=samples();m.mcu_tick_ms=10;m.feedback_tick_ms=2**32-40
    g=UnifiedPlatformGuard(4,.008059);g.update(m,replace(l,mcu_tick_ms=10,observed_ms=2**32-30),20,20.01)
    assert g.stamp==19.95


def test_native_surface_plane_is_explicit_for_a_mesh_support():
    import numpy as np
    from tools.diagnostics.prepare_carry_execution import audit_native_surfaces
    from home_robot_tasks.carry_scene import SceneBox
    from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
    from home_robot_tasks.carry_program import CarryStage
    class Jaw:
        def plane_clearance(self, pose, angle, support):
            return {'fixed':{'clearance_m':support[2,3]},'moving':{'clearance_m':support[2,3]}}
    arm=NS(jaw=Jaw(),tcp_from_stock=np.eye(4),fk=lambda q:(np.eye(4),None))
    import json
    from pathlib import Path
    profile=json.loads((Path(__file__).resolve().parents[1]/'config/home_assistant/s2_coordinates.development.json').read_text())
    stage=CarryStage('close',(TimedJointPoint(100,(0.,)*12),),{},False)
    program=NS(stages=(stage,))
    mesh=SceneBox('table',np.eye(4),(.5,.5,.1),'support',{'vertices':[],'faces':[]})
    with pytest.raises(ValueError,match='explicitly observed'):
        audit_native_surfaces(program,profile,arm,[mesh])
    result=audit_native_surfaces(program,profile,arm,[mesh],support_plane=[0,0,2,-.6])
    assert result['base_from_support'][2][3]==.3
    with pytest.raises(ValueError,match='upward'):
        audit_native_surfaces(program,profile,arm,[mesh],support_plane=[1,0,0,-.3])


@pytest.mark.parametrize('failure', ['worker_start', 'invalid_bundle'])
def test_pick_preflight_failure_closes_workers_before_control(tmp_path, monkeypatch, failure):
    from concurrent.futures import Future
    from tools.run import unified_pick as pick

    closed = []

    class Worker:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            closed.append(self)

        def submit(self, function, *args, **kwargs):
            result = Future()
            if failure == 'worker_start':
                result.set_exception(RuntimeError('worker unavailable'))
            else:
                result.set_result(123)
            return result

    monkeypatch.setattr(pick, 'ThreadPoolExecutor', lambda **kwargs: Worker())
    monkeypatch.setattr(pick, 'ProcessPoolExecutor', lambda **kwargs: Worker())
    monkeypatch.setattr(pick.signal, 'signal', lambda *args: None)
    monkeypatch.setattr(pick, 'load_capture_profile', lambda *args: ({}, {}))
    monkeypatch.setattr(pick, 'acquire_backend_lease', lambda *args: pytest.fail('controller accessed before preflight'))
    args = NS(execute=True, supported_empty_bottom=True, lift_target_um=0,
              empty_return=False, output=tmp_path/'result', bundle=tmp_path/'missing')
    if failure == 'worker_start':
        with pytest.raises(RuntimeError, match='worker unavailable'):
            pick.run(args)
    else:
        assert pick.run(args) == 2
        import json
        report = json.loads((args.output/'state.json').read_text())
        assert report['status'] == 'REJECTED_BEFORE_CONTROL'
    assert len(closed) == 2
