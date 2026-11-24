"""Web/replay parity at the actual planner boundary; no hardware access."""
import json
from types import SimpleNamespace

import pytest

from home_robot_tasks.pick_calibration import TOP_BASIS
from tools.lib.pick_compute import PickCompute
from tools.lib.pick_planning import station_plan_options, save_plan_options


@pytest.mark.parametrize('review,start,expected', [
    (TOP_BASIS, None, 'vertical'),
    ('bottom', None, 'tool_axis'),
    ('bottom', TOP_BASIS, 'vertical'),
    (TOP_BASIS, 'bottom', 'tool_axis'),
])
def test_web_and_saved_capture_use_identical_geometry_and_arrival_basis(
        tmp_path, monkeypatch, review, start, expected):
    from tools.diagnostics import plan_capture_carry

    data = {'review': {'lift_reference_basis': review}}
    if start is not None:
        data['start_revalidation'] = {'lift_reference_basis': start}
    prepared = tmp_path/'prepared.json'
    original = json.dumps(data)
    prepared.write_text(original)
    output = tmp_path/'checked'
    meshes = tmp_path/'meshes'
    # Stop exactly at the planner boundary. Nothing downstream may be compiled
    # or sent when the planner refuses this capture.
    calls = []
    def rejected(args, **kwargs):
        calls.append(vars(args))
        raise ValueError('fixture scene cannot be planned')
    monkeypatch.setattr(plan_capture_carry, 'plan', rejected)
    runtime = PickCompute({'mobile_mesh_dir': str(meshes)})
    with pytest.raises(ValueError, match='no executable route: fixture scene'):
        runtime.plan(prepared, None, None, output)
    replay = station_plan_options(prepared, output/'planning', meshes)
    assert calls == [vars(replay)]
    assert replay.approach_mode == expected
    assert replay.nominal_jaw and replay.exhaust_candidates
    assert replay.entry_clearance_mm-replay.contact_clearance_mm == 4
    assert replay.contact_clearance_mm == 6 and replay.close_extra_raw == 50
    assert prepared.read_text() == original
    saved = json.loads((output/'planning-options.json').read_text())
    assert saved['approach_mode'] == expected
    assert saved['prepared'] == str(prepared)
    assert saved['entry_clearance_mm'] == 10


@pytest.mark.parametrize('budget', [0, 33, True, 1.5])
def test_invalid_search_budget_rejected_before_reading_capture(tmp_path, budget):
    with pytest.raises(ValueError, match='maximum_grasps'):
        station_plan_options(tmp_path/'absent.json', tmp_path, tmp_path,
                             maximum_grasps=budget)


def test_settings_record_rejects_nonfinite_geometry(tmp_path):
    with pytest.raises(ValueError):
        save_plan_options(SimpleNamespace(entry_clearance_mm=float('nan')),
                          tmp_path/'options.json')


def test_all_binding_rejections_reach_web_error_before_execution(tmp_path, monkeypatch):
    from tools.diagnostics import plan_capture_carry
    from tools.diagnostics import prepare_carry_execution
    prepared = tmp_path/'prepared.json'
    prepared.write_text(json.dumps({'review': {'lift_reference_basis': TOP_BASIS}}))
    report = dict(status='NO_COMPLETE_ROUTE', candidates=[], binding_failures=[
        dict(reason='CANDIDATE_FRAME_MISMATCH', diagnostics=None) for _ in range(64)],
        plan={'failures': []})
    monkeypatch.setattr(plan_capture_carry, 'plan', lambda *a, **kw: report)
    def forbidden(*args, **kwargs):
        pytest.fail('failed planning must not build execution inputs')
    monkeypatch.setattr(prepare_carry_execution, 'load_inputs', forbidden)
    runtime = PickCompute({'mobile_mesh_dir': str(tmp_path/'meshes')})
    with pytest.raises(ValueError) as error:
        runtime.plan(prepared, None, None, tmp_path/'checked')
    assert 'CANDIDATE_FRAME_MISMATCH: 64회' in str(error.value)
    assert '경로 검사 입력 후보 0개' in str(error.value)


def test_ik_diagnostics_do_not_combine_different_candidate_residuals():
    from tools.lib.pick_planning import planning_failure_message
    message = planning_failure_message(dict(status='NO_COMPLETE_ROUTE', candidates=[],
        binding_failures=[dict(reason='pregrasp IK failed bounded multi-seed search: residual',
                              diagnostics=dict(position_error_m=p, orientation_error_deg=o))
                          for p, o in ((.0001, 20.), (.002, 1.))]))
    assert '사전 접근 역기구학(IK) 조건 미충족: 2회' in message
    assert '위치 2.0 mm, 자세 1.0°' in message
    assert '위치 0.1 mm, 자세 1.0°' not in message


def test_initial_collision_and_route_stage_evidence_still_visible():
    from tools.lib.pick_planning import planning_failure_message
    message = planning_failure_message(dict(status='NO_COMPLETE_ROUTE', candidates=[{}],
        reason='captured state collision', binding_failures=[], plan={'failures': [
            dict(stage='approach', diagnostics={'violations': ['orientation']})]}))
    assert 'captured state collision' in message
    assert '접근 1회' in message
    assert '접근 자세 조건 1개' in message
