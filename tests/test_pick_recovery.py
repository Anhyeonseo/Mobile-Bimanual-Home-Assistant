"""No-route evidence must not turn controller/data faults into base movement."""
import json
from types import SimpleNamespace

import pytest

from home_robot_tasks.pick_recovery import planning_recovery, IK_PREFIX
from home_robot_tasks.pick_service import PlanningRejected, Rejected
from home_robot_tasks.pick_backend import PipelineBackend
from test_pick_service import RecordingBackend, start, wait_for


def no_route():
    return dict(status='NO_COMPLETE_ROUTE', capture_id='capture-one', initial_state_valid=True,
                binding_failures=[dict(reason=IK_PREFIX+' 0.2 mm / 2 deg residual')],
                candidates=[], plan={'failures': []})


def test_geometric_rejection_proposes_review_not_motion_or_old_capture_reuse():
    outcome = planning_recovery(no_route())
    assert outcome['action'] == 'REPOSITION_BASE_AND_REOBSERVE'
    assert outcome['base_reposition_candidate']
    assert not outcome['motion_authorized'] and not outcome['automatic_base_motion']
    assert outcome['requires_new_capture_after_move']
    assert not outcome['reuse_previous_plan_after_move']
    assert outcome['retry_sequence'] == ['confirm_base_stopped', 'capture', 'detect',
                                        'candidates', 'plan', 'execute', 'verify']


@pytest.mark.parametrize('reason', ['CANDIDATE_FRAME_MISMATCH', 'CANDIDATE_IDENTITY_MISMATCH',
    'payload, grasp and robot must share the capture and fixed lift', 'unknown defect'])
def test_one_metadata_failure_blocks_relocation_even_with_many_ik_failures(reason):
    report = no_route()
    report['binding_failures'].append(dict(reason=reason))
    assert planning_recovery(report)['action'] == 'CHECK_PLANNING_DIAGNOSTICS'
    assert not planning_recovery(report)['base_reposition_candidate']


@pytest.mark.parametrize('patch', [dict(status='PLANNER_ERROR'), dict(initial_state_valid=False),
    dict(initial_state_valid=None), dict(plan={'failures':[dict(stage='pregrasp', reason='planning scene update failed')]}),
    dict(plan={'failures':[dict(stage='binding', reason='STALE_SCENE_OR_CALIBRATION')]}),
    dict(plan={'failures':[dict(stage='pregrasp', reason='OMPL error -6')]}),
    dict(status='OFFLINE_ROUTE_FOUND')])
def test_fault_or_success_is_not_relocation(patch):
    report = dict(no_route(), **patch)
    assert not planning_recovery(report)['base_reposition_candidate']


@pytest.mark.parametrize('reason', ['CANDIDATE_OFF_TARGET', 'OUTSIDE_NOMINAL_STL_RANGE',
                                  'closing direction cannot slide along support'])
def test_object_geometry_rejection_alone_is_not_reachability_failure(reason):
    report = no_route();report['binding_failures']=[dict(reason=reason)]
    assert planning_recovery(report)['action'] == 'REVIEW_GRASP_GEOMETRY'


def test_unexamined_bound_candidate_must_be_searched_before_relocation():
    report = no_route()
    report['candidates'] = [dict(candidate={'candidate_id':i}) for i in ('a','b')]
    report['plan']['failures'] = [dict(candidate='a', stage='pregrasp', reason='OMPL error -1')]
    assert planning_recovery(report)['action'] == 'CONTINUE_CANDIDATE_SEARCH'
    report['plan']['failures'].append(dict(candidate='b', stage='pregrasp', reason='OMPL error -1'))
    assert planning_recovery(report)['base_reposition_candidate']


@pytest.mark.parametrize('native_failure', [False, True])
def test_backend_uses_current_saved_failure_not_exception_text(tmp_path, native_failure):
    report = no_route()
    if native_failure: report['status']='OFFLINE_ROUTE_FOUND'
    path=tmp_path/'checked/planning/report.json';path.parent.mkdir(parents=True)
    path.write_text(json.dumps(report))
    backend=PipelineBackend({'mode':'replay'})
    def fail(ctx, method, **kwargs):
        if method=='planning_failure':
            from tools.lib.pick_compute import PickCompute
            return PickCompute.planning_failure(None,**kwargs)
        raise RuntimeError('no executable route: test rejection')
    backend._compute=fail
    ctx=SimpleNamespace(folder=tmp_path, artifact=lambda *a:None, note=lambda *a:None)
    with pytest.raises(Rejected) as error:
        backend.plan(ctx, dict(prepared='prepared', snapshot='snapshot', detections='detections'))
    assert isinstance(error.value, PlanningRejected) != native_failure


@pytest.mark.parametrize('stop_fails', [False, True])
def test_station_records_recovery_skips_execution_and_restarts_with_fresh_capture(tmp_path, stop_fails):
    class Backend(RecordingBackend):
        def plan(self, ctx, value):
            self.calls.append('plan')
            raise PlanningRejected('no executable route: NO_COMPLETE_ROUTE', planning_recovery(no_route()))
        def abort(self):
            super().abort()
            if stop_fails:raise RuntimeError('stop not confirmed')
    backend=Backend();station=start(tmp_path,backend)
    try:
        identifier=station.submit('지우개 집어줘')
        wait_for(lambda:station.snapshot()['history'])
        result=station.snapshot()['last_result']
        assert result['status']==('STOP_UNCONFIRMED' if stop_fails else 'REPOSITION_REVIEW_REQUIRED')
        assert not result['physical_success']
        assert result['recovery']['capture_id']=='capture-one'
        assert 'execute' not in backend.calls
        saved=json.loads((tmp_path/identifier/'result.json').read_text())
        assert saved['result']==result
        assert all(s['status']=='skipped' for s in saved['stages'] if s['id'] in ('EXECUTING','VERIFYING'))
        if not stop_fails:
            wait_for(lambda:station.snapshot()['accepting'])
            station.submit('지우개 집어줘')
            wait_for(lambda:len(station.snapshot()['history'])==2)
            assert backend.calls==['prepare']+['capture','detect','candidates','plan']*2
    finally:station.close()
