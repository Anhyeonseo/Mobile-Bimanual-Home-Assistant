"""Regression checks for unchanged sample coverage and hard rejection paths."""
from types import SimpleNamespace
import numpy as np
import pytest
from home_robot_tasks.carry_moveit import MoveItCarryPlanner
from home_robot_tasks.carry_planning import PlanRejected
from tools.diagnostics.benchmark_planning_latency import summary


def backend(batch=None):
    p = object.__new__(MoveItCarryPlanner)
    p.transport = SimpleNamespace()
    if batch is not None: p.transport.check_states = batch
    p.timings = {}; p.checked_states = 0
    p.state = lambda q, c, a, jaw=None: (tuple(q), a, jaw)
    p.arm = SimpleNamespace(fk=lambda q: (np.eye(4), None))
    return p


def test_batch_preserves_all_states_jaw_angles_and_attachment():
    seen = []
    p = backend(lambda states: (seen.extend(states) or SimpleNamespace(valid=True, failed_index=-1)))
    samples = np.arange(15).reshape(3, 5)
    p.valid_many(samples, SimpleNamespace(object_constraint=None), True, jaws=[.1, .2, .3])
    assert seen == [(tuple(q), True, jaw) for q, jaw in zip(samples, [.1, .2, .3])]
    assert p.checked_states == 3


def test_baseline_retains_per_state_checks():
    p = backend(); seen = []
    p.valid = lambda q, c, a, jaw=None: seen.append((tuple(q), a, jaw))
    p.valid_many([[0]*5, [1]*5], None, False, jaws=[.1, .2])
    assert seen == [((0,)*5, False, .1), ((1,)*5, False, .2)]


def test_collision_rejects_batch():
    p = backend(lambda states: SimpleNamespace(valid=False, failed_index=1, reason='collision'))
    with pytest.raises(PlanRejected, match='state 1'):
        p.valid_many([[0]*5]*3, SimpleNamespace(object_constraint=None), False)
    assert p.checked_states == 2


def test_payload_orientation_rejected_before_native_call():
    p = backend(lambda states: pytest.fail('invalid orientation reached native checker'))
    candidate = SimpleNamespace(object_constraint=SimpleNamespace(satisfied=lambda pose: False))
    with pytest.raises(PlanRejected, match='orientation'):
        p.valid_many([[0]*5], candidate, True)


def test_transport_error_not_success_and_accounted():
    p = backend()
    def broken(name, request): raise RuntimeError('unavailable')
    p.transport = broken
    with pytest.raises(RuntimeError): p.call('plan_kinematic_path', None)
    assert p.timings['plan_kinematic_path']['calls'] == 1


def test_summary_never_reports_error_as_zero_latency():
    result = summary([dict(mode='parallel', report=dict(status='VALIDATION_ERROR'))])['parallel']
    assert result['median_first_complete_route_ms'] is None
    assert result['median_search_wall_ms'] is None
    assert result['successes'] == 0 and result['errors'] == 1


def test_unsuccessful_search_retained_in_wall_time():
    rows = [dict(mode='batch', report=dict(status='NO_COMPLETE_ROUTE', validation_ms=200.)),
            dict(mode='batch', report=dict(status='OFFLINE_ROUTE_FOUND', validation_ms=100.,
                                          first_feasible_validation_ms=90.))]
    result = summary(rows)['batch']
    assert result['median_search_wall_ms'] == 150.
    assert result['median_first_complete_route_ms'] == 90.
    assert result['successes'] == 1
