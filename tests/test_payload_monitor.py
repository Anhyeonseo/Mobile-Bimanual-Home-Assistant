from dataclasses import replace
import pytest
from home_robot_tasks.payload_monitor import GripMeasurement, PayloadObservation, PayloadMonitor


def monitor():
    return PayloadMonitor(gap_min_mm=5, gap_max_mm=30, open_gap_mm=40,
        effort_min_raw=20, effort_max_raw=100, empty_effort_max_raw=5,
        max_age_s=0.25, max_skew_s=0.05, dwell_s=0.1)


def sample(m, seq, t, *, held=True, destination=None, present=False):
    m.observe_grip(GripMeasurement(seq, t, 15 if held else 45, 40 if held else 0, True), t)
    m.observe_vision(PayloadObservation(seq, t, "remote", "held" if held else "empty", destination, present), t)
    return m.proof(t)


def test_commands_or_polling_do_not_establish_grasp_or_empty_state():
    m = monitor(); m.expect("grasp", "remote", 0)
    assert not m.proof(0).conditions["load_retained"]
    assert not sample(m, 1, 0).conditions["grasp_verified"]
    assert not m.proof(0.15).conditions["grasp_verified"]
    assert sample(m, 2, 0.15).conditions["grasp_verified"]
    assert not m.proof(0.41).conditions["load_retained"]
    assert m.carrying  # Missing evidence never silently erases the carried-object obligation.


def test_motor_effort_alone_or_observation_gap_is_not_hold_proof():
    m = monitor(); m.expect("grasp", "remote", 0)
    sample(m, 1, 0)
    m.observe_grip(GripMeasurement(2, .15, 15, 40, True), .15)
    assert not m.proof(.15).conditions["grasp_verified"]
    assert not sample(m, 3, 1.0).conditions["grasp_verified"]  # Unobserved gap restarts dwell.
    assert sample(m, 4, 1.15).conditions["load_retained"]


def test_release_requires_new_empty_measurements_and_the_registered_destination():
    m = monitor(); m.expect("grasp", "remote", 0)
    sample(m, 1, 0); assert sample(m, 2, .15).conditions["load_retained"]
    with pytest.raises(ValueError): m.expect("empty", "remote", .16)
    m.expect("release", "remote", .2, destination="bed")
    assert not m.proof(.2).conditions["release_verified"]
    assert not sample(m, 3, .21, held=False, destination="sofa", present=True).conditions["release_verified"]
    assert not sample(m, 4, .25, held=False, destination="bed", present=True).conditions["release_verified"]
    result = sample(m, 5, .4, held=False, destination="bed", present=True)
    assert result.conditions["release_verified"] and result.conditions["payload_safe"] and not result.conditions["load_retained"]
    assert not m.carrying


@pytest.mark.parametrize("change", [{"sequence": 1}, {"observed_s": .3}, {"gap_mm": float("nan")}, {"healthy": 1}, {"effort_raw": -1}])
def test_bad_or_replayed_measurement_revokes_proof(change):
    m = monitor(); m.expect("grasp", "remote", 0)
    sample(m, 1, 0); sample(m, 2, .15)
    with pytest.raises(ValueError): m.observe_grip(replace(GripMeasurement(3,.2,15,40,True), **change), .2)
    assert not m.proof(.2).conditions["load_retained"]


def test_empty_gripper_and_wrong_object_are_distinct():
    m = monitor(); m.expect("empty", "remote", 0)
    sample(m, 1, 0, held=False)
    assert sample(m, 2, .15, held=False).conditions["payload_safe"]
    assert not m.proof(.15).conditions["load_retained"]
    m.expect("grasp", "remote", .2)
    m.observe_grip(GripMeasurement(3,.21,15,40,True),.21)
    m.observe_vision(PayloadObservation(3,.21,"other","held"),.21)
    assert not m.proof(.21).conditions["grasp_verified"]


@pytest.mark.parametrize('source', ['grip', 'vision'])
def test_contradiction_between_task_polls_restarts_payload_dwell(source):
    m = monitor(); m.expect('grasp', 'remote', 0)
    sample(m, 1, 0); assert sample(m, 2, .15).conditions['load_retained']
    if source == 'grip':
        m.observe_grip(GripMeasurement(3, .17, 45, 0, True), .17)
    else:
        m.observe_vision(PayloadObservation(3, .17, 'remote', 'unknown'), .17)
    # No proof() call between the contradictory and good sensor messages.
    assert not sample(m, 4, .2).conditions['load_retained']
    assert m.carrying
    assert sample(m, 5, .35).conditions['load_retained']
