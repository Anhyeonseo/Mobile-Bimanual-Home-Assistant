import copy
import json
from pathlib import Path

import pytest

from home_robot_tasks.fetch import FetchRequest, InvalidTask
from home_robot_tasks.replay import FetchReplay, replay_fetch
from home_robot_tasks.replay_cli import main

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def inputs():
    def read(name):
        return json.loads((ROOT / "config" / name).read_text())
    return (FetchRequest.from_dict(read("fetch_remote.example.json")),
            read("home.example.json"), read("fetch_remote.replay.example.json"))


def test_sofa_to_bed_fixture_requires_all_phases_without_claiming_delivery(inputs):
    request, world, recording = inputs
    result = replay_fetch(request, world, recording)
    assert result["status"] == "COMPLETED"
    assert result["completed_steps"] == 15
    assert result["current_step"] is None
    assert result["motion_commands"] == 0
    assert result["physical_task_completed"] is False
    assert result["stop_command_sent"] is False
    assert result["executable"] is False
    assert result["motion_authorized"] is False
    assert result["evidence_source"] == "synthetic"
    assert [e["event"]["step_id"] for e in result["history"] if e["event"]["kind"] == "success"] == [
        "01_prepare_navigation", "02_navigate", "03_search", "04_align_for_pick",
        "05_adjust_lift_for_pick", "06_reobserve_target", "07_pick", "08_prepare_transport",
        "09_navigate_with_load", "10_inspect_destination", "11_align_for_place",
        "12_adjust_lift_for_place", "13_reobserve_destination", "14_place", "15_verify_delivery",
    ]


def runner_before(inputs, index):
    request, world, recording = inputs
    runner = FetchReplay(request, world, recording["run_id"])
    for event in recording["events"][:index]:
        runner.accept(event)
    return runner, copy.deepcopy(recording["events"][index])


@pytest.mark.parametrize("index,field", [
    (2, "arm_in_transport_pose"), (2, "lift_in_transport_position"),
    (6, "base_stopped"), (6, "base_control_available"), (8, "lift_stopped"),
    (8, "base_stopped"), (8, "lift_homed"), (12, "fresh_target_pose"),
    (12, "collision_checked"), (13, "grasp_verified"),
    (16, "load_retained"), (24, "base_stopped"),
    (26, "fresh_work_surface"), (29, "object_at_destination"),
])
def test_missing_or_negative_evidence_stops_sequence(inputs, index, field):
    runner, event = runner_before(inputs, index)
    event["evidence"][field] = False
    result = runner.accept(event)
    assert result["status"] == "FAILED"
    assert result["reason"] == "unmet_conditions"
    assert result["required_response"] == "stop_base_stop_lift_preserve_load_then_report"
    assert not result["stop_command_sent"]
    with pytest.raises(InvalidTask, match="terminal"):
        runner.accept(event)


def test_omitted_condition_cannot_count_as_success(inputs):
    runner, event = runner_before(inputs, 13)
    del event["evidence"]["grasp_verified"]
    assert runner.accept(event)["reason"] == "unmet_conditions"


@pytest.mark.parametrize("stamp", [0.0, 8.0, 13.0])
def test_reobservation_rejects_pre_lift_old_and_future_evidence(inputs, stamp):
    runner, event = runner_before(inputs, 11)
    # The fixture completes the lift at t=10 and starts reobservation at t=11.
    event["evidence_at_s"] = stamp
    assert runner.accept(event)["reason"] == "stale_or_out_of_phase_evidence"


def test_success_requires_start_and_correct_step(inputs):
    runner, event = runner_before(inputs, 0)
    with pytest.raises(InvalidTask, match="started"):
        runner.accept(inputs[2]["events"][1])
    skipped = copy.deepcopy(inputs[2]["events"][2])
    with pytest.raises(InvalidTask, match="current step"):
        runner.accept(skipped)
    assert runner.result()["history"] == []
    assert runner.accept(event)["status"] == "RUNNING"


@pytest.mark.parametrize("changes", [
    {"run_id": "other-run"}, {"at_s": -1}, {"at_s": float("nan")},
    {"at_s": float("inf")}, {"at_s": True}, {"evidence_at_s": False},
    {"at_s": 10 ** 400},
    {"evidence": {"base_stopped": "true"}}, {"motion_authorized": True},
    {"evidence": {"motion_authorized": True}}, {"kind": []},
])
def test_invalid_event_is_atomic(inputs, changes):
    runner, event = runner_before(inputs, 0)
    previous = runner.result()
    event.update(changes)
    with pytest.raises(InvalidTask):
        runner.accept(event)
    assert runner.result() == previous


def test_duplicate_and_backwards_events_do_not_advance(inputs):
    runner, event = runner_before(inputs, 1)
    previous = runner.result()
    with pytest.raises(InvalidTask, match="duplicate"):
        runner.accept(inputs[2]["events"][0])
    event["at_s"] = 0
    with pytest.raises(InvalidTask, match="backwards"):
        runner.accept(event)
    assert runner.result() == previous


@pytest.mark.parametrize("kind,expected", [("cancel", "CANCELLED"), ("failure", "FAILED")])
def test_fault_or_cancel_with_load_terminates_without_retries(inputs, kind, expected):
    runner, _ = runner_before(inputs, 17)
    event = {"run_id": inputs[2]["run_id"], "event_id": "interrupt", "at_s": 17.5, "kind": kind}
    if kind == "failure":
        event.update(step_id="09_navigate_with_load", reason="localization_lost")
    result = runner.accept(event)
    assert result["status"] == expected
    assert result["completed_steps"] == 8
    assert "preserve_load" in result["required_response"]
    assert result["motion_commands"] == 0


@pytest.mark.parametrize("started", [False, True])
def test_deadline_includes_wait_for_preconditions_and_fires_without_result(inputs, started):
    runner, event = runner_before(inputs, 0)
    if started:
        event["at_s"] = event["evidence_at_s"] = 29
        runner.accept(event)
    result = runner.accept({"run_id": inputs[2]["run_id"], "event_id": "deadline",
                            "at_s": 30, "kind": "tick"})
    assert result["reason"] == "timeout"
    assert result["status"] == "FAILED"


def test_late_success_cannot_override_timeout(inputs):
    runner, event = runner_before(inputs, 1)
    event["at_s"] = event["evidence_at_s"] = 30
    assert runner.accept(event)["reason"] == "timeout"


def test_end_of_incomplete_recording_is_not_success(inputs):
    request, world, recording = inputs
    recording["events"] = recording["events"][:2]
    result = replay_fetch(request, world, recording)
    assert result["status"] == "WAITING"
    assert result["replay_completed"] is False


def test_public_snapshots_cannot_mutate_replay(inputs):
    runner, event = runner_before(inputs, 0)
    runner.current_step["timeout_s"] = 9999
    snapshot = runner.accept(event)
    event["evidence"]["hardware_ready"] = False
    snapshot["history"].clear()
    assert len(runner.result()["history"]) == 1
    assert runner.current_step["timeout_s"] == 30


def test_replay_cli_has_success_failure_and_invalid_exit_codes(inputs, tmp_path, capsys):
    recording_path = tmp_path / "recording.json"
    args = ["--world", str(ROOT / "config/home.example.json"),
            "--request", str(ROOT / "config/fetch_remote.example.json"),
            "--recording", str(recording_path)]
    recording_path.write_text(json.dumps(inputs[2]))
    assert main(args) == 0
    assert json.loads(capsys.readouterr().out)["status"] == "COMPLETED"
    inputs[2]["events"] = []
    recording_path.write_text(json.dumps(inputs[2]))
    assert main(args) == 1
    assert json.loads(capsys.readouterr().out)["status"] == "WAITING"
    recording_path.write_text("{")
    with pytest.raises(SystemExit) as error:
        main(args)
    assert error.value.code == 2
