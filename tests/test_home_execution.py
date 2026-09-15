import json
from pathlib import Path
import pytest
from home_robot_tasks.application import RobotApplication
from home_robot_tasks.execution import TaskExecutor, TaskLease, Feedback
from home_robot_tasks.fake_robot import FakeRobot
from home_robot_tasks.fetch import FetchRequest, InvalidTask, plan_fetch
from home_robot_tasks.navigation import NavigationMap

ROOT = Path(__file__).resolve().parents[1]


def read(name):
    return json.loads((ROOT / "config" / name).read_text())


def steps():
    return plan_fetch(
        FetchRequest.from_dict(read("fetch_remote.example.json")),
        read("home.example.json"),
    )["steps"]


def executor(robot=None):
    return TaskExecutor("test-run", steps(), robot or FakeRobot(), TaskLease())


def finish(task, start=0, count=300):
    for i in range(count):
        result = task.tick(start + i / 10)
        if result["status"] in task.TERMINAL:
            return result
    return result


def test_executes_all_fifteen_skills_through_adapter():
    robot = FakeRobot()
    task = executor(robot)
    result = finish(task)
    assert result["status"] == "SUCCEEDED" and result["completed_steps"] == 15
    assert result["simulated_skill_commands"] == 15 and len(robot.log) == 15
    assert not result["physical_task_completed"] and result["hardware_commands"] == 0
    assert not result["control_owned"]


@pytest.mark.parametrize("index", range(15))
def test_cancel_at_every_step_waits_for_stop_and_keeps_ownership(index):
    robot = FakeRobot(stop_delay_s=0.3)
    task = executor(robot)
    for i in range(300):
        now = i / 10
        task.tick(now)
        if task.index == index and task.state == "RUNNING":
            break
    result = task.cancel(now)
    assert result["status"] == "STOPPING" and result["control_owned"]
    assert not result["simulated_stop_confirmed"]
    assert task.tick(now + 0.4)["status"] == "CANCELLED"
    assert task.commands == index + 1 and task.lease.owner is None
    if index in range(6, 14):
        assert robot.log[-1]["preserve_load"]


def test_never_confirmed_stop_retains_lease_even_after_timeout():
    robot = FakeRobot()
    robot.never_stop = True
    task = executor(robot)
    task.tick(0)
    task.cancel(0.1)
    result = task.tick(2.2)
    assert result["status"] == "STOP_UNCONFIRMED" and result["control_owned"]
    with pytest.raises(InvalidTask, match="robot_busy"):
        TaskExecutor("other", steps(), FakeRobot(), task.lease, 2.2)
    robot.never_stop = False
    assert task.tick(2.3)["status"] == "CANCELLED"


@pytest.mark.parametrize(
    "fault", ["stale", "wrong_goal", "load_lost", "localization_lost", "failure"]
)
def test_running_fault_stops_instead_of_advancing(fault):
    robot = FakeRobot(duration_s=1)
    task = executor(robot)
    # Directly select a valid load-transport phase for continuous monitoring.
    task.index = 8
    task.carrying = True
    task.tick(0)
    if fault == "stale":
        robot.stale_by_s = 1
    if fault == "wrong_goal":
        robot.wrong_goal = True
    if fault == "load_lost":
        robot.false_conditions.add("load_retained")
    if fault == "localization_lost":
        robot.false_conditions.add("localized")
    if fault == "failure":
        robot.fail_skill = "navigate_with_load"
    result = task.tick(0.2)
    assert result["status"] == "STOPPING" and task.index == 8
    result = task.tick(0.4)
    # Lost load is not falsely acknowledged as preserved.
    assert result["status"] == ("STOPPING" if fault == "load_lost" else "FAILED")


def test_ready_conditions_wait_until_timeout_without_starting():
    robot = FakeRobot()
    robot.false_conditions.add("hardware_ready")
    task = executor(robot)
    assert task.tick(0)["status"] == "WAITING"
    assert task.tick(30)["status"] == "STOPPING"
    assert task.tick(30.2)["status"] == "FAILED"
    assert task.commands == 0


def test_adapter_start_exception_requests_stop():
    class Broken(FakeRobot):
        def start(self, *args):
            super().start(*args)
            raise RuntimeError("partial_dispatch")

    task = executor(Broken())
    result = task.tick(0)
    assert result["status"] == "STOPPING"
    assert task.tick(0.2)["status"] == "FAILED"


def test_clock_regression_rejected_and_physical_adapter_refused():
    task = executor()
    task.tick(0.2)
    with pytest.raises(InvalidTask):
        task.tick(0.1)
    assert task.now == 0.2
    robot = FakeRobot()
    robot.mode = "physical"
    with pytest.raises(InvalidTask):
        executor(robot)


def test_app_fetch_and_phone_navigation_share_control():
    app = RobotApplication(
        read("home.example.json"),
        NavigationMap(read("navigation_map.simulation.json")),
        FakeRobot(),
    )
    request = {
        "schema_version": 1,
        "operation": "fetch_object",
        "request_id": "fetch1",
        "request": read("fetch_remote.example.json"),
    }
    app.submit(request, 0)
    with pytest.raises(InvalidTask, match="robot_busy"):
        app.submit(read("navigate_to.simulation.json"), 0)
    for i in range(200):
        app.tick(i / 10)
    assert app.status("fetch1")["status"] == "SUCCEEDED"
    with pytest.raises(InvalidTask, match="localization_required"):
        app.submit(read("navigate_to.simulation.json"), 20)
    app.localize_simulated((1.5, 1.5), 20)
    assert app.submit(read("navigate_to.simulation.json"), 20)["status"] == "WAITING"


@pytest.mark.parametrize("budget", [0, -1, True, float("nan"), 3601])
def test_invalid_total_budget_does_not_acquire_control(budget):
    lease = TaskLease()
    with pytest.raises(InvalidTask):
        TaskExecutor("budget", steps(), FakeRobot(), lease, task_timeout_s=budget)
    assert lease.owner is None


def test_total_deadline_is_shared_across_steps_and_readiness_waits():
    robot = FakeRobot(duration_s=0.2)
    task = TaskExecutor("budget", steps(), robot, TaskLease(), task_timeout_s=0.5)
    task.tick(0)
    task.tick(0.2)
    robot.false_conditions.add("hardware_ready")
    assert task.tick(0.3)["status"] == "WAITING"
    result = task.tick(0.5)
    assert result["reason"] == "task_timeout" and result["control_owned"]
    assert task.commands == 1
    result = task.tick(0.7)
    assert result["status"] == "FAILED" and not result["control_owned"]
    assert result["history"][0]["execution_s"] == 0.2
    assert result["history"][0]["wait_s"] == 0
    assert task.tick(10)["elapsed_s"] == 0.7


def test_total_deadline_does_not_release_unconfirmed_loaded_stop():
    robot = FakeRobot()
    robot.never_stop = True
    task = TaskExecutor("budget", steps(), robot, TaskLease(), task_timeout_s=0.5)
    task.index, task.carrying = 8, True
    task.tick(0)
    result = task.tick(0.5)
    assert result["reason"] == "task_timeout"
    assert robot.log[-1]["preserve_load"]
    assert task.tick(3)["status"] == "STOP_UNCONFIRMED"
    assert task.lease.owner == "budget"
    robot.never_stop = False
    assert task.tick(3.1)["status"] == "FAILED"


def test_application_passes_operator_task_budget():
    app = RobotApplication(
        read("home.example.json"),
        NavigationMap(read("navigation_map.simulation.json")),
        FakeRobot(duration_s=5),
        task_timeout_s=0.5,
    )
    req = read("navigate_to.simulation.json")
    app.submit(req, 0)
    app.tick(0)
    assert app.tick(0.5)["reason"] == "task_timeout"
    assert app.tick(0.7)["task_timeout_s"] == 0.5
