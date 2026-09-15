from dataclasses import replace
import pytest
from home_robot_tasks.manipulation_port import ManipulationPort, PlanningContext
from home_robot_tasks.skill_ports import PortResult


class Backend:
    def __init__(self):
        self.state = "RUNNING"
        self.starts = []
        self.cancels = []

    def ready(self):
        return True

    def start(self, g, p):
        self.starts.append((g, p))

    def poll(self, g):
        return PortResult(self.state)

    def result(self, g):
        return ("checked-trajectory",)

    def cancel(self, g):
        self.cancels.append(g)


def make():
    planner, executor = Backend(), Backend()
    now = [1.0]
    context = [
        PlanningContext(
            1.0,
            "capture",
            "pose",
            "scene",
            "calibration",
            "base_link",
            (0.0,) * 12,
            True,
            True,
        )
    ]
    port = ManipulationPort(planner, executor, lambda: context[0], lambda: now[0])
    port.start("pick", {"target": "remote_control"})
    return port, planner, executor, context, now


@pytest.mark.parametrize(
    "change",
    [
        {"scene_revision": "new"},
        {"pose_revision": "new"},
        {"calibration_id": "new"},
        {"anchor_rad": (0.2,) * 12},
        {"base_stopped": False},
        {"observed_s": 0.0},
    ],
)
def test_no_execution_of_plan_with_changed_or_stale_context(change):
    port, p, e, c, _ = make()
    p.state = "SUCCEEDED"
    c[0] = replace(c[0], **change)
    port.poll("pick")
    assert not e.starts
    assert port.poll("pick").state == "FAILED"


def test_plan_success_executes_but_cancellation_keeps_owner_until_child_ends():
    port, p, e, _, _ = make()
    p.state = "SUCCEEDED"
    assert port.poll("pick").state == "RUNNING" and e.starts
    port.cancel("pick")
    assert port.poll("pick").state == "RUNNING" and port.owner == "pick"
    e.state = "CANCELLED"
    assert port.poll("pick").state == "CANCELLED" and port.owner is None


def test_late_plan_after_cancel_cannot_execute():
    port, p, e, _, _ = make()
    port.cancel("pick")
    assert port.poll("pick").state == "RUNNING"
    p.state = "SUCCEEDED"
    assert port.poll("pick").state == "CANCELLED" and not e.starts
