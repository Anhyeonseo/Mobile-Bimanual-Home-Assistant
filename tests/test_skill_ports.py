from home_robot_tasks.skill_ports import RoutedSkillAdapter, PortResult
from home_robot_tasks.execution import Feedback


class Port:
    def __init__(self):
        self.state = "RUNNING"
        self.cancelled = False

    def ready(self):
        return True

    def start(self, g, p):
        self.goal = g

    def poll(self, g):
        return PortResult(self.state)

    def cancel(self, g):
        self.cancelled = True


class Stop:
    def request(self, *args):
        pass

    def poll(self, g, now):
        return Feedback(
            g,
            now,
            "STOPPED",
            {"base_stopped": True, "lift_stopped": True, "arms_holding": True},
        )


def test_action_success_does_not_create_evidence_and_late_goal_blocks_stop():
    port = Port()
    monitor = lambda g, s, t: Feedback(g, t, "READY", {"grasp_verified": False})
    adapter = RoutedSkillAdapter({"pick": port}, monitor, Stop(), mode="simulation")
    step = {"skill": "pick", "parameters": {}}
    assert not adapter.observe("g", step, 0).conditions["grasp_verified"]
    adapter.start("g", step, 0)
    adapter.request_stop("run", True, 0.1)
    assert port.cancelled and adapter.poll_stop("run", 0.1).status == "RUNNING"
    port.state = "CANCELLED"
    assert adapter.poll_stop("run", 0.2).status == "STOPPED"


def test_success_waits_for_independent_evidence_and_remains_cancellable():
    port = Port()
    evidence = {"base_stopped": False}
    adapter = RoutedSkillAdapter(
        {"navigate_to": port},
        lambda g, s, t: Feedback(g, t, "READY", dict(evidence)),
        Stop(),
        mode="simulation",
    )
    step = {
        "skill": "navigate_to",
        "parameters": {},
        "success_requires": ("base_stopped",),
    }
    adapter.start("g", step, 0)
    port.state = "SUCCEEDED"
    result = adapter.poll("g", step, 0.1)
    assert result.status == "RUNNING"
    assert not result.conditions["base_stopped"]
    assert adapter.active is not None
    adapter.request_stop("run", False, 0.2)
    assert port.cancelled
    assert adapter.poll_stop("run", 0.3).status == "STOPPED"


def test_success_releases_port_only_after_evidence_arrives():
    port = Port()
    stopped = [False]
    adapter = RoutedSkillAdapter(
        {"navigate_to": port},
        lambda g, s, t: Feedback(g, t, "READY", {"base_stopped": stopped[0]}),
        Stop(),
        mode="simulation",
    )
    step = {
        "skill": "navigate_to",
        "parameters": {},
        "success_requires": ("base_stopped",),
    }
    adapter.start("g", step, 0)
    port.state = "SUCCEEDED"
    assert adapter.poll("g", step, 0.1).status == "RUNNING"
    stopped[0] = True
    assert adapter.poll("g", step, 0.2).status == "SUCCEEDED"
    assert adapter.active is None


def test_cancel_exception_still_dispatches_whole_stop_and_retains_active_child():
    import pytest
    port = Port()
    def broken_cancel(g):
        raise RuntimeError('lost cancel ACK')
    port.cancel = broken_cancel
    stop = Stop(); calls = []
    stop.request = lambda *args: calls.append(args)
    adapter = RoutedSkillAdapter({'pick': port}, lambda *a: None, stop, mode='simulation')
    adapter.start('run/pick', {'skill': 'pick', 'parameters': {}}, 0)
    with pytest.raises(RuntimeError): adapter.request_stop('run', True, .1)
    assert len(calls) == 1 and adapter.active is not None
    assert adapter.poll_stop('run', .2).status == 'RUNNING'
    port.state = 'CANCELLED'
    assert adapter.poll_stop('run', .3).status == 'STOPPED'
