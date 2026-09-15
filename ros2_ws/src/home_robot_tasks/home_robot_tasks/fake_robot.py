"""Deterministic fault-injectable adapter, with no device or network imports."""
from __future__ import annotations
from .execution import Feedback, invariants


class FakeRobot:
    mode = "simulation"

    def __init__(self, *, duration_s: float = 0.2, stop_delay_s: float = 0.1):
        from .replay import _seconds
        self.duration = _seconds(duration_s, "duration_s")
        self.stop_delay = _seconds(stop_delay_s, "stop_delay_s")
        self.active: tuple[str, float] | None = None
        self.stopping: tuple[str, float] | None = None
        self.fail_skill: str | None = None
        self.false_conditions: set[str] = set()
        self.stale_by_s = 0.0
        self.wrong_goal = False
        self.never_stop = False
        self.stop_done = False
        self.log: list[dict] = []

    def _feedback(self, goal_id, step, now_s, status):
        keys = set(step['start_requires']) | set(step['success_requires']) | set(invariants(step['skill']))
        return Feedback("wrong-goal" if self.wrong_goal else goal_id,
                        now_s - self.stale_by_s, status,
                        {key: key not in self.false_conditions for key in keys})

    def observe(self, goal_id, step, now_s):
        return self._feedback(goal_id, step, now_s, "READY")

    def start(self, goal_id, step, now_s):
        if self.active is not None or (self.stopping is not None and not self.stop_done):
            raise RuntimeError("fake device already owned")
        self.stopping, self.stop_done = None, False
        self.active = (goal_id, now_s)
        self.log.append({"kind": "start", "goal_id": goal_id, "skill": step['skill'], "at_s": now_s})

    def poll(self, goal_id, step, now_s):
        if self.active is None or self.active[0] != goal_id:
            raise RuntimeError("unknown fake goal")
        status = "SUCCEEDED" if now_s - self.active[1] >= self.duration else "RUNNING"
        if step['skill'] == self.fail_skill:
            status = "FAILED"
        feedback = self._feedback(goal_id, step, now_s, status)
        if status == "SUCCEEDED":
            self.active = None
        return feedback

    def request_stop(self, run_id, preserve_load, now_s):
        if self.stopping is None or (self.stop_done and self.stopping[0] != run_id):
            self.stop_done = False
            self.stopping = (run_id, now_s)
            self.log.append({"kind": "stop", "run_id": run_id, "preserve_load": preserve_load, "at_s": now_s})
        elif self.stopping[0] != run_id:
            raise RuntimeError("wrong stop owner")

    def poll_stop(self, run_id, now_s):
        if self.stopping is None or self.stopping[0] != run_id:
            raise RuntimeError("unknown stop")
        stopped = not self.never_stop and now_s - self.stopping[1] >= self.stop_delay
        feedback = Feedback(run_id, now_s, "STOPPED" if stopped else "RUNNING",
                            {key: stopped and key not in self.false_conditions for key in
                             ('base_stopped', 'lift_stopped', 'arms_holding', 'load_retained')})
        if stopped:
            self.active = None
            self.stop_done = True
        return feedback
