"""Clock-driven task execution with adapter calls and confirmed cancellation.

Current deployment is simulation-only. The adapter boundary is intended for
future ROS/hardware integration; physical adapters are deliberately not enabled.
Unlike replay, this module starts/polls/cancels skills and retains ownership
until completion or a fresh stop acknowledgement.
"""

from __future__ import annotations
from copy import deepcopy
from dataclasses import dataclass
from typing import Protocol

from .fetch import InvalidTask
from .replay import _seconds

DEFAULT_TASK_TIMEOUT_S = (
    300.0  # PC operating policy, not measured delivery performance.
)


@dataclass(frozen=True)
class Feedback:
    goal_id: str
    observed_s: float
    status: str
    conditions: dict[str, bool]
    reason: str = ""


class SkillAdapter(Protocol):
    mode: str

    def observe(self, goal_id: str, step: dict, now_s: float) -> Feedback: ...
    def start(self, goal_id: str, step: dict, now_s: float) -> None: ...
    def poll(self, goal_id: str, step: dict, now_s: float) -> Feedback: ...
    def request_stop(self, run_id: str, preserve_load: bool, now_s: float) -> None: ...
    def poll_stop(self, run_id: str, now_s: float) -> Feedback: ...


class TaskLease:
    """Shared by fetch and app navigation; no implicit preemption or timeout release."""

    def __init__(self):
        self.owner: str | None = None

    def acquire(self, owner: str):
        if self.owner is not None:
            raise InvalidTask("robot_busy")
        self.owner = owner

    def release(self, owner: str):
        if self.owner != owner:
            raise InvalidTask("wrong_task_owner")
        self.owner = None


def invariants(skill: str) -> tuple[str, ...]:
    """Continuous conditions, distinct from one-time start/completion proofs."""
    required = ["hardware_ready"]
    if skill in {"navigate", "navigate_with_load", "navigate_to"}:
        required += [
            "localized",
            "lift_stopped",
            "arm_in_transport_pose",
            "lift_in_transport_position",
        ]
    elif skill == "search":
        # Composite search checks movement/observation invariants in each child.
        required += ["search_phase_safe"]
    elif skill in {"prepare_navigation", "prepare_transport"}:
        # The composite alternates stationary-platform arm movement and
        # stationary-base lift movement; its phase monitor checks each case.
        required += ["base_stopped", "preparation_phase_safe"]
    elif skill.startswith("align_"):
        required += ["alignment_phase_safe"]
    elif skill.startswith("adjust_lift_"):
        required += ["base_stopped", "arms_safe_for_lift", "lift_homed"]
    else:
        required += ["base_stopped", "lift_stopped"]
    if skill in {
        "prepare_transport",
        "navigate_with_load",
        "inspect_destination",
        "align_for_place",
        "adjust_lift_for_place",
        "reobserve_destination",
    }:
        required += ["load_retained"]
    return tuple(required)


class TaskExecutor:
    TERMINAL = {"SUCCEEDED", "FAILED", "CANCELLED"}

    def __init__(
        self,
        run_id: str,
        steps: list[dict],
        adapter: SkillAdapter,
        lease: TaskLease,
        now_s: float = 0,
        *,
        feedback_max_age_s: float = 0.5,
        stop_timeout_s: float = 2,
        task_timeout_s: float = DEFAULT_TASK_TIMEOUT_S,
    ):
        if not isinstance(run_id, str) or not run_id or len(run_id) > 128:
            raise InvalidTask("invalid run id")
        if adapter.mode != "simulation":
            raise InvalidTask("physical execution is not commissioned")
        now = _seconds(now_s, "now_s")
        age, stop = _seconds(feedback_max_age_s, "feedback_max_age_s"), _seconds(
            stop_timeout_s, "stop_timeout_s"
        )
        task_budget = _seconds(task_timeout_s, "task_timeout_s")
        if (
            not 0 < task_budget <= 3600
            or not age
            or not stop
            or not steps
            or len(steps) > 64
        ):
            raise InvalidTask("invalid execution budget")
        ids = set()
        for step in steps:
            if not isinstance(step, dict) or not isinstance(step.get("skill"), str):
                raise InvalidTask("invalid step")
            sid = step.get("step_id")
            if not isinstance(sid, str) or not sid or sid in ids:
                raise InvalidTask("invalid step id")
            ids.add(sid)
            if not _seconds(step.get("timeout_s"), "timeout_s"):
                raise InvalidTask("step timeout must be positive")
            for key in ("start_requires", "success_requires"):
                values = step.get(key)
                if not isinstance(values, (list, tuple)) or not all(
                    isinstance(v, str) for v in values
                ):
                    raise InvalidTask("invalid condition list")
        self.run_id, self.steps, self.adapter, self.lease = (
            run_id,
            deepcopy(steps),
            adapter,
            lease,
        )
        self.index, self.state, self.reason = 0, "WAITING", None
        self.now = self.ready_at = self.started_at = now
        self.created_at, self.deadline = now, now + task_budget
        self.task_budget, self.finished_at = task_budget, None
        self.max_age, self.stop_timeout = age, stop
        self.stop_at, self.stop_sent, self.stop_confirmed = now, False, False
        self.stop_destination, self.carrying = "FAILED", False
        self.commands, self.history = 0, []
        lease.acquire(run_id)

    @property
    def step(self) -> dict:
        return self.steps[min(self.index, len(self.steps) - 1)]

    @property
    def goal_id(self) -> str:
        return f"{self.run_id}/{self.step['step_id']}"

    def _clock(self, now_s: float):
        now = _seconds(now_s, "now_s")
        if now < self.now:
            raise InvalidTask("executor clock moved backwards")
        self.now = now

    def _valid_feedback(self, feedback: Feedback, goal_id: str, earliest: float):
        if not isinstance(feedback, Feedback) or feedback.goal_id != goal_id:
            raise InvalidTask("wrong_goal_feedback")
        stamp = _seconds(feedback.observed_s, "feedback stamp")
        if stamp < earliest or stamp > self.now or self.now - stamp > self.max_age:
            raise InvalidTask("stale_feedback")
        if feedback.status not in {
            "READY",
            "RUNNING",
            "SUCCEEDED",
            "FAILED",
            "STOPPED",
        }:
            raise InvalidTask("invalid_adapter_status")
        if not isinstance(feedback.conditions, dict) or any(
            type(v) is not bool for v in feedback.conditions.values()
        ):
            raise InvalidTask("invalid_condition_evidence")

    @staticmethod
    def _satisfies(feedback: Feedback, keys) -> bool:
        return all(feedback.conditions.get(key) is True for key in keys)

    def _stopping(self, reason: str, destination: str = "FAILED"):
        if self.state in self.TERMINAL or self.state in {
            "STOPPING",
            "STOP_UNCONFIRMED",
        }:
            return
        self.reason, self.stop_destination, self.stop_at = reason, destination, self.now
        self.state = "STOPPING"
        self._poll_stop()

    def _poll_stop(self):
        try:
            if not self.stop_sent:
                self.adapter.request_stop(
                    self.run_id,
                    self.carrying or self.step["skill"] in {"pick", "place"},
                    self.now,
                )
                self.stop_sent = True
            feedback = self.adapter.poll_stop(self.run_id, self.now)
            self._valid_feedback(feedback, self.run_id, self.stop_at)
            required = ["base_stopped", "lift_stopped", "arms_holding"]
            if self.carrying:
                required += ["load_retained"]
            if feedback.status == "STOPPED" and self._satisfies(feedback, required):
                self.stop_confirmed = True
                self.state = self.stop_destination
                self.finished_at = self.now
                self.lease.release(self.run_id)
                return
        except Exception:
            # A broken stop transport must never release control ownership.
            pass
        if self.now - self.stop_at >= self.stop_timeout:
            self.state = "STOP_UNCONFIRMED"

    def cancel(self, now_s: float, reason: str = "cancelled") -> dict:
        self._clock(now_s)
        self._stopping(reason, "CANCELLED")
        return self.result()

    def tick(self, now_s: float) -> dict:
        self._clock(now_s)
        if self.state in self.TERMINAL:
            return self.result()
        if self.state in {"STOPPING", "STOP_UNCONFIRMED"}:
            self._poll_stop()
            return self.result()
        if self.now >= self.deadline:
            self._stopping("task_timeout")
            return self.result()
        if self.now - self.ready_at >= self.step["timeout_s"]:
            self._stopping("step_timeout")
            return self.result()
        try:
            if self.state == "WAITING":
                feedback = self.adapter.observe(
                    self.goal_id, deepcopy(self.step), self.now
                )
                self._valid_feedback(feedback, self.goal_id, self.ready_at)
                if feedback.status == "FAILED":
                    self._stopping(feedback.reason or "readiness_failed")
                elif feedback.status == "READY" and self._satisfies(
                    feedback,
                    (*self.step["start_requires"], *invariants(self.step["skill"])),
                ):
                    # Starting can partially dispatch before raising. Stop all
                    # resources on exception, even if no accepted count exists.
                    self.started_at, self.state = self.now, "RUNNING"
                    self.adapter.start(self.goal_id, deepcopy(self.step), self.now)
                    self.commands += 1
            else:
                feedback = self.adapter.poll(
                    self.goal_id, deepcopy(self.step), self.now
                )
                self._valid_feedback(feedback, self.goal_id, self.started_at)
                if feedback.status == "FAILED":
                    self._stopping(feedback.reason or "skill_failed")
                elif not self._satisfies(feedback, invariants(self.step["skill"])):
                    self._stopping("continuous_condition_lost")
                elif feedback.status == "SUCCEEDED":
                    if not self._satisfies(feedback, self.step["success_requires"]):
                        self._stopping("completion_not_verified")
                    else:
                        self.history.append(
                            {
                                "step_id": self.step["step_id"],
                                "completed_at_s": self.now,
                                "started_at_s": self.started_at,
                                "wait_s": self.started_at - self.ready_at,
                                "execution_s": self.now - self.started_at,
                            }
                        )
                        if self.step["skill"] == "pick":
                            self.carrying = True
                        elif self.step["skill"] == "place":
                            self.carrying = False
                        self.index += 1
                        self.ready_at = self.now
                        if self.index == len(self.steps):
                            self.state = "SUCCEEDED"
                            self.finished_at = self.now
                            self.lease.release(self.run_id)
                        else:
                            self.state = "WAITING"
        except Exception as error:
            self._stopping(f"adapter_error:{type(error).__name__}:{error}")
        return self.result()

    def result(self) -> dict:
        elapsed = (
            self.now if self.finished_at is None else self.finished_at
        ) - self.created_at
        return {
            "run_id": self.run_id,
            "mode": "simulation",
            "status": self.state,
            "elapsed_s": elapsed,
            "task_timeout_s": self.task_budget,
            "task_remaining_s": max(0.0, self.task_budget - elapsed),
            "reason": self.reason,
            "completed_steps": self.index,
            "current_step_id": (
                None if self.state in self.TERMINAL else self.step["step_id"]
            ),
            "simulated_skill_commands": self.commands,
            "stop_requested": self.stop_sent,
            "simulated_stop_confirmed": self.stop_confirmed,
            "control_owned": self.lease.owner == self.run_id,
            "physical_task_completed": False,
            "hardware_commands": 0,
            "history": deepcopy(self.history),
        }
