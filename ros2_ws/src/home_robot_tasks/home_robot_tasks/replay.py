"""Replay task-level evidence; never connect to hardware or issue commands.

Evidence consists of assertions from a recorded/synthetic fixture, not measured
robot feedback. A live executor and continuous motion supervision remain separate.
All timestamps use seconds since the start of this replay, not device clocks.
"""
from __future__ import annotations

from copy import deepcopy
import math
from typing import Any

from .fetch import FetchRequest, InvalidTask, _identifier, plan_fetch


def _seconds(value: Any, field: str) -> float:
    if type(value) not in (int, float):
        raise InvalidTask(f"{field} must be finite nonnegative seconds")
    try:
        seconds = float(value)
    except OverflowError as error:
        raise InvalidTask(f"{field} is too large") from error
    if not math.isfinite(seconds) or seconds < 0:
        raise InvalidTask(f"{field} must be finite nonnegative seconds")
    return seconds


class FetchReplay:
    """Serial start/result transitions with bounded waiting and no auto retry."""

    def __init__(self, request: FetchRequest, world: dict, run_id: str,
                 *, evidence_max_age_s: float = 1.0):
        self._plan = plan_fetch(request, world)
        self.run_id = _identifier(run_id, "run_id")
        self._max_age = _seconds(evidence_max_age_s, "evidence_max_age_s")
        if self._max_age == 0:
            raise InvalidTask("evidence_max_age_s must be positive")
        self._index = 0
        self._status = "WAITING"
        self._ready_at = 0.0
        self._started_at = 0.0
        self._now = 0.0
        self._seen: set[str] = set()
        self._history: list[dict] = []
        self._reason: str | None = None

    @property
    def current_step(self) -> dict | None:
        if self._status in {"COMPLETED", "FAILED", "CANCELLED"}:
            return None
        return deepcopy(self._plan["steps"][self._index])

    def _finish(self, status: str, reason: str) -> None:
        self._status, self._reason = status, reason

    def _evidence(self, event: dict, required: list | tuple, earliest: float) -> str | None:
        stamp = _seconds(event["evidence_at_s"], "evidence_at_s")
        evidence = event["evidence"]
        if not isinstance(evidence, dict) or any(type(v) is not bool for v in evidence.values()):
            raise InvalidTask("evidence must contain boolean assertions")
        if set(evidence) - set(required):
            raise InvalidTask("unexpected evidence fields")
        if stamp > self._now or stamp < earliest or self._now - stamp > self._max_age:
            return "stale_or_out_of_phase_evidence"
        if any(evidence.get(key) is not True for key in required):
            return "unmet_conditions"
        return None

    def accept(self, event: dict) -> dict:
        """Consume one event. Invalid input raises without changing replay state."""
        candidate = deepcopy(self)
        candidate._accept(event)
        self.__dict__.update(candidate.__dict__)
        return self.result()

    def _accept(self, event: dict) -> None:
        step = self.current_step
        if step is None:
            raise InvalidTask("replay is terminal")
        if not isinstance(event, dict):
            raise InvalidTask("event must be an object")
        kind = event.get("kind")
        if kind not in ("start", "success", "failure", "cancel", "tick"):
            raise InvalidTask("unsupported event kind")
        fields = {"run_id", "event_id", "at_s", "kind"}
        if kind in ("start", "success", "failure"):
            fields.add("step_id")
        if kind in ("start", "success"):
            fields.update(("evidence_at_s", "evidence"))
        if kind == "failure":
            fields.add("reason")
        if set(event) != fields:
            raise InvalidTask(f"{kind} event fields must be {sorted(fields)}")
        event_id = _identifier(event["event_id"], "event_id")
        if event["run_id"] != self.run_id or event_id in self._seen:
            raise InvalidTask("wrong run or duplicate event")
        now = _seconds(event["at_s"], "at_s")
        if now < self._now:
            raise InvalidTask("event clock moved backwards")
        if "step_id" in event and event["step_id"] != step["step_id"]:
            raise InvalidTask("event does not match the current step")
        self._now = now
        if kind == "cancel":
            self._finish("CANCELLED", "cancelled")
        elif now - self._ready_at >= step["timeout_s"]:
            self._finish("FAILED", "timeout")
        elif kind == "failure":
            self._finish("FAILED", _identifier(event["reason"], "failure reason"))
        elif kind == "start":
            if self._status != "WAITING":
                raise InvalidTask("step already started")
            reason = self._evidence(event, step["start_requires"], self._ready_at)
            if reason:
                self._finish("FAILED", reason)
            else:
                self._status, self._started_at = "RUNNING", now
        elif kind == "success":
            if self._status != "RUNNING":
                raise InvalidTask("success requires a started step")
            reason = self._evidence(event, step["success_requires"], self._started_at)
            if reason:
                self._finish("FAILED", reason)
            else:
                self._index += 1
                self._ready_at = now
                self._status = "COMPLETED" if self._index == len(self._plan["steps"]) else "WAITING"
        self._seen.add(event_id)
        self._history.append({"event": deepcopy(event), "status": self._status,
                              "reason": self._reason})

    def result(self) -> dict:
        return {
            "schema_version": 1, "run_id": self.run_id, "mode": "offline_replay",
            "status": self._status, "reason": self._reason,
            "replay_completed": self._status == "COMPLETED",
            "physical_task_completed": False, "motion_authorized": False,
            "executable": False, "motion_commands": 0, "stop_command_sent": False,
            "completed_steps": self._index, "current_step": self.current_step,
            "required_response": (self._plan["failure_policy"]["required_response"]
                                  if self._status in {"FAILED", "CANCELLED"} else None),
            "evidence_max_age_s": self._max_age,
            "history": deepcopy(self._history),
        }


def replay_fetch(request: FetchRequest, world: dict, recording: dict) -> dict:
    if (not isinstance(recording, dict) or type(recording.get("schema_version")) is not int
            or recording["schema_version"] != 1):
        raise InvalidTask("recording schema_version must be 1")
    if set(recording) != {"schema_version", "run_id", "evidence_source", "events"}:
        raise InvalidTask("unexpected recording fields")
    if recording["evidence_source"] not in ("synthetic", "recorded_assertions"):
        raise InvalidTask("evidence_source must be synthetic or recorded_assertions")
    if not isinstance(recording["events"], list):
        raise InvalidTask("events must be a list")
    runner = FetchReplay(request, world, recording["run_id"])
    for event in recording["events"]:
        runner.accept(event)
    return {**runner.result(), "evidence_source": recording["evidence_source"]}
