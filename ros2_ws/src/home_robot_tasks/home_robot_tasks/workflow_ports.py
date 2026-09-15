"""Checked asynchronous skill composition, including moving viewpoint search.

Children own motion only in their declared phase. Monitors are independent of
backend results; cancellation waits for the active child to become terminal.
"""

from copy import deepcopy
from dataclasses import dataclass
from .execution import Feedback
from .fetch import InvalidTask
from .skill_ports import PortResult
from .search import SearchSession
from .navigation import number

TERMINAL = {"SUCCEEDED", "FAILED", "CANCELLED"}


@dataclass(frozen=True)
class Phase:
    name: str
    port: object
    parameters: dict
    before: tuple
    during: tuple
    after: tuple
    timeout_s: float


class SequencePort:
    def __init__(self, build, monitor, clock, *, feedback_age_s=0.5):
        self.build, self.monitor, self.clock = build, monitor, clock
        self.max_age = number(feedback_age_s, "phase feedback age")
        if self.max_age <= 0:
            raise InvalidTask("positive feedback age required")
        self.entries = {}
        self.owner = None

    def ready(self):
        return self.owner is None

    def start(self, goal_id, parameters):
        if (
            self.owner is not None
            or goal_id in self.entries
            or len(self.entries) >= 128
        ):
            raise InvalidTask("sequence_owned_or_duplicate")
        phases = tuple(self.build(deepcopy(parameters)))
        if not 1 <= len(phases) <= 64 or any(
            not isinstance(p, Phase)
            or not 0 < number(p.timeout_s, "phase timeout") <= 3600
            for p in phases
        ):
            raise InvalidTask("bounded phases required")
        self.entries[goal_id] = dict(
            phases=phases,
            index=0,
            started=False,
            phase_at=number(self.clock(), "phase start"),
            status="RUNNING",
            cancel=False,
            reason="",
            history=[],
        )
        self.owner = goal_id

    def _evidence(self, child, phase, earliest):
        now = number(self.clock(), "phase clock")
        f = self.monitor(child, phase.name, now)
        if (
            not isinstance(f, Feedback)
            or f.goal_id != child
            or not earliest <= number(f.observed_s, "feedback time") <= now
            or now - f.observed_s > self.max_age
        ):
            raise InvalidTask("stale_phase_evidence")
        if any(type(v) is not bool for v in f.conditions.values()):
            raise InvalidTask("invalid_phase_evidence")
        return f.conditions

    def _cancel(self, goal_id, reason, cancelled=False):
        e = self.entries[goal_id]
        if e["status"] in TERMINAL or e["cancel"]:
            return
        e.update(
            cancel=True,
            reason=reason,
            destination="CANCELLED" if cancelled else "FAILED",
        )
        if e["started"]:
            phase = e["phases"][e["index"]]
            try:
                phase.port.cancel(f"{goal_id}/{e['index']}")
            except Exception as error:
                # A failed cancel is not evidence of stopped motion. Retain ownership.
                e["reason"] += f";cancel_unconfirmed:{error}"

    def cancel(self, goal_id):
        self._cancel(goal_id, "cancelled", True)

    def poll(self, goal_id):
        e = self.entries[goal_id]
        if e["status"] in TERMINAL:
            return PortResult(e["status"], e["reason"])
        phase = e["phases"][e["index"]]
        child = f"{goal_id}/{e['index']}"
        try:
            if not e["cancel"]:
                if self.clock() - e["phase_at"] >= phase.timeout_s:
                    self._cancel(goal_id, "phase_timeout:" + phase.name)
                else:
                    conditions = self._evidence(child, phase, e["phase_at"])
                    if e["started"]:
                        if not all(conditions.get(k) is True for k in phase.during):
                            self._cancel(goal_id, "phase_condition_lost:" + phase.name)
                        else:
                            result = phase.port.poll(child)
                            if result.state in {"FAILED", "CANCELLED"}:
                                self._cancel(goal_id, result.reason or "child_failed")
                            elif result.state == "SUCCEEDED" and all(
                                conditions.get(k) is True for k in phase.after
                            ):
                                e["history"].append(
                                    dict(
                                        phase=phase.name,
                                        elapsed_s=self.clock() - e["phase_at"],
                                    )
                                )
                                e["index"] += 1
                                e.update(started=False, phase_at=self.clock())
                                if e["index"] == len(e["phases"]):
                                    e["status"] = "SUCCEEDED"
                                    self.owner = None
                    elif phase.port.ready() and all(
                        conditions.get(k) is True
                        for k in (*phase.before, *phase.during)
                    ):
                        # Record ownership before a potentially partial dispatch.
                        e["started"] = True
                        phase.port.start(child, deepcopy(phase.parameters))
        except Exception as error:
            self._cancel(goal_id, f"phase_error:{error}")
        if e["cancel"]:
            try:
                idle = not e["started"] or phase.port.poll(child).state in TERMINAL
            except Exception:
                idle = False
            if idle:
                e["status"] = e["destination"]
                self.owner = None
        return PortResult(e["status"], e["reason"])

    def phase_safe(self, goal_id):
        e = self.entries[goal_id]
        if e["cancel"] or e["status"] == "FAILED":
            return False
        if e["status"] == "SUCCEEDED":
            return True
        p = e["phases"][e["index"]]
        try:
            f = self._evidence(f"{goal_id}/{e['index']}", p, e["phase_at"])
            return all(f.get(k) is True for k in p.during)
        except Exception:
            return False


class SearchSkillPort:
    """Execute SearchSession actions through real/mocked navigation/lift/camera ports."""

    def __init__(
        self, make_session, views, move, capture, motion, stop, clock, target_sink
    ):
        self.make_session, self.views = make_session, deepcopy(views)
        self.move, self.capture, self.motion, self.stop, self.clock = (
            move,
            capture,
            motion,
            stop,
            clock,
        )
        self.target_sink = target_sink
        self.entries = {}
        self.owner = None

    def ready(self):
        return self.owner is None and self.move.ready() and self.capture.ready()

    def start(self, goal_id, parameters):
        if not self.ready() or goal_id in self.entries or len(self.entries) >= 128:
            raise InvalidTask("search_owned_or_duplicate")
        session = self.make_session(goal_id, parameters, self.clock())
        if not isinstance(session, SearchSession):
            raise InvalidTask("search_session_required")
        self.entries[goal_id] = dict(
            session=session, action=None, port=None, stop_sent=False, delivered=False
        )
        self.owner = goal_id

    def cancel(self, goal_id):
        self.entries[goal_id]["session"].cancel(self.clock())

    def phase_safe(self, goal_id):
        e = self.entries[goal_id]
        if e["session"].state == "MOVING":
            return e["port"] is self.move and self.move.phase_safe(e["action"])
        try:
            e["session"].camera.stationary_since(self.motion(), self.clock())
            return e["session"].state not in {"STOPPING", "FAILED"}
        except Exception:
            return False

    def poll(self, goal_id):
        e = self.entries[goal_id]
        s, now = e["session"], self.clock()
        try:
            action = s.poll(self.motion(), now)
            if action is not None and action.kind == "STOP_SEARCH":
                if not e["stop_sent"]:
                    if e["port"] is not None:
                        e["port"].cancel(e["action"])
                    self.stop.request(action.action_id, True, now)
                    e["stop_sent"] = True
                f = self.stop.poll(action.action_id, now)
                idle = (
                    e["port"] is None or e["port"].poll(e["action"]).state in TERMINAL
                )
                if f.status == "STOPPED" and idle:
                    s.confirm_stop(
                        action.action_id,
                        self.motion(),
                        now,
                        backend_idle=True,
                        arms_holding=f.conditions.get("arms_holding") is True,
                    )
            elif action is not None:
                if e["action"] != action.action_id:
                    port = self.move if action.kind == "MOVE_TO_VIEW" else self.capture
                    if not port.ready():
                        return PortResult("RUNNING")
                    e.update(action=action.action_id, port=port)
                    params = (
                        self.views[action.view_id]
                        if port is self.move
                        else {
                            "object_id": s.object_id,
                            "view_id": action.view_id,
                            "requested_s": action.issued_s,
                        }
                    )
                    port.start(action.action_id, deepcopy(params))
                result = e["port"].poll(e["action"])
                if result.state == "SUCCEEDED":
                    if action.kind == "MOVE_TO_VIEW":
                        s.arrive(
                            action.action_id,
                            self.motion(),
                            now,
                            view_reached=True,
                            backend_idle=True,
                        )
                    else:
                        frame, kwargs = self.capture.result(action.action_id)
                        s.observation(
                            action.action_id, frame, self.motion(), now, **kwargs
                        )
                    e.update(port=None, action=None)
                elif result.state in {"FAILED", "CANCELLED"}:
                    # Movement failure does not authorize another view before stop.
                    s._stop(result.reason or "search_child_failed", "FAILED")
        except Exception as error:
            s._stop(f"search_adapter_error:{error}", "FAILED")
        if s.state in s.TERMINAL:
            self.owner = None
            if s.state == "FOUND":
                if not e["delivered"]:
                    try:
                        self.target_sink(deepcopy(s.target))
                        e["delivered"] = True
                    except Exception as error:
                        self.owner = goal_id
                        s._stop(f"target_sink_failed:{error}", "FAILED")
                        return PortResult("RUNNING")
                return PortResult("SUCCEEDED")
            return PortResult(
                "CANCELLED" if s.state == "CANCELLED" else "FAILED", s.reason
            )
        return PortResult("RUNNING")
