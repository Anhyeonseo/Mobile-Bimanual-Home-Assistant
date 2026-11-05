"""Pick planning facade over the repository's single execution implementation.

This prototype uses synthetic evidence only. No motor or hardware transport is
created. An adapter is injected per task; TaskExecutor enforces simulation mode.
"""
from copy import deepcopy
import json
import re

from home_robot_tasks.execution import TaskExecutor, TaskLease
from home_robot_tasks.fake_robot import FakeRobot
from home_robot_tasks.fetch import InvalidTask
from home_robot_tasks.replay import _seconds


STAGES = (
    ("SEARCHING", "search", "object_found"),
    ("LOCALIZING", "localize_object", "object_pose_valid"),
    ("ALIGNING", "align_for_pick", "aligned_for_pick"),
    ("GRASPING", "pick", "object_grasped"),
    ("VERIFYING", "verify_grasp", "load_retained"),
)


def parse_request(raw):
    if not isinstance(raw, str) or len(raw.encode("utf-8")) > 16384:
        raise InvalidTask("request must be JSON text within 16 KiB")

    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise InvalidTask("duplicate JSON field")
            result[key] = value
        return result

    try:
        request = json.loads(raw, object_pairs_hook=unique)
    except (ValueError, RecursionError) as error:
        raise InvalidTask("invalid JSON: " + str(error)) from error
    if not isinstance(request, dict) or set(request) != {
        "schema_version", "task_id", "operation", "target"
    }:
        raise InvalidTask("expected schema_version, task_id, operation, target")
    if type(request["schema_version"]) is not int or request["schema_version"] != 1:
        raise InvalidTask("schema_version must be 1")
    if request["operation"] != "pick":
        raise InvalidTask("only pick is supported")
    if not isinstance(request["task_id"], str) or not re.fullmatch(
        r"[A-Za-z0-9_-]{1,64}", request["task_id"]
    ):
        raise InvalidTask("task_id must contain 1-64 letters, digits, _ or -")
    target = request["target"]
    if not isinstance(target, dict) or set(target) not in (
        {"object", "location"}, {"object", "location", "color"}
    ):
        raise InvalidTask("target needs object, location and optional color")
    if any(not isinstance(v, str) or not v.strip() or len(v) > 128
           for v in target.values()):
        raise InvalidTask("target values must be nonempty strings up to 128 characters")
    return request


def plan_pick(target, timeout_s):
    return [dict(
        step_id=state, skill=skill, parameters=deepcopy(target),
        timeout_s=timeout_s, start_requires=[], success_requires=[evidence],
    ) for state, skill, evidence in STAGES]


class TaskManager:
    def __init__(self, adapter_factory=FakeRobot, *, lease=None, step_timeout_s=10.0):
        self.step_timeout = _seconds(step_timeout_s, "step_timeout_s")
        if not 0 < self.step_timeout <= 600:
            raise InvalidTask("step timeout must be within (0, 600]")
        self.factory = adapter_factory
        self.lease = lease if lease is not None else TaskLease()
        self.tasks = {}
        self.requests = {}
        self.active = None
        self.last_task_id = None
        self.validating = False
        self.now = 0.0

    def _clock(self, now):
        now = _seconds(now, "now_s")
        if now < self.now:
            raise InvalidTask("manager clock moved backwards")
        self.now = now

    def submit(self, raw, now):
        self._clock(now)
        request = parse_request(raw)
        task_id = request["task_id"]
        if task_id in self.requests:
            if self.requests[task_id] != request:
                raise InvalidTask("task_id_conflict")
            return self.status(task_id)
        if self.lease.owner is not None:
            raise InvalidTask("robot_busy")
        if len(self.tasks) >= 128:
            raise InvalidTask("session_history_full")
        executor = TaskExecutor(
            task_id, plan_pick(request["target"], self.step_timeout),
            self.factory(), self.lease, now,
            task_timeout_s=self.step_timeout * 5,
        )
        self.tasks[task_id], self.requests[task_id] = executor, request
        self.active = self.last_task_id = task_id
        self.validating = True
        return self.status(task_id)

    def tick(self, now):
        self._clock(now)
        if self.active is not None:
            task = self.tasks[self.active]
            if task.state in task.TERMINAL:
                self.active = None
            else:
                self.validating = False
                task.tick(now)
        return self.status()

    def cancel(self, task_id, now):
        self._clock(now)
        if task_id not in self.tasks:
            raise InvalidTask("unknown_task")
        self.tasks[task_id].cancel(now)
        if task_id == self.active:
            self.validating = False
        return self.status(task_id)

    def status(self, task_id=""):
        selected = task_id or self.active
        if selected is None:
            return {"schema_version": 1, "state": "IDLE", "task_id": None,
                    "last_task_id": self.last_task_id, "mode": "simulation",
                    "control_owned": self.lease.owner is not None}
        if selected not in self.tasks:
            raise InvalidTask("unknown_task")
        task = self.tasks[selected]
        state = {"SUCCEEDED": "COMPLETED"}.get(task.state, task.state)
        if state in {"WAITING", "RUNNING"}:
            state = ("VALIDATING" if self.validating and selected == self.active
                     else task.step["step_id"])
        return {**task.result(), "schema_version": 1, "task_id": selected,
                "state": state, "target": deepcopy(self.requests[selected]["target"])}
