"""Transport-neutral application service for future phone/desktop clients.

No HTTP listener, authentication substitute or public robot endpoint is created.
Navigation and fetch share one executor/lease. Requests are idempotent within
this service lifetime; production persistence/auth belong in the future gateway.
"""

from __future__ import annotations
from copy import deepcopy

from .execution import TaskExecutor, TaskLease, DEFAULT_TASK_TIMEOUT_S
from .fetch import FetchRequest, InvalidTask, plan_fetch
from .navigation import NavigateRequest, NavigationMap, identifier
from .replay import _seconds


class RobotApplication:
    def __init__(
        self,
        world: dict,
        navigation_map: NavigationMap,
        adapter,
        *,
        initial_xy=(1.5, 1.5),
        exact_goal_simulation=True,
        task_timeout_s=DEFAULT_TASK_TIMEOUT_S,
        lease=None,
        admission=None,
    ):
        if adapter.mode != "simulation":
            raise InvalidTask("physical application is not commissioned")
        if type(exact_goal_simulation) is not bool:
            raise InvalidTask("exact goal simulation flag must be boolean")
        self.task_timeout_s = _seconds(task_timeout_s, "task_timeout_s")
        if not 0 < self.task_timeout_s <= 3600:
            raise InvalidTask("task timeout must be within (0, 3600] seconds")
        if admission is not None and not callable(admission):
            raise InvalidTask("admission callback required")
        self.admission = admission
        self.exact_goal_simulation = exact_goal_simulation
        navigation_map._cell(*initial_xy)
        self.world, self.map, self.adapter = deepcopy(world), navigation_map, adapter
        self.xy = tuple(initial_xy)
        if lease is not None and not isinstance(lease, TaskLease):
            raise InvalidTask("application requires a task lease")
        self.lease = lease if lease is not None else TaskLease()
        self.tasks: dict[str, TaskExecutor] = {}
        self.requests: dict[str, dict] = {}
        self.plans: dict[str, dict] = {}
        self.now = 0.0
        self.pose_known = True
        self.pending_map = None

    def _clock(self, now_s):
        now = _seconds(now_s, "now_s")
        if now < self.now:
            raise InvalidTask("application clock moved backwards")
        self.now = now

    def submit(self, request: dict, now_s: float) -> dict:
        self._clock(now_s)
        if not isinstance(request, dict):
            raise InvalidTask("request must be an object")
        rid = identifier(request.get("request_id"), "request_id")
        if rid in self.requests:
            if self.requests[rid] != request:
                raise InvalidTask("request_id_conflict")
            return self.status(rid)
        if self.admission is not None and not self.admission():
            raise InvalidTask("runtime not ready; explicit recovery may be required")
        if len(self.tasks) >= 128:
            raise InvalidTask("session_history_full")
        if self.lease.owner is not None:
            raise InvalidTask("robot_busy")
        operation = request.get("operation")
        if operation == "navigate_to":
            goal = NavigateRequest.from_dict(request)
            if not self.pose_known:
                raise InvalidTask("localization_required")
            plan = self.map.plan(goal, self.xy)
            steps = [
                {
                    "step_id": "01_navigate_to",
                    "skill": "navigate_to",
                    "parameters": deepcopy(plan),
                    "timeout_s": 180,
                    "start_requires": (
                        "hardware_ready",
                        "localized",
                        "base_stopped",
                        "lift_stopped",
                        "arm_in_transport_pose",
                        "lift_in_transport_position",
                    ),
                    "success_requires": (
                        "localized_at_destination",
                        "base_stopped",
                        "lift_stopped",
                        "arm_in_transport_pose",
                        "lift_in_transport_position",
                    ),
                }
            ]
        elif operation == "fetch_object":
            if (
                set(request) != {"schema_version", "operation", "request_id", "request"}
                or type(request["schema_version"]) is not int
                or request["schema_version"] != 1
            ):
                raise InvalidTask("invalid fetch envelope")
            plan = plan_fetch(FetchRequest.from_dict(request["request"]), self.world)
            steps = plan["steps"]
        else:
            raise InvalidTask("unsupported operation")
        executor = TaskExecutor(
            rid,
            steps,
            self.adapter,
            self.lease,
            self.now,
            task_timeout_s=self.task_timeout_s,
        )
        self.requests[rid], self.tasks[rid], self.plans[rid] = (
            deepcopy(request),
            executor,
            plan,
        )
        return self.status(rid)

    def status(self, request_id: str) -> dict:
        if request_id not in self.tasks:
            raise InvalidTask("unknown_request")
        return {
            **self.tasks[request_id].result(),
            "schema_version": 1,
            "operation": self.requests[request_id]["operation"],
            "request_id": request_id,
            "plan": deepcopy(self.plans[request_id]),
        }

    def tick(self, now_s: float) -> dict | None:
        self._clock(now_s)
        owner = self.lease.owner
        if owner is None:
            self._commit_pending_map()
            return None
        task = self.tasks[owner]
        task.tick(self.now)
        # A fake completed map move has a known target. An interrupted move or
        # semantic fetch cannot provide a pose; require relocalization next.
        # Nav2/feedback-driven simulations must disable exact_goal_simulation
        # and supply a fresh estimate through localize_simulated while idle.
        if task.state in task.TERMINAL:
            if (
                self.exact_goal_simulation
                and task.state == "SUCCEEDED"
                and self.requests[owner]["operation"] == "navigate_to"
            ):
                goal = self.plans[owner]["goal"]
                self.xy, self.pose_known = (goal["x"], goal["y"]), True
            else:
                self.pose_known = False
        self._commit_pending_map()
        return self.status(owner)

    def cancel(self, request_id: str, now_s: float) -> dict:
        self._clock(now_s)
        if request_id not in self.tasks:
            raise InvalidTask("unknown_request")
        task = self.tasks[request_id]
        if task.state not in task.TERMINAL:
            self.pose_known = False
        task.cancel(self.now)
        return self.status(request_id)

    def replace_map(self, new_map: NavigationMap, now_s: float):
        self._clock(now_s)
        # A revision is immutable: never silently replace its geometry.
        if (new_map.map_id, new_map.revision) == (self.map.map_id, self.map.revision):
            if new_map.descriptor() != self.map.descriptor():
                raise InvalidTask("map_revision_reused")
            return
        if self.pending_map is not None:
            if new_map.descriptor() != self.pending_map.descriptor():
                raise InvalidTask("map_update_pending")
            return
        self.pending_map = new_map
        self.pose_known = False
        owner = self.lease.owner
        if owner is not None:
            self.tasks[owner].cancel(self.now, "map_changed")
        self._commit_pending_map()

    def _commit_pending_map(self):
        # Cancellation is asynchronous. Keep old revision until whole stop
        # confirmation releases the task lease, then require relocalization.
        if self.pending_map is not None and self.lease.owner is None:
            self.map, self.pending_map = self.pending_map, None
            self.pose_known = False

    def localize_simulated(self, xy: tuple[float, float], now_s: float):
        self._clock(now_s)
        if self.lease.owner is not None:
            raise InvalidTask("robot_busy")
        self.map._cell(*xy)
        self.xy, self.pose_known = tuple(xy), True
