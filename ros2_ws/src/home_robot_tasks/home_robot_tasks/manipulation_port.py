"""Plan then execute on a stationary platform, with context checked at handoff.

Planning success alone cannot report grasp/release success. RoutedSkillAdapter's
independent monitor supplies contact, payload retention and destination evidence.
"""

from dataclasses import dataclass
from copy import deepcopy
from .fetch import InvalidTask
from .navigation import number
from .skill_ports import PortResult


@dataclass(frozen=True)
class PlanningContext:
    observed_s: float
    capture_id: str
    pose_revision: str
    scene_revision: str
    calibration_id: str
    frame_id: str
    anchor_rad: tuple
    base_stopped: bool
    lift_stopped: bool
    # Target age is checked at admission/handoff, state age throughout execution.
    target_observed_s: float | None = None

    def key(self):
        return (
            self.capture_id,
            self.pose_revision,
            self.scene_revision,
            self.calibration_id,
            self.frame_id,
        )


class ManipulationPort:
    def __init__(
        self,
        planner,
        executor,
        context,
        clock,
        *,
        maximum_age_s=0.5,
        anchor_tolerance_rad=0.02
    ):
        self.planner, self.executor, self.context, self.clock = (
            planner,
            executor,
            context,
            clock,
        )
        self.age = number(maximum_age_s, "planning age")
        self.tolerance = number(anchor_tolerance_rad, "anchor tolerance")
        if not 0 < self.age <= 2 or not 0 < self.tolerance <= 0.1:
            raise InvalidTask("invalid planning bounds")
        self.entries = {}
        self.owner = None

    def ready(self):
        return self.owner is None and self.planner.ready() and self.executor.ready()

    def _context(self, *, check_target=True):
        c = self.context()
        now = number(self.clock(), "planning clock")
        if (
            not isinstance(c, PlanningContext)
            or not 0 <= number(c.observed_s, "planning observation") <= now
            or now - c.observed_s > self.age
        ):
            raise InvalidTask("stale planning context")
        if c.base_stopped is not True or c.lift_stopped is not True:
            raise InvalidTask("moving planning platform")
        if c.target_observed_s is not None:
            stamp=number(c.target_observed_s,"target exposure")
            if not 0<=stamp<=now or check_target and now-stamp>self.age:
                raise InvalidTask("stale planning target")
        if len(c.anchor_rad) != 12 or any(
            not isinstance(v, (int, float)) for v in c.anchor_rad
        ):
            raise InvalidTask("twelve joint anchor required")
        for v in c.anchor_rad:
            number(v, "anchor")
        if any(not isinstance(v, str) or not v for v in c.key()):
            raise InvalidTask("planning context identities required")
        return c

    def start(self, goal_id, parameters):
        if not self.ready() or goal_id in self.entries or len(self.entries) >= 128:
            raise InvalidTask("manipulation busy or duplicate")
        context = self._context()
        self.entries[goal_id] = {
            "state": "RUNNING",
            "port": self.planner,
            "phase": "plan",
            "context": context,
            "reason": "",
            "cancel": False,
        }
        self.owner = goal_id
        try:
            self.planner.start(
                goal_id, {"task": deepcopy(parameters), "context": context}
            )
        except Exception as error:
            self._cancel(goal_id, str(error))

    def _cancel(self, g, reason, cancelled=False):
        e = self.entries[g]
        if e["cancel"] or e["state"] != "RUNNING":
            return
        e.update(
            cancel=True,
            reason=reason,
            destination="CANCELLED" if cancelled else "FAILED",
        )
        try:
            e["port"].cancel(g)
        except Exception:
            e["reason"] += ";cancel_unconfirmed"

    def cancel(self, g):
        self._cancel(g, "cancelled", True)

    def poll(self, g):
        e = self.entries[g]
        if e["state"] != "RUNNING":
            return PortResult(e["state"], e["reason"])
        try:
            if e["cancel"]:
                if e["port"].poll(g).state in {"SUCCEEDED", "FAILED", "CANCELLED"}:
                    e["state"] = e["destination"]
                    self.owner = None
                return PortResult(e["state"], e["reason"])
            current = self._context(check_target=e['phase']=='plan')
            if (current.key() != e["context"].key()
                    or current.target_observed_s != e['context'].target_observed_s):
                raise InvalidTask("planning context changed")
            result = e["port"].poll(g)
            if result.state in {"FAILED", "CANCELLED"}:
                self._cancel(g, result.reason or "manipulation backend failed")
            elif result.state == "SUCCEEDED":
                if e["phase"] == "plan":
                    if any(
                        abs(a - b) > self.tolerance
                        for a, b in zip(current.anchor_rad, e["context"].anchor_rad)
                    ):
                        raise InvalidTask("planning anchor moved")
                    plan = self.planner.result(g)
                    if not self.executor.ready():
                        return PortResult("RUNNING", "waiting for arm executor")
                    e["port"] = self.executor
                    e["phase"] = "execute"
                    self.executor.start(g, {"trajectory": plan, "context": current})
                else:
                    e["state"] = "SUCCEEDED"
                    self.owner = None
        except Exception as error:
            self._cancel(g, str(error))
        return PortResult(e["state"], e["reason"])


class MoveItPlanPort:
    """Asynchronous GetMotionPlan service; planning only, no controller output.

    request_factory maps the grasp orientation/scene/anchor into a complete
    MotionPlanRequest. trajectory_mapper validates and maps named model joints
    to twelve runtime joint coordinates, preserving the opposite-arm anchor.
    """

    def __init__(
        self, node, request_factory, trajectory_mapper, service="/plan_kinematic_path"
    ):
        from moveit_msgs.srv import GetMotionPlan

        self.service_type = GetMotionPlan
        self.client = node.create_client(GetMotionPlan, service)
        self.factory, self.map = request_factory, trajectory_mapper
        self.entries = {}
        self.future = None

    def ready(self):
        return self.client.service_is_ready() and (
            self.future is None or self.future.done()
        )

    def start(self, g, parameters):
        if not self.ready() or g in self.entries or len(self.entries) >= 128:
            raise InvalidTask("planner busy or duplicate")
        request = self.service_type.Request()
        request.motion_plan_request = self.factory(parameters)
        self.future = self.client.call_async(request)
        self.entries[g] = {
            "future": self.future,
            "cancel": False,
            "context": parameters["context"],
            "mapped": None,
        }

    def cancel(self, g):
        # Service requests have no action cancellation. Results are quarantined;
        # no motion is emitted here, and a late response cannot reach executor.
        self.entries[g]["cancel"] = True

    def poll(self, g):
        e = self.entries[g]
        if e["cancel"]:
            return PortResult("CANCELLED")
        if not e["future"].done():
            return PortResult("RUNNING")
        try:
            response = e["future"].result().motion_plan_response
            if response.error_code.val != 1:
                return PortResult("FAILED", "moveit planning failed")
            if e["mapped"] is None:
                e["mapped"] = tuple(self.map(response.trajectory, e["context"]))
            if not 2 <= len(e["mapped"]) <= 20000:
                raise InvalidTask("bounded executable trajectory required")
            return PortResult("SUCCEEDED")
        except Exception as error:
            return PortResult("FAILED", str(error))

    def result(self, g):
        if self.poll(g).state != "SUCCEEDED":
            raise InvalidTask("plan incomplete")
        return self.entries[g]["mapped"]
