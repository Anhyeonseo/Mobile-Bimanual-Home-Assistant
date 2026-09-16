"""Connect every fetch skill to explicit, injected asynchronous ports.

This is dependency assembly, not physical commissioning. The resolver supplies
registered geometry/routes and fresh targets. The monitor supplies independent
conditions. Neither is replaced by a port's successful action result.
"""
from copy import deepcopy
from .fetch import InvalidTask
from .skill_ports import PortResult, RoutedSkillAdapter
from .workflow_ports import Phase, SequencePort
from .execution import Feedback

ROLES = {
    "navigate": "navigation", "navigate_with_load": "navigation", "navigate_to": "navigation",
    "search": "search", "align_for_pick": "alignment", "align_for_place": "alignment",
    "adjust_lift_for_pick": "lift", "adjust_lift_for_place": "lift",
    "reobserve_target": "capture", "inspect_destination": "capture",
    "reobserve_destination": "capture", "verify_delivery": "capture",
    "pick": "manipulation", "place": "manipulation",
}


class ResolvedPort:
    """Resolve semantic parameters at start, and deliver capture results once."""
    def __init__(self, skill, port, resolve, clock, *, before=None, result_sink=None):
        self.skill, self.port, self.resolve, self.clock = skill, port, resolve, clock
        self.before, self.sink = before, result_sink
        self.entries = {}
        self.owner = None

    def ready(self):
        return self.owner is None and self.port.ready()

    def start(self, goal_id, parameters):
        if not self.ready() or goal_id in self.entries or len(self.entries) >= 128:
            raise InvalidTask("resolved port owned or duplicate")
        e = dict(attempted=False, status="RUNNING", reason="", delivered=False,
                 parameters=deepcopy(parameters))
        self.entries[goal_id], self.owner = e, goal_id
        try:
            resolved = self.resolve(self.skill, deepcopy(parameters), self.clock())
            if not isinstance(resolved, dict):
                raise InvalidTask("resolved port parameters must be a dictionary")
            if self.before is not None:
                self.before(self.skill, deepcopy(parameters), self.clock())
            e["attempted"] = True
            self.port.start(goal_id, deepcopy(resolved))
        except Exception as error:
            e["reason"] = str(error)
            if not e["attempted"]:
                e["status"] = "FAILED"
                self.owner = None
            raise

    def poll(self, goal_id):
        e = self.entries[goal_id]
        if e["status"] != "RUNNING":
            return PortResult(e["status"], e["reason"])
        result = self.port.poll(goal_id)
        if result.state == "SUCCEEDED" and self.sink is not None and not e["delivered"]:
            try:
                self.sink(goal_id, self.skill, deepcopy(e["parameters"]), self.port.result(goal_id))
                e["delivered"] = True
            except Exception as error:
                # The capture child is already terminal. Do not replay a
                # partially applied sink on every subsequent stop poll.
                result = PortResult("FAILED", f"capture_sink_failed:{error}")
        if result.state in {"SUCCEEDED", "FAILED", "CANCELLED"}:
            e["status"], e["reason"] = result.state, result.reason
            self.owner = None
        return result

    def cancel(self, goal_id):
        e = self.entries[goal_id]
        if e["status"] != "RUNNING":
            return
        if e["attempted"]:
            self.port.cancel(goal_id)
        else:
            e["status"], self.owner = "CANCELLED", None

    def phase_safe(self, goal_id):
        return self.port.phase_safe(goal_id)


def compose_fetch_adapter(*, backends, resolve, monitor, phase_monitor,
                          stop_backend, clock, capture_sink, before_start=None, alignment_transport=None):
    """Use real or mocked ports, retaining the simulation-only execution gate.

    Required roles: arm, lift, navigation, alignment, search, capture,
    manipulation. SearchSkillPort and ManipulationPort can own nested ports.
    Resolve prepare_* to {arm: checked-route-parameters, height_um: integer};
    resolve all other skills to the selected port's documented parameters.
    Capture sinks update target/context from CapturePort's validated frame.
    """
    required = set(ROLES.values()) | {"arm"}
    if set(backends) != required or not all(callable(f) for f in (
        resolve, monitor, phase_monitor, clock, capture_sink
    )):
        raise InvalidTask("complete explicit fetch dependencies required")
    for port in backends.values():
        if not all(callable(getattr(port, method, None)) for method in ("ready", "start", "poll", "cancel")):
            raise InvalidTask("asynchronous skill port required")
    if not callable(getattr(backends["capture"], "result", None)):
        raise InvalidTask("capture must return validated observations")
    ports = {
        skill: ResolvedPort(skill, backends[role], resolve, clock, before=before_start,
                            result_sink=capture_sink if role == "capture" else None)
        for skill, role in ROLES.items()
    }
    for skill in ("prepare_navigation", "prepare_transport"):
        def phases(parameters, carrying=skill == "prepare_transport"):
            if (set(parameters) != {"arm", "height_um"} or not isinstance(parameters["arm"], dict)
                    or type(parameters["height_um"]) is not int):
                raise InvalidTask("explicit transport arm route and lift height required")
            load = ("load_retained",) if carrying else ()
            stationary = ("base_stopped", "lift_stopped", *load)
            return (
                Phase("transport_arm", backends["arm"], parameters["arm"], stationary, stationary,
                      ("arm_in_transport_pose", *load), 20),
                Phase("transport_lift", backends["lift"], {"height_um": parameters["height_um"]},
                      (*stationary, "arms_safe_for_lift", "lift_homed"),
                      ("base_stopped", "arms_safe_for_lift", *load),
                      ("lift_stopped", "lift_in_transport_position", *load), 20),
            )
        sequence = SequencePort(phases, phase_monitor, clock)
        ports[skill] = ResolvedPort(skill, sequence, resolve, clock, before=before_start)
    if alignment_transport is not None:
        for skill in ("align_for_pick", "align_for_place"):
            def alignment_phases(plan, carrying=skill == "align_for_place"):
                load = ("load_retained",) if carrying else ()
                stationary = ("base_stopped", "lift_stopped", *load)
                travel = ("localized", "arm_in_transport_pose", "lift_in_transport_position", "lift_stopped", *load)
                return (
                    Phase("alignment_arm_transport", backends["arm"], alignment_transport["arm"],
                          stationary, stationary, ("arm_in_transport_pose", *load), 20),
                    Phase("alignment_lift_transport", backends["lift"],
                          {"height_um": alignment_transport["height_um"]},
                          (*stationary, "arms_safe_for_lift", "lift_homed"),
                          ("base_stopped", "arms_safe_for_lift", *load),
                          ("lift_stopped", "lift_in_transport_position", *load), 20),
                    Phase("alignment_navigate", backends["alignment"], plan,
                          ("base_stopped", *travel), travel, ("base_stopped", *load), 60),
                )
            ports[skill] = ResolvedPort(skill, SequencePort(alignment_phases, phase_monitor, clock),
                                       resolve, clock, before=before_start)
    def composed_monitor(goal_id, step, now_s):
        evidence = monitor(goal_id, step, now_s)
        conditions = dict(evidence.conditions)
        skill = step["skill"]
        if skill in {"search", "prepare_navigation", "prepare_transport"}:
            port = ports[skill]
            if goal_id in port.entries and port.entries[goal_id]["attempted"]:
                safe = port.phase_safe(goal_id)
            else:
                safe = port.ready() and all(conditions.get(k) is True
                                           for k in ("base_stopped", "lift_stopped"))
            conditions["search_phase_safe" if skill == "search" else "preparation_phase_safe"] = safe
        if skill.startswith("align_"):
            port = ports[skill]
            if alignment_transport is None:
                safe = all(conditions.get(k) is True for k in ("lift_stopped", "arms_safe_for_alignment"))
            elif goal_id in port.entries and port.entries[goal_id]["attempted"]:
                safe = port.phase_safe(goal_id)
            else:
                safe = port.ready() and all(conditions.get(k) is True for k in ("base_stopped", "lift_stopped"))
            conditions["alignment_phase_safe"] = safe
        return Feedback(evidence.goal_id, evidence.observed_s, evidence.status, conditions, evidence.reason)

    return RoutedSkillAdapter(ports, composed_monitor, stop_backend, mode="simulation")
