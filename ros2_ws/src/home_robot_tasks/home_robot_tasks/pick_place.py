"""Pick/place stages with independent payload proof and scene acknowledgement.

The resolver supplies each stage's geometry/command at dispatch time, so retreat
is planned from a new measured anchor AFTER the scene update. It must not turn
an old observation into a fresh one. No pose, jaw calibration or collision
permission is inferred here. Whole-system stopping belongs to RoutedSkillAdapter.
"""
from copy import deepcopy

from .fetch import InvalidTask
from .fetch_composition import ResolvedPort
from .navigation import identifier, number
from .workflow_ports import Phase, SequencePort


class PickPlacePort:
    def __init__(self, motion, gripper, scene, resolve_stage, monitor, clock,
                 *, probe=None, motion_timeout_s=20, gripper_timeout_s=5, scene_timeout_s=3):
        self.motion, self.gripper, self.scene = motion, gripper, scene
        self.resolve, self.clock = resolve_stage, clock
        self.probe = probe
        if not callable(resolve_stage):
            raise InvalidTask("explicit manipulation stage resolver required")
        self.timeouts = {}
        for name, value in (("motion", motion_timeout_s), ("gripper", gripper_timeout_s),
                            ("scene", scene_timeout_s)):
            value = number(value, name + " timeout")
            if not 0 < value <= 60:
                raise InvalidTask("bounded manipulation timeout required")
            self.timeouts[name] = value
        ports = (motion, gripper, scene) if probe is None else (motion, gripper, scene, probe)
        for port in ports:
            if not all(callable(getattr(port, m, None)) for m in ("ready", "start", "poll", "cancel")):
                raise InvalidTask("asynchronous manipulation ports required")
        self.sequence = SequencePort(self._build, monitor, clock)

    @property
    def owner(self):
        return self.sequence.owner

    def ready(self):
        return self.sequence.ready()

    def _build(self, parameters):
        operation = parameters.get("operation")
        if operation not in {"pick", "place"} or parameters.get("arm") not in {"left", "right"}:
            raise InvalidTask("explicit pick/place and arm required")
        identifier(parameters.get("object_id"), "manipulated object")
        stationary = ("hardware_ready", "base_stopped", "lift_stopped")
        held = ("load_retained",)
        attached = ("object_attached",)
        released = ("release_verified",)
        detached = ("object_detached",)
        # During a jaw transition, do not demand the completed grasp/release
        # state. AFTER requires fresh independent evidence from the monitor.
        if operation == "pick":
            close_after = ("grasp_verified",) if self.probe is None else ("contact_plausible",)
            probe_stage = () if self.probe is None else (
                ("probe", "probe", ("contact_plausible",), ("probe_clearance_verified", "probe_contact_live"),
                 ("probe_follow_verified", "grasp_verified", "load_retained")),
            )
            spec = (
                ("open", "gripper", ("gripper_empty",), (), ("gripper_open", "gripper_empty")),
                ("pregrasp", "motion", ("gripper_open", "gripper_empty"), ("gripper_empty",), ()),
                ("approach", "motion", ("gripper_open", "gripper_empty"), ("gripper_empty",), ()),
                ("close", "gripper", ("gripper_empty",), (), close_after),
                *probe_stage,
                ("attach", "scene", held, held, attached),
                ("retreat", "motion", held + attached, held + attached, ("arm_clear",)),
            )
        else:
            spec = (
                ("preplace", "motion", held + attached, held + attached, ()),
                ("approach", "motion", held + attached, held + attached, ()),
                ("open", "gripper", held + attached, attached, released),
                ("detach", "scene", released + attached, released, detached),
                ("retreat", "motion", released + detached, released + detached, ("arm_clear",)),
            )
        phases = []
        for name, role, before, during, after in spec:
            stage = operation + "_" + name
            # One wrapper per stage, while shared underlying ports serialize
            # motion/gripper ownership. Resolution is lazy, never preplanned.
            if role == "probe":
                # The dedicated backend owns the checked short stroke. A caller cannot supply a pose.
                port = self.probe
                args = {key: parameters[key] for key in ("object_id", "arm")}
                timeout = self.timeouts["motion"]
            else:
                port = ResolvedPort(stage, getattr(self, role), self.resolve, self.clock)
                args, timeout = deepcopy(parameters), self.timeouts[role]
            phases.append(Phase(stage, port, args, stationary + before,
                                stationary + during, after, timeout))
        return tuple(phases)

    def start(self, goal_id, parameters):
        self.sequence.start(goal_id, parameters)

    def poll(self, goal_id):
        return self.sequence.poll(goal_id)

    def cancel(self, goal_id):
        # No automatic reopen, detach or reverse motion during cancellation.
        self.sequence.cancel(goal_id)

    def phase_safe(self, goal_id):
        return self.sequence.phase_safe(goal_id)

    def history(self, goal_id):
        return deepcopy(self.sequence.entries[goal_id]["history"])
