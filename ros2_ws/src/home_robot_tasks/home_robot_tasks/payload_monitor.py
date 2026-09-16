"""Payload proof from independent gripper measurements and visual observations.

Thresholds are explicit commissioning inputs. Motion completion, commanded jaw
position and a detector's old image cannot establish grasp/release or retention.
No sensor driver, model, physical calibration or MCU evidence writer is implied.
"""
from dataclasses import dataclass
from .fetch import InvalidTask
from .navigation import number, identifier
from .execution import Feedback


@dataclass(frozen=True)
class GripMeasurement:
    sequence: int
    observed_s: float
    gap_mm: float
    effort_raw: float
    healthy: bool


@dataclass(frozen=True)
class PayloadObservation:
    sequence: int
    observed_s: float
    object_id: str
    relation: str  # held / empty / unknown, independently observed at gripper
    destination: str | None = None
    object_at_destination: bool = False
    camera_frame: str = ""


@dataclass(frozen=True)
class PayloadProof:
    state: str
    observed_s: float
    conditions: dict


class PayloadMonitor:
    def __init__(self, *, gap_min_mm, gap_max_mm, open_gap_mm,
                 effort_min_raw, effort_max_raw, empty_effort_max_raw,
                 max_age_s, max_skew_s, dwell_s, camera_frames=None):
        values = tuple(number(v, "payload threshold") for v in (
            gap_min_mm, gap_max_mm, open_gap_mm, effort_min_raw, effort_max_raw,
            empty_effort_max_raw, max_age_s, max_skew_s, dwell_s))
        (self.gap_min, self.gap_max, self.open_gap, self.effort_min, self.effort_max,
         self.empty_effort, self.age, self.skew, self.dwell) = values
        if not (0 < self.gap_min < self.gap_max < self.open_gap
                and 0 <= self.empty_effort < self.effort_min < self.effort_max
                and 0 < self.skew <= self.age <= 2 and 0 < self.dwell <= 2):
            raise InvalidTask("invalid commissioned payload thresholds")
        if camera_frames is not None and not isinstance(camera_frames,(list,tuple)):
            raise InvalidTask("payload camera frame list required")
        self.camera_frames = None if camera_frames is None else tuple(camera_frames)
        if self.camera_frames is not None:
            if not self.camera_frames or len(set(self.camera_frames)) != len(self.camera_frames):
                raise InvalidTask("explicit distinct payload camera frames required")
            for frame in self.camera_frames: identifier(frame, "payload camera")
        self.grip = self.vision = None
        self.sequences = [0, 0]
        self.stamps = [None, None]
        self.intent = "unknown"
        self.object_id = self.destination = None
        self.since = self.signature = self.candidate = self.last_pair_stamp = None
        self.verified = False
        self.intent_at = self.now = 0.0
        self.carrying = False

    def _clock(self, now):
        now = number(now, "payload clock")
        if now < self.now:
            self.grip = self.vision = None
            self._invalidate()
            raise InvalidTask("payload clock moved backwards")
        self.now = now
        return now

    def _invalidate(self):
        self.since = self.signature = self.candidate = self.last_pair_stamp = None
        self.verified = False

    def expect(self, intent, object_id, now_s, *, destination=None):
        now = self._clock(now_s)
        if intent not in {"empty", "grasp", "release"}:
            raise InvalidTask("explicit payload intent required")
        if self.carrying and intent != "release":
            raise InvalidTask("carried object cannot be cleared by a new command")
        obj = identifier(object_id, "payload object")
        if intent == "release":
            if not self.carrying or obj != self.object_id:
                raise InvalidTask("release requires the verified carried object")
            destination = identifier(destination, "payload destination")
        elif destination is not None:
            raise InvalidTask("destination only belongs to release")
        self.intent, self.object_id, self.destination, self.intent_at = intent, obj, destination, now
        self._invalidate()

    def _accept(self, sample, index, now):
        stamp = number(sample.observed_s, "payload sample time")
        seq = sample.sequence
        if (type(seq) is not int or not self.sequences[index] < seq <= 0xFFFFFFFF
                or not 0 <= stamp <= now or now - stamp > self.age
                or (self.stamps[index] is not None and stamp <= self.stamps[index])):
            raise InvalidTask("old, repeated or future payload sample")
        self.sequences[index], self.stamps[index] = seq, stamp

    def observe_grip(self, sample, now_s):
        now = self._clock(now_s)
        try:
            if not isinstance(sample, GripMeasurement) or type(sample.healthy) is not bool:
                raise InvalidTask("measured gripper feedback required")
            if number(sample.gap_mm, "gap") < 0 or number(sample.effort_raw, "effort") < 0:
                raise InvalidTask("invalid measured gripper values")
            self._accept(sample, 0, now)
            self.grip = sample
            if self.candidate is not None:
                matches = (self.gap_min <= sample.gap_mm <= self.gap_max
                           and self.effort_min <= sample.effort_raw <= self.effort_max
                           if self.candidate == "HELD" else
                           sample.gap_mm >= self.open_gap and sample.effort_raw <= self.empty_effort)
                if not sample.healthy or not matches:
                    self._invalidate()
        except Exception:
            self.grip = None
            self._invalidate()
            raise

    def observe_vision(self, sample, now_s):
        now = self._clock(now_s)
        try:
            if (not isinstance(sample, PayloadObservation) or sample.relation not in {"held", "empty", "unknown"}
                    or type(sample.object_at_destination) is not bool):
                raise InvalidTask("independent payload observation required")
            if self.camera_frames is not None and sample.camera_frame not in self.camera_frames:
                raise InvalidTask("unqualified payload camera frame")
            identifier(sample.object_id, "observed object")
            if sample.destination is not None:
                identifier(sample.destination, "observed destination")
            self._accept(sample, 1, now)
            if self.vision is not None and self.vision.camera_frame != sample.camera_frame:
                self._invalidate()
            self.vision = sample
            if self.candidate is not None and (
                sample.object_id != self.object_id
                or sample.relation != ("held" if self.candidate == "HELD" else "empty")
                or (self.candidate == "RELEASED" and (
                    sample.destination != self.destination or not sample.object_at_destination))
            ):
                # A contradictory sample cannot disappear just because a
                # newer good sample arrived before the task's next poll.
                self._invalidate()
        except Exception:
            self.vision = None
            self._invalidate()
            raise

    def proof(self, now_s):
        now = self._clock(now_s)
        g, v = self.grip, self.vision
        valid = (g is not None and v is not None and g.healthy
                 and v.object_id == self.object_id
                 and min(g.observed_s, v.observed_s) >= self.intent_at
                 and now - min(g.observed_s, v.observed_s) <= self.age
                 and abs(g.observed_s - v.observed_s) <= self.skew)
        state = "UNKNOWN"
        stamp = min(g.observed_s, v.observed_s) if g is not None and v is not None else now
        if valid:
            held = (self.gap_min <= g.gap_mm <= self.gap_max
                    and self.effort_min <= g.effort_raw <= self.effort_max and v.relation == "held")
            empty = g.gap_mm >= self.open_gap and g.effort_raw <= self.empty_effort and v.relation == "empty"
            if held and self.intent in {"grasp", "release"}:
                state = "HELD"
            elif empty and self.intent in {"empty", "grasp"} and not self.carrying:
                state = "EMPTY"
            elif empty and self.intent == "release" and v.destination == self.destination and v.object_at_destination:
                state = "RELEASED"
        signature = (g.sequence, v.sequence) if valid else None
        if state == "UNKNOWN":
            self._invalidate()
        elif self.candidate != state:
            self.candidate, self.since, self.signature, self.verified = state, stamp, signature, False
            self.last_pair_stamp = stamp
        elif all(a > b for a, b in zip(signature, self.signature)):
            # Both sources must advance. Repeated polls or one fresh source do
            # not turn a cached image/measurement into a dwell interval.
            if stamp - self.last_pair_stamp > self.age:
                self.since = stamp
            self.signature, self.last_pair_stamp = signature, stamp
            self.verified = stamp - self.since >= self.dwell
        verified = self.verified and state != "UNKNOWN"
        if verified and state == "HELD":
            self.carrying = True
        if verified and state == "RELEASED":
            self.carrying = False
        return PayloadProof(state if verified else "UNKNOWN", stamp, {
            "grasp_verified": verified and state == "HELD",
            "load_retained": verified and state == "HELD",
            "release_verified": verified and state == "RELEASED",
            "gripper_empty": verified and state in {"EMPTY", "RELEASED"},
            "gripper_open": verified and state in {"EMPTY", "RELEASED"},
            "payload_safe": verified and state in {"HELD", "EMPTY", "RELEASED"},
        })


class PayloadTaskMonitor:
    """Overlay independent payload proof on task/phase evidence.

    Pass this callable as both composition monitors (or wrap each environment
    monitor separately around the same PayloadMonitor), and before_start as
    the composition hook. The caller still supplies actual sensor samples.
    """

    def __init__(self, environment_monitor, payload):
        if not callable(environment_monitor) or not isinstance(payload, PayloadMonitor):
            raise InvalidTask("environment and payload monitors required")
        self.environment, self.payload = environment_monitor, payload

    def __call__(self, goal_id, step_or_phase, now_s):
        evidence = self.environment(goal_id, step_or_phase, now_s)
        if not isinstance(evidence, Feedback):
            raise InvalidTask("independent environment feedback required")
        proof = self.payload.proof(now_s)
        return Feedback(evidence.goal_id, min(evidence.observed_s, proof.observed_s),
                        evidence.status, {**evidence.conditions, **proof.conditions}, evidence.reason)

    def before_start(self, skill, parameters, now_s):
        if skill == "pick":
            self.payload.expect("grasp", parameters["object_id"], now_s)
        elif skill == "place":
            self.payload.expect("release", parameters["object_id"], now_s,
                                destination=parameters["destination_place"])
