"""Bounded viewpoint search, driven by explicit arrivals and RGB-D observations.

This is a PC coordinator, not a navigation/arm controller. MOVE_TO_VIEW names a
catalog entry; a future adapter must check the current map/path, transport pose,
lift limits and collision clearance before moving, then supply measured arrival.
Visibility comes from the observer, never from intended camera coverage. Missing
objects are never declared absent. All positions remain unapproved grasp inputs.
"""

from copy import deepcopy
from dataclasses import dataclass

from .fetch import InvalidTask
from .navigation import identifier, number
from .stationary_rgbd import StationaryRGBD


@dataclass(frozen=True)
class Viewpoint:
    view_id: str
    location: str
    expected_regions: tuple[str, ...]


@dataclass(frozen=True)
class SearchAction:
    action_id: str
    kind: str
    view_id: str | None
    issued_s: float


@dataclass(frozen=True)
class Detection:
    object_id: str
    confidence: float
    samples_uv_depth: tuple


class SearchSession:
    TERMINAL = {"FOUND", "NOT_FOUND_UNCONFIRMED", "CANCELLED", "FAILED"}

    def __init__(
        self,
        run_id,
        object_id,
        location,
        scene_revision,
        viewpoints,
        camera: StationaryRGBD,
        *,
        now_s=0,
        timeout_s=120,
        captures_per_view=2,
        minimum_confidence=0.7,
    ):
        self.run_id = identifier(run_id, "search run")
        self.object_id = identifier(object_id, "object id")
        self.location = identifier(location, "location")
        self.scene_revision = identifier(scene_revision, "scene revision")
        if not isinstance(viewpoints, (list, tuple)) or not 1 <= len(viewpoints) <= 32:
            raise InvalidTask("bounded viewpoint catalog required")
        self.views = tuple(viewpoints)
        seen = set()
        for view in self.views:
            if not isinstance(view, Viewpoint):
                raise InvalidTask("viewpoint required")
            identifier(view.view_id, "view id")
            if view.view_id in seen or view.location != location:
                raise InvalidTask("duplicate or wrong-location viewpoint")
            seen.add(view.view_id)
            if (
                not isinstance(view.expected_regions, tuple)
                or not 1 <= len(view.expected_regions) <= 64
                or len(set(view.expected_regions)) != len(view.expected_regions)
            ):
                raise InvalidTask("bounded unique regions required")
            for region in view.expected_regions:
                identifier(region, "region")
        if type(captures_per_view) is not int or not 1 <= captures_per_view <= 3:
            raise InvalidTask("invalid capture retry budget")
        self.timeout = number(timeout_s, "timeout")
        self.confidence = number(minimum_confidence, "confidence")
        self.now = number(now_s, "now")
        if self.timeout <= 0 or self.now < 0 or not 0 < self.confidence <= 1:
            raise InvalidTask("invalid search budget")
        self.started_s = self.now
        self.deadline = self.now + self.timeout
        self.camera, self.capture_limit = camera, captures_per_view
        self.regions = set(r for view in self.views for r in view.expected_regions)
        self.visible, self.occluded, self.visited = set(), set(), set()
        self.capture_ids = set()
        self.state, self.reason = "SELECT_VIEW", ""
        self.action = None
        self.view = None
        self.pose_revision = None
        self.target = None
        self.captures = self.sequence = 0
        self.history = []
        self.stop_destination = "FAILED"

    def _clock(self, now_s):
        now = number(now_s, "now")
        if now < self.now:
            raise InvalidTask("search clock moved backwards")
        self.now = now
        if self.state not in self.TERMINAL | {"STOPPING"} and now >= self.deadline:
            self._stop("search_timeout", "FAILED")

    def _issue(self, kind):
        self.sequence += 1
        self.action = SearchAction(
            f"{self.run_id}:{self.sequence}",
            kind,
            self.view.view_id if self.view else None,
            self.now,
        )
        self.history.append(
            {"action": kind, "view_id": self.action.view_id, "at_s": self.now}
        )
        return self.action

    def _matching(self, action_id, kind):
        if (
            self.action is None
            or self.action.action_id != action_id
            or self.action.kind != kind
        ):
            raise InvalidTask("wrong_or_late_search_action")

    def _stop(self, reason, destination):
        if self.state == "STOPPING":
            return
        self.state, self.reason, self.stop_destination = "STOPPING", reason, destination
        self.target = None
        self._issue("STOP_SEARCH")

    def cancel(self, now_s):
        self._clock(now_s)
        if self.state not in self.TERMINAL:
            self._stop("cancelled", "CANCELLED")

    def invalidate_scene(self, scene_revision, now_s):
        identifier(scene_revision, "scene revision")
        self._clock(now_s)
        if scene_revision != self.scene_revision and self.state not in self.TERMINAL:
            self._stop("scene_changed", "FAILED")

    def poll(self, motion, now_s):
        self._clock(now_s)
        if self.state in self.TERMINAL:
            return None
        if self.state in {"STOPPING", "MOVING"}:
            return self.action
        try:
            since = self.camera.stationary_since(motion, self.now)
            if (
                self.state in {"SETTLING", "OBSERVING"}
                and motion.pose_revision != self.pose_revision
            ):
                raise InvalidTask("pose_changed_during_observation")
        except InvalidTask as error:
            self._stop(str(error), "FAILED")
            return self.action
        if self.state == "SELECT_VIEW":
            remaining = [v for v in self.views if v.view_id not in self.visited]
            if not remaining:
                self.state, self.reason = (
                    "NOT_FOUND_UNCONFIRMED",
                    "viewpoints_exhausted",
                )
                return None
            # Stable ties preserve catalog priority; revisit neither blocked nor observed views.
            self.view = max(
                remaining, key=lambda v: len(set(v.expected_regions) - self.visible)
            )
            self.captures = 0
            self.state = "MOVING"
            return self._issue("MOVE_TO_VIEW")
        if self.state == "SETTLING" and self.now >= since + self.camera.policy.settle_s:
            self.state = "OBSERVING"
            return self._issue("CAPTURE_RGBD")
        return self.action

    def arrive(self, action_id, motion, now_s, *, view_reached, backend_idle):
        self._clock(now_s)
        self._matching(action_id, "MOVE_TO_VIEW")
        since = self.camera.stationary_since(motion, self.now)
        if (
            view_reached is not True
            or backend_idle is not True
            or motion.observed_s < self.action.issued_s
            or since < self.action.issued_s
        ):
            raise InvalidTask("fresh_measured_arrival_required")
        self.pose_revision = motion.pose_revision
        self.action, self.state = None, "SETTLING"

    def skip_unavailable(self, action_id, motion, now_s, *, backend_idle):
        self._clock(now_s)
        self._matching(action_id, "MOVE_TO_VIEW")
        self.camera.stationary_since(motion, self.now)
        if backend_idle is not True or motion.observed_s < self.action.issued_s:
            raise InvalidTask("unavailable_view_still_active")
        self.history.append({"event": "view_unavailable", "view_id": self.view.view_id})
        self._next_view()

    def _next_view(self):
        self.visited.add(self.view.view_id)
        self.action, self.state = None, "SELECT_VIEW"

    def observation(
        self,
        action_id,
        frame,
        motion,
        now_s,
        *,
        visible_regions=(),
        occluded_regions=(),
        detections=(),
    ):
        self._clock(now_s)
        self._matching(action_id, "CAPTURE_RGBD")
        if motion.pose_revision != self.pose_revision:
            self._stop("pose_changed_during_observation", "FAILED")
            return False
        try:
            self.camera.validate_frame(
                frame, motion, now_s=self.now, requested_s=self.action.issued_s
            )
            if frame.capture_id in self.capture_ids:
                raise InvalidTask("duplicate_capture")
            visible, occluded = self._regions(visible_regions), self._regions(
                occluded_regions
            )
            if visible & occluded:
                raise InvalidTask("contradictory_visibility")
            if not isinstance(detections, (tuple, list)) or len(detections) > 64:
                raise InvalidTask("bounded detections required")
            candidates = []
            for item in detections:
                if not isinstance(item, Detection):
                    raise InvalidTask("detection required")
                identifier(item.object_id, "detected object")
                confidence = number(item.confidence, "detection confidence")
                if not 0 <= confidence <= 1:
                    raise InvalidTask("invalid detection confidence")
                if item.object_id == self.object_id and confidence >= self.confidence:
                    try:
                        position = self.camera.locate(
                            frame,
                            motion,
                            item.samples_uv_depth,
                            now_s=self.now,
                            requested_s=self.action.issued_s,
                        )
                        candidates.append((confidence, position))
                    except InvalidTask:
                        # RGB recognition is not a usable 3D target without valid depth.
                        pass
        except InvalidTask as error:
            self.captures += 1
            self.history.append({"event": "rejected_observation", "reason": str(error)})
            if self.captures >= self.capture_limit:
                self._next_view()
            else:
                self.action, self.state = None, "SETTLING"
            return False
        self.capture_ids.add(frame.capture_id)
        self.captures += 1
        self.visible |= visible
        self.occluded = (self.occluded | occluded) - self.visible
        self.history.append(
            {
                "event": "observed",
                "view_id": self.view.view_id,
                "capture_id": frame.capture_id,
                "visible_regions": sorted(visible),
            }
        )
        if candidates:
            self.visited.add(self.view.view_id)
            self.target = max(candidates, key=lambda value: value[0])[1]
            self.state, self.action = "FOUND", None
        elif self.captures < self.capture_limit and any(
            d.object_id == self.object_id for d in detections
        ):
            # One bounded retry for uncertain RGB/depth; empty views move on immediately.
            self.action, self.state = None, "SETTLING"
        else:
            self._next_view()
        return True

    def _regions(self, values):
        if not isinstance(values, (tuple, list)) or len(values) > 64:
            raise InvalidTask("bounded observed regions required")
        for value in values:
            identifier(value, "observed region")
        if len(set(values)) != len(values) or not set(values) <= self.regions:
            raise InvalidTask("unknown_or_duplicate_region")
        return set(values)

    def confirm_stop(self, action_id, motion, now_s, *, backend_idle, arms_holding):
        self._clock(now_s)
        self._matching(action_id, "STOP_SEARCH")
        self.camera.stationary_since(motion, self.now)
        if (
            motion.observed_s <= self.action.issued_s
            or backend_idle is not True
            or arms_holding is not True
        ):
            raise InvalidTask("independent_stop_confirmation_required")
        self.state, self.action = self.stop_destination, None

    def report(self):
        return deepcopy(
            {
                "state": self.state,
                "reason": self.reason,
                "object_id": self.object_id,
                "elapsed_s": self.now - self.started_s,
                "capture_requests": sum(
                    e.get("action") == "CAPTURE_RGBD" for e in self.history
                ),
                "location": self.location,
                "scene_revision": self.scene_revision,
                "visible_regions": sorted(self.visible),
                "occluded_regions": sorted(self.occluded),
                "unverified_regions": sorted(self.regions - self.visible),
                "visited_views": sorted(self.visited),
                "target": self.target,
                "history": self.history,
                "absence_proven": False,
                "motion_authorized": False,
                "physical_task_completed": False,
            }
        )
