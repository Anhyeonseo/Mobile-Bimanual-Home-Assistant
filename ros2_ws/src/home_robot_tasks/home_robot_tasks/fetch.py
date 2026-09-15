"""Compile a household fetch request into a bounded, non-executable task plan.

Rooms and locations are semantic names, not navigation poses. A physical
executor must resolve them against a commissioned map and fresh observations.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Mapping


class InvalidTask(ValueError):
    """The request or world description cannot define an unambiguous plan."""


def _identifier(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise InvalidTask(f"{field} must be a non-empty string")
    return value.strip()


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, dict) or not value:
        raise InvalidTask(f"{field} must be a non-empty object")
    return value


@dataclass(frozen=True)
class FetchRequest:
    object_id: str
    source_room: str
    destination_place: str

    @classmethod
    def from_dict(cls, document: Mapping[str, Any]) -> FetchRequest:
        if not isinstance(document, dict):
            raise InvalidTask("request must be an object")
        if document.get("task") != "fetch_object":
            raise InvalidTask("only fetch_object is implemented")
        expected = {"task", "object_id", "source_room", "destination_place"}
        if set(document) != expected:
            raise InvalidTask("request fields must be task, object_id, source_room, destination_place")
        return cls(*(_identifier(document[key], key) for key in
                     ("object_id", "source_room", "destination_place")))


@dataclass(frozen=True)
class TaskStep:
    skill: str
    parameters: dict[str, Any]
    success_requires: tuple[str, ...]
    timeout_s: int
    max_attempts: int = 1
    start_requires: tuple[str, ...] = ()


def plan_fetch(request: FetchRequest, world: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(world, dict) or type(world.get("schema_version")) is not int or world["schema_version"] != 1:
        raise InvalidTask("world schema_version must be 1")
    if world.get("transport_mode") != "carry_in_gripper":
        raise InvalidTask("this planning example supports carry_in_gripper only")
    rooms = _mapping(world.get("rooms"), "rooms")
    places = _mapping(world.get("places"), "places")
    objects = _mapping(world.get("objects"), "objects")
    object_id = _identifier(request.object_id, "object_id")
    room_id = _identifier(request.source_room, "source_room")
    destination = _identifier(request.destination_place, "destination_place")
    if object_id not in objects or not isinstance(objects[object_id], dict):
        raise InvalidTask(f"unknown object: {object_id}")
    if room_id not in rooms or not isinstance(rooms[room_id], dict):
        raise InvalidTask(f"unknown source room: {room_id}")
    place = places.get(destination)
    if not isinstance(place, dict):
        raise InvalidTask("destination must name a registered place with a known room")
    destination_room = _identifier(place.get("room"), "destination room")
    if destination_room not in rooms or not isinstance(rooms[destination_room], dict):
        raise InvalidTask("destination must belong to a known room")
    locations = rooms[room_id].get("search_locations")
    if not isinstance(locations, list) or not locations:
        raise InvalidTask("source room needs a finite list of search locations")
    locations = [_identifier(location, "search_location") for location in locations]
    if len(set(locations)) != len(locations):
        raise InvalidTask("search locations must be unique")
    if any(not isinstance(places.get(location), dict) or places[location].get("room") != room_id
           for location in locations):
        raise InvalidTask("each search location must belong to the source room")
    target = {"object_id": object_id}
    stationary = ("base_stopped", "lift_stopped")
    travel_ready = ("arm_in_transport_pose", "lift_in_transport_position")
    destination_target = {**target, "destination_place": destination}
    steps = (
        TaskStep("prepare_navigation", {}, (*stationary, *travel_ready), 30,
                 start_requires=(*stationary, "hardware_ready")),
        TaskStep("navigate", {"room": room_id},
                 ("localized_in_source_room", "base_stopped", *travel_ready), 180,
                 start_requires=(*stationary, *travel_ready, "localized")),
        TaskStep("search", {**target, "locations": locations},
                 ("target_observed", "target_pose_resolved", "fresh_observation"), 120,
                 start_requires=stationary),
        TaskStep("align_for_pick", target, ("base_stopped", "alignment_verified"), 30,
                 start_requires=(*stationary, "base_control_available", "arms_safe_for_alignment", "fresh_target_pose")),
        TaskStep("adjust_lift_for_pick", target, ("lift_stopped", "lift_height_verified"), 30,
                 start_requires=(*stationary, "arms_safe_for_lift", "lift_homed", "fresh_work_surface")),
        TaskStep("reobserve_target", target,
                 ("fresh_observation", "target_pose_resolved", "transform_valid", "reachable"), 30,
                 start_requires=stationary),
        TaskStep("pick", target, (*stationary, "grasp_verified"), 60,
                 start_requires=(*stationary, "fresh_target_pose", "transform_valid", "collision_checked")),
        TaskStep("prepare_transport", target, (*stationary, *travel_ready, "load_retained"), 30,
                 start_requires=(*stationary, "grasp_verified")),
        TaskStep("navigate_with_load", {"destination_place": destination},
                 ("localized_at_destination", "load_retained", "base_stopped", *travel_ready), 180,
                 start_requires=(*stationary, *travel_ready, "load_retained", "localized")),
        TaskStep("inspect_destination", destination_target,
                 ("fresh_observation", "placement_surface_valid"), 30,
                 start_requires=(*stationary, "load_retained")),
        TaskStep("align_for_place", destination_target, ("base_stopped", "alignment_verified", "load_retained"), 30,
                 start_requires=(*stationary, "base_control_available", "arms_safe_for_alignment", "load_retained", "fresh_work_surface")),
        TaskStep("adjust_lift_for_place", destination_target,
                 ("lift_stopped", "lift_height_verified", "load_retained"), 30,
                 start_requires=(*stationary, "arms_safe_for_lift", "load_retained", "lift_homed", "fresh_work_surface")),
        TaskStep("reobserve_destination", destination_target,
                 ("fresh_observation", "placement_surface_valid", "transform_valid", "reachable"), 30,
                 start_requires=(*stationary, "load_retained")),
        TaskStep("place", destination_target,
                 (*stationary, "release_verified", "arm_clear"), 60,
                 start_requires=(*stationary, "load_retained", "fresh_work_surface", "transform_valid", "collision_checked")),
        TaskStep("verify_delivery", destination_target,
                 ("fresh_observation", "object_at_destination"), 30,
                 start_requires=stationary),
    )
    return {
        "schema_version": 1,
        "task": "fetch_object",
        "request": asdict(FetchRequest(object_id, room_id, destination)),
        "mode": "plan_only",
        "motion_authorized": False,
        "executable": False,
        "physical_task_completed": False,
        "transport_mode": "carry_in_gripper",
        "steps": [{"step_id": f"{index:02d}_{step.skill}", **asdict(step)}
                  for index, step in enumerate(steps, 1)],
        "failure_policy": {
            "automatic_retry": False,
            "abort_on": ["timeout", "object_not_found", "stale_observation", "localization_lost",
                         "grasp_failed", "load_lost", "path_blocked", "release_failed", "cancelled"],
            "required_response": "stop_base_stop_lift_preserve_load_then_report",
        },
        "unimplemented_dependencies": ["navigation", "perception", "fine_alignment", "lift",
                                       "pick_and_place", "physical_task_executor"],
    }
