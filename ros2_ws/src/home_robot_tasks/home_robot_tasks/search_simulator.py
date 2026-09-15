"""Deterministic search-driver examples. No devices, ROS actions or image model.

Arrivals/visibility/ROI samples are synthetic fixtures, not physical simulation.
Run with `python -m home_robot_tasks.search_simulator --config ... --scenario ...`.
"""

import argparse
import json
from pathlib import Path

from .fetch import InvalidTask
from .perception import CameraIntrinsics
from .search import Detection, SearchSession, Viewpoint
from .stationary_rgbd import CapturePolicy, MotionEvidence, RGBDFrame, StationaryRGBD

SCENARIOS = (
    "alternate_found",
    "hidden",
    "invalid_depth",
    "stale_transform",
    "cancel_pending",
)


def simulate_search(config, scenario="alternate_found"):
    if (
        config.get("mode") != "simulation"
        or type(config.get("schema_version")) is not int
        or config["schema_version"] != 1
    ):
        raise InvalidTask("simulation search configuration required")
    if scenario not in SCENARIOS:
        raise InvalidTask("unknown search scenario")
    policy = CapturePolicy(**config["camera"])
    views = [
        Viewpoint(v["view_id"], v["location"], tuple(v["expected_regions"]))
        for v in config["viewpoints"]
    ]
    session = SearchSession(
        "offline-search",
        config["object_id"],
        config["location"],
        config["scene_revision"],
        views,
        StationaryRGBD(policy),
    )
    since, pose = 0.0, "initial"
    current, started = None, 0.0
    for tick in range(1, 3000):
        now = tick / 20
        motion = MotionEvidence(now, since, pose, True, True)
        action = session.poll(motion, now)
        if action is None:
            if session.state in session.TERMINAL:
                break
            continue
        if action.action_id != current:
            current, started = action.action_id, now
        if scenario == "cancel_pending" and action.kind == "MOVE_TO_VIEW":
            session.cancel(now)
            continue
        if now - started < 0.1:
            continue
        if action.kind == "MOVE_TO_VIEW":
            since, pose = now, action.view_id
            session.arrive(
                action.action_id,
                MotionEvidence(now, since, pose, True, True),
                now,
                view_reached=True,
                backend_idle=True,
            )
        elif action.kind == "CAPTURE_RGBD":
            visible = (
                ("seat_left",)
                if action.view_id == "sofa_front"
                else ("seat_right", "armrest")
            )
            occluded = (
                ("seat_right", "armrest") if action.view_id == "sofa_front" else ()
            )
            depth = 0.2 if scenario == "invalid_depth" else 0.8
            detections = ()
            if action.view_id != "sofa_front" and scenario != "hidden":
                detections = (
                    Detection(config["object_id"], 0.95, ((640, 360, depth),) * 6),
                )
            tf_stamp = now - 0.1 if scenario == "stale_transform" else now
            frame = RGBDFrame(
                action.action_id,
                now,
                pose,
                policy.camera_frame,
                policy.calibration_id,
                policy.width,
                policy.height,
                CameraIntrinsics(900, 900, 640, 360),
                policy.root_frame,
                tf_stamp,
                ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0.3), (0, 0, 0, 1)),
                True,
            )
            session.observation(
                action.action_id,
                frame,
                motion,
                now,
                visible_regions=visible,
                occluded_regions=occluded,
                detections=detections,
            )
        elif action.kind == "STOP_SEARCH":
            session.confirm_stop(
                action.action_id, motion, now, backend_idle=True, arms_holding=True
            )
    else:
        raise RuntimeError("simulation exceeded its finite step budget")
    return {
        **session.report(),
        "mode": "simulation",
        "scenario": scenario,
        "hardware_commands": 0,
        "synthetic_observations": True,
        "camera_model": policy.model,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--scenario", choices=SCENARIOS, default="alternate_found")
    args = parser.parse_args()
    print(
        json.dumps(
            simulate_search(json.loads(args.config.read_text()), args.scenario),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
