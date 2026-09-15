from dataclasses import replace
import json
from pathlib import Path

import pytest

from home_robot_tasks.fetch import InvalidTask
from home_robot_tasks.perception import CameraIntrinsics
from home_robot_tasks.stationary_rgbd import (
    CapturePolicy,
    MotionEvidence,
    RGBDFrame,
    StationaryRGBD,
)
from home_robot_tasks.search import Detection, SearchSession, Viewpoint
from home_robot_tasks.search_simulator import simulate_search

ROOT = Path(__file__).resolve().parents[1]


def frame(**changes):
    value = RGBDFrame(
        "frame1",
        1.0,
        "pose1",
        "camera_color_optical_frame",
        "simulation-only",
        1280,
        720,
        CameraIntrinsics(500, 500, 640, 360),
        "base_link",
        1.0,
        ((1, 0, 0, 0.1), (0, 1, 0, 0.2), (0, 0, 1, 0.3), (0, 0, 0, 1)),
        True,
    )
    return replace(value, **changes)


def motion(**changes):
    return replace(MotionEvidence(1.05, 0.5, "pose1", True, True), **changes)


def test_stationary_target_uses_current_transform_and_remains_unapproved():
    camera = StationaryRGBD(CapturePolicy())
    result = camera.locate(
        frame(),
        motion(),
        [(640, 360, 0.8)] * 6 + [(640, 360, 0), (640, 360, 4)],
        now_s=1.05,
        requested_s=0.9,
    )
    assert result["position_m"] == pytest.approx((0.1, 0.2, 1.1))
    assert not result["motion_authorized"] and not result["grasp_pose_resolved"]
    # Current lift transform moves the target in the selected root frame.
    higher = replace(
        frame(),
        root_from_camera=((1, 0, 0, 0.1), (0, 1, 0, 0.2), (0, 0, 1, 0.5), (0, 0, 0, 1)),
    )
    result2 = camera.locate(
        higher, motion(), [(640, 360, 0.8)] * 6, now_s=1.05, requested_s=0.9
    )
    assert result2["position_m"][2] - result["position_m"][2] == pytest.approx(0.2)


@pytest.mark.parametrize(
    "changes",
    [
        {"observed_s": 0.8},
        {"observed_s": 1.2},
        {"pose_revision": "before-lift"},
        {"transform_observed_s": 0.97},
        {"transform_observed_s": 1.1},
        {"calibration_id": "old-calibration"},
        {"camera_frame": "depth_optical"},
        {"root_frame": "map"},
        {"width": 640},
        {"registered_rectified": False},
        {"intrinsics": CameraIntrinsics(0, 500, 640, 360)},
        {"root_from_camera": ((-1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))},
    ],
)
def test_incompatible_or_old_image_never_produces_target(changes):
    with pytest.raises(InvalidTask):
        StationaryRGBD(CapturePolicy()).validate_frame(
            frame(**changes), motion(), now_s=1.05, requested_s=0.9
        )


@pytest.mark.parametrize(
    "changes",
    [
        {"base_stopped": False},
        {"lift_stopped": False},
        {"base_stopped": 1},
        {"stationary_since_s": 0.9},
        {"stationary_since_s": 1.1},
        {"observed_s": 0.4},
        {"observed_s": 2},
        {"pose_revision": "moved-and-returned"},
    ],
)
def test_motion_settling_and_stale_state_rejected(changes):
    with pytest.raises(InvalidTask):
        StationaryRGBD(CapturePolicy()).validate_frame(
            frame(), motion(**changes), now_s=1.05, requested_s=0.9
        )


@pytest.mark.parametrize(
    "samples",
    [
        [(640, 360, 0.2)] * 6,
        [(1280, 360, 0.8)] * 6,
        [(640, 360, float("nan"))] * 6,
        [(640, 360, 0.8)] * 2,
    ],
)
def test_depth_range_pixels_and_minimum_evidence(samples):
    with pytest.raises(InvalidTask):
        StationaryRGBD(CapturePolicy()).locate(
            frame(), motion(), samples, now_s=1.05, requested_s=0.9
        )


def session(**changes):
    args = dict(now_s=0, timeout_s=10, captures_per_view=2)
    args.update(changes)
    return SearchSession(
        "run",
        "remote_control",
        "sofa",
        "map1",
        [
            Viewpoint("front", "sofa", ("left", "right")),
            Viewpoint("side", "sofa", ("right",)),
        ],
        StationaryRGBD(CapturePolicy()),
        **args
    )


def capture_ready(s):
    action = s.poll(motion(observed_s=0, stationary_since_s=0), 0)
    assert action.kind == "MOVE_TO_VIEW"
    assert s.poll(None, 0.1) == action  # no second dispatch while arrival is pending
    s.arrive(
        action.action_id,
        motion(observed_s=0.2, stationary_since_s=0.2),
        0.2,
        view_reached=True,
        backend_idle=True,
    )
    assert s.poll(motion(observed_s=0.3, stationary_since_s=0.2), 0.3) is None
    capture = s.poll(motion(observed_s=0.5, stationary_since_s=0.2), 0.5)
    assert capture.kind == "CAPTURE_RGBD"
    return capture


def observe(s, action, **changes):
    args = dict(visible_regions=("left",), occluded_regions=("right",), detections=())
    args.update(changes)
    return s.observation(
        action.action_id, frame(), motion(stationary_since_s=0.2), 1.05, **args
    )


def test_unseen_regions_prioritized_without_inventing_visibility():
    s = session()
    capture = capture_ready(s)
    assert observe(s, capture)
    assert s.report()["visible_regions"] == ["left"]
    assert s.report()["occluded_regions"] == ["right"]
    assert s.poll(motion(observed_s=1.1, stationary_since_s=0.2), 1.1).view_id == "side"
    assert not s.report()["absence_proven"]


def test_wrong_object_and_invalid_depth_do_not_complete_search():
    s = session()
    capture = capture_ready(s)
    assert observe(
        s, capture, detections=(Detection("cup", 1, ((640, 360, 0.8),) * 6),)
    )
    assert s.state != "FOUND"
    s = session()
    capture = capture_ready(s)
    assert observe(
        s, capture, detections=(Detection("remote_control", 1, ((640, 360, 0.1),) * 6),)
    )
    assert s.state == "SETTLING" and s.target is None


def test_found_requires_identity_confidence_and_depth():
    s = session()
    action = capture_ready(s)
    assert observe(
        s,
        action,
        detections=(Detection("remote_control", 0.95, ((640, 360, 0.8),) * 6),),
    )
    assert s.state == "FOUND"
    report = s.report()
    assert report["target"]["pose_revision"] == "pose1"
    report["target"]["position_m"] = (100, 100, 100)
    assert s.report()["target"]["position_m"] != (100, 100, 100)


def test_late_callback_and_capture_after_motion_never_complete():
    s = session()
    action = capture_ready(s)
    with pytest.raises(InvalidTask, match="late"):
        s.observation("old", frame(), motion(), 1.05)
    assert not s.observation(
        action.action_id, frame(), motion(pose_revision="pose2"), 1.05
    )
    assert s.state == "STOPPING" and s.target is None


def test_rejected_frame_cannot_claim_coverage_and_retries_are_bounded():
    s = session()
    action = capture_ready(s)
    assert not observe(s, action, visible_regions=("invented",))
    assert s.visible == set()
    action2 = s.poll(motion(observed_s=1.1, stationary_since_s=0.2), 1.1)
    assert action2.action_id != action.action_id
    assert not s.observation(action2.action_id, frame(), motion(observed_s=1.15), 1.15)
    assert s.state == "SELECT_VIEW"  # pre-request frame consumed final retry
    assert "front" in s.visited


def test_cancel_requires_fresh_stop_and_idle_backend_before_terminal():
    s = session()
    original = s.poll(motion(observed_s=0, stationary_since_s=0), 0)
    s.cancel(0.1)
    stop = s.action
    with pytest.raises(InvalidTask):
        s.arrive(
            original.action_id,
            motion(observed_s=0.2, stationary_since_s=0.2),
            0.2,
            view_reached=True,
            backend_idle=True,
        )
    with pytest.raises(InvalidTask):
        s.confirm_stop(
            stop.action_id,
            motion(observed_s=0.2, stationary_since_s=0.1),
            0.2,
            backend_idle=False,
            arms_holding=True,
        )
    assert s.state == "STOPPING"
    s.confirm_stop(
        stop.action_id,
        motion(observed_s=0.3, stationary_since_s=0.1),
        0.3,
        backend_idle=True,
        arms_holding=True,
    )
    assert s.state == "CANCELLED"


def test_timeout_and_changed_scene_cancel_inflight_work():
    s = session(timeout_s=1)
    s.poll(motion(observed_s=0, stationary_since_s=0), 0)
    assert s.poll(None, 1).kind == "STOP_SEARCH"
    assert s.reason == "search_timeout"
    s2 = session()
    capture_ready(s2)
    s2.invalidate_scene("map2", 0.6)
    assert s2.state == "STOPPING" and s2.reason == "scene_changed"


def test_unavailable_view_needs_idle_proof_and_is_not_marked_seen():
    s = session()
    action = s.poll(motion(observed_s=0, stationary_since_s=0), 0)
    with pytest.raises(InvalidTask):
        s.skip_unavailable(
            action.action_id,
            motion(observed_s=0.1, stationary_since_s=0),
            0.1,
            backend_idle=False,
        )
    s.skip_unavailable(
        action.action_id,
        motion(observed_s=0.2, stationary_since_s=0),
        0.2,
        backend_idle=True,
    )
    assert s.visible == set() and s.visited == {"front"}


@pytest.mark.parametrize(
    "scenario,state",
    [
        ("alternate_found", "FOUND"),
        ("hidden", "NOT_FOUND_UNCONFIRMED"),
        ("invalid_depth", "NOT_FOUND_UNCONFIRMED"),
        ("stale_transform", "NOT_FOUND_UNCONFIRMED"),
        ("cancel_pending", "CANCELLED"),
    ],
)
def test_end_to_end_pc_search_scenarios(scenario, state):
    config = json.loads((ROOT / "config/search_sofa.simulation.json").read_text())
    report = simulate_search(config, scenario)
    assert report["state"] == state
    assert report["hardware_commands"] == 0
    assert not report["absence_proven"] and not report["physical_task_completed"]
    if state == "FOUND":
        moves = [
            e["view_id"] for e in report["history"] if e.get("action") == "MOVE_TO_VIEW"
        ]
        assert moves == ["sofa_front", "sofa_high"]


def test_d435_is_an_explicit_profile_switch_without_firmware_change():
    config = json.loads((ROOT / "config/search_sofa.simulation.json").read_text())
    config["camera"]["model"] = "D435"
    assert simulate_search(config)["camera_model"] == "D435"
