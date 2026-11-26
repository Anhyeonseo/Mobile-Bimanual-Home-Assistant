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


def surface_window(heights):
    return [dict(capture_id=f'frame-{i}',observed_s=2+i*.25,height_m=h,
                 pose_revision='stopped-1',surface_id='table',calibration_id='cal-1')
            for i,h in enumerate(heights)]


def test_surface_window_rejects_bounce_but_not_one_depth_spike():
    from home_robot_tasks.stationary_rgbd import settled_surface_height
    steady=surface_window([.318,.319,.318,.340,.318,.317,.318,.318])
    r=settled_surface_height(steady,stopped_s=0.)
    assert r['status']=='SETTLED_SURFACE' and r['height_m']==pytest.approx(.318)
    assert 'frame-3' not in r['retained_capture_ids']
    assert not r['motion_authorized'] and not r['physical_accuracy_verified']
    assert settled_surface_height(surface_window([.31,.33]*4),stopped_s=0.)['status']=='RETRY_CAPTURE'
    assert settled_surface_height(surface_window([.318]*8),stopped_s=2.)['status']=='RETRY_CAPTURE'


def test_surface_window_never_mixes_moved_pose_or_duplicate_frames():
    from home_robot_tasks.stationary_rgbd import settled_surface_height
    rows=surface_window([.318]*8);rows[-1]['pose_revision']='moved-2'
    with pytest.raises(InvalidTask,match='configuration changed'):
        settled_surface_height(rows,stopped_s=0.)
    rows=surface_window([.318]*8);rows[-1]['capture_id']='frame-0'
    with pytest.raises(InvalidTask,match='unique ordered'):
        settled_surface_height(rows,stopped_s=0.)


def test_surface_window_does_not_confuse_stability_with_height_accuracy():
    from home_robot_tasks.stationary_rgbd import settled_surface_height
    r=settled_surface_height(surface_window([.333]*8),stopped_s=0.)
    assert r['status']=='SETTLED_SURFACE' and r['height_m']==pytest.approx(.333)
    assert not r['physical_accuracy_verified']


def test_observed_floor_height_handles_pose_offset_without_changing_arm_transform():
    import numpy as np
    from home_robot_tasks.stationary_rgbd import observed_floor_height_model
    profile=json.loads((ROOT/'config/home_assistant/s2_coordinates.development.json').read_text())
    transform=np.array(profile['base_from_optical']);original=transform.copy()
    x,y=np.meshgrid(np.linspace(.25,1.45,50),np.linspace(-.85,.85,50))
    floor=np.c_[x.ravel(),y.ravel(),profile['ground_z_m']+.01+.015*x.ravel()+.008*y.ravel()]
    normal=np.array([-.015,-.008,1.]);normal/=np.linalg.norm(normal)
    target=np.array([.8,.2,profile['ground_z_m']+.01+.015*.8+.008*.2])+.318*normal
    optical=(floor-transform[:3,3])@transform[:3,:3]
    target_optical=(target-transform[:3,3])@transform[:3,:3]
    model,evidence=observed_floor_height_model(ROOT,optical)
    assert evidence['floor_observed'] and not evidence['rigid_transform_changed']
    assert not evidence['arm_calibration_changed']
    assert model.heights(target_optical)==pytest.approx(.318,abs=1e-7)
    np.testing.assert_array_equal(transform,original)


def test_observed_floor_height_explicitly_falls_back_without_floor():
    import numpy as np
    from home_robot_tasks.stationary_rgbd import observed_floor_height_model,load_surface_height_model
    cloud=np.array([[0.,0.,1.],[.1,0.,1.],[0.,.1,1.]])
    model,evidence=observed_floor_height_model(ROOT,cloud)
    assert evidence['source']=='calibrated_height_fallback' and not evidence['floor_observed']
    assert model.model_id==load_surface_height_model(ROOT).model_id


def test_calibrated_height_is_absolute_distance_dependent_and_leaves_input_unchanged():
    import numpy as np
    from home_robot_tasks.stationary_rgbd import load_surface_height_model
    height=load_surface_height_model(ROOT)
    points=np.array([[0.,0.,.4],[0.,0.,1.],[.1,0.,1.]])
    original=points.copy();actual=height.heights(points)
    # Independent full precision row: z row of the measured camera in
    # measured_calibration.development.json (2026-11-17 joint calibration).
    # A fixed 9.1mm or 14.6mm subtraction cannot satisfy both depths and lateral roll.
    r=json.loads((ROOT/'config/home_assistant/measured_calibration.development.json').read_text())['base_from_optical'][2]
    expected=r[3]-height.ground_z_m + np.array([r[2]*.4,r[2],r[2]+r[0]*.1])
    np.testing.assert_allclose(actual,expected,atol=1e-12,rtol=0)
    np.testing.assert_array_equal(points,original)
    assert height.heights([0.,0.,0.])==pytest.approx(r[3]-height.ground_z_m)
    assert not np.isclose(actual[1]-actual[0],-.7071067811865475*.6)


def test_height_model_rejects_tampered_projection_and_wrong_profile(tmp_path):
    from home_robot_tasks.stationary_rgbd import load_surface_height_model
    folder=tmp_path/'config/home_assistant';folder.mkdir(parents=True)
    for name in ('camera_height.calibrated.json','s2_coordinates.development.json'):
        (folder/name).write_bytes((ROOT/'config/home_assistant'/name).read_bytes())
    cfg=json.loads((folder/'camera_height.calibrated.json').read_text())
    cfg['base_z_from_optical'][3]-=.0091
    (folder/'camera_height.calibrated.json').write_text(json.dumps(cfg))
    with pytest.raises(InvalidTask,match='binding mismatch'):load_surface_height_model(tmp_path)
    (folder/'camera_height.calibrated.json').write_bytes((ROOT/'config/home_assistant/camera_height.calibrated.json').read_bytes())
    profile=json.loads((folder/'s2_coordinates.development.json').read_text());profile['calibration_id']='other'
    (folder/'s2_coordinates.development.json').write_text(json.dumps(profile))
    with pytest.raises(InvalidTask,match='binding mismatch'):load_surface_height_model(tmp_path)


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
