import pytest
from home_robot_tasks.classical import route_for_arm
from home_robot_tasks.perception import CameraIntrinsics, depth_target
from home_robot_tasks.fetch import InvalidTask


def route(arm="left", **changes):
    values = dict(
        current=(0,) * 12,
        opposite_hold=(0,) * 6,
        arm=arm,
        steps=[
            {"kind": "arm", "target_positions_rad": [0.2, 0.1, 0, 0, 0]},
            {"kind": "gripper", "target_position_rad": 0.3},
        ],
        minimum_rad=(-1,) * 12,
        maximum_rad=(1,) * 12,
        max_speed_rad_s=(0.5,) * 12,
    )
    values.update(changes)
    return route_for_arm(**values)


@pytest.mark.parametrize("arm", ["left", "right"])
def test_reused_route_preserves_other_arm_and_bounded_velocity(arm):
    points = route(arm)
    idx = 0 if arm == "left" else 6
    other = 6 - idx
    assert points[0]["positions_rad"] == (0,) * 12
    assert points[-1]["positions_rad"][idx : idx + 6] == (0.2, 0.1, 0, 0, 0, 0.3)
    for a, b in zip(points, points[1:]):
        assert b["offset_ms"] > a["offset_ms"]
        assert b["positions_rad"][other : other + 6] == (0,) * 6
        assert (
            max(abs(x - y) for x, y in zip(a["positions_rad"], b["positions_rad"]))
            <= 0.025000001
        )


@pytest.mark.parametrize(
    "change",
    [
        {"opposite_hold": (0.1,) * 6},
        {"max_speed_rad_s": (0,) * 12},
        {"maximum_rad": (0.1,) * 12},
        {"current": (float("nan"),) * 12},
    ],
)
def test_route_rejects_unmeasured_hold_bad_limits_and_nonfinite(change):
    with pytest.raises(InvalidTask):
        route(**change)


def target(**changes):
    values = dict(
        samples_uv_depth=[(320, 240, 1)] * 6 + [(320, 240, 9), (320, 240, 0)],
        intrinsics=CameraIntrinsics(500, 500, 320, 240),
        observed_s=1,
        now_s=1.1,
        max_age_s=0.5,
        camera_frame="optical",
        expected_camera_frame="optical",
        calibration_id="cal1",
        expected_calibration_id="cal1",
        root_from_camera=[[1, 0, 0, 0.1], [0, 1, 0, 0.2], [0, 0, 1, 0.3], [0, 0, 0, 1]],
        root_frame="base_link",
        transform_observed_s=1,
    )
    values.update(changes)
    return depth_target(**values)


def test_depth_roi_filters_invalid_and_outliers_in_named_fresh_frame():
    result = target()
    assert result["position_m"] == pytest.approx((0.1, 0.2, 1.3))
    assert (
        result["valid_depth_samples"] == 6
        and result["frame_id"] == "base_link"
        and not result["motion_authorized"]
    )


@pytest.mark.parametrize(
    "change",
    [
        {"observed_s": 0},
        {"transform_observed_s": 0},
        {"calibration_id": "old"},
        {"camera_frame": "wrong"},
        {"samples_uv_depth": [(0, 0, 0)] * 6},
        {"root_from_camera": [[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]},
    ],
)
def test_stale_missing_or_wrong_geometry_rejected(change):
    with pytest.raises(InvalidTask):
        target(**change)
