"""Recorded/synthetic RGB-D geometry with explicit freshness and transform IDs."""

from dataclasses import dataclass
import math
from statistics import median
from .navigation import number, identifier
from .fetch import InvalidTask


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


def depth_target(
    samples_uv_depth,
    intrinsics: CameraIntrinsics,
    *,
    observed_s,
    now_s,
    max_age_s,
    camera_frame,
    expected_camera_frame,
    calibration_id,
    expected_calibration_id,
    root_from_camera,
    root_frame,
    transform_observed_s,
    minimum_valid_samples=5,
    maximum_depth_spread_m=0.05
):
    stamp, now, age = (number(v, "time") for v in (observed_s, now_s, max_age_s))
    if age <= 0 or stamp < 0 or now < stamp or now - stamp > age:
        raise InvalidTask("stale_observation")
    if (
        not camera_frame
        or camera_frame != expected_camera_frame
        or not calibration_id
        or calibration_id != expected_calibration_id
    ):
        raise InvalidTask("frame_or_calibration_mismatch")
    identifier(root_frame, "root frame")
    tf_stamp = number(transform_observed_s, "transform time")
    if (
        tf_stamp < 0
        or tf_stamp > now
        or now - tf_stamp > age
        or abs(tf_stamp - stamp) > age
    ):
        raise InvalidTask("stale_transform")
    fx, fy, cx, cy = (
        number(v, "intrinsics")
        for v in (intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy)
    )
    if (
        fx <= 0
        or fy <= 0
        or type(minimum_valid_samples) is not int
        or minimum_valid_samples < 3
    ):
        raise InvalidTask("invalid depth configuration")
    spread = number(maximum_depth_spread_m, "depth spread")
    if spread <= 0:
        raise InvalidTask("invalid depth spread")
    if not isinstance(samples_uv_depth, (list, tuple)) or len(samples_uv_depth) > 10000:
        raise InvalidTask("bounded depth ROI required")
    points = []
    for sample in samples_uv_depth:
        if not isinstance(sample, (list, tuple)) or len(sample) != 3:
            raise InvalidTask("invalid depth sample")
        u, v, z = (number(x, "depth sample") for x in sample)
        if z > 0:
            points.append(((u - cx) * z / fx, (v - cy) * z / fy, z))
    if len(points) < minimum_valid_samples:
        raise InvalidTask("insufficient_depth")
    depth = median(p[2] for p in points)
    points = [p for p in points if abs(p[2] - depth) <= spread]
    if len(points) < minimum_valid_samples:
        raise InvalidTask("inconsistent_depth")
    xyz = tuple(median(p[i] for p in points) for i in range(3))
    matrix = root_from_camera
    if (
        not isinstance(matrix, (list, tuple))
        or len(matrix) != 4
        or any(not isinstance(row, (list, tuple)) or len(row) != 4 for row in matrix)
    ):
        raise InvalidTask("4x4 transform required")
    matrix = [[number(v, "transform") for v in row] for row in matrix]
    if matrix[3] != [0, 0, 0, 1]:
        raise InvalidTask("invalid homogeneous transform")
    for i in range(3):
        for j in range(3):
            if (
                abs(
                    sum(matrix[i][k] * matrix[j][k] for k in range(3))
                    - (1 if i == j else 0)
                )
                > 1e-6
            ):
                raise InvalidTask("transform rotation is not orthonormal")
    r = matrix
    det = (
        r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
    )
    if abs(det - 1) > 1e-6:
        raise InvalidTask("reflected transform")
    return {
        "position_m": tuple(
            sum(matrix[i][j] * xyz[j] for j in range(3)) + matrix[i][3]
            for i in range(3)
        ),
        "frame_id": root_frame,
        "observed_s": stamp,
        "calibration_id": calibration_id,
        "valid_depth_samples": len(points),
        "motion_authorized": False,
    }
