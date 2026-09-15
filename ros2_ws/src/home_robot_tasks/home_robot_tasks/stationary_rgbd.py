"""Stationary manipulation observations, independent of the navigation sensors.

Input pixels must be rectified colour pixels with depth registered to that same
optical frame, in metres. The producer owns image registration and TF lookup at
exposure time. A pose revision must change on ANY base/lift motion, even if the
robot returns to its former pose. This module never authorizes motor commands.
"""

from dataclasses import dataclass

from .fetch import InvalidTask
from .navigation import identifier, number
from .perception import CameraIntrinsics, depth_target


@dataclass(frozen=True)
class CapturePolicy:
    model: str = "D415"
    camera_frame: str = "camera_color_optical_frame"
    root_frame: str = "base_link"
    calibration_id: str = "simulation-only"
    width: int = 1280
    height: int = 720
    minimum_depth_m: float = 0.5
    maximum_depth_m: float = 3.0
    settle_s: float = 0.25
    maximum_age_s: float = 0.5
    maximum_tf_skew_s: float = 0.02

    def __post_init__(self):
        if self.model not in {"D415", "D435"}:
            raise InvalidTask("unsupported camera profile")
        for name in ("camera_frame", "root_frame", "calibration_id"):
            identifier(getattr(self, name), name)
        for name in ("width", "height"):
            if (
                type(getattr(self, name)) is not int
                or not 1 <= getattr(self, name) <= 8192
            ):
                raise InvalidTask("invalid image dimensions")
        for name in (
            "minimum_depth_m",
            "maximum_depth_m",
            "settle_s",
            "maximum_age_s",
            "maximum_tf_skew_s",
        ):
            if number(getattr(self, name), name) <= 0:
                raise InvalidTask("positive capture limits required")
        if (
            self.minimum_depth_m >= self.maximum_depth_m
            or self.maximum_tf_skew_s > self.maximum_age_s
        ):
            raise InvalidTask("invalid capture limits")


@dataclass(frozen=True)
class MotionEvidence:
    observed_s: float
    stationary_since_s: float
    pose_revision: str
    base_stopped: bool
    lift_stopped: bool


@dataclass(frozen=True)
class RGBDFrame:
    capture_id: str
    observed_s: float
    pose_revision: str
    camera_frame: str
    calibration_id: str
    width: int
    height: int
    intrinsics: CameraIntrinsics
    root_frame: str
    transform_observed_s: float
    root_from_camera: tuple
    registered_rectified: bool


class StationaryRGBD:
    def __init__(self, policy: CapturePolicy):
        self.policy = policy

    def stationary_since(self, motion: MotionEvidence, now_s):
        if not isinstance(motion, MotionEvidence):
            raise InvalidTask("motion evidence required")
        now = number(now_s, "now")
        stamp = number(motion.observed_s, "motion timestamp")
        since = number(motion.stationary_since_s, "stationary timestamp")
        identifier(motion.pose_revision, "pose revision")
        if not 0 <= since <= stamp <= now or now - stamp > self.policy.maximum_age_s:
            raise InvalidTask("stale_motion_evidence")
        if motion.base_stopped is not True or motion.lift_stopped is not True:
            raise InvalidTask("platform_not_stationary")
        return since

    def validate_frame(
        self, frame: RGBDFrame, motion: MotionEvidence, *, now_s, requested_s
    ):
        since = self.stationary_since(motion, now_s)
        if not isinstance(frame, RGBDFrame):
            raise InvalidTask("RGB-D frame required")
        p = self.policy
        identifier(frame.capture_id, "capture id")
        stamp = number(frame.observed_s, "exposure timestamp")
        request = number(requested_s, "capture request timestamp")
        tf_stamp = number(frame.transform_observed_s, "transform timestamp")
        if not 0 <= request <= stamp <= now_s or now_s - stamp > p.maximum_age_s:
            raise InvalidTask("stale_or_pre_request_frame")
        if stamp < since + p.settle_s or frame.pose_revision != motion.pose_revision:
            raise InvalidTask("frame_before_settling_or_pose_changed")
        if (
            tf_stamp < since
            or tf_stamp > now_s
            or abs(tf_stamp - stamp) > p.maximum_tf_skew_s
        ):
            raise InvalidTask("transform_not_at_exposure")
        if (
            frame.camera_frame != p.camera_frame
            or frame.root_frame != p.root_frame
            or frame.calibration_id != p.calibration_id
        ):
            raise InvalidTask("frame_or_calibration_mismatch")
        if (
            type(frame.width) is not int
            or type(frame.height) is not int
            or (frame.width, frame.height) != (p.width, p.height)
            or frame.registered_rectified is not True
        ):
            raise InvalidTask("registered_rectified_profile_required")
        # Validate intrinsics and rigid transform even for a frame with no detections.
        self._project(frame, [(0, 0, p.minimum_depth_m)] * 5, now_s)

    def _project(self, frame, samples, now_s):
        p = self.policy
        if not isinstance(frame.intrinsics, CameraIntrinsics):
            raise InvalidTask("camera intrinsics required")
        return depth_target(
            samples,
            frame.intrinsics,
            observed_s=frame.observed_s,
            now_s=now_s,
            max_age_s=p.maximum_age_s,
            camera_frame=frame.camera_frame,
            expected_camera_frame=p.camera_frame,
            calibration_id=frame.calibration_id,
            expected_calibration_id=p.calibration_id,
            root_from_camera=frame.root_from_camera,
            root_frame=p.root_frame,
            transform_observed_s=frame.transform_observed_s,
        )

    def locate(self, frame, motion, samples_uv_depth, *, now_s, requested_s):
        self.validate_frame(frame, motion, now_s=now_s, requested_s=requested_s)
        if (
            not isinstance(samples_uv_depth, (tuple, list))
            or len(samples_uv_depth) > 10000
        ):
            raise InvalidTask("bounded depth ROI required")
        samples = []
        for sample in samples_uv_depth:
            if not isinstance(sample, (tuple, list)) or len(sample) != 3:
                raise InvalidTask("invalid depth sample")
            u, v, z = (number(value, "depth sample") for value in sample)
            if not 0 <= u < frame.width or not 0 <= v < frame.height:
                raise InvalidTask("pixel_outside_image")
            if self.policy.minimum_depth_m <= z <= self.policy.maximum_depth_m:
                samples.append((u, v, z))
        result = self._project(frame, samples, now_s)
        result.update(
            capture_id=frame.capture_id,
            pose_revision=frame.pose_revision,
            camera_model=self.policy.model,
            grasp_pose_resolved=False,
        )
        return result
