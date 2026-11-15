"""Stationary manipulation observations, independent of the navigation sensors.

Input pixels must be rectified colour pixels with depth registered to that same
optical frame, in metres. The producer owns image registration and TF lookup at
exposure time. A pose revision must change on ANY base/lift motion, even if the
robot returns to its former pose. This module never authorizes motor commands.
"""

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import numpy as np

from .fetch import InvalidTask
from .navigation import identifier, number
from .perception import CameraIntrinsics, depth_target


# Camera-only A/B/A and independent restart trials, 2026-11-15. This is an
# acquisition choice for stopped height review, not a new geometric offset or
# a change to navigation/arm capture defaults. Capture restores it on exit.
HEIGHT_DEPTH_EXPOSURE_US = 16000


def height_frame_issue(record):
    """Check exposure-time evidence; never substitute sensor option readback.

    A failed check requests another height frame, not a controller fault.
    Automatic exposure's get_option(exposure) can remain 33000 while the actual
    frame exposure is different. Gain/laser metadata are retained for review.
    """
    settings=record.get('depth_frame',{}).get('settings',{})
    if settings.get('auto_exposure')!=0:return 'depth_manual_exposure_unconfirmed'
    if settings.get('actual_exposure')!=HEIGHT_DEPTH_EXPOSURE_US:
        return 'depth_exposure_not_settled'
    return None


@dataclass(frozen=True)
class SurfaceHeightModel:
    """Absolute optical-point height model; does not replace a rigid XYZ TF.

    Use raw registered optical XYZ, never previously corrected base/map XYZ.
    Only the Z row of the measured camera calibration is adopted. Arm zeros,
    arm mounts, map XY, and the released manipulation TF remain untouched.
    Requires the stopped upright base/floor convention used by the capture.
    """
    model_id: str
    base_z_from_optical: tuple
    ground_z_m: float

    def __post_init__(self):
        identifier(self.model_id,'height model id')
        row=np.asarray(self.base_z_from_optical,dtype=float)
        if (row.shape!=(4,) or not np.isfinite(row).all() or
                not np.isclose(np.linalg.norm(row[:3]),1.,atol=1e-8,rtol=0) or
                not 0<row[3]<3 or not np.isfinite(self.ground_z_m)):
            raise InvalidTask('valid absolute optical height projection required')

    def heights(self,optical_xyz):
        xyz=np.asarray(optical_xyz,dtype=float)
        if xyz.ndim<1 or xyz.shape[-1]!=3 or not np.isfinite(xyz).all():
            raise InvalidTask('finite raw optical XYZ required')
        row=np.asarray(self.base_z_from_optical)
        # The calibrated translation is absolute: no legacy -9.1mm adjustment.
        return xyz@row[:3]+row[3]-self.ground_z_m


def load_surface_height_model(root):
    root=Path(root)
    config=json.loads((root/'config/home_assistant/camera_height.calibrated.json').read_text())
    unsigned={k:v for k,v in config.items() if k!='model_id'}
    digest=hashlib.sha256(json.dumps(unsigned,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
    profile=json.loads((root/'config/home_assistant/s2_coordinates.development.json').read_text())
    if (config.get('model_id')!='height-'+digest[:24] or
            config.get('mode')!='absolute_optical_height_only' or
            config.get('profile_calibration_id')!=profile['calibration_id'] or
            config.get('source_frame')!=profile['source_frame'] or
            config.get('ground_z_m')!=profile['ground_z_m'] or
            config.get('legacy_z_adjustment_applied') is not False):
        raise InvalidTask('height model/profile binding mismatch')
    return SurfaceHeightModel(config['model_id'],tuple(config['base_z_from_optical']),config['ground_z_m'])


def observed_floor_height_model(root, optical_xyz):
    """Height-only reference for one stopped frame; never changes a rigid TF.

    Reuse navigation's bounded floor-plane fit on the original rigid transform.
    The measured optical plane supplies only signed height above that floor.
    A missing/ambiguous floor returns the calibrated model with explicit source;
    callers must not label that fallback as an observed floor correction.
    Base/lift stillness and exposure identity remain the caller's responsibility.
    """
    from .navigation_depth import measured_floor_heights
    root=Path(root)
    nominal=load_surface_height_model(root)
    xyz=np.asarray(optical_xyz,dtype=float)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or not np.isfinite(xyz).all():
        raise InvalidTask('finite optical point cloud required')
    profile=json.loads((root/'config/home_assistant/s2_coordinates.development.json').read_text())
    rigid=np.asarray(profile['base_from_optical'],dtype=float)
    base=xyz@rigid[:3,:3].T+rigid[:3,3]
    _,plane=measured_floor_heights(base,profile['ground_z_m'])
    evidence=dict(source='calibrated_height_fallback',parent_model_id=nominal.model_id,
                  floor_observed=False,rigid_transform_changed=False,arm_calibration_changed=False)
    if plane is None:return nominal,evidence
    normal=np.asarray(plane['normal'])
    optical_normal=normal@rigid[:3,:3]
    distance=float((rigid[:3,3]-np.asarray(plane['point_m']))@normal)
    row=tuple(optical_normal.tolist()+[distance+nominal.ground_z_m])
    identity='floor-height-'+hashlib.sha256(json.dumps(row).encode()).hexdigest()[:24]
    observed=SurfaceHeightModel(identity,row,nominal.ground_z_m)
    return observed,{**evidence,'source':'observed_floor_plane','floor_observed':True,
        'floor_fit':plane,'base_z_from_optical':list(row),'model_id':identity}


def settled_surface_height(samples, *, stopped_s, settle_s=1., minimum_span_s=1.,
                           maximum_spread_m=.006):
    """Review a stopped, exposure-bound surface sequence without motor authority.

    Each sample supplies capture_id, observed_s, pose_revision, surface_id,
    calibration_id and height_m. The caller fits the SAME surface at each
    exposure and binds pose_revision to stopped base/lift evidence. Discarding
    transient samples is allowed, but a persistent height change is not hidden
    by averaging. The selected frame is real, not a synthetic averaged image.
    Stable depth does not prove a correct extrinsic calibration or level camera.
    """
    for name,value in (('stopped_s',stopped_s),('settle_s',settle_s),
                       ('minimum_span_s',minimum_span_s),('maximum_spread_m',maximum_spread_m)):
        if not np.isfinite(value) or value<0:raise InvalidTask('invalid surface window '+name)
    if not 0<maximum_spread_m<=.02 or minimum_span_s<=0:
        raise InvalidTask('bounded surface window required')
    if not isinstance(samples,(list,tuple)) or len(samples)>300:
        raise InvalidTask('bounded surface samples required')
    rows=[];identities=set();ids=set();previous=-float('inf')
    for sample in samples:
        stamp=number(sample['observed_s'],'surface exposure')
        height=number(sample['height_m'],'surface height')
        capture=identifier(sample['capture_id'],'surface capture')
        identity=tuple(identifier(sample[k],k) for k in ('pose_revision','surface_id','calibration_id'))
        if not 0<=height<=3 or stamp<=previous or capture in ids:
            raise InvalidTask('unique ordered surface exposures required')
        previous=stamp;ids.add(capture)
        if stamp>=stopped_s+settle_s:
            rows.append(sample);identities.add(identity)
    if len(identities)>1:raise InvalidTask('surface or stopped configuration changed')
    result=dict(status='RETRY_CAPTURE',motion_authorized=False,physical_accuracy_verified=False,
                retained_capture_ids=[],height_m=None,reason='insufficient_settled_exposures')
    if len(rows)<5 or rows[-1]['observed_s']-rows[0]['observed_s']<minimum_span_s:return result
    heights=np.array([s['height_m'] for s in rows]);median=float(np.median(heights))
    # A single sensor spike must not force a lift step. A changed or oscillating
    # plateau occupying more than 20% of the window requests another capture.
    keep=np.abs(heights-median)<=maximum_spread_m/2
    kept=[r for r,k in zip(rows,keep) if k]
    result.update(total_frames=len(rows),inlier_frames=len(kept),raw_range_m=float(np.ptp(heights)))
    if (len(kept)<5 or keep.mean()<.8 or
            kept[-1]['observed_s']-kept[0]['observed_s']<minimum_span_s):
        result['reason']='surface_height_not_settled';return result
    median=float(np.median(heights[keep]))
    representative=min(kept,key=lambda r:(abs(r['height_m']-median),-r['observed_s']))
    return {**result,'status':'SETTLED_SURFACE','reason':'stable_height_window',
            'height_m':median,'spread_m':float(np.ptp(heights[keep])),
            'representative_capture_id':representative['capture_id'],
            'retained_capture_ids':[r['capture_id'] for r in kept],
            'window_s':[kept[0]['observed_s'],kept[-1]['observed_s']]}


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
