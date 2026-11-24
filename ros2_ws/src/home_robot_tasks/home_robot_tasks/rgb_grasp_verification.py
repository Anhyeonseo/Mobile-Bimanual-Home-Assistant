"""Markerless RGB evidence: compare 'stayed on support' and 'followed TCP'.

RGB does not supply new metric depth. Both hypotheses originate in the same
pre-grasp RGB-D point cloud; measured FK and calibrated camera extrinsics project
them into each NEW rectified RGB image. A detector miss is never an empty hand.
This is visual evidence, not a force measurement or hardware motion authority.
"""
from dataclasses import dataclass
import math

import numpy as np

from .s2_geometry import rigid


@dataclass(frozen=True)
class RGBGraspResult:
    arm: str
    attempt_id: str
    capture_id: str
    target_id: str
    camera_frame: str
    sequence: int
    observed_s: float
    state: str  # SUCCESS / FAILURE / UNKNOWN
    reason: str
    source_overlap: float = 0.
    held_overlap: float = 0.


class RGBGraspVerifier:
    """One camera, arm, target and attempt; no automatic reference replacement.

    The caller supplies a target-instance mask from its detector/segmenter and
    an independently established occlusion mask (robot, foreground obstacles,
    invalid pixels). Missing either input produces UNKNOWN. Never substitute a
    zero occlusion mask merely because foreground segmentation failed.
    """
    def __init__(self, *, arm, attempt_id, capture_id, target_id, camera_frame,
                 parent_frame, parent_from_optical, camera_matrix, image_size,
                 target_points_base, reference_base_from_tcp,
                 min_motion_m=.01, overlap=.65, separation=.15,
                 max_occlusion=.15, dwell_s=.3, min_frames=3, max_age_s=.2):
        if arm not in ('left', 'right') or not all(isinstance(s, str) and s.strip() for s in
                (attempt_id, capture_id, target_id, camera_frame, parent_frame)):
            raise ValueError('explicit arm, attempt, target and calibrated frames required')
        if camera_frame == parent_frame:
            raise ValueError('camera optical frame must be distinct from its mounting link')
        self.binding = (arm, attempt_id, capture_id, target_id, camera_frame)
        self.parent_frame = parent_frame
        self.parent_from_optical = rigid(parent_from_optical).copy()
        self.k = np.asarray(camera_matrix, float)
        if (self.k.shape != (3, 3) or not np.isfinite(self.k).all()
                or min(self.k[0, 0], self.k[1, 1]) <= 0
                or not np.allclose(self.k[2], [0, 0, 1])):
            raise ValueError('rectified RGB intrinsics required')
        if len(image_size) != 2 or any(type(v) is not int or not 16 <= v <= 8192 for v in image_size):
            raise ValueError('image_size is [width, height]')
        self.shape = (image_size[1], image_size[0])
        points = np.asarray(target_points_base, float)
        if (points.ndim != 2 or points.shape[1] != 3 or not 4 <= len(points) <= 100000
                or not np.isfinite(points).all()):
            raise ValueError('measured target RGB-D cloud in stationary base frame required')
        self.points = np.c_[points, np.ones(len(points))].T
        self.reference = rigid(reference_base_from_tcp).copy()
        self.in_tcp = np.linalg.inv(self.reference) @ self.points
        values = (min_motion_m, overlap, separation, max_occlusion, dwell_s, max_age_s)
        if (not all(math.isfinite(v) for v in values) or not .004 <= min_motion_m <= .05
                or not .5 <= overlap <= 1 or not 0 <= separation < overlap
                or not 0 <= max_occlusion <= .3 or not .1 <= dwell_s <= 2
                or not .05 <= max_age_s <= .5 or type(min_frames) is not int or min_frames < 3):
            raise ValueError('invalid commissioned verification thresholds')
        self.motion, self.overlap, self.separation = min_motion_m, overlap, separation
        self.occlusion, self.dwell, self.frames, self.age = max_occlusion, dwell_s, min_frames, max_age_s
        self.last_sequence, self.last_stamp, self.last_now = -1, -1., -1.
        self.candidate, self.since, self.count = None, None, 0
        self.had_success = False
        self.lost_after_success = False

    def _result(self, sequence, stamp, state, reason, source=0., held=0.):
        return RGBGraspResult(*self.binding, sequence, stamp, state, reason, source, held)

    def _unknown(self, sequence, stamp, reason):
        self.candidate, self.since, self.count = None, None, 0
        # Once a verified payload is lost, do not turn that into a clean miss
        # and initiate an empty-hand retry under the same attempt.
        if self.had_success:
            self.lost_after_success = True
        return self._result(sequence, stamp, 'UNKNOWN', reason)

    def _project(self, optical_from_base, points):
        import cv2
        xyz = (optical_from_base @ points)[:3]
        if np.any(xyz[2] <= .01):
            return None
        uv = (self.k @ xyz)[:2] / xyz[2]
        h, w = self.shape
        if np.any(uv < 1) or np.any(uv[0] >= w-1) or np.any(uv[1] >= h-1):
            return None
        mask = np.zeros(self.shape, np.uint8)
        hull = cv2.convexHull(np.rint(uv.T).astype(np.int32))
        cv2.fillConvexPoly(mask, hull, 1)
        return mask.astype(bool) if mask.sum() >= 25 else None

    def observe(self, *, arm, camera_frame, sequence, observed_s, now,
                base_from_parent, base_from_tcp, pose_observed_s,
                target_masks, occluded_mask, jaw_state, jaw_observed_s,
                verification_phase=True):
        """Masks share this exposure. jaw_state is measured contact/empty/unknown.

        'empty' means a commissioned no-object CLOSED jaw measurement, not a
        requested close command. Multiple matching target instances are unknown.
        Pose timestamps must belong to measured FK interpolated at image time.
        """
        unknown = lambda reason: self._unknown(sequence, observed_s, reason)
        times = (observed_s, now, pose_observed_s, jaw_observed_s)
        if (not all(isinstance(t, (int, float)) and not isinstance(t, bool) and math.isfinite(t) for t in times)
                or min(times) < 0 or type(sequence) is not int or sequence < 0):
            return unknown('INVALID_MEASUREMENT')
        if (arm, camera_frame) != (self.binding[0], self.binding[4]):
            return unknown('WRONG_ARM_OR_CAMERA')
        if (sequence <= self.last_sequence or observed_s <= self.last_stamp or now < self.last_now):
            return unknown('REPEATED_OR_REGRESSED_SAMPLE')
        previous_stamp = self.last_stamp
        self.last_sequence, self.last_stamp, self.last_now = sequence, observed_s, now
        if (not 0 <= now-min(observed_s, pose_observed_s, jaw_observed_s) <= self.age
                or max(observed_s, pose_observed_s, jaw_observed_s) > now
                or max(observed_s, pose_observed_s, jaw_observed_s)-min(observed_s, pose_observed_s, jaw_observed_s) > .05):
            return unknown('STALE_OR_UNSYNCHRONIZED')
        if previous_stamp >= 0 and observed_s-previous_stamp > self.age:
            if self.had_success:
                return unknown('PAYLOAD_OBSERVATION_GAP')
            self.candidate, self.since, self.count = None, None, 0
        if self.lost_after_success:
            return unknown('PAYLOAD_LOST_REQUIRES_REVIEW')
        if verification_phase is not True:
            return unknown('WAIT_VERIFICATION_PHASE')
        try:
            tcp = rigid(base_from_tcp)
            optical_from_base = np.linalg.inv(rigid(base_from_parent) @ self.parent_from_optical)
        except (ValueError, TypeError):
            return unknown('INVALID_MEASURED_TRANSFORM')
        if np.linalg.norm(tcp[:3, 3]-self.reference[:3, 3]) < self.motion:
            return unknown('INSUFFICIENT_MEASURED_MOTION')
        source = self._project(optical_from_base, self.points)
        held = self._project(optical_from_base, tcp @ self.in_tcp)
        if source is None or held is None:
            return unknown('HYPOTHESIS_OUT_OF_VIEW')
        if np.count_nonzero(source & held)/min(source.sum(), held.sum()) > self.separation:
            return unknown('HYPOTHESES_NOT_SEPARABLE')
        if (not isinstance(occluded_mask, np.ndarray) or occluded_mask.dtype != bool
                or occluded_mask.shape != self.shape):
            return unknown('VISIBILITY_EVIDENCE_REQUIRED')
        if any(np.count_nonzero(region & occluded_mask)/region.sum() > self.occlusion for region in (source, held)):
            return unknown('OCCLUDED')
        if not isinstance(target_masks, (list, tuple)) or len(target_masks) != 1:
            return unknown('TARGET_MISSING_OR_AMBIGUOUS')
        target = target_masks[0]
        if (not isinstance(target, np.ndarray) or target.dtype != bool or target.shape != self.shape
                or target.sum() < 25):
            return unknown('INVALID_TARGET_MASK')
        # Precision against both competing projections; a huge/broad mask cannot
        # prove either hypothesis. Also require substantial projected coverage.
        score = lambda region: min(np.count_nonzero(target & region)/target.sum(),
                                   np.count_nonzero(target & region)/region.sum())
        a, b = score(source), score(held)
        if b >= self.overlap and a <= self.separation and jaw_state == 'contact':
            candidate, reason = 'SUCCESS', 'TARGET_FOLLOWED_GRIPPER'
        elif a >= self.overlap and b <= self.separation and jaw_state == 'empty' and not self.had_success:
            candidate, reason = 'FAILURE', 'TARGET_REMAINED_AND_JAW_EMPTY'
        else:
            return unknown('CONTRADICTORY_OR_INSUFFICIENT_EVIDENCE')
        if candidate != self.candidate:
            self.candidate, self.since, self.count = candidate, observed_s, 0
        self.count += 1
        if self.count < self.frames or observed_s-self.since+1e-9 < self.dwell:
            return self._result(sequence, observed_s, 'UNKNOWN', 'WAIT_STABLE_EVIDENCE', a, b)
        self.had_success |= candidate == 'SUCCESS'
        return self._result(sequence, observed_s, candidate, reason, a, b)

    def observe_rgb(self, rgb, *, detector, visibility, **measurements):
        """Run on the camera worker, never the motor heartbeat thread.

        detector follows GroundingSAMDetector's RGB -> ImageDetection[] API.
        visibility(rgb, measurements) supplies the independently observed robot/
        foreground occlusion mask. Category ambiguity is intentionally rejected.
        """
        if (not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8
                or rgb.shape != (*self.shape, 3)):
            raise ValueError('rectified uint8 RGB image matching calibration required')
        detected = detector(rgb)
        masks = [d.mask for d in detected if d.object_id == self.binding[3] and d.confidence >= .5]
        return self.observe(target_masks=masks, occluded_mask=visibility(rgb, measurements), **measurements)


class GraspRetryGate:
    """Single-use retry admission; never opens jaws, auto-homes or rearms a fault.

    A recovery planner/executor must explicitly acknowledge a collision-checked
    empty-hand retreat. A NEW RGB-D scene and measured anchor are then required.
    """
    def __init__(self, arm, max_retries=2):
        if arm not in ('left', 'right') or type(max_retries) is not int or not 0 <= max_retries <= 5:
            raise ValueError('bounded per-arm retry budget required')
        self.arm, self.limit, self.used = arm, max_retries, 0
        self.attempts = set()

    def admit(self, result, *, now, stopped, controller_ok, empty_hand,
              recovery_complete, new_capture_id, capture_s, measured_state_s):
        if not isinstance(result, RGBGraspResult):
            return False, 'VERIFICATION_REQUIRED'
        if result.arm != self.arm or result.state != 'FAILURE' or result.reason != 'TARGET_REMAINED_AND_JAW_EMPTY':
            return False, 'NO_CONFIRMED_EMPTY_GRASP'
        if result.attempt_id in self.attempts or self.used >= self.limit:
            return False, 'RETRY_ALREADY_USED_OR_EXHAUSTED'
        if any(v is not True for v in (stopped, controller_ok, empty_hand, recovery_complete)):
            return False, 'MEASURED_RECOVERY_REQUIRED'
        # The miss can precede recovery. Require fresh *post-recovery* capture
        # and empty-hand/current-state proofs instead of restamping the miss.
        if (not all(isinstance(t, (float, int)) and not isinstance(t, bool) and math.isfinite(t)
                    for t in (now, capture_s, measured_state_s, result.observed_s))
                or not 0 <= now-capture_s <= 2 or not 0 <= now-measured_state_s <= .2
                or capture_s <= result.observed_s or measured_state_s < capture_s
                or not isinstance(new_capture_id, str) or not new_capture_id
                or new_capture_id == result.capture_id):
            return False, 'FRESH_POST_RECOVERY_SCENE_REQUIRED'
        self.attempts.add(result.attempt_id)
        self.used += 1
        return True, 'REPLAN_FIRST_FEASIBLE_FROM_CURRENT_STATE'
