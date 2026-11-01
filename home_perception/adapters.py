"""Wrap existing tool reports in contracts without changing their meaning."""
from pathlib import Path

from home_perception.contracts import ACCEPTED, INVALID_DEPTH, ObjectObservation, REJECTED


def observation_from_grounding(report, detection):
    """A grounding_rgbd surface pixel stays a surface pixel, never a centre."""
    point = detection['surface_point']
    segmentation = detection.get('segmentation') or {}
    valid = point['valid']
    return ObjectObservation(
        capture_id=Path(report['source_snapshot']).name, captured_at_utc=report['captured_at_utc'],
        frame_id=report['coordinate_frame'], label=detection['label'],
        box_xyxy=[float(v) for v in detection['box_xyxy']], detector=report['model_id'],
        detector_score=detection['score'], status=ACCEPTED if valid else REJECTED,
        mask_quality=segmentation.get('predicted_iou'), mask_pixels=segmentation.get('pixels'),
        valid_depth_fraction=point.get('valid_fraction'),
        surface_xyz_m=[float(v) for v in point['xyz_m']] if valid else None,
        point_semantics=report['point_semantics'],
        failure=None if valid else INVALID_DEPTH, reason=None if valid else point['reason'],
        robot_transform=report['robot_transform'])
