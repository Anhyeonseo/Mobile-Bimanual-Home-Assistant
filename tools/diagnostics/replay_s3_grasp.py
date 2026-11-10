#!/usr/bin/env python3
"""Evaluate where to grasp a named object in a saved D415 exposure. No device access."""
import argparse
import hashlib
import json
from io import BytesIO
from pathlib import Path
import sys

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'ros2_ws/src/home_robot_tasks'), str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_perception.geometry.mask_cloud import measure
from home_perception.geometry.plane import backproject, fit_plane, target_support_plane
from home_perception.geometry.pick_scene import build_pick_scene, InsufficientScene
from home_perception.grasp import graspnet
from home_perception.grasp.handoff import flat_candidates, observed_closing_section
from home_robot_tasks import s2_geometry
from home_robot_tasks.camera_evidence import near_depth_profile
from home_robot_tasks.snapshot_io import hashes
from home_robot_tasks.jaw_closure import closure_plan
from home_robot_tasks.installed_jaw import InstalledLeftJaw
from home_robot_tasks.stationary_lift import ARRIVAL_TOLERANCE_M, LiftHeightRule
from home_robot_tasks.stationary_pick_sim import pad_tip_optical, plan_pick
from tools.diagnostics import review_s3_capture, yeoeun_grasp_export
from tools.diagnostics.run_s3_stationary_lab import CONFIG, model, read, stock_jaw


def installed_jaw_for_replay(profile, record=None, *, nominal_jaw=False):
    """Nominal geometry must be explicit; never silently accept stale pad data."""
    if nominal_jaw:
        if record is not None:
            raise ValueError('choose nominal STL geometry or measured pad record, not both')
        return None
    default = CONFIG / 'left_pad_gap.observed.json'
    if record is None and default.is_file():
        record = read(default)
    return InstalledLeftJaw(record, profile) if record is not None else None


def load_scene(snapshot):
    meta = read(snapshot / 'metadata.json')
    with Image.open(snapshot / 'depth_aligned.png') as image:
        if image.format != 'PNG' or image.mode not in ('I', 'I;16', 'I;16B'):
            raise ValueError('16-bit depth PNG required')
        # Older Pillow exposes 16-bit grayscale PNGs as mode I (int32).
        depth = np.asarray(image, dtype=np.uint16)
    with Image.open(snapshot / 'color.png') as color:
        if depth.shape[::-1] != color.size:
            raise ValueError('RGB/depth dimensions differ')
    intr = meta['aligned_depth_intrinsics']
    scale = meta['depth_scale_m']
    if (depth.ndim != 2 or depth.dtype != np.uint16 or depth.shape[::-1] != (intr['width'], intr['height'])
            or not 0 < scale <= .1 or any(intr['coeffs'])
            or not np.isfinite([intr[k] for k in ('fx','fy','ppx','ppy')]).all() or min(intr['fx'], intr['fy']) <= 0
            or meta['coordinate_frame'] != 'color_camera_optical_frame'
            or not meta['alignment'].startswith('depth to color')):
        raise ValueError('rectified D415 depth aligned to the color camera required')
    depth_m = depth.astype(float) * scale
    scene = backproject(depth_m, intr, depth > 0)
    return meta, depth_m, scene, fit_plane(scene)


def verified_mask(path, expected_hash, size):
    content = path.read_bytes()
    if hashlib.sha256(content).hexdigest() != expected_hash:
        raise ValueError('SAM mask digest mismatch')
    with Image.open(BytesIO(content)) as image:
        mask = np.asarray(image)
    if mask.shape != size or mask.dtype != np.uint8 or not np.isin(mask, [0, 255]).all():
        raise ValueError('binary mask aligned to the original pixels required')
    return mask > 0


def grounding_observations(snapshot, detection_file, meta, depth, plane, source, clouds=None, *, support_planes=None, up_in_optical=None):
    report = read(detection_file)
    if (report['schema_version'] != 2 or not report.get('segmentation')
            or report['captured_at_utc'] != meta['captured_at_utc']
            or report['coordinate_frame'] != meta['coordinate_frame'] or report.get('robot_transform') is not None
            or Path(report['source_snapshot']).name != snapshot.name or report['source_sha256'] != source):
        raise ValueError('detection report does not match this segmented exposure')
    observations, masks = [], []
    for index, detection in enumerate(report['detections']):
        seg = detection['segmentation']
        if (type(detection['id']) is not int or detection['id'] != index
                or seg['mask_file'] != f'mask_{index:03}.png' or not 0 <= seg['predicted_iou'] <= 1):
            raise ValueError('invalid observation-to-mask binding')
        mask = verified_mask(detection_file.parent / seg['mask_file'], seg['sha256'], depth.shape)
        if int(mask.sum()) != seg['pixels']:
            raise ValueError('SAM mask pixel count mismatch')
        support = plane
        if support_planes is not None:
            support = target_support_plane(depth, meta['aligned_depth_intrinsics'], mask, plane, up_in_optical)
            support_planes[index] = support
        geometry = measure(mask, depth, meta['aligned_depth_intrinsics'], support, seg['predicted_iou'])
        if support_planes is not None and support is None and geometry.status == 'accepted':
            geometry.status, geometry.reason = 'held', 'no_locally_verified_support'
        if clouds is not None:
            clouds[index] = geometry.points
        observations.append(dict(capture_id=snapshot.name, captured_at_utc=meta['captured_at_utc'],
                                 frame_id=meta['coordinate_frame'], source_sha256=source,
                                 box_xyxy=detection['box_xyxy'], label=detection['label'],
                                 status=geometry.status, failure=geometry.failure, reason=geometry.reason,
                                 center_xyz_m=geometry.center_xyz_m, mask_sha256=seg['sha256'],
                                 clock_domain=meta['color_frame']['timestamp_domain'].split('.')[-1], calibration_id='unmeasured',
                                 support_height_m=geometry.support_height_m, valid_depth_fraction=geometry.valid_depth_fraction))
        masks.append(mask)
    return observations, masks


def lift_setting_at_target(profile, plane, base_points, above_support_m=0., previous_preset=None, tabletop_calibration=False):
    """Use the fitted surface directly below the selected object, before generating grasps."""
    base = s2_geometry.rigid(profile['base_from_optical'])
    normal = base[:3,:3] @ plane.normal
    point = base[:3,:3] @ plane.point + base[:3,3]
    position = base_points.get('center_xyz_m', base_points.get('surface_xyz_m'))
    if position is None or normal[2] < np.cos(np.deg2rad(20)):
        return dict(status='SUPPORT_UNRESOLVED', motion_authorized=False)
    xy = np.asarray(position[:2])
    support_z = point[2] - normal[:2] @ (xy-point[:2])/normal[2]
    support_height = float(support_z - profile['ground_z_m'])
    if not np.isfinite(support_height) or support_height < 0:
        return dict(status='SUPPORT_UNRESOLVED', motion_authorized=False)
    return LiftHeightRule.from_profile(profile).setting(support_height, above_support_m, previous_preset=previous_preset,
                                                    tabletop_calibration=tabletop_calibration)


def capture_scene_boxes(scene, review):
    # Boxes expressed in base_link cease to describe the scene when the base moves.
    identity = ('capture_id', 'session_id', 'calibration_id', 'frame', 'source_sha256')
    if not isinstance(scene, dict) or any(scene.get(key) != review[key] for key in identity):
        raise ValueError('base-frame obstacle boxes must belong to this capture and robot session')
    boxes = scene.get('scene_boxes_m')
    if not isinstance(boxes, list) or not boxes:
        raise ValueError('observed workspace geometry required')
    return boxes


def replay(snapshot, detections, label, side=None, *, robot_report=None, scene_boxes=None,
           grasp_run=None, perception_run=None, surface_variants=False, height_only=False, above_support_m=0., previous_preset=None,
           tabletop_calibration=False, candidate_source='graspnet-first', scene_preview=False, near_field=False,
           left_pad_record=None, prepare_carry=False, trial_contact_offset_m=0., nominal_jaw=False):
    if trial_contact_offset_m and not prepare_carry:
        raise ValueError('trial contact offset requires capture-bound carry preparation')
    if candidate_source not in ('graspnet-first', 'graspnet', 'flat-rule') or (surface_variants and candidate_source != 'flat-rule'):
        raise ValueError('surface variants require the explicit flat-rule comparison')
    if height_only and (robot_report is None or scene_boxes is not None):
        raise ValueError('height-only replay requires the capture-time robot report and no arm scene')
    if scene_boxes is not None and robot_report is None:
        raise ValueError('base-frame scene boxes require the capture-time robot report')
    if prepare_carry and (robot_report is None or scene_boxes is not None or height_only):
        raise ValueError('carry preparation requires an anchored capture and automatically observed scene')
    if scene_preview and robot_report is not None:
        raise ValueError('scene-preview is for exposures without a robot report')
    automatic_scene = scene_preview or (robot_report is not None and scene_boxes is None and not height_only)
    clouds = {}
    if (bool(grasp_run) != bool(perception_run) or bool(detections) == bool(grasp_run)
            or (grasp_run and surface_variants)):
        raise ValueError('choose bound DINO/SAM detections or a matched pair of upstream perception/grasp exports')
    profile = read(CONFIG / 's2_coordinates.development.json')
    installed_jaw = installed_jaw_for_replay(profile, left_pad_record, nominal_jaw=nominal_jaw)
    if near_field:
        profile = near_depth_profile(profile, read(CONFIG/'camera_device.observed.json'),
                                     read(CONFIG/'camera_device.near.observed.json'))
    source = hashes(snapshot)
    meta, depth, scene, plane = load_scene(snapshot)
    if near_field:
        from home_robot_tasks.camera_evidence import from_metadata, match_observed_identity
        match_observed_identity(from_metadata(meta), read(CONFIG/'camera_device.near.observed.json'))
    support_planes = {}
    if grasp_run:
        observations = yeoeun_grasp_export.original_observations(perception_run, snapshot.name)
    else:
        up_in_optical = s2_geometry.rigid(profile['base_from_optical'])[2, :3]
        observations, masks = grounding_observations(snapshot, detections, meta, depth, plane, source, clouds,
                                                     support_planes=support_planes, up_in_optical=up_in_optical)
    result = dict(mode='offline_grasp_replay', capture_id=snapshot.name, label=label,
                  jaw_geometry_source='measured_installed_pads' if installed_jaw is not None else 'nominal_official_stl',
                  base_placement='manual_stationary', candidate_source=candidate_source,
                  coordinate_frame=meta['coordinate_frame'], source_sha256=source,
                  motion_authorized=False, hardware_commands=0, physical_task_completed=False,
                  support_plane=plane.to_dict(), observations=observations)
    review = None
    if robot_report is not None:
        geometry = s2_geometry.load_geometry(CONFIG / 'official_geometry.json')
        review = review_s3_capture.review(snapshot, robot_report, observations, profile, geometry, label, side)
        result['review'] = review
        if review['status'] != 'TARGET_DISPLAY_ONLY':
            return dict(result, status=review['status'])
        index = review['observation_index']
    else:
        choice = review_s3_capture.choose_observation(observations, snapshot.name, profile, meta, source, label, side)
        if choice['status'] != 'SELECTED':
            return dict(result, **choice)
        index = choice['observation_index']
    selected = observations[index]
    if index in support_planes:
        support = support_planes[index]
        result['support_plane_source'] = 'target_neighborhood' if support is not plane else 'global'
        plane = support
        result['support_plane'] = plane.to_dict()
    exported_groups = {}

    def exported(name):
        if name not in exported_groups:
            exported_groups[name] = yeoeun_grasp_export.refined_group(
                grasp_run, perception_run, snapshot.name, index, selected, source, meta, candidate_source=name)
        return exported_groups[name]

    observed_scene = None
    if automatic_scene:
        if grasp_run:
            source_name = 'flat-rule' if candidate_source == 'flat-rule' else 'graspnet'
            mask_sha = exported(source_name)['mask_sha256']
            mask = verified_mask(perception_run / snapshot.name / f'mask_{index:03}.png', mask_sha, depth.shape)
            geometry = measure(mask, depth, meta['aligned_depth_intrinsics'], plane)
            if geometry.status != 'accepted':
                return dict(result, status='SCENE_REOBSERVE', reason=geometry.reason)
            cloud = clouds[index] = geometry.points
        else:
            mask, cloud, mask_sha = masks[index], clouds[index], selected['mask_sha256']
        try:
            protected=np.zeros_like(mask)
            if not grasp_run:
                for other_index,other_mask in enumerate(masks):
                    # Preserve distinct detections; overlapping proposals may
                    # be duplicate masks for this same object, not neighbours.
                    overlap=np.count_nonzero(mask&other_mask)/max(1,min(mask.sum(),other_mask.sum()))
                    if other_index!=index and overlap<=.05:
                        protected|=other_mask&~mask
            observed_scene = build_pick_scene(depth, meta['aligned_depth_intrinsics'], plane, mask, cloud,
                                             protected=protected)
        except InsufficientScene as exc:
            return dict(result, status='SCENE_REOBSERVE', reason=str(exc))
        result['observed_scene'] = dict(observed_scene.describe(), capture_id=snapshot.name, source_sha256=source,
                                        mask_sha256=mask_sha, observation_index=index,
                                        session_id=None if review is None else review['session_id'],
                                        calibration_id=None if review is None else review['calibration_id'])
        assessment=observed_scene.ownership_assessment
        if assessment['status']!='CONSISTENT':
            return dict(result,status='SCENE_REOBSERVE',reason=assessment['status'],
                        next_observation=dict(action=assessment['next_action'],roi_xyxy=assessment['roi_xyxy'],
                                              retryable=True,robot_motion_required=False))
    if review is not None:
        result['lift_setting'] = lift_setting_at_target(profile, plane, review['base_points_m'], above_support_m, previous_preset, tabletop_calibration)
        if height_only:
            return dict(result, status=result['lift_setting']['status'])
        setting = result['lift_setting']
        if setting['status'] != 'HEIGHT_SETTING':
            return dict(result, status=setting['status'])
        if not prepare_carry and abs(review['lift_height_m'] - setting['target_travel_from_bottom_m']) > ARRIVAL_TOLERANCE_M:
            return dict(result, status='LIFT_ALIGNMENT_AND_NEW_CAPTURE_REQUIRED')
        if scene_boxes is not None:
            scene_boxes = capture_scene_boxes(scene_boxes, review)
    jaw = stock_jaw()

    def group_for(source_name):
        if grasp_run:
            group = exported(source_name)
            if not group['refined_grasps']:
                return group
            if index not in clouds:
                mask = verified_mask(perception_run / snapshot.name / f'mask_{index:03}.png',
                                     group['mask_sha256'], depth.shape)
                geometry = measure(mask, depth, meta['aligned_depth_intrinsics'], plane)
                clouds[index] = geometry.points if geometry.status == 'accepted' else np.empty((0,3))
            return dict(group, refined_grasps=[dict(c, closing_section=observed_closing_section(c, clouds[index]))
                                               for c in group['refined_grasps']])
        intr = meta['aligned_depth_intrinsics']
        section_points = clouds.get(index, np.empty((0, 3)))
        if source_name == 'graspnet':
            group = graspnet.candidates(depth, intr, masks, index, scene, plane, selected['support_height_m'],
                                        (jaw.gaps[0], jaw.gaps[-1]), section_points=section_points)
        else:
            group = flat_candidates(section_points, scene, plane, surface_variants=surface_variants)
        return dict(group, mask_sha256=selected['mask_sha256'])

    if candidate_source == 'graspnet-first':
        group = group_for('graspnet')
        auxiliary = group_for('flat-rule')
        if (group['mask_sha256'], group['tcp_convention']) != (auxiliary['mask_sha256'], auxiliary['tcp_convention']):
            raise ValueError('candidate groups do not describe the same target and gripper convention')
        group = dict(group, flat_rule_reason=auxiliary['candidate_reason'],
                     refined_grasps=group['refined_grasps'] + auxiliary['refined_grasps'])
    else:
        group = group_for(candidate_source)
    if trial_contact_offset_m:
        from home_perception.grasp.trial_contact_offset import apply_trial_offset
        group = apply_trial_offset(group, clouds[index], plane, profile['base_from_optical'],
                                   trial_contact_offset_m, (jaw.gaps[0], jaw.gaps[-1]))
    group.update(capture_id=snapshot.name, observation_index=index, label=label, source_sha256=source,
                 coordinate_frame=meta['coordinate_frame'], perception_status=selected['status'])
    result['group'] = group
    result['stock_jaw_section'] = jaw.describe()
    result['candidate_diagnostics'] = [
        dict(source=c['source'], opening_mm=1000*c['width_m'], within_stock_jaw_section=bool(jaw.accepts(c['width_m'])),
             nominal_open_angle_deg=float(np.degrees(np.interp(c['width_m'], jaw.gaps, jaw.angles)))
             if jaw.accepts(c['width_m']) else None,
             jaw_closure=closure_plan(jaw,c), pad_tip_optical_m=pad_tip_optical(c)[:3, 3].tolist()) for c in group['refined_grasps']]
    if installed_jaw:
        for diagnostic in result['candidate_diagnostics']:
            diagnostic['left_installed_pad_preview'] = installed_jaw.preview(diagnostic['jaw_closure'])
    result['bounded_closure_candidates'] = sum(d['jaw_closure']['status'] == 'BOUNDED_CLOSE'
                                               for d in result['candidate_diagnostics'])
    if not group['refined_grasps']:
        return dict(result, status='NO_PERCEPTION_CANDIDATE')
    fitting = [d for d in result['candidate_diagnostics'] if d['within_stock_jaw_section']]
    if not fitting:
        return dict(result, status='NO_STOCK_JAW_CANDIDATE')
    if installed_jaw is not None:
        calibrated = [d for d in fitting if d['left_installed_pad_preview']['status'] == 'MEASURED_GAP_PREVIEW']
        result['measured_closure_candidates'] = len(calibrated)
        result['first_measured_closure_source'] = calibrated[0]['source'] if calibrated else None
    if robot_report is None:
        return dict(result, status='CAMERA_CANDIDATES_ONLY', first_width_fit_source=fitting[0]['source'])
    group.update(session_id=review['session_id'], calibration_id=review['calibration_id'])
    if prepare_carry:
        # Evaluate the actually captured fixed height. Preserve the recommendation
        # but never substitute it for the measured state or silently move the lift.
        return dict(result,status='CAPTURE_CANDIDATES_READY',planning_lift='captured_fixed_height',
                    lift_recommendation_requires_move=abs(review['lift_height_m']-setting['target_travel_from_bottom_m'])>ARRIVAL_TOLERANCE_M)
    arm = model(review['lift_height_m'],profile=profile) if near_field else model(review['lift_height_m'])
    path = plan_pick(arm, review, group, scene_boxes, observed_scene=observed_scene,
                     installed_left_jaw=installed_jaw)
    if path['status'] == 'VIRTUAL_PATH_FOUND':
        result['planned_source'] = group['refined_grasps'][path['candidate_index']]['source']
    return dict(result, path=path, status=path['status'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', required=True, type=Path)
    evidence = parser.add_mutually_exclusive_group(required=True)
    evidence.add_argument('--detections', type=Path, help='hash-bound Grounding DINO + SAM detections.json')
    evidence.add_argument('--grasp-run', type=Path, help='original Yeoeun tabletop grasp export directory')
    parser.add_argument('--perception-run', type=Path, help='original perception export used by --grasp-run')
    parser.add_argument('--candidate-source', choices=['graspnet-first', 'graspnet', 'flat-rule'], default='graspnet-first',
                        help='try network poses before the flat-object rule, or compare either method alone')
    parser.add_argument('--surface-variants', action='store_true', help='try filtered surface-height poses after the original rule')
    parser.add_argument('--label', required=True)
    parser.add_argument('--screen-side', choices=['left','right'])
    parser.add_argument('--robot-report', type=Path)
    parser.add_argument('--scene-boxes', type=Path, help='override the observed scene for a capture-bound manual comparison')
    parser.add_argument('--near-field', action='store_true', help='verify the separately registered 640x360 D415 capture')
    parser.add_argument('--scene-preview', action='store_true', help='also reconstruct obstacles in the capture frame without robot state')
    parser.add_argument('--height-only', action='store_true', help='return the lift setting without evaluating any grasps')
    parser.add_argument('--prepare-carry', action='store_true', help='export bound candidates/scene at the captured fixed height for offline MoveIt')
    parser.add_argument('--tabletop-calibration', action='store_true', help='experimental left-arm high tabletop offset')
    parser.add_argument('--previous-preset', metavar='PRESET_ID',
                        help='last preset on this support, e.g. 45cm; omit for first acquisition')
    parser.add_argument('--above-support-cm', type=float, default=0., help='optional work level above the support')
    parser.add_argument('--left-pad-record',type=Path,help='override the installed left ID6 ruler calibration for encoder previews')
    parser.add_argument('--nominal-jaw',action='store_true',help='explicitly use official STL jaw geometry, without claiming measured pad accuracy')
    parser.add_argument('--trial-contact-offset-m', type=float, default=0.,
                        help='single-trial shift from observed length centre toward base +X (0..0.03 m); not calibration')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    try:
        result = replay(args.snapshot, args.detections, args.label, args.screen_side,
                        grasp_run=args.grasp_run, perception_run=args.perception_run, surface_variants=args.surface_variants,
                        height_only=args.height_only, above_support_m=args.above_support_cm/100, previous_preset=args.previous_preset,
                        tabletop_calibration=args.tabletop_calibration, candidate_source=args.candidate_source, scene_preview=args.scene_preview, near_field=args.near_field,
                        left_pad_record=read(args.left_pad_record) if args.left_pad_record else None,
                        robot_report=read(args.robot_report) if args.robot_report else None,
                        scene_boxes=read(args.scene_boxes) if args.scene_boxes else None,prepare_carry=args.prepare_carry,
                        trial_contact_offset_m=args.trial_contact_offset_m,nominal_jaw=args.nominal_jaw)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(result['status'])
    except (KeyError, TypeError, ValueError, OSError, ImportError, RuntimeError) as exc:
        parser.exit(2, str(exc) + '\n')


if __name__ == '__main__':
    main()
