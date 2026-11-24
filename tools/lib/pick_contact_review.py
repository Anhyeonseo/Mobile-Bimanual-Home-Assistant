"""Explain model contact height relative to the observed table; no correction.

This separates an image/goal-height error from an unmeasured model-to-real-arm
error. Encoder FK and planned TCP height cannot establish actual fingertip height.
"""
import numpy as np


def contact_height_review(report, prepared):
    plan = report.get('plan') or {}
    grasp = plan.get('resolved_grasp')
    plane = report.get('observed_support_plane_base')
    if report.get('status') != 'OFFLINE_ROUTE_FOUND' or grasp is None or plane is None:
        return None
    plane = np.asarray(plane, dtype=float)
    if plane.shape != (4,) or not np.isfinite(plane).all() or np.linalg.norm(plane[:3]) < 1e-8:
        raise ValueError('finite support plane required for contact review')
    plane = plane/np.linalg.norm(plane[:3])
    pose = np.asarray(grasp['base_from_grasp_tcp'], dtype=float)
    def height(point):return float(np.asarray(point)@plane[:3]+plane[3])
    observation_index = prepared['review']['observation_index']
    observation = prepared['observations'][observation_index]
    result = dict(capture_id=report['capture_id'], candidate_id=plan['candidate_id'],
        frame='base_link', calibration_id=report['calibration_id'],
        observation_index=observation_index,
        object_observed_height_m=observation.get('support_height_m'),
        planned_tcp_base_z_m=float(pose[2,3]),
        planned_tcp_height_above_support_m=height(pose[:3,3]),
        planning_lift_m=report['initial_model_joint_positions_si']['s2_lift_joint'],
        height_basis=report.get('lift_reference_basis'),
        physical_contact_height_verified=False, correction_applied_m=0.,
        calibration_posture_review=(plan.get('carry_quality') or {}).get('top_calibration'),
        interpretation='planned nominal TCP, not an optical measurement of real fingertips')
    mesh = grasp.get('payload_mesh')
    if mesh:
        observed = grasp.get('observed_base_from_object')
        object_pose = (np.asarray(observed) if observed is not None else
                       pose@np.asarray(grasp['tcp_from_object']))
        vertices = np.asarray(mesh['vertices'])@object_pose[:3,:3].T+object_pose[:3,3]
        heights = vertices@plane[:3]+plane[3]
        result['observed_object_envelope_height_m'] = [float(heights.min()),float(heights.max())]
    selected = next((row for row in report.get('candidates', [])
                     if row.get('candidate', {}).get('candidate_id') == plan['candidate_id']), None)
    pre = ((selected or {}).get('jaw_preview', {}).get('contact_fit') or {}).get('desired_base_from_tcp')
    if pre is not None:
        result['pregrasp_tcp_height_above_support_m'] = height(np.asarray(pre)[:3,3])
        result['planned_vertical_drop_m'] = float(np.asarray(pre)[2,3]-pose[2,3])
    return result
