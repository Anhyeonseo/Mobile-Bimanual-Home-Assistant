"""L3C -> L3D adapter for existing Yeoeun/GraspNet capture-bound candidates.

Reuse the observation/frame binding and explicitly selected left jaw conversion.
Payload geometry must be supplied from the same scene.
"""
from dataclasses import dataclass

import numpy as np

from .carry_planning import GraspRouteCandidate, PlanRejected
from .carry_posture import checked_joints, ObjectAxisConstraint
from .stationary_pick_sim import _bound_candidate
from .jaw_closure import closure_plan, RIGID_CLOSE_ALLOWANCE_M
from .carry_contact_ik import fit_pregrasp
from . import s2_geometry as geo


def oriented_source(source, jaw_sign):
    """Exchange grasp contacts before binding the asymmetric physical hand.

    This is not a symmetric TCP rotation: the fixed finger changes sides, so
    its translation, section bounds, IK and collision path must all be rebuilt.
    """
    if jaw_sign not in (-1, 1):
        raise ValueError('jaw sign must be +1 or -1')
    if jaw_sign == 1:
        return source
    rotation=np.asarray(source['rotation_matrix'],float).copy()
    rotation[:,1:] *= -1
    section=dict(source.get('closing_section',{}))
    if section.get('status') == 'VISIBLE_SPAN':
        section.update(low_m=-section['high_m'],high_m=-section['low_m'])
    return dict(source,rotation_matrix=rotation.tolist(),closing_section=section)


@dataclass(frozen=True)
class BoundPayload:
    capture_id: str
    source_sha256: dict
    calibration_id: str
    base_from_object: np.ndarray
    size_m: tuple
    mesh: object = None


class PregraspRejected(PlanRejected):
    def __init__(self, diagnostics):
        self.diagnostics=diagnostics
        super().__init__('pregrasp IK failed bounded multi-seed search: '
            f"{diagnostics['position_error_m']*1000:.1f} mm / {diagnostics['orientation_error_deg']:.1f} deg residual")


def bind_observed_fixed_contact(arm, grip, vertices_base, *, entry_clearance_m=.004):
    """Offline corrected TCP/width for the actual stock closing direction.

    Call before IK/pregrasp/route generation. Rebinding moves the hand target,
    never the observed object, and invalidates every earlier path to that target.
    """
    if not np.isfinite(entry_clearance_m) or not .004<=entry_clearance_m<=.020:
        raise ValueError('bounded entry clearance required')
    grip=geo.rigid(grip).copy()
    section=arm.jaw.contact_section(grip@arm.tcp_from_stock,vertices_base)
    width=section['width_m']
    if not arm.jaw.accepts(width+entry_clearance_m):
        raise PlanRejected('observed directed section and entry clearance exceed stock jaw')
    grip[:3,3]+=section['tcp_correction_base_m']
    section.update(entry_gap_m=width+entry_clearance_m,requires_new_ik_and_route=True)
    source=dict(width_m=width,closing_section=dict(status='VISIBLE_SPAN',
        low_m=-width/2,high_m=width/2,method='observed_contact_line'))
    return grip,source,section


def bind_grasp(arm, review, group, index, payload, installed_left_jaw, *,
               approach_distance_m=.02, upright_object_axis=None,
               maximum_tilt_rad=None, orientation_evidence=None, hypothetical_lift_m=None,
               jaw_sign=1, entry_clearance_m=.004, fixed_finger_clearance_m=0., grasp_policy=None,
               close_allowance_m=RIGID_CLOSE_ALLOWANCE_M, close_extra_raw=0, approach_mode='tool_axis',
               contact_clearance_m=0., support_normal_base=(0.,0.,1.)):
    """Return (offline candidate, jaw command preview); neither authorizes motion.

    An IK pregrasp seed is not a path. MoveIt must subsequently plan/check the
    complete path; endpoint tolerances cannot substitute for that step.
    """
    if arm.side != 'left' or installed_left_jaw is None:
        raise PlanRejected('explicit left jaw conversion required; right-hand aperture is not configured')
    planning_lift=review.get('lift_height_m') if hypothetical_lift_m is None else hypothetical_lift_m
    if hypothetical_lift_m is not None and (not np.isfinite(hypothetical_lift_m) or not 0<=hypothetical_lift_m<=arm.profile['lift']['travel_m']):
        raise ValueError('hypothetical lift must be an explicit bounded offline setting')
    if (planning_lift != arm.lift_m or
            payload.capture_id != review.get('capture_id') or
            payload.source_sha256 != review.get('source_sha256') or
            payload.calibration_id != arm.profile['calibration_id']):
        raise PlanRejected('payload, grasp and robot must share the capture and fixed lift')
    if not .005 <= approach_distance_m <= .05:
        raise ValueError('bounded approach distance required')
    if (not np.isfinite(fixed_finger_clearance_m)
            or not 0 <= fixed_finger_clearance_m <= .010
            or fixed_finger_clearance_m > entry_clearance_m-.002):
        raise ValueError('fixed-finger entry clearance must leave at least 2 mm for moving finger')
    if not np.isfinite(contact_clearance_m) or not 0<=contact_clearance_m<=.010 or contact_clearance_m>entry_clearance_m-.002:
        raise ValueError('fixed contact clearance must retain moving-side entry room')
    if contact_clearance_m and fixed_finger_clearance_m:
        raise ValueError('do not combine entry-only and retained contact clearances')
    candidates=group['refined_grasps']
    if not isinstance(candidates,list) or len(candidates)>64 or type(index) is not int or not 0<=index<len(candidates):
        raise ValueError('invalid bounded candidate index')
    source=oriented_source(candidates[index],jaw_sign)
    target_vertices=None
    if payload.mesh is not None:
        object_pose=geo.rigid(payload.base_from_object)
        target_vertices=np.asarray(payload.mesh['vertices'])@object_pose[:3,:3].T+object_pose[:3,3]
    grip=_bound_candidate(review,group,source,arm,target_vertices_base=target_vertices)
    close=closure_plan(arm.jaw,source,entry_clearance_m=entry_clearance_m,close_allowance_m=close_allowance_m,close_extra_raw=close_extra_raw)
    gap=installed_left_jaw.preview(close)
    if gap['status'] not in ('MEASURED_GAP_PREVIEW','NOMINAL_STL_GAP_PREVIEW'):
        raise PlanRejected(gap['status'])
    # GraspNet Y points from the fixed side to the moving side (-TCP X).
    # Align the fixed face with the observed low contact, independent of the
    # entry opening. Centring the OPEN gap would leave the fixed finger outside
    # the object and require an unplanned lateral push during closing.
    fixed_extent=-source['closing_section']['low_m']
    shift=fixed_extent-source['width_m']/2
    grip[:3,3]+=grip[:3,0]*shift
    section_binding=None
    if target_vertices is not None:
        grip,effective,section_binding=bind_observed_fixed_contact(
            arm,grip,target_vertices,entry_clearance_m=entry_clearance_m)
        source=dict(source,**effective)
        close=closure_plan(arm.jaw,source,entry_clearance_m=entry_clearance_m,close_allowance_m=close_allowance_m,close_extra_raw=close_extra_raw)
        gap=installed_left_jaw.preview(close)
        if gap['status'] not in ('MEASURED_GAP_PREVIEW','NOMINAL_STL_GAP_PREVIEW'):
            raise PlanRejected(gap['status'])
        fixed_extent=source['width_m']/2
    observed_object=geo.rigid(payload.base_from_object)
    held_object=observed_object.copy()
    planned_slide=np.zeros(3)
    if contact_clearance_m:
        # Leave both sides clear through the whole descent. The moving jaw may
        # slide the object across this small gap while closing, along its support.
        normal=np.asarray(support_normal_base,float)
        if normal.shape!=(3,) or not np.isfinite(normal).all() or np.linalg.norm(normal)<1e-8:
            raise ValueError('finite support normal required')
        normal=normal/np.linalg.norm(normal)
        outward=(grip@arm.tcp_from_stock)[:3,0]
        tangent=outward-normal*(normal@outward)
        projection=float(tangent@outward)
        if projection<.5:raise PlanRejected('closing direction cannot slide along support')
        planned_slide=contact_clearance_m*tangent/projection
        grip[:3,3]+=planned_slide
        held_object[:3,3]+=planned_slide
    seed=checked_joints(arm,[review['left_anchor_rad'][n] for n in geo.JOINTS[:5]])
    # Roll is free only for an observed pinch-line section. Legacy rectangular
    # sampling bands cannot justify changing the approach orientation.
    contact_roll=source.get('closing_section',{}).get('method')=='observed_contact_line'
    from .pick_calibration import TOP_BASIS, TOP_BASES
    if review.get('lift_reference_basis') in TOP_BASES:
        # Seed the contact IK with the field-observed extended arm posture.
        # MoveIt still connects from the actual measured twelve-joint start.
        seed = np.clip([seed[0], 1.032, -.239, .127, seed[4]], arm.lo, arm.hi)
    ok,pre_joints,grip,fit=fit_pregrasp(arm,grip,seed,approach_distance_m,contact_roll=contact_roll,
                                     fixed_finger_clearance_m=fixed_finger_clearance_m,grasp_policy=grasp_policy,
                                     approach_mode=approach_mode)
    if not ok:
        raise PregraspRejected(fit)
    relative=np.linalg.inv(grip)@held_object
    constraint=None
    if upright_object_axis is not None:
        if maximum_tilt_rad is None or not orientation_evidence:
            raise ValueError('upright axis requires an explicit tolerance and evidence')
        constraint=ObjectAxisConstraint(relative,object_axis=tuple(upright_object_axis),
                                         maximum_tilt_rad=maximum_tilt_rad,evidence=orientation_evidence)
    elif maximum_tilt_rad is not None or orientation_evidence is not None:
        raise ValueError('orientation metadata without an object axis')
    alignment=arm.profile['arms']['left']['alignment']['gripper']
    angle=lambda raw: geo.model_angle_from_raw(raw,alignment,'gripper')
    # Collision geometry uses the observed rigid span, not the smaller motor
    # command target. A rigid object cannot shrink by the closing allowance.
    held_raw=installed_left_jaw.raw_at_least(close['visible_span_m']*1000)
    suffix='' if jaw_sign==1 else ':reversed'
    candidate=GraspRouteCandidate(f"{review['capture_id']}:{index}{suffix}",arm.profile['calibration_id'],
        review['capture_id'],pre_joints,grip,relative,tuple(payload.size_m),
        angle(gap['entry_raw']),angle(held_raw),constraint,payload.mesh,
        observed_object if contact_clearance_m else None,contact_clearance_m)
    preview_key='measured_gap_preview' if gap['status']=='MEASURED_GAP_PREVIEW' else 'nominal_gap_preview'
    return candidate, dict(closure=close,**{preview_key:gap},
                           observed_directed_section=section_binding,
                           held_geometry_raw=held_raw,contact_fit=fit,jaw_sign=jaw_sign,
                           fixed_contact_binding=dict(policy=('clear_entry_then_close_into_fixed_finger' if contact_clearance_m else 'observed_low_contact_not_open_gap_center'),
                               fixed_extent_m=fixed_extent,entry_gap_m=close['entry_gap_m'],
                               fixed_side_clearance_m=contact_clearance_m,
                               moving_side_clearance_m=entry_clearance_m-contact_clearance_m,
                               planned_object_translation_base_m=planned_slide.tolist(),
                               object_translation_physically_verified=False,
                               pregrasp_fixed_side_clearance_m=fixed_finger_clearance_m,
                               entry_extra_gap_m=entry_clearance_m,
                               unplanned_object_translation_m=0.),
                           hypothetical_lift_m=hypothetical_lift_m,motion_authorized=False)
