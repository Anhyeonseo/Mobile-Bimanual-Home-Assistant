"""Fit a contact-line grasp to five DOF without changing its pinch contacts.

Only the approach rotation about the fixed closing line may change. Returned
poses are proposals, never collision-free paths. Full MoveIt checking follows.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .grasp_preferences import contact_posture, contact_roll_interval


def fit_pregrasp(arm, grip, seed, distance, *, contact_roll=False, fixed_finger_clearance_m=0., grasp_policy=None,
                approach_mode='tool_axis'):
    if not np.isfinite(fixed_finger_clearance_m) or not 0 <= fixed_finger_clearance_m <= .010:
        raise ValueError('fixed-finger pregrasp clearance must be within 0..10 mm')
    if approach_mode not in ('tool_axis','vertical'):
        raise ValueError('unknown approach mode')
    if approach_mode=='vertical' and fixed_finger_clearance_m!=0:
        raise ValueError('vertical descent requires lateral alignment before descent')
    seeds=[seed,arm.anchor,(arm.lo+arm.hi)/2]
    seeds.extend(np.clip(seed+np.deg2rad(delta),arm.lo,arm.hi)
                 for delta in ([0,30,-45,30,0],[0,-30,45,-30,0]))

    def target(angle):
        pose=grip.copy()
        pose[:3,:3]=grip[:3,:3]@Rotation.from_rotvec([angle,0.,0.]).as_matrix()
        pre=pose.copy()
        if approach_mode=='vertical':
            # Keep the reachable contact attitude; vertical travel does not
            # require a vertical wrist. Align XY above the object first.
            pre[2,3]+=distance
        else:
            pre[:3,3]-=distance*pre[:3,2]
        # Only the entry pose shifts away from the object on the fixed side.
        # The Cartesian approach converges to the original contact, so closing
        # does not require translating the object across a residual side gap.
        pre[:3,3]+=fixed_finger_clearance_m*pre[:3,0]
        return pose,pre

    def error(q,angle=0.):
        actual=arm.fk(q)[0];pre=target(angle)[1]
        return np.r_[(actual[:3,3]-pre[:3,3])/.001,
            Rotation.from_matrix(pre[:3,:3].T@actual[:3,:3]).as_rotvec()/np.deg2rad(1)]

    def accepted(result,residual):
        return (result.success and np.linalg.norm(residual[:3])<=1 and np.linalg.norm(residual[3:])<=1
                and (grasp_policy is None or contact_posture(arm.fk(result.x[:5])[0],grasp_policy)['accepted']))

    exact=[least_squares(error,q,bounds=(arm.lo,arm.hi),max_nfev=150) for q in seeds]
    passing=[r for r in exact if accepted(r,error(r.x))]
    # Do not select a distant joint branch for an insignificant residual gain.
    # Differences are continuous motor travel, never modulo 2*pi.
    def preference(q):
        if grasp_policy is not None:
            return .25*float(np.sum((q-seed)**2))+contact_posture(arm.fk(q)[0],grasp_policy)['cost']
        return float(np.sum((q-seed)**2)+(q[4]-arm.anchor[4])**2)
    best=min(passing,key=lambda r:preference(r.x)) if passing else min(exact,key=lambda r:np.linalg.norm(error(r.x)))
    angle=0.;solution=best;residual=error(best.x)
    tried=False
    roll_bounds=(-np.pi/3,np.pi/3) if grasp_policy is None else contact_roll_interval(grip,grasp_policy)
    if not accepted(best,residual) and contact_roll and roll_bounds is not None:
        tried=True
        bounds=(np.r_[arm.lo,roll_bounds[0]],np.r_[arm.hi,roll_bounds[1]])
        def constrained(values):return error(values[:5],values[5])
        fits=[least_squares(constrained,np.r_[q,np.clip(0.,*roll_bounds)],bounds=bounds,max_nfev=200) for q in seeds]
        passing=[r for r in fits if accepted(r,constrained(r.x))]
        if passing:
            solution=min(passing,key=lambda r:preference(r.x[:5])+.1*r.x[5]**2)
        else:
            solution=min(fits,key=lambda r:np.linalg.norm(constrained(r.x)))
        angle=float(solution.x[5]);residual=constrained(solution.x)
    pose,pre=target(angle)
    q=solution.x[:5]
    diagnostics=dict(seeds=len(seeds),contact_roll_search=tried,
        position_error_m=float(np.linalg.norm(residual[:3])*.001),
        orientation_error_deg=float(np.linalg.norm(residual[3:])),nearest_model_rad=q.tolist(),
        desired_base_from_tcp=pre.tolist(),nearest_base_from_tcp=arm.fk(q)[0].tolist(),
        contact_roll_deg=float(np.rad2deg(angle)),source_base_from_grasp_tcp=grip.tolist(),
        planned_base_from_grasp_tcp=pose.tolist(),pinch_center_displacement_m=float(np.linalg.norm(pose[:3,3]-grip[:3,3])),
        closing_axis_change_deg=float(np.rad2deg(np.arccos(np.clip(pose[:3,0]@grip[:3,0],-1,1)))),
        fixed_finger_pregrasp_clearance_m=fixed_finger_clearance_m,
        approach_mode=approach_mode,
        full_path_collision_checked=False,motion_authorized=False)
    diagnostics.update(wrist_travel_deg=float(np.rad2deg(q[4]-seed[4])),
        wrist_from_home_deg=float(np.rad2deg(q[4]-arm.anchor[4])))
    if grasp_policy is not None:
        diagnostics.update(contact_posture=contact_posture(pose,grasp_policy),
            actual_pregrasp_posture=contact_posture(arm.fk(q)[0],grasp_policy),
            allowed_contact_roll_rad=roll_bounds)
    return accepted(solution,residual),q,pose,diagnostics
