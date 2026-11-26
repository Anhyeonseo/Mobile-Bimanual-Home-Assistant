"""Bounded 5-DOF Cartesian seeds; caller must check every segment for collision.

Full-pose 6D IK can reject an otherwise usable 5-DOF insertion for sub-degree
orientation mismatch. Keep position primary, then enforce the same explicit
position/path limits as the MoveIt path validator. Tabletop vertical picking
may explicitly allow 5 degrees of attitude change; contact geometry, finger
slope and collision checks remain separate requirements. No wrap, no
collision exemption, no hardware timing or execution authority.
"""
import math
from collections import deque
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .carry_planning import PlanRejected
from .carry_posture import checked_joints
from .s2_geometry import rigid


def validate_vertical_descent(poses):
    """Check actual FK samples, including retimed wire samples, not just seeds."""
    xyz=np.asarray([rigid(p)[:3,3] for p in poses])
    if len(xyz)<2 or not .003<=xyz[0,2]-xyz[-1,2]<=.052:
        raise ValueError('vertical descent requires a bounded downward stroke')
    lateral=float(np.max(np.linalg.norm(xyz[:,:2]-xyz[-1,:2],axis=1)))
    upward=float(np.max(np.diff(xyz[:,2])))
    if lateral>.002 or upward>.0002:
        raise ValueError('vertical descent exceeds 2 mm lateral drift or rises during insertion')
    return dict(drop_m=float(xyz[0,2]-xyz[-1,2]),maximum_lateral_m=lateral,maximum_upward_step_m=upward)


def bounded_cartesian(arm,start,goal, *, orientation_tolerance_deg=2.):
    if not np.isfinite(orientation_tolerance_deg) or not 0 < orientation_tolerance_deg <= 5:
        raise ValueError('Cartesian orientation tolerance must be within (0, 5] degrees')
    orientation_limit=math.radians(orientation_tolerance_deg)
    start=checked_joints(arm,start);goal=rigid(goal)
    origin=arm.fk(start)[0];delta=goal[:3,3]-origin[:3,3];distance=np.linalg.norm(delta)
    if not 0<distance<=.11:
        raise PlanRejected('bounded Cartesian stroke outside (0, 11 cm]')
    if Rotation.from_matrix(origin[:3,:3].T@goal[:3,:3]).magnitude()>orientation_limit:
        raise PlanRejected(f'bounded Cartesian start/goal orientation exceeds {orientation_tolerance_deg:g} degrees')
    path=[start.copy()]
    previous_fraction=0.
    pending=deque((float(f),0) for f in np.linspace(0,1,max(2,math.ceil(distance/.002)+1))[1:])
    while pending:
        fraction,subdivisions=pending.popleft()
        point=origin[:3,3]+fraction*delta
        def residual(q):
            pose=arm.fk(q)[0]
            return np.r_[(pose[:3,3]-point)/.0001,
                Rotation.from_matrix(goal[:3,:3].T@pose[:3,:3]).as_rotvec()/math.radians(1)]
        result=least_squares(residual,path[-1],bounds=(arm.lo,arm.hi),max_nfev=80)
        pose=arm.fk(result.x)[0]
        position_error=float(np.linalg.norm(pose[:3,3]-point))
        goal_angle=float(Rotation.from_matrix(goal[:3,:3].T@pose[:3,:3]).magnitude())
        start_angle=float(Rotation.from_matrix(origin[:3,:3].T@pose[:3,:3]).magnitude())
        joint_step=float(np.max(abs(result.x-path[-1])))
        violations=[]
        if not result.success:violations.append('solver')
        if position_error>.001:violations.append('position')
        if max(goal_angle,start_angle)>orientation_limit:violations.append('orientation')
        if joint_step>.1:violations.append('joint_step')
        if violations==['joint_step'] and subdivisions<4:
            # Near a singularity 2 mm can require a large yet continuous joint
            # change. Retry shorter Cartesian intervals; never enlarge the
            # branch bound or interpolate across a rejected jump.
            pending.appendleft((fraction,subdivisions+1))
            pending.appendleft(((previous_fraction+fraction)/2,subdivisions+1))
            continue
        if violations:
            error=PlanRejected('bounded 5-DOF Cartesian IK rejected: '+', '.join(violations))
            error.diagnostics=dict(violations=violations,fraction=float(fraction),
                position_error_mm=position_error*1000,goal_orientation_deg=math.degrees(goal_angle),
                start_orientation_deg=math.degrees(start_angle),joint_step_deg=math.degrees(joint_step),
                orientation_limit_deg=orientation_tolerance_deg,position_limit_mm=1.,
                minimum_joint_limit_margin_deg=float(np.rad2deg(np.min(np.minimum(result.x-arm.lo,arm.hi-result.x)))),
                solver_success=bool(result.success),nearest_model_rad=result.x.tolist(),
                target_point_m=point.tolist())
            raise error
        # Sample intervening joint-space positions too; checking endpoints alone
        # would miss lateral drift and payload/world collisions.
        count=max(1,math.ceil(np.max(abs(result.x-path[-1]))/math.radians(1)))
        path.extend(np.linspace(path[-1],result.x,count+1)[1:])
        previous_fraction=fraction
    return np.asarray(path)
