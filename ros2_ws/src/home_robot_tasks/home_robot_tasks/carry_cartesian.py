"""Bounded 5-DOF Cartesian seeds; caller must check every segment for collision.

Full-pose 6D IK can reject an otherwise usable 5-DOF insertion for sub-degree
orientation mismatch. Keep position primary, then enforce the same explicit
1 mm / 2 degree geometric limits as the MoveIt path validator. No wrap, no
collision exemption, no hardware timing or execution authority.
"""
import math
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


def bounded_cartesian(arm,start,goal):
    start=checked_joints(arm,start);goal=rigid(goal)
    origin=arm.fk(start)[0];delta=goal[:3,3]-origin[:3,3];distance=np.linalg.norm(delta)
    if not 0<distance<=.11:
        raise PlanRejected('bounded Cartesian stroke outside (0, 11 cm]')
    if Rotation.from_matrix(origin[:3,:3].T@goal[:3,:3]).magnitude()>math.radians(2):
        raise PlanRejected('bounded Cartesian start/goal orientation differs by over 2 degrees')
    path=[start.copy()]
    for fraction in np.linspace(0,1,max(2,math.ceil(distance/.002)+1))[1:]:
        point=origin[:3,3]+fraction*delta
        def residual(q):
            pose=arm.fk(q)[0]
            return np.r_[(pose[:3,3]-point)/.0001,
                Rotation.from_matrix(goal[:3,:3].T@pose[:3,:3]).as_rotvec()/math.radians(1)]
        result=least_squares(residual,path[-1],bounds=(arm.lo,arm.hi),max_nfev=80)
        pose=arm.fk(result.x)[0]
        if (not result.success or np.linalg.norm(pose[:3,3]-point)>.001
                or Rotation.from_matrix(goal[:3,:3].T@pose[:3,:3]).magnitude()>math.radians(2)
                or Rotation.from_matrix(origin[:3,:3].T@pose[:3,:3]).magnitude()>math.radians(2)
                or np.max(abs(result.x-path[-1]))>.1):
            raise PlanRejected('bounded 5-DOF Cartesian IK failed position/orientation/branch limits')
        # Sample intervening joint-space positions too; checking endpoints alone
        # would miss lateral drift and payload/world collisions.
        count=max(1,math.ceil(np.max(abs(result.x-path[-1]))/math.radians(1)))
        path.extend(np.linspace(path[-1],result.x,count+1)[1:])
    return np.asarray(path)
