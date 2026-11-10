"""L3D: bounded carry-goal search, independent of ROS and motor control.

The common posture is a preference, not an exact goal. Object orientation is
expressed in the object frame; it is never inferred from a grasp score/name.
All angles are official SO101 model angles, not controller angles.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from .s2_geometry import rigid
from .carry_load import payload_load


@dataclass(frozen=True)
class ObjectAxisConstraint:
    tcp_from_object: np.ndarray
    object_axis: tuple = (0., 0., 1.)
    base_axis: tuple = (0., 0., 1.)
    maximum_tilt_rad: float = math.radians(10)
    evidence: str = ""

    def __post_init__(self):
        rigid(self.tcp_from_object)
        if not self.evidence.strip() or not 0 < self.maximum_tilt_rad < math.pi:
            raise ValueError("explicit object orientation evidence and finite tolerance required")
        for axis in (self.object_axis, self.base_axis):
            if np.shape(axis) != (3,) or not np.isfinite(axis).all() or not np.isclose(np.linalg.norm(axis), 1.):
                raise ValueError("object and base axes must be unit vectors")

    def tilt(self, base_from_tcp):
        actual = rigid(base_from_tcp)[:3, :3] @ rigid(self.tcp_from_object)[:3, :3] @ self.object_axis
        return float(np.arccos(np.clip(np.dot(actual, self.base_axis), -1., 1.)))

    def satisfied(self, base_from_tcp):
        return self.tilt(base_from_tcp) <= self.maximum_tilt_rad + 1e-7


@dataclass(frozen=True)
class CarryPolicy:
    # Offline tuning defaults, not measured robot performance or torque limits.
    joint_weights: tuple = (4., 3., 3., .6, .25)
    reference_position_weight: float = .3
    grasp_orientation_weight: float = 2.
    travel_weight: float = .08
    maximum_results: int = 6
    maximum_iterations: int = 100
    payload_moment_weight: float = .35
    joint_margin_weight: float = .015
    # Among complete feasible routes, keep only near-best grasp balance before
    # comparing carry comfort/travel. This is a preference, not a force limit.
    grasp_balance_band: float = .03
    reference_model_deg: tuple = (0.,-70.,35.,90.,90.)
    # Same folded-arm family, progressively higher/forward. Bounded search
    # alternatives, not object-name rules or motor command waypoints.
    exploration_offsets_deg: tuple = ((0.,10.,-10.,0.,0.),(0.,25.,-25.,0.,0.))

    def __post_init__(self):
        weights = np.asarray(self.joint_weights, float)
        if weights.shape != (5,) or not np.isfinite(weights).all() or np.any(weights <= 0):
            raise ValueError("five positive joint weights required")
        for weight in (self.reference_position_weight, self.grasp_orientation_weight, self.travel_weight,
                       self.payload_moment_weight,self.joint_margin_weight):
            if not np.isfinite(weight) or weight < 0:
                raise ValueError("invalid carry preference")
        if not 1 <= self.maximum_results <= 8 or not 1 <= self.maximum_iterations <= 300:
            raise ValueError("carry search must remain bounded")
        if not np.isfinite(self.grasp_balance_band) or not 0<=self.grasp_balance_band<=1:
            raise ValueError('bounded grasp balance preference required')
        if np.shape(self.reference_model_deg)!=(5,) or not np.isfinite(self.reference_model_deg).all():
            raise ValueError('five finite carry reference angles required')
        if len(self.exploration_offsets_deg)>2 or any(np.shape(x)!=(5,) or not np.isfinite(x).all()
                or np.max(abs(np.asarray(x)))>30 for x in self.exploration_offsets_deg):
            raise ValueError('at most two bounded carry reference offsets required')


@dataclass(frozen=True)
class CarryGoal:
    model_rad: np.ndarray
    base_from_tcp: np.ndarray
    cost: float
    grasp_orientation_change_rad: float
    object_tilt_rad: float | None


def checked_joints(arm, value):
    value = np.asarray(value, float)
    if value.shape != (5,) or not np.isfinite(value).all() or np.any(value < arm.lo) or np.any(value > arm.hi):
        raise ValueError("five finite in-range model joint angles required")
    return value


def carry_goals(arm, lifted_model_rad, grasp_base_from_tcp, *, reference_model_rad=None,
                object_constraint=None, policy=CarryPolicy(), payload=None):
    """Bounded soft-reference alternatives; collision/route feasibility belongs to MoveIt.

    The grasp rotation remains a preference even for an unconstrained rigid
    object. A supplied object-axis constraint is hard and never auto-relaxed.
    Joint differences are continuous, without modulo/shortest-wrap conversion.
    """
    lifted = checked_joints(arm, lifted_model_rad)
    reference = checked_joints(arm, np.deg2rad(policy.reference_model_deg) if reference_model_rad is None else reference_model_rad)
    grasp = rigid(grasp_base_from_tcp)
    reference_pose = arm.fk(reference)[0]
    if object_constraint and not object_constraint.satisfied(arm.fk(lifted)[0]):
        raise ValueError("lifted state violates the requested object orientation")

    def score(q,reference_scale=1.,target_reference=None):
        target=reference if target_reference is None else target_reference
        target_pose=reference_pose if target_reference is None else arm.fk(target)[0]
        pose = arm.fk(q)[0]
        rotation = Rotation.from_matrix(grasp[:3, :3].T @ pose[:3, :3]).as_rotvec()
        load=payload_load(arm,q,payload) if payload is not None else None
        margin=np.maximum(np.minimum(q-arm.lo,arm.hi-q),.02)
        return float(reference_scale*np.dot(policy.joint_weights, (q-target)**2)
                     + reference_scale*policy.reference_position_weight*np.sum(((pose[:3, 3]-target_pose[:3, 3])/.1)**2)
                     + policy.grasp_orientation_weight*np.dot(rotation, rotation)
                     + policy.travel_weight*np.sum((q-lifted)**2)
                     + (.5*(object_constraint.tilt(pose)/object_constraint.maximum_tilt_rad)**2
                        if object_constraint else 0.)
                     + (policy.payload_moment_weight*((load['shoulder_horizontal_lever_m']/.15)**2
                         + .5*(load['elbow_horizontal_lever_m']/.15)**2) if load else 0.)
                     + policy.joint_margin_weight*np.sum((.12/margin)**2))

    constraints = []
    if object_constraint:
        constraints = [{'type': 'ineq', 'fun': lambda q: object_constraint.maximum_tilt_rad
                        - object_constraint.tilt(arm.fk(q)[0])}]
    seeds = [reference, lifted, (reference+lifted)/2]
    for sign in (-1, 1):
        seed = reference.copy()
        seed[-1] += sign*.6
        seeds.append(np.clip(seed, arm.lo, arm.hi))
    answers = []
    # Preserve alternatives when the closest-to-reference optimum collides with a
    # long payload. Vary only the soft reference preference; all hard orientation
    # constraints and the subsequent complete MoveIt checks remain identical.
    # Eleven preference solves plus at most four clearance-reference solves,
    # independent of object name/category. All are ranked against one reference.
    for scale in (1.,.5,.2,.05):
        local=[]
        for seed in seeds if scale==1. else [reference,lifted]:
            solved = minimize(lambda q:score(q,scale), seed, method='SLSQP', bounds=list(zip(arm.lo, arm.hi)),
                              constraints=constraints, options={'maxiter': policy.maximum_iterations, 'ftol': 1e-8})
            if not solved.success or not np.isfinite(solved.x).all():continue
            q = checked_joints(arm, solved.x);pose = arm.fk(q)[0]
            if object_constraint and not object_constraint.satisfied(pose):continue
            if any(np.max(abs(q-a.model_rad)) < .01 for a in answers):continue
            change = Rotation.from_matrix(grasp[:3, :3].T @ pose[:3, :3]).magnitude()
            local.append(CarryGoal(q, pose, score(q), float(change),object_constraint.tilt(pose) if object_constraint else None))
        if local:answers.append(min(local,key=lambda a:a.cost))
    for offset in policy.exploration_offsets_deg:
        target=reference+np.deg2rad(offset)
        if np.any(target<arm.lo) or np.any(target>arm.hi):continue
        local=[]
        for seed in (target,lifted):
            solved=minimize(lambda q:score(q,target_reference=target),seed,method='SLSQP',
                bounds=list(zip(arm.lo,arm.hi)),constraints=constraints,
                options={'maxiter':policy.maximum_iterations,'ftol':1e-8})
            if not solved.success or not np.isfinite(solved.x).all():continue
            q=checked_joints(arm,solved.x);pose=arm.fk(q)[0]
            if object_constraint and not object_constraint.satisfied(pose):continue
            if any(np.max(abs(q-a.model_rad))<.01 for a in answers):continue
            change=Rotation.from_matrix(grasp[:3,:3].T@pose[:3,:3]).magnitude()
            local.append(CarryGoal(q,pose,score(q),float(change),object_constraint.tilt(pose) if object_constraint else None))
        if local:answers.append(min(local,key=lambda a:a.cost))
    return sorted(answers, key=lambda a: a.cost)[:policy.maximum_results]
