"""Bounded numerical pose IK; no collision, path or hardware execution.

All six pose residuals are checked against explicit tolerances even when only
five joints are available. Position-only diagnostics never approve a grasp.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from tools.lib.grasp_preview import JOINTS, joint_positions, pose_matrix, vector


def rotation_matrix(value, name):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
            or not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(matrix), 1., atol=1e-5)):
        raise ValueError(name + ' must be a proper rotation')
    return matrix


def rigid_transform(value, name):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1]):
        raise ValueError(name + ' must be a finite rigid transform')
    rotation_matrix(matrix[:3, :3], name)
    return matrix


@dataclass(frozen=True)
class IKPolicy:
    position_tolerance_m: float = .005
    orientation_tolerance_degrees: float = 10.
    allow_parallel_jaw_half_turn: bool = True
    maximum_starts: int = 10
    maximum_function_evaluations: int = 120
    random_seed: int = 0

    def __post_init__(self):
        for name, low, high in (('position_tolerance_m', 0, .05),
                                ('orientation_tolerance_degrees', 0, 30)):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or not low < value <= high:
                raise ValueError('invalid ' + name)
        for name, low, high in (('maximum_starts', 2, 64),
                                ('maximum_function_evaluations', 10, 2000), ('random_seed', 0, 2**32 - 1)):
            value = getattr(self, name)
            if type(value) is not int or not low <= value <= high:
                raise ValueError('invalid ' + name)
        if type(self.allow_parallel_jaw_half_turn) is not bool:
            raise ValueError('half-turn symmetry must be boolean')


def load_settings(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if type(data.get('schema_version')) is not int or data['schema_version'] != 1 or data.get('mode') != 'virtual':
        raise ValueError('virtual IK settings schema_version 1 required')
    rotation_matrix(data['grasp_from_tcp_rotation'], 'GraspNet-to-TCP convention')
    vector(data['grasp_to_tcp_translation_m'], 3, 'GraspNet-to-TCP offset')
    IKPolicy(**data['policy'])
    return data


def target_from_grasp(grasp, root_from_camera, settings):
    camera = rigid_transform(root_from_camera, 'camera transform')
    frame = np.eye(4)
    frame[:3, :3] = rotation_matrix(grasp['rotation_matrix'], 'grasp rotation')
    frame[:3, 3] = vector(grasp['translation_m'], 3, 'grasp translation')
    tcp = np.eye(4)
    tcp[:3, :3] = rotation_matrix(settings['grasp_from_tcp_rotation'], 'TCP convention')
    tcp[:3, 3] = vector(settings['grasp_to_tcp_translation_m'], 3, 'TCP offset')
    return camera @ frame @ tcp


class BoundedPoseIK:
    def __init__(self, robot, joint_names, tcp_link, policy):
        self.robot, self.names, self.tcp, self.policy = robot, tuple(joint_names), tcp_link, policy
        if not self.names or len(set(self.names)) != len(self.names) or not set(self.names).issubset(robot.bounds):
            raise ValueError('distinct bounded IK joint names required')
        self.bounds = np.asarray([robot.bounds[name] for name in self.names], dtype=float)
        if not np.isfinite(self.bounds).all() or not (self.bounds[:, 0] < self.bounds[:, 1]).all():
            raise ValueError('finite, nonempty IK bounds required')

    def solve(self, target, anchor):
        target = rigid_transform(target, 'IK target')
        # Validate all fixed/active positions, including the visualization jaw.
        self.robot.link_transforms(anchor)
        initial = np.asarray([anchor[name] for name in self.names], dtype=float)
        midpoint = self.bounds.mean(axis=1)
        rng = np.random.default_rng(self.policy.random_seed)
        seeds = [initial, midpoint, *rng.uniform(self.bounds[:, 0], self.bounds[:, 1],
                                                size=(self.policy.maximum_starts - 2, len(self.names)))]
        angular_tolerance = math.radians(self.policy.orientation_tolerance_degrees)
        flips = (0, 1) if self.policy.allow_parallel_jaw_half_turn else (0,)
        attempts, evaluations, solutions = 0, 0, []

        def fk(values):
            positions = dict(anchor)
            positions.update(zip(self.names, map(float, values)))
            return self.robot.link_transforms(positions)[self.tcp]

        def position_error(values):
            return (fk(values)[:3, 3] - target[:3, 3]) / self.policy.position_tolerance_m

        for flip in flips:
            desired = target[:3, :3] @ Rotation.from_rotvec([0, 0, math.pi * flip]).as_matrix()
            desired_transform = target.copy()
            desired_transform[:3, :3] = desired

            def residual(values):
                actual = fk(values)
                return np.r_[(actual[:3, 3] - target[:3, 3]) / self.policy.position_tolerance_m,
                             Rotation.from_matrix(desired.T @ actual[:3, :3]).as_rotvec() / angular_tolerance]

            for seed in seeds:
                optimized = least_squares(residual, seed, bounds=(self.bounds[:, 0], self.bounds[:, 1]),
                                          max_nfev=self.policy.maximum_function_evaluations,
                                          ftol=1e-9, xtol=1e-9, gtol=1e-9)
                attempts += 1
                evaluations += optimized.nfev
                actual = fk(optimized.x)
                position = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
                angle = float(Rotation.from_matrix(desired.T @ actual[:3, :3]).magnitude())
                feasible = position <= self.policy.position_tolerance_m and angle <= angular_tolerance
                normalized = max(position / self.policy.position_tolerance_m, angle / angular_tolerance)
                solutions.append({'kinematically_feasible': feasible, 'position_error_m': position,
                                  'orientation_error_degrees': math.degrees(angle),
                                  'normalized_error': normalized, 'half_turn_applied': bool(flip),
                                  'joint_positions_rad': {name: float(value) for name, value in zip(self.names, optimized.x)},
                                  'joint_displacement_rad': float(np.linalg.norm(optimized.x - initial)),
                                  'minimum_joint_margin_rad': float(np.min(np.minimum(
                                      optimized.x - self.bounds[:, 0], self.bounds[:, 1] - optimized.x))),
                                  'achieved_tcp_transform': actual.tolist(),
                                  'target_tcp_transform': desired_transform.tolist()})
        # A lower least-squares sum must not displace a solution that passes
        # both individual thresholds with one that fails either threshold.
        best = min(solutions, key=lambda r: (not r['kinematically_feasible'], r['normalized_error'], r['joint_displacement_rad']))
        position_only = None
        if not best['kinematically_feasible']:
            for seed in seeds:
                diagnostic = least_squares(position_error, seed, bounds=(self.bounds[:, 0], self.bounds[:, 1]),
                                            max_nfev=self.policy.maximum_function_evaluations)
                error = float(np.linalg.norm(position_error(diagnostic.x)) * self.policy.position_tolerance_m)
                if position_only is None or error < position_only['position_error_m']:
                    position_only = {'position_error_m': error,
                                     'joint_positions_rad': {n: float(v) for n, v in zip(self.names, diagnostic.x)}}
        status = ('IK_FEASIBLE' if best['kinematically_feasible'] else
                  'ORIENTATION_NOT_MATCHED' if position_only['position_error_m'] <= self.policy.position_tolerance_m
                  else 'NO_POSITION_SOLUTION_FOUND')
        return {**best, 'status': status, 'pose_search_attempts': attempts,
                'pose_function_evaluations': evaluations, 'position_only_diagnostic': position_only,
                'search_complete': False, 'collision_checked': False, 'path_checked': False,
                'gripper_width_checked': False}


def evaluate_candidates(robot, layout, grasps, settings):
    policy = IKPolicy(**settings['policy'])
    arm = layout['arm']
    names = [f'{arm}_{name}_joint' for name in JOINTS if name != 'gripper']
    solver = BoundedPoseIK(robot, names, arm + '_gripper_frame_link', policy)
    anchor = joint_positions(layout)
    camera = pose_matrix(layout['camera_translation_m'], layout['camera_rpy_degrees'])
    if not isinstance(grasps, list) or len(grasps) > 128:
        raise ValueError('bounded list of up to 128 grasp candidates required')
    results = []
    for index, grasp in enumerate(grasps):
        try:
            for field in ('width_m', 'depth_m'):
                value = grasp[field]
                if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= .5:
                    raise ValueError('invalid grasp ' + field)
            score = grasp['score']
            if type(score) not in (int, float) or not math.isfinite(score):
                raise ValueError('finite GraspNet score required')
            target = target_from_grasp(grasp, camera, settings)
            result = solver.solve(target, anchor)
            results.append({'candidate_index': index, 'graspnet_score': score, **result})
        except (ValueError, TypeError, KeyError) as error:
            results.append({'candidate_index': index, 'status': 'INVALID_CANDIDATE',
                            'kinematically_feasible': False, 'error': str(error)})
    accepted = [result for result in results if result['kinematically_feasible']]
    selected = min(accepted, key=lambda r: (r['normalized_error'], r['joint_displacement_rad'], -r['graspnet_score'])) if accepted else None
    return {'schema_version': 1, 'mode': 'virtual_kinematic_check',
            'robot_description_sha256': hashlib.sha256(robot.xml.encode()).hexdigest(),
            'active_joint_limits_rad': {name: list(robot.bounds[name]) for name in names},
            'policy': asdict(policy), 'tool_mapping': {
                'grasp_from_tcp_rotation': settings['grasp_from_tcp_rotation'],
                'grasp_to_tcp_translation_m': settings['grasp_to_tcp_translation_m']},
            'calibration_measured': False, 'ik_checked': True, 'collision_checked': False,
            'path_checked': False, 'gripper_width_checked': False,
            'hardware_commands': 0, 'physical_task_completed': False,
            'candidate_count': len(results), 'accepted_count': len(accepted),
            'selected_index': selected['candidate_index'] if selected is not None else None,
            'candidates': results}
