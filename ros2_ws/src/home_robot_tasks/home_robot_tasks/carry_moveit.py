"""Offline MoveIt service adapter; deliberately no controller/action execution.

OMPL generates free-space paths. GetCartesianPath generates insertion/lift.
Returned paths are additionally sampled for payload tilt and full-model collision.
This is sampled checking, not a continuous collision or physical holding proof.
"""
from copy import deepcopy
from dataclasses import dataclass, replace
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .carry_planning import PlanRejected
from .carry_posture import checked_joints
from .s2_geometry import JOINTS, rigid


@dataclass
class StagePath:
    label: str
    joints: np.ndarray
    planner: str
    duration_s: float = 0.

    @property
    def end(self):
        return self.joints[-1].copy()


class MoveItCarryPlanner:
    def __init__(self, arm, initial_state, call):
        self.arm, self.initial_state, self.call = arm, deepcopy(initial_state), call
        self.names = [arm.side+'_model_'+name for name in JOINTS[:5]]
        self.jaw_name = arm.side+'_model_gripper'
        self.tcp = arm.side+'_tcp'
        self.group = arm.side+'_arm'
        lift = self.initial_state.joint_state.position[self.initial_state.joint_state.name.index('s2_lift_joint')]
        if not math.isclose(lift, arm.lift_m, abs_tol=1e-9):
            raise ValueError('MoveIt and carry generator must use the same fixed lift')
        self.touch_links = [arm.side+'_s2_'+name for name in ('gripper_link', 'moving_jaw_so101_v1_link', 'gripper_frame_link')]
        self._scene_key = None
        self.checked_states = 0

    @staticmethod
    def pose(matrix):
        from geometry_msgs.msg import Pose
        matrix = rigid(matrix)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, Rotation.from_matrix(matrix[:3, :3]).as_quat())
        return pose

    def payload(self, candidate, attached):
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        obj = CollisionObject()
        obj.header.frame_id = self.tcp if attached else 'base_link'
        obj.id = 'carry_target'
        obj.operation = CollisionObject.ADD
        primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, candidate.object_size_m)))
        obj.primitives = [primitive]
        pose = candidate.tcp_from_object if attached else candidate.base_from_grasp_tcp @ candidate.tcp_from_object
        obj.primitive_poses = [self.pose(pose)]
        if candidate.payload_mesh is not None:
            from shape_msgs.msg import Mesh, MeshTriangle
            from geometry_msgs.msg import Point
            mesh=candidate.payload_mesh
            obj.primitives=[];obj.primitive_poses=[]
            obj.meshes=[Mesh(vertices=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in mesh['vertices']],
                             triangles=[MeshTriangle(vertex_indices=list(map(int,f))) for f in mesh['faces']])]
            obj.mesh_poses=[self.pose(pose)]
        return obj

    def prepare(self, candidate, attached):
        from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
        from moveit_msgs.msg import CollisionObject, AllowedCollisionEntry, PlanningSceneComponents
        key = (id(candidate), attached)
        if key == self._scene_key:
            return
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        query = GetPlanningScene.Request(components=PlanningSceneComponents(components=(
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX | PlanningSceneComponents.WORLD_OBJECT_NAMES)))
        current = self.call('get_planning_scene', query).scene
        obj = self.payload(candidate, False)
        if attached:
            obj.operation = CollisionObject.REMOVE
        if not attached or any(o.id == obj.id for o in current.world.collision_objects):
            request.scene.world.collision_objects = [obj]
        # Contact permission is local to this offline scene and only jaw/target.
        # A nonempty ACM replaces the matrix even in a scene diff. Preserve the
        # robot's existing adjacent-link allowances and all obstacle restrictions.
        matrix = current.allowed_collision_matrix
        for name in ['carry_target']+self.touch_links:
            if name not in matrix.entry_names:
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_names.append(name)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        target = matrix.entry_names.index('carry_target')
        for i, name in enumerate(matrix.entry_names):
            allowed = name in self.touch_links
            matrix.entry_values[target].enabled[i] = allowed
            matrix.entry_values[i].enabled[target] = allowed
        request.scene.allowed_collision_matrix = matrix
        if not self.call('apply_planning_scene', request).success:
            raise PlanRejected('planning scene update failed')
        self._scene_key = key

    def state(self, q, candidate=None, attached=False, jaw=None):
        from moveit_msgs.msg import AttachedCollisionObject
        state = deepcopy(self.initial_state)
        state.is_diff = False
        for name, value in zip(self.names, checked_joints(self.arm, q)):
            state.joint_state.position[state.joint_state.name.index(name)] = float(value)
        if candidate:
            value = candidate.held_jaw_model_rad if attached else candidate.open_jaw_model_rad
            state.joint_state.position[state.joint_state.name.index(self.jaw_name)] = float(value if jaw is None else jaw)
        state.attached_collision_objects = []
        if attached:
            obj = AttachedCollisionObject(link_name=self.tcp, object=self.payload(candidate, True), touch_links=self.touch_links)
            state.attached_collision_objects = [obj]
        return state

    def valid(self, q, candidate, attached, jaw=None):
        from moveit_msgs.srv import GetStateValidity
        if attached and candidate.object_constraint and not candidate.object_constraint.satisfied(self.arm.fk(q)[0]):
            raise PlanRejected('payload orientation violated along path')
        request = GetStateValidity.Request(robot_state=self.state(q, candidate, attached, jaw))
        result = self.call('check_state_validity', request)
        self.checked_states += 1
        if not result.valid:
            contacts = [(c.contact_body_1, c.contact_body_2) for c in result.contacts]
            failure=PlanRejected('invalid full robot state: '+str(contacts))
            failure.diagnostics=dict(model_rad=np.asarray(q).tolist(),jaw_model_rad=jaw,
                attached=attached,contacts=[dict(bodies=[c.contact_body_1,c.contact_body_2],
                    position_m=[c.position.x,c.position.y,c.position.z],depth_m=c.depth) for c in result.contacts])
            raise failure

    def payload_clearance(self,q,candidate):
        """Soft endpoint preference; finger contacts excluded, not exact distance."""
        from .carry_load import CarriedGeometry, inflated_payload_mesh
        geometry=CarriedGeometry(candidate.tcp_from_object,candidate.object_size_m,candidate.payload_mesh)
        passed=0.
        self.prepare(candidate,True)
        self.valid(q,candidate,True)
        try:
            for margin in (.005,.01,.02):
                probe=replace(candidate,payload_mesh=inflated_payload_mesh(geometry,margin))
                self.prepare(probe,True)
                try:self.valid(q,probe,True)
                except PlanRejected:break
                passed=margin
        finally:self.prepare(candidate,True)
        return dict(passed_margin_m=passed,ceiling_m=.02,kind='convex_envelope_offset_lower_bound',
                    scope='payload_endpoint_to_non_touch_robot_and_observed_world',
                    exact_distance=False,whole_route_margin_proven=False)

    def checked_path(self, trajectory, start, candidate, attached, label, planner):
        jt = trajectory.joint_trajectory
        if set(jt.joint_names) != set(self.names) or len(jt.joint_names) != 5 or not jt.points:
            raise PlanRejected('incomplete or unexpected MoveIt trajectory')
        indexes = [jt.joint_names.index(n) for n in self.names]
        q = np.array([[p.positions[i] for i in indexes] for p in jt.points])
        if not np.isfinite(q).all() or np.max(abs(q[0]-start)) > .002:
            raise PlanRejected('MoveIt path start differs from requested anchor')
        if np.any(abs(q-start) >= math.pi) or np.any(abs(np.diff(q, axis=0)) >= math.pi):
            raise PlanRejected('large joint branch change; no encoder wrapping allowed')
        samples = [q[0]]
        for a, b in zip(q[:-1], q[1:]):
            steps = max(1, math.ceil(np.max(abs(b-a))/math.radians(1)))
            samples.extend(np.linspace(a, b, steps+1)[1:])
        for value in samples:
            self.valid(value, candidate, attached)
        duration = jt.points[-1].time_from_start
        return StagePath(label, np.asarray(samples), planner, duration.sec+duration.nanosec/1e9)

    def connect(self, start, goal, candidate, *, attached, label):
        from moveit_msgs.srv import GetMotionPlan
        from moveit_msgs.msg import Constraints, JointConstraint
        self.prepare(candidate, attached)
        self.valid(start, candidate, attached); self.valid(goal, candidate, attached)
        request = GetMotionPlan.Request()
        r = request.motion_plan_request
        r.group_name = self.group; r.pipeline_id = 'ompl'; r.planner_id = 'RRTConnect'
        r.start_state = self.state(start, candidate, attached)
        r.num_planning_attempts = 2; r.allowed_planning_time = 5.
        r.max_velocity_scaling_factor = .2; r.max_acceleration_scaling_factor = .2
        if attached and candidate.object_constraint:
            r.path_constraints = self.orientation_hint(candidate)
            # First try the nearby carry goal directly in joint space. This
            # avoids random six-dimensional pose sampling on a five-DOF arm.
            r.planner_id = 'RRTGoal'
        r.goal_constraints = [Constraints(joint_constraints=[JointConstraint(
            joint_name=n, position=float(q), tolerance_above=.0001, tolerance_below=.0001, weight=1.)
            for n, q in zip(self.names, goal)])]
        response = self.call('plan_kinematic_path', request).motion_plan_response
        planner_id = r.planner_id
        if response.error_code.val != 1 and planner_id == 'RRTGoal':
            r.planner_id = 'RRTConnect'
            response = self.call('plan_kinematic_path', request).motion_plan_response
            planner_id = r.planner_id
        if response.error_code.val != 1:
            raise PlanRejected('OMPL error '+str(response.error_code.val))
        path = self.checked_path(response.trajectory, start, candidate, attached, label, 'MoveIt OMPL/'+planner_id)
        if np.max(abs(path.end-goal)) > .002:
            raise PlanRejected('MoveIt missed carry/pregrasp joint goal')
        return path

    def orientation_hint(self, candidate):
        """Guide OMPL with world-frame rotation-vector bounds, then check exact tilt.

        The rectangular rotation-vector region is a search hint, not the exact
        spherical object-axis limit. checked_path always enforces that limit.
        World-Z tasks keep yaw free. Other axes use a conservative rotation box.
        """
        from moveit_msgs.msg import Constraints, OrientationConstraint
        constraint = candidate.object_constraint
        grasp_rotation = candidate.base_from_grasp_tcp[:3, :3]
        current_axis = grasp_rotation @ constraint.tcp_from_object[:3,:3] @ constraint.object_axis
        cross = np.cross(current_axis, constraint.base_axis)
        sine = np.linalg.norm(cross)
        cosine = np.clip(np.dot(current_axis, constraint.base_axis), -1., 1.)
        if sine < 1e-10:
            if cosine < 0:
                raise PlanRejected('grasp is upside-down relative to the required object axis')
            correction = np.eye(3)
        else:
            correction = Rotation.from_rotvec(cross/sine*np.arctan2(sine, cosine)).as_matrix()
        desired = np.eye(4); desired[:3,:3] = correction@grasp_rotation
        result = OrientationConstraint()
        result.header.frame_id = 'base_link'; result.link_name = self.tcp
        result.orientation = self.pose(desired).orientation
        result.parameterization = OrientationConstraint.ROTATION_VECTOR
        result.absolute_x_axis_tolerance = constraint.maximum_tilt_rad
        result.absolute_y_axis_tolerance = constraint.maximum_tilt_rad
        result.absolute_z_axis_tolerance = (math.pi if np.allclose(constraint.base_axis,[0,0,1])
                                            else constraint.maximum_tilt_rad)
        result.weight = 1.
        return Constraints(orientation_constraints=[result])

    def cartesian(self, start, goal, candidate, *, attached, label):
        from moveit_msgs.srv import GetCartesianPath
        self.prepare(candidate, attached)
        request = GetCartesianPath.Request()
        request.header.frame_id = 'base_link'
        request.start_state = self.state(start, candidate, attached)
        request.group_name = self.group; request.link_name = self.tcp
        request.waypoints = [self.pose(goal)]
        request.max_step = .002; request.revolute_jump_threshold = .2
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = .2; request.max_acceleration_scaling_factor = .2
        response = self.call('compute_cartesian_path', request)
        if response.error_code.val != 1 or response.fraction < .999999:
            from .carry_cartesian import bounded_cartesian
            samples=bounded_cartesian(self.arm,start,goal)
            for q in samples:self.valid(q,candidate,attached)
            path=StagePath(label,samples,'bounded 5DOF IK + MoveIt collision checks; Cartesian fraction='+str(response.fraction))
        else:
            path = self.checked_path(response.solution, start, candidate, attached, label, 'MoveIt Cartesian')
        actual = self.arm.fk(path.end)[0]
        if np.linalg.norm(actual[:3, 3]-goal[:3, 3]) > .001 or Rotation.from_matrix(goal[:3, :3].T@actual[:3, :3]).magnitude() > math.radians(2):
            raise PlanRejected('Cartesian endpoint pose mismatch')
        # Cartesian interpolation must preserve the requested orientation and line.
        origin = self.arm.fk(start)[0]
        direction = goal[:3, 3]-origin[:3, 3]
        distance = np.linalg.norm(direction)
        previous = -.001
        for q in path.joints:
            pose = self.arm.fk(q)[0]
            if Rotation.from_matrix(origin[:3, :3].T@pose[:3, :3]).magnitude() > math.radians(2):
                raise PlanRejected('Cartesian orientation drift')
            if distance > 1e-6:
                along = np.dot(pose[:3, 3]-origin[:3, 3], direction/distance)
                lateral = np.linalg.norm(pose[:3, 3]-origin[:3, 3]-along*direction/distance)
                if lateral > .001 or along < previous-.0001 or along > distance+.001:
                    raise PlanRejected('Cartesian lateral drift or backward motion')
                previous = along
        return path

    def open(self, q, candidate):
        self.prepare(candidate, False)
        current = self.initial_state.joint_state.position[self.initial_state.joint_state.name.index(self.jaw_name)]
        for angle in np.linspace(current, candidate.open_jaw_model_rad, 16):
            self.valid(q, candidate, False, jaw=angle)
        return StagePath('open_geometry', np.array([q]), 'sampled opening sweep')

    def close(self, q, candidate):
        self.prepare(candidate, False)
        for angle in np.linspace(candidate.open_jaw_model_rad, candidate.held_jaw_model_rad, 16):
            self.valid(q, candidate, False, jaw=angle)
        self.prepare(candidate, True)
        self.valid(q, candidate, True)
        return StagePath('close_geometry', np.array([q]), 'sampled jaw sweep; holding not verified')
