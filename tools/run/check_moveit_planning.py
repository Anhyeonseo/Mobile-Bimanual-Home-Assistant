#!/usr/bin/env python3
"""Run real MoveIt IK, collision and planning services against synthetic states.

No trajectory execution/controller or device is started. Source ROS Jazzy first.
All newly assembled geometry and states are simulation candidates only.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "output/moveit-planning")
    parser.add_argument(
        "--retain-moveit-plugin",
        action="store_true",
        help="Keep the capability library loaded in this child process (Jazzy 2.12.4 shutdown workaround)",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    from home_robot_tasks.moveit_config import simulation_parameters
    from ament_index_python.packages import (
        get_package_prefix,
        get_package_share_directory,
    )
    import rclpy
    from moveit_msgs.srv import (
        GetPositionFK,
        GetPositionIK,
        GetStateValidity,
        GetMotionPlan,
        ApplyPlanningScene,
    )
    from moveit_msgs.msg import (
        RobotState,
        Constraints,
        JointConstraint,
        CollisionObject,
    )
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState

    parameters = simulation_parameters(
        ROOT / "ros2_ws/src/so101_description",
        ROOT / "config/bimanual_operational_limits.json",
    )
    robot = ET.fromstring(parameters["robot_description"])
    names = [
        j.attrib["name"] for j in robot.findall("joint") if j.attrib["type"] != "fixed"
    ]
    state = RobotState()
    state.joint_state.name = names
    state.joint_state.position = [0.0] * len(names)
    state.is_diff = False
    executable = (
        Path(get_package_prefix("moveit_ros_move_group"))
        / "lib/moveit_ros_move_group/move_group"
    )
    report = {
        "mode": "simulation",
        "hardware_commands": 0,
        "physical_task_completed": False,
        "dimensions_measured": False,
        "response_adapters": parameters["ompl"]["response_adapters"],
        "motion_limits": {k: parameters['robot_description_planning']['joint_limits']['left_base_joint'][k]
                          for k in ('max_velocity','max_acceleration','max_jerk')},
        "checks": [],
        "passed": False,
    }
    report["ros_packages"] = {
        package: ET.parse(
            Path(get_package_share_directory(package)) / "package.xml"
        ).findtext("version")
        for package in (
            "moveit_ros_move_group",
            "moveit_kinematics",
            "moveit_planners_ompl",
            "rclcpp",
        )
    }
    child_env = os.environ.copy()
    if args.retain_moveit_plugin:
        plugin = (
            Path(get_package_prefix("moveit_ros_move_group"))
            / "lib/libmoveit_move_group_default_capabilities.so"
        )
        if not plugin.is_file():
            raise RuntimeError("MoveIt capability library unavailable")
        child_env["LD_PRELOAD"] = str(plugin) + (
            ":" + child_env["LD_PRELOAD"] if child_env.get("LD_PRELOAD") else ""
        )
        report["plugin_lifetime_workaround"] = {
            "path": str(plugin.resolve()),
            "sha256": hashlib.sha256(plugin.read_bytes()).hexdigest(),
        }
    else:
        report["plugin_lifetime_workaround"] = False
    process = None
    rsp = None
    node = None
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory(prefix="alohamini-moveit-") as tmp, (
            args.output / "move_group.log"
        ).open("w") as log:
            param = Path(tmp) / "parameters.yaml"
            param.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
            process = subprocess.Popen(
                [
                    str(executable),
                    "--ros-args",
                    "-r",
                    "__ns:=/offline_moveit",
                    "--params-file",
                    str(param),
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                env=child_env,
            )
            rsp_executable = (
                Path(get_package_prefix("robot_state_publisher"))
                / "lib/robot_state_publisher/robot_state_publisher"
            )
            rsp = subprocess.Popen(
                [
                    str(rsp_executable),
                    "--ros-args",
                    "-r",
                    "__ns:=/offline_moveit",
                    "--params-file",
                    str(param),
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            node = rclpy.create_node(
                "offline_moveit_check", namespace="/offline_moveit"
            )
            publisher = node.create_publisher(JointState, "joint_states", 10)

            def publish_state():
                state.joint_state.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(state.joint_state)

            node.create_timer(0.02, publish_state)
            clients = {
                name: node.create_client(kind, name)
                for name, kind in [
                    ("compute_fk", GetPositionFK),
                    ("compute_ik", GetPositionIK),
                    ("check_state_validity", GetStateValidity),
                    ("plan_kinematic_path", GetMotionPlan),
                    ("apply_planning_scene", ApplyPlanningScene),
                ]
            }
            ready_by = time.monotonic() + 20
            for client in clients.values():
                while not client.wait_for_service(timeout_sec=0.2):
                    if process.poll() is not None or time.monotonic() >= ready_by:
                        raise RuntimeError(
                            "MoveIt services unavailable; inspect move_group.log"
                        )

            def call(name, request):
                future = clients[name].call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=8)
                if not future.done():
                    raise RuntimeError("MoveIt service timeout: " + name)
                return future.result()

            def valid(candidate):
                req = GetStateValidity.Request()
                req.robot_state = candidate
                return call("check_state_validity", req)

            initial = valid(state)
            if not initial.valid:
                raise RuntimeError(
                    "Initial model collision: "
                    + str(
                        [(c.contact_body_1, c.contact_body_2) for c in initial.contacts]
                    )
                )
            report["checks"].append("collision_free_start")
            prior_z = {}
            for lift in (0.0, 0.2):
                state.joint_state.position[names.index("lift_joint")] = lift
                assert valid(state).valid
                for arm in ("left", "right"):
                    link = arm + "_gripper_frame_link"
                    fk = GetPositionFK.Request()
                    fk.header.frame_id = "base_link"
                    fk.fk_link_names = [link]
                    fk.robot_state = state
                    pose = call("compute_fk", fk)
                    assert pose.error_code.val == 1 and len(pose.pose_stamped) == 1
                    ik = GetPositionIK.Request()
                    ik.ik_request.group_name = arm + "_arm"
                    ik.ik_request.robot_state = deepcopy(state)
                    ik.ik_request.robot_state.joint_state.position[
                        names.index(arm + "_base_joint")
                    ] = 0.05
                    ik.ik_request.avoid_collisions = True
                    ik.ik_request.ik_link_name = link
                    ik.ik_request.pose_stamped = pose.pose_stamped[0]
                    ik.ik_request.timeout.sec = 1
                    solved = call("compute_ik", ik)
                    assert solved.error_code.val == 1 and valid(solved.solution).valid
                    solved_fk = deepcopy(fk)
                    solved_fk.robot_state = solved.solution
                    achieved = call("compute_fk", solved_fk)
                    assert achieved.error_code.val == 1
                    a, b = (
                        achieved.pose_stamped[0].pose.position,
                        pose.pose_stamped[0].pose.position,
                    )
                    error = (
                        sum(
                            (getattr(a, axis) - getattr(b, axis)) ** 2
                            for axis in ("x", "y", "z")
                        )
                        ** 0.5
                    )
                    assert error < 0.005
                    if arm in prior_z:
                        assert abs((b.z - prior_z[arm]) - 0.2) < 1e-6
                    prior_z[arm] = b.z
                    report["checks"].append(f"{arm}_ik_lift_{lift}")
            # Plan a small joint displacement with the base/lift fixed.
            request = GetMotionPlan.Request()
            request.motion_plan_request.group_name = "left_arm"
            request.motion_plan_request.start_state = state
            request.motion_plan_request.allowed_planning_time = 2.0
            request.motion_plan_request.num_planning_attempts = 1
            request.motion_plan_request.planner_id = "RRTConnect"
            request.motion_plan_request.workspace_parameters.header.frame_id = (
                "base_footprint"
            )
            request.motion_plan_request.workspace_parameters.min_corner.x = -2.0
            request.motion_plan_request.workspace_parameters.min_corner.y = -2.0
            request.motion_plan_request.workspace_parameters.min_corner.z = -1.0
            request.motion_plan_request.workspace_parameters.max_corner.x = 2.0
            request.motion_plan_request.workspace_parameters.max_corner.y = 2.0
            request.motion_plan_request.workspace_parameters.max_corner.z = 2.0
            request.motion_plan_request.max_velocity_scaling_factor = 0.2
            request.motion_plan_request.max_acceleration_scaling_factor = 0.2
            goal = Constraints()
            for name in names:
                if (
                    name.startswith("left_")
                    and name in parameters["robot_description_planning"]["joint_limits"]
                    and name != "left_gripper_joint"
                ):
                    j = JointConstraint()
                    j.joint_name = name
                    j.position = 0.1 if name == "left_base_joint" else 0.0
                    j.tolerance_above = j.tolerance_below = 0.001
                    j.weight = 1.0
                    goal.joint_constraints.append(j)
            request.motion_plan_request.goal_constraints = [goal]
            plan = call("plan_kinematic_path", request).motion_plan_response
            assert plan.error_code.val == 1 and plan.trajectory.joint_trajectory.points
            assert set(plan.trajectory.joint_trajectory.joint_names) == {
                j.joint_name for j in goal.joint_constraints
            }
            report["checks"].append("arm_only_trajectory")
            assert "default_planning_response_adapters/AddRuckigTrajectorySmoothing" in report['response_adapters']
            planned = plan.trajectory.joint_trajectory.points
            assert all(len(p.velocities)==5 and len(p.accelerations)==5 for p in planned)
            report['planned_peak_velocity_rad_s'] = max(abs(v) for p in planned for v in p.velocities)
            report['planned_peak_acceleration_rad_s2'] = max(abs(v) for p in planned for v in p.accelerations)
            assert report['planned_peak_velocity_rad_s'] <= .5 + 1e-6
            assert report['planned_peak_acceleration_rad_s2'] <= 1.0 + 1e-6
            report['checks'].append('ruckig_response_with_explicit_jerk_bound')
            from home_robot_tasks.moveit_bridge import MoveItBridge
            from home_robot_tasks.manipulation_port import PlanningContext
            canonical = json.loads((ROOT / "config/bimanual_operational_limits.json").read_text())['joint_order']
            bounds = parameters['robot_description_planning']['joint_limits']
            bridge = MoveItBridge(canonical, [bounds[n]['min_position'] for n in canonical],
                [bounds[n]['max_position'] for n in canonical],
                lambda: state.joint_state.position[names.index('lift_joint')])
            context = PlanningContext(1., 'synthetic-capture', 'pose-1', 'scene-1',
                'simulation-only', 'base_footprint', tuple(state.joint_state.position[names.index(n)] for n in canonical), True, True)
            mapped = bridge.map(plan.trajectory, context)
            assert len(mapped) >= 2 and all(p.positions_rad[6:] == context.anchor_rad[6:] for p in mapped)
            report['checks'].append('resident_12_axis_trajectory_mapping')
            # Known reachable Cartesian target generated by FK of the planned
            # goal, then planned again through the runtime request factory.
            fk_goal = GetPositionFK.Request();fk_goal.header.frame_id = context.frame_id
            fk_goal.fk_link_names = ['left_gripper_frame_link'];fk_goal.robot_state = deepcopy(state)
            fk_goal.robot_state.joint_state.position[names.index('left_base_joint')] = 0.1
            pose_goal = call('compute_fk', fk_goal).pose_stamped[0].pose
            xyz = pose_goal.position;q = pose_goal.orientation
            cartesian = GetMotionPlan.Request()
            cartesian.motion_plan_request = bridge.request({'context':context,'task':{
                'arm':'left','target':{'frame_id':context.frame_id},'position_m':[xyz.x,xyz.y,xyz.z],
                'quaternion_xyzw':[q.x,q.y,q.z,q.w]}})
            cartesian_plan = call('plan_kinematic_path', cartesian).motion_plan_response
            assert cartesian_plan.error_code.val == 1
            assert len(bridge.map(cartesian_plan.trajectory,context)) >= 2
            report['checks'].append('runtime_cartesian_pose_request')

            # Actual Apply/Get scene transactions, including a payload that
            # must remain present in the runtime request's collision checking.
            from home_robot_tasks.planning_scene_port import MoveItSceneClient, SceneUpdatePort
            scene_client = MoveItSceneClient(node, apply_service='/offline_moveit/apply_planning_scene',
                get_service='/offline_moveit/get_planning_scene')
            deadline = time.monotonic()+5
            while not scene_client.ready():
                assert time.monotonic()<deadline, 'scene services unavailable'
                rclpy.spin_once(node,timeout_sec=.02)
            scene_port = SceneUpdatePort(scene_client,time.monotonic,world_frame='base_footprint',
                gripper_links={a:[a+'_gripper_frame_link'] for a in ('left','right')})
            change = dict(operation='attach',object_id='synthetic_payload',arm='left',
                link_name='left_gripper_frame_link',frame_id='left_gripper_frame_link',
                position_m=[0.,0.,0.],quaternion_xyzw=[0.,0.,0.,1.],size_m=[.5,.5,.5],
                touch_links=['left_gripper_frame_link'],observed_s=time.monotonic())
            # Reproduce the perceived-world-object -> attached-object transition.
            seed = ApplyPlanningScene.Request(); seed.scene.is_diff=True
            seed.scene.world.collision_objects=[scene_client._box(change)]
            assert call('apply_planning_scene',seed).success
            def update_scene(goal, p):
                scene_port.start(goal,p)
                deadline=time.monotonic()+5
                while True:
                    rclpy.spin_once(node,timeout_sec=.01)
                    status=scene_port.poll(goal)
                    if status.state!='RUNNING':break
                    assert time.monotonic()<deadline,(status,scene_port.fault)
                assert status.state=='SUCCEEDED',status
            update_scene('attach-payload',change)
            assert cartesian.motion_plan_request.start_state.is_diff
            loaded=call('plan_kinematic_path',cartesian).motion_plan_response
            assert loaded.error_code.val!=1, 'runtime planning discarded the colliding attached object'
            report['checks'].append('attached_payload_inherited_by_runtime_plan')
            update_scene('release-payload',{**change,'operation':'detach','frame_id':'base_footprint',
                'position_m':[2.,2.,2.],'observed_s':time.monotonic()})
            unloaded=call('plan_kinematic_path',cartesian).motion_plan_response
            assert unloaded.error_code.val==1
            assert scene_port.revision==2
            report['checks'].append('scene_attach_detach_geometry_readback')

            # A world obstacle enclosing the arm must invalidate the same state.
            scene = ApplyPlanningScene.Request()
            scene.scene.is_diff = True
            obstacle = CollisionObject()
            obstacle.id = "synthetic_blocker"
            obstacle.header.frame_id = "base_link"
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [2.0, 2.0, 2.0]
            placement = Pose()
            placement.position.z = 0.5
            placement.orientation.w = 1.0
            obstacle.primitives = [box]
            obstacle.primitive_poses = [placement]
            obstacle.operation = CollisionObject.ADD
            scene.scene.world.collision_objects = [obstacle]
            assert call("apply_planning_scene", scene).success
            assert not valid(state).valid
            blocked = call("plan_kinematic_path", request).motion_plan_response
            assert blocked.error_code.val != 1
            report["checks"].append("world_collision_rejects_plan")
            report["trajectory_points"] = len(plan.trajectory.joint_trajectory.points)
            report["passed"] = True
    finally:
        if rsp is not None:
            rsp.terminate()
            try:
                rsp.wait(timeout=5)
            except subprocess.TimeoutExpired:
                rsp.kill()
                rsp.wait(timeout=5)
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        report["move_group_exit_code"] = (
            process.returncode if process is not None else None
        )
        report["publisher_exit_code"] = rsp.returncode if rsp is not None else None
        report["passed"] = (
            report["passed"]
            and report["move_group_exit_code"] == 0
            and report["publisher_exit_code"] == 0
        )
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        (args.output / "report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report))
    if not report["passed"]:
        raise RuntimeError("MoveIt validation or clean shutdown failed")


if __name__ == "__main__":
    main()
