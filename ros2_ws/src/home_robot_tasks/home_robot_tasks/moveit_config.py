"""Build simulation-only MoveIt parameters for a stationary mobile assembly.

No controller plugin or trajectory execution is enabled. Canonical arm bounds
are intersected with URDF bounds; lift/base remain outside arm planning groups.
"""

import json
from pathlib import Path
import xml.etree.ElementTree as ET

from .fetch import InvalidTask


def simulation_parameters(description_dir: Path, limits_path: Path):
    import xacro

    xml = xacro.process_file(
        str(description_dir / "urdf/alohamini.simulation.urdf.xacro")
    ).toxml()
    robot = ET.fromstring(xml)
    # Source-tree checks need not install an ament overlay to resolve meshes.
    for mesh in robot.iter("mesh"):
        prefix = "package://so101_description/"
        name = mesh.attrib["filename"]
        if not name.startswith(prefix):
            raise InvalidTask("unexpected mesh source")
        local = (description_dir / name[len(prefix) :]).resolve()
        if not local.is_relative_to(description_dir.resolve()) or not local.is_file():
            raise InvalidTask("missing or external robot mesh")
        mesh.set("filename", "file://" + str(local))
    limits = json.loads(limits_path.read_text())
    planning_limits = {}
    for joint in robot.findall("joint"):
        name = joint.attrib["name"]
        if name not in limits["joint_order"]:
            continue
        arm, suffix = name.split("_", 1)
        values = limits["arms"][arm][suffix.removesuffix("_joint")]
        bound = joint.find("limit")
        low = max(float(bound.attrib["lower"]), values["minimum_urad"] / 1e6)
        high = min(float(bound.attrib["upper"]), values["maximum_urad"] / 1e6)
        if low >= high:
            raise InvalidTask("URDF and canonical joint ranges do not overlap")
        # Apply the intersection to the model as well as planner overrides.
        bound.set("lower", str(low))
        bound.set("upper", str(high))
        planning_limits[name] = dict(
            has_position_limits=True,
            min_position=low,
            max_position=high,
            has_velocity_limits=True,
            max_velocity=min(0.5, float(bound.attrib["velocity"])),
            has_acceleration_limits=True,
            max_acceleration=1.0,
            # Explicit simulation candidate; no implicit Ruckig fallback.
            has_jerk_limits=True,
            max_jerk=8.0,
        )
    if set(planning_limits) != set(limits["joint_order"]):
        raise InvalidTask("incomplete arm planning limits")
    return {
        "robot_description": ET.tostring(robot, encoding="unicode"),
        "robot_description_semantic": (
            description_dir / "config/alohamini.simulation.srdf"
        ).read_text(),
        "robot_description_kinematics": {
            f"{arm}_arm": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.05,
                "position_only_ik": True,
            }
            for arm in ("left", "right")
        },
        "robot_description_planning": {"joint_limits": planning_limits},
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/AddRuckigTrajectorySmoothing",
                "default_planning_response_adapters/ValidateSolution",
            ],
            "planner_configs": {
                "RRTConnect": {"type": "geometric::RRTConnect", "range": 0.0}
            },
            **{
                f"{arm}_arm": {"planner_configs": ["RRTConnect"]}
                for arm in ("left", "right")
            },
        },
        "disable_capabilities": "move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction",
        "allow_trajectory_execution": False,
        "moveit_manage_controllers": False,
        "publish_robot_description": False,
        "publish_robot_description_semantic": False,
        "use_sim_time": False,
    }
