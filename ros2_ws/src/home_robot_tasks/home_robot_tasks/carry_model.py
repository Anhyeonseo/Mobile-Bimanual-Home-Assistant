"""Current S2 nominal geometry for offline MoveIt; no execution configuration.

Keep the observation transforms and original SO101 collision meshes. Mobile
meshes are required explicitly, never replaced with the old example chassis.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from . import s2_geometry as geo, s2_development as dev
from .s2_urdf import render_observation_urdf, _rpy
from .d415_geometry import nominal_model


def parameters(root, profile, geometry, arms, mobile_mesh_dir):
    root, mobile_mesh_dir = Path(root), Path(mobile_mesh_dir)
    dev.validate(profile, geometry)
    robot = ET.fromstring(render_observation_urdf(
        profile, geometry, lift_axis=' '.join(map(str, profile['lift']['axis_base'])),
        lift_travel=profile['lift']['travel_m'], ground_z=profile['ground_z_m']))
    official_path = root/'config/home_assistant/official_sources/so101_new_calib.urdf'
    expected = next(s['sha256'] for s in geometry['sources'] if s['file'] == official_path.name)
    if hashlib.sha256(official_path.read_bytes()).hexdigest() != expected:
        raise ValueError('official arm geometry changed')
    source = ET.parse(official_path).getroot()
    links = {link.get('name'): link for link in robot.findall('link')}
    for material in source.findall('material'):
        robot.append(deepcopy(material))
    for side in ('left', 'right'):
        for link in source.findall('link'):
            target = links[side+'_s2_'+link.get('name')]
            for child in link:
                target.append(deepcopy(child))
        for link in robot.findall('link'):
            for mesh in link.iter('mesh'):
                name = mesh.get('filename')
                if name.startswith('file://'):
                    continue
                path = root/'ros2_ws/src/so101_description/meshes'/Path(name).name
                if not path.is_file():
                    raise ValueError('missing official arm mesh: '+str(path))
                mesh.set('filename', path.resolve().as_uri())

    def body(name, parent, matrix, filename):
        path = (mobile_mesh_dir/filename).resolve()
        # Hashes of the original meshes were recorded when the viewer was built.
        record = json.loads((root/'tools/diagnostics/s2_viewer/meshes.json').read_text())['meshes'][filename]
        if hashlib.sha256(path.read_bytes()).hexdigest() != record['source_sha256']:
            raise ValueError('official mobile mesh changed: '+filename)
        link = ET.SubElement(robot, 'link', name=name)
        for tag in ('visual', 'collision'):
            shape = ET.SubElement(link, tag)
            ET.SubElement(ET.SubElement(shape, 'geometry'), 'mesh', filename=path.as_uri())
        joint = ET.SubElement(robot, 'joint', name=name+'_mount', type='fixed')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=name)
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, matrix[:3, 3])), rpy=' '.join(map(str, _rpy(matrix))))

    base = geo.rigid(profile['base_from_cad'])
    body('chassis', 'base_link', base, 'base_link.STL')
    body('platform', 's2_lift_link', geo.rigid(profile['lift']['platform_at_lift_zero']), 'vertical_link.STL')
    for j in geometry['mobile_reference_joints']:
        if j['name'] in ('wheel1', 'wheel2', 'wheel3'):
            body(j['name'], 'base_link', base@geo.origin(j), 'wheel1.STL')

    camera = nominal_model(root/'config/home_assistant/d415_official_geometry.json')
    center = np.eye(4); center[2, 3] = -camera['depth_m']/2
    camera_shape = geo.rigid(camera['color_optical_from_mesh'])@center
    for tag in ('visual', 'collision'):
        shape = ET.SubElement(links[profile['source_frame']], tag)
        ET.SubElement(shape, 'origin', xyz=' '.join(map(str, camera_shape[:3,3])),
                      rpy=' '.join(map(str, _rpy(camera_shape))))
        ET.SubElement(ET.SubElement(shape, 'geometry'), 'box',
                      size=' '.join(str(camera[k]) for k in ('width_m','height_m','depth_m')))

    semantics = ET.Element('robot', name=robot.get('name'))
    limits = {}
    for side, arm in arms.items():
        group = ET.SubElement(semantics, 'group', name=side+'_arm')
        ET.SubElement(group, 'chain', base_link=side+'_s2_base_link', tip_link=side+'_tcp')
        ET.SubElement(semantics, 'passive_joint', name=side+'_model_gripper')
        for i, name in enumerate(geo.JOINTS[:5]):
            joint_name = side+'_model_'+name
            bound = robot.find(f"joint[@name='{joint_name}']/limit")
            bound.set('lower', str(arm.lo[i])); bound.set('upper', str(arm.hi[i]))
            bound.set('velocity', '0.5')
            limits[joint_name] = dict(has_velocity_limits=True, max_velocity=.5,
                                     has_acceleration_limits=True, max_acceleration=1.,
                                     has_jerk_limits=True, max_jerk=8.)
    ET.SubElement(semantics, 'passive_joint', name='s2_lift_joint')
    # Only physical neighbours. No inherited "Never" exclusions for forearm/jaw.
    excluded = set()
    def exclude(a, b, reason):
        key = tuple(sorted((a, b)))
        if key not in excluded:
            ET.SubElement(semantics, 'disable_collisions', link1=a, link2=b, reason=reason)
            excluded.add(key)
    for joint in robot.findall('joint'):
        exclude(joint.find('parent').get('link'), joint.find('child').get('link'), 'Adjacent')
    exclude('chassis', 'platform', 'Lift rack and slider mechanical mating surfaces')
    exclude('chassis', profile['source_frame'], 'Fixed front antenna camera mounting interface')
    for side in ('left', 'right'):
        exclude('platform', side+'_s2_base_link', 'Arm mounting interface')
    for wheel in ('wheel1', 'wheel2', 'wheel3'):
        exclude('chassis', wheel, 'Wheel mounting interface')
    return {
        'robot_description': ET.tostring(robot, encoding='unicode'),
        'robot_description_semantic': ET.tostring(semantics, encoding='unicode'),
        'robot_description_kinematics': {side+'_arm': {
            'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_timeout': .1, 'position_only_ik': False,
        } for side in arms},
        'robot_description_planning': {'joint_limits': limits},
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': ['default_planning_request_adapters/'+s for s in (
                'ResolveConstraintFrames', 'ValidateWorkspaceBounds', 'CheckStartStateBounds', 'CheckStartStateCollision')],
            'response_adapters': ['default_planning_response_adapters/'+s for s in (
                'AddTimeOptimalParameterization', 'AddRuckigTrajectorySmoothing', 'ValidateSolution')],
            'planner_configs': {'RRTConnect': {'type': 'geometric::RRTConnect', 'range': 0.},
                                'RRTGoal': {'type': 'geometric::RRT', 'range': .1, 'goal_bias': 1.}},
            **{side+'_arm': {'planner_configs': ['RRTConnect', 'RRTGoal'], 'longest_valid_segment_fraction': .005,
                            'enforce_joint_model_state_space': True} for side in arms},
        },
        'disable_capabilities': 'move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction',
        'allow_trajectory_execution': False, 'moveit_manage_controllers': False,
        'publish_robot_description': False, 'publish_robot_description_semantic': False,
        'use_sim_time': False,
    }
