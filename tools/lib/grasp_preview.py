"""URDF visual geometry and FK for a read-only, virtual grasp preview.

The camera transform is explicitly illustrative. No IK, collision tests,
controller, ROS node, serial connection, or motor commands are provided here.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from home_robot_tasks.grasp_yaw import _axis_matrix, _rpy_matrix

JOINTS = ('base', 'shoulder', 'elbow', 'wrist_flex', 'wrist_roll', 'gripper')


def vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f'{name} must contain {size} numbers')
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError(f'{name} must contain finite numbers')
    return np.asarray(value, dtype=float)


def pose_matrix(xyz, rpy_degrees):
    transform = np.eye(4)
    transform[:3, :3] = _rpy_matrix(*np.radians(vector(rpy_degrees, 3, 'rotation')))
    transform[:3, 3] = vector(xyz, 3, 'translation')
    return transform


def load_layout(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if type(data.get('schema_version')) is not int or data['schema_version'] != 1 or data.get('mode') != 'virtual':
        raise ValueError('explicit virtual layout schema_version 1 required')
    if data.get('arm') not in ('left', 'right'):
        raise ValueError('arm must be left or right')
    vector(data['camera_translation_m'], 3, 'camera translation')
    vector(data['camera_rpy_degrees'], 3, 'camera rotation')
    joints = data['joint_positions_degrees']
    if not isinstance(joints, dict) or set(joints) != set(JOINTS):
        raise ValueError('all five arm joints and gripper must be explicit')
    vector([joints[name] for name in JOINTS], 6, 'joint positions')
    return data


def _origin(element):
    origin = element.find('origin')
    if origin is None:
        return np.eye(4)
    xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
    return pose_matrix(xyz, np.degrees(rpy).tolist())


class VisualRobot:
    def __init__(self, xml, description_dir, operational_limits=None):
        self.xml = xml
        self.description_dir = Path(description_dir).resolve()
        self.robot = ET.fromstring(xml)
        self.links = {link.get('name'): link for link in self.robot.findall('link')}
        self.joints = {joint.get('name'): joint for joint in self.robot.findall('joint')}
        self.by_child = {joint.find('child').get('link'): joint for joint in self.joints.values()}
        roots = set(self.links) - set(self.by_child)
        if len(roots) != 1 or len(self.by_child) != len(self.joints):
            raise ValueError('URDF must be a single tree')
        self.root = roots.pop()
        self.bounds = {}
        for name, joint in self.joints.items():
            if joint.get('type') == 'fixed':
                continue
            if joint.get('type') not in ('revolute', 'prismatic'):
                raise ValueError('unsupported preview joint type')
            limit = joint.find('limit')
            low, high = float(limit.get('lower')), float(limit.get('upper'))
            # Servo gripper coordinates are not a calibrated jaw angle. Keep
            # its URDF visual limits, while intersecting the five arm joints.
            if operational_limits is not None and not name.endswith('_gripper_joint'):
                arm, suffix = name.split('_', 1)
                values = operational_limits['arms'][arm][suffix.removesuffix('_joint')]
                low = max(low, values['minimum_urad'] / 1e6)
                high = min(high, values['maximum_urad'] / 1e6)
            if not math.isfinite(low + high) or low >= high:
                raise ValueError('empty joint limit intersection')
            self.bounds[name] = (low, high)

    def link_transforms(self, positions):
        if set(positions) != set(self.bounds):
            raise ValueError('every movable joint position is required')
        for name, value in positions.items():
            low, high = self.bounds[name]
            if type(value) not in (int, float) or not math.isfinite(value) or not low - 1e-8 <= value <= high + 1e-8:
                raise ValueError('joint outside preview limits: ' + name)
        transforms, visiting = {self.root: np.eye(4)}, set()

        def resolve(link):
            if link in transforms:
                return transforms[link]
            if link in visiting or link not in self.by_child:
                raise ValueError('disconnected/cyclic URDF')
            visiting.add(link)
            joint = self.by_child[link]
            parent = resolve(joint.find('parent').get('link'))
            motion = np.eye(4)
            if joint.get('type') != 'fixed':
                axis_element = joint.find('axis')
                axis = np.asarray([float(v) for v in axis_element.get('xyz', '1 0 0').split()]) if axis_element is not None else np.array([1., 0., 0.])
                if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-10:
                    raise ValueError('invalid joint axis')
                if joint.get('type') == 'revolute':
                    motion[:3, :3] = _axis_matrix(axis, positions[joint.get('name')])
                else:
                    motion[:3, 3] = axis / np.linalg.norm(axis) * positions[joint.get('name')]
            transforms[link] = parent @ _origin(joint) @ motion
            visiting.remove(link)
            return transforms[link]

        for link in self.links:
            resolve(link)
        return transforms

    def visual_meshes(self):
        import open3d as o3d
        materials = {m.get('name'): [float(v) for v in m.find('color').get('rgba').split()][:3]
                     for m in self.robot.findall('material') if m.find('color') is not None}
        result = {}
        for link, element in self.links.items():
            merged = o3d.geometry.TriangleMesh()
            for visual in element.findall('visual'):
                geometry = visual.find('geometry')
                mesh_element, box = geometry.find('mesh'), geometry.find('box')
                if mesh_element is not None:
                    prefix = 'package://so101_description/'
                    filename = mesh_element.get('filename')
                    if not filename.startswith(prefix):
                        raise ValueError('unexpected mesh package')
                    path = (self.description_dir / filename[len(prefix):]).resolve()
                    if not path.is_relative_to(self.description_dir) or not path.is_file():
                        raise ValueError('missing or external mesh')
                    mesh = o3d.io.read_triangle_mesh(str(path))
                    if mesh.is_empty():
                        raise ValueError('empty mesh: ' + str(path))
                    scale = vector([float(v) for v in mesh_element.get('scale', '1 1 1').split()], 3, 'mesh scale')
                    mesh.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices) * scale)
                elif box is not None:
                    size = [float(v) for v in box.get('size').split()]
                    mesh = o3d.geometry.TriangleMesh.create_box(*size)
                    mesh.translate(-np.asarray(size) / 2)
                else:
                    raise ValueError('unsupported visual geometry')
                material = visual.find('material')
                color = materials.get(material.get('name'), [.85, .68, .12]) if material is not None else [.85, .68, .12]
                if material is not None and material.find('color') is not None:
                    color = [float(v) for v in material.find('color').get('rgba').split()][:3]
                mesh.paint_uniform_color(color)
                mesh.transform(_origin(visual))
                merged += mesh
            if not merged.is_empty():
                merged.compute_vertex_normals()
                result[link] = merged
        return result


def load_robot(description_dir, arm, limits_path):
    import xacro
    description_dir = Path(description_dir)
    xml = xacro.process_file(str(description_dir / 'urdf/so101_arm.urdf.xacro'),
                             mappings={'arm_slot': arm, 'use_wrist_camera_mount': 'false'}).toxml()
    limits = json.loads(Path(limits_path).read_text())
    return VisualRobot(xml, description_dir, limits)


def joint_positions(layout):
    return {f'{layout["arm"]}_{name}_joint': math.radians(layout['joint_positions_degrees'][name]) for name in JOINTS}


def candidate_meshes(report, colors=None):
    """Generic parallel-jaw glyphs, not SO-ARM101 TCP/jaw geometry."""
    import open3d as o3d
    mesh = o3d.geometry.TriangleMesh()
    for index, grasp in enumerate(report['grasps']):
        width, depth = float(grasp['width_m']), float(grasp['depth_m'])
        rotation = np.asarray(grasp['rotation_matrix'], dtype=float)
        center = vector(grasp['translation_m'], 3, 'grasp translation')
        if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5)
                or not 0 < width <= .5 or not 0 < depth <= .5):
            raise ValueError('invalid saved grasp geometry')
        t = np.eye(4)
        t[:3, :3], t[:3, 3] = rotation, center
        pieces = [((depth + .024, .004, .004), (-.024, -width / 2 - .004, -.002)),
                  ((depth + .024, .004, .004), (-.024, width / 2, -.002)),
                  ((.004, width, .004), (-.024, -width / 2, -.002)),
                  ((.04, .004, .004), (-.064, -.002, -.002))]
        for size, translation in pieces:
            part = o3d.geometry.TriangleMesh.create_box(*size)
            part.translate(translation).transform(t)
            part.paint_uniform_color(colors[index] if colors is not None else [1., .32, .04])
            mesh += part
    if not mesh.is_empty():
        mesh.compute_vertex_normals()
    return mesh


def transformed(geometry, transform):
    result = deepcopy(geometry)
    result.transform(transform)
    return result
