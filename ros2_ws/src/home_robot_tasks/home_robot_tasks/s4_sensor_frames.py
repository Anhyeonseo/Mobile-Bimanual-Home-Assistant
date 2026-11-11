"""Nominal stationary sensor TFs, sharing the released S3 camera/base convention.

This observation tree has no odom/map transform, joint state or motion permission.
The mount is supported by real scan comparison, not precision calibration.
"""
import json
import math
from pathlib import Path

import numpy as np

from . import s2_development, s2_geometry
from .s2_urdf import _rpy


def sensor_frames(root, mount, *, depth_to_color=None, navigation_root=False):
    root = Path(root)
    geometry = s2_geometry.load_geometry(root/'config/home_assistant/official_geometry.json')
    profile = s2_development.released_profile(root, geometry)
    if (mount.get('mode') != 'operator_reviewed_nominal_mount'
            or mount.get('mount_confirmed') is not True
            or mount.get('accuracy_qualified') is not False
            or mount.get('range_calibration_applied') is not False
            or mount.get('user_confirmed', {}).get('approximate_position_and_orientation') is not True
            or mount.get('setup_method', {}).get('physical_comparison_performed') is not True):
        raise ValueError('reviewed nominal mount with real scan comparison required')
    xyz = s2_geometry.vector(mount['base_from_lidar_translation_m'], 3, 'lidar mount')
    yaw = s2_geometry.number(mount['yaw_rad'], 'lidar yaw', math.pi)
    if not math.isclose(xyz[2] - profile['ground_z_m'],
                        mount['scan_height_above_floor_m'], abs_tol=1e-9):
        raise ValueError('lidar/base floor reference mismatch')
    lidar = np.eye(4)
    lidar[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0],
                     [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    lidar[:3, 3] = xyz
    floor = np.eye(4)
    floor[2, 3] = profile['ground_z_m']
    frames = [('base_link', 'base_footprint', floor),
              ('base_link', 'lidar_a', lidar),
              ('base_link', profile['source_frame'], s2_geometry.rigid(profile['base_from_optical']))]
    if navigation_root:
        # RosBaseIO supplies odom -> base_footprint. Do not give that link a
        # second parent when the observation-only tree is used for navigation.
        frames[0] = ('base_footprint', 'base_link', np.linalg.inv(floor))
    if depth_to_color is not None:
        color_from_depth = np.eye(4)
        color_from_depth[:3, :3] = np.asarray(depth_to_color['rotation_column_major']).reshape(3, 3, order='F')
        color_from_depth[:3, 3] = depth_to_color['translation_m']
        frames.append((profile['source_frame'], 'depth_camera_optical_frame',
                       s2_geometry.rigid(color_from_depth)))
    result = []
    for parent, child, matrix in frames:
        roll, pitch, yaw = _rpy(matrix)
        cr, cp, cy = (math.cos(a/2) for a in (roll, pitch, yaw))
        sr, sp, sy = (math.sin(a/2) for a in (roll, pitch, yaw))
        result.append(dict(parent=parent, child=child, matrix=matrix.tolist(),
                           translation_m=matrix[:3, 3].tolist(),
                           quaternion_xyzw=[sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                                            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]))
    return dict(mode='stationary_nominal_observation', motion_authorized=False,
                precision_extrinsics_qualified=False, moving_mapping_qualified=False,
                calibration_id=profile['calibration_id'],
                mount_setup_method=mount['setup_method'], transforms=result)


def load_sensor_frames(root, *, depth_to_color=None, navigation_root=False):
    mount = json.loads((Path(root)/'config/home_assistant/lidar_mount.review.json').read_text())
    return sensor_frames(root, mount, depth_to_color=depth_to_color, navigation_root=navigation_root)
