"""Read-only conversion between controller radians (zero at raw 2048) and official SO-ARM101 angles."""
import math
import xml.etree.ElementTree as ET

import numpy as np

from . import s2_geometry as geo

CONVENTION = 'so101_new_calib_from_controller_v1'
CONTROL_MODEL = 'ros2_ws/src/so101_description/urdf/so101_arm_macro.xacro'


def _axis_in_parent(joint):
    axis = np.fromstring(joint['axis']['xyz'], sep=' ')
    if axis.shape != (3,) or not np.isfinite(axis).all() or not np.isclose(np.linalg.norm(axis), 1.):
        raise ValueError('unit joint axis required')
    return geo.origin(joint)[:3, :3] @ axis


def arm_axis_signs(control_xml, geometry):
    """Compare each arm joint in its parent link frame; the moving jaw uses opening evidence."""
    macro = ET.fromstring(control_xml)
    official = {j['name']:j for j in geometry['arm_joints']}
    signs = {}
    for name, upstream in zip(geo.JOINTS[:-1], geo.UPSTREAM[:-1], strict=True):
        matches = macro.findall('.//joint[@name="${prefix}' + name + '_joint"]')
        if len(matches) != 1:
            raise ValueError(f'{name}: expected one control-model joint')
        control = {k: dict(matches[0].find(k).attrib) for k in ('origin', 'axis', 'parent')}
        source = official[upstream]
        if control['parent']['link'] != '${prefix}' + source['parent']['link']:
            raise ValueError(f'{name}: different parent link frames')
        dot = _axis_in_parent(control) @ _axis_in_parent(source)
        if not np.isclose(abs(dot), 1., atol=1e-7, rtol=0.):
            raise ValueError(f'{name}: incompatible control and model axes')
        signs[name] = 1 if dot > 0 else -1
    return signs


def model_alignment(reference, ranges, arm_signs):
    """Anchor the new mapping at the recorded raw pose, not at a nominal encoder zero."""
    mapping = {}
    for name in geo.JOINTS:
        control_sign = ranges[name]['positive_raw_direction']
        if control_sign not in (-1, 1) or control_sign != reference['alignment'][name]['sign']:
            raise ValueError(f'{name}: historical controller direction mismatch')
        if name == 'gripper':
            raw_sign = ranges[name]['opening_raw_direction']
            if raw_sign not in (-1, 1) or ranges[name]['closing_raw_direction'] != -raw_sign:
                raise ValueError('opposing jaw opening and closing directions required')
            control_to_model = control_sign * raw_sign
        else:
            control_to_model = arm_signs[name]
            raw_sign = control_sign * control_to_model
        reference_delta = (reference['reference_raw'][name] - 2048) * math.tau / 4096
        mapping[name] = dict(sign=raw_sign, raw_to_control_sign=control_sign, control_to_model_sign=control_to_model,
                             home_model_rad=reference['reference_model_rad'][name] - raw_sign*reference_delta)
    return mapping
