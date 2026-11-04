import json
import math
from pathlib import Path

import numpy as np
import pytest

from tools.lib.grasp_preview import VisualRobot, joint_positions, load_layout, pose_matrix

ROOT = Path(__file__).resolve().parents[1]

XML = '''<robot name="test">
<link name="mount"/><link name="elbow"/><link name="tool"/>
<joint name="tool_fixed" type="fixed"><parent link="elbow"/><child link="tool"/>
  <origin xyz="1 0 0" rpy="0 0 0"/></joint>
<joint name="left_base_joint" type="revolute"><parent link="mount"/><child link="elbow"/>
  <origin xyz="0 0 2" rpy="0 0 1.5707963267948966"/>
  <axis xyz="0 0 1"/><limit lower="-2" upper="2"/></joint>
</robot>'''


def test_optical_camera_axes_map_to_z_up_without_a_reflection():
    layout = load_layout(ROOT / 'config/grasp_preview.virtual.json')
    transform = pose_matrix(layout['camera_translation_m'], layout['camera_rpy_degrees'])
    rotation = transform[:3, :3]
    assert np.allclose(rotation.T @ rotation, np.eye(3))
    assert np.linalg.det(rotation) == pytest.approx(1)
    # Camera down points towards -arm X; camera forward points down.
    assert np.allclose(rotation @ [0, 1, 0], [-1, 0, 0])
    assert np.allclose(rotation @ [0, 0, 1], [0, 0, -1])
    point = transform @ [0, -.1, .45, 1]
    assert point[:3] == pytest.approx([.22, -.1, .02])


def test_fk_composes_origin_then_joint_motion_and_handles_reverse_xml_order(tmp_path):
    model = VisualRobot(XML, tmp_path)
    links = model.link_transforms({'left_base_joint': math.pi / 2})
    assert links['elbow'][:3, 3] == pytest.approx([0, 0, 2])
    assert links['tool'][:3, 3] == pytest.approx([-1, 0, 2])
    assert links['tool'][:3, :3] == pytest.approx(np.diag([-1, -1, 1]))


@pytest.mark.parametrize('positions', [{}, {'left_base_joint': 3.}, {'left_base_joint': float('nan')},
                                      {'left_base_joint': True}, {'left_base_joint': 0, 'unknown': 0}])
def test_missing_invalid_or_out_of_bounds_joints_do_not_silently_become_zero(tmp_path, positions):
    with pytest.raises(ValueError):
        VisualRobot(XML, tmp_path).link_transforms(positions)


def test_arm_bounds_intersect_the_registered_operational_limits(tmp_path):
    limits = {'arms': {'left': {'base': {'minimum_urad': -500000, 'maximum_urad': 400000}}}}
    model = VisualRobot(XML, tmp_path, limits)
    assert model.bounds['left_base_joint'] == (-.5, .4)
    with pytest.raises(ValueError, match='outside preview limits'):
        model.link_transforms({'left_base_joint': .6})


def test_measured_layout_cannot_be_passed_as_the_virtual_demo(tmp_path):
    data = json.loads((ROOT / 'config/grasp_preview.virtual.json').read_text())
    data['mode'] = 'measured'
    path = tmp_path / 'layout.json'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='virtual layout'):
        load_layout(path)


def test_saved_joint_angles_are_converted_to_the_correct_arm_prefix():
    layout = load_layout(ROOT / 'config/grasp_preview.virtual.json')
    layout['arm'] = 'right'
    layout['joint_positions_degrees']['shoulder'] = 90
    result = joint_positions(layout)
    assert len(result) == 6 and all(name.startswith('right_') for name in result)
    assert result['right_shoulder_joint'] == pytest.approx(math.pi / 2)
