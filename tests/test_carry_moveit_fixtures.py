"""Keep real-MoveIt synthetic payloads consistent with asymmetric jaw contact."""
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.carry_planning import fixed_contact_review
from home_robot_tasks.so101_jaw import load_stock_jaw
from home_robot_tasks.stationary_pick_sim import StationaryArm
from tools.diagnostics.check_carry_moveit import synthetic_grasp

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def arm():
    config = ROOT / 'config/home_assistant'
    return StationaryArm(
        json.loads((config / 's2_coordinates.development.json').read_text()),
        geo.load_geometry(config / 'official_geometry.json'),
        json.loads((config / 'arm_ranges.development.json').read_text()), 0.,
        load_stock_jaw(config / 'official_sources/so101_new_calib.urdf',
                       ROOT / 'ros2_ws/src/so101_description/meshes'),
    )


@pytest.mark.parametrize('roll,upright', [(0., False), (30., False), (30., True)])
def test_fixture_binds_both_fingers_at_each_orientation(arm, roll, upright):
    candidate = synthetic_grasp(arm, roll, upright=upright)
    review = fixed_contact_review(arm, candidate)
    assert review['accepted']
    assert review['fixed_intrusion_m'] == pytest.approx(0, abs=1e-9)
    assert review['moving_contact_error_m'] == pytest.approx(0, abs=1e-9)
    assert review['open_moving_clearance_m'] == pytest.approx(.004)
    if upright:
        pose = candidate.base_from_grasp_tcp @ candidate.tcp_from_object
        np.testing.assert_allclose(pose[:3, :3], np.eye(3), atol=1e-9)
        assert candidate.object_constraint.satisfied(candidate.base_from_grasp_tcp)


def test_old_floating_payload_is_rejected_without_relaxing_contact_checks(arm):
    old_attachment = np.eye(4)
    old_attachment[:3, 3] = [-.018, 0., 0.]
    candidate = replace(synthetic_grasp(arm, 0.), tcp_from_object=old_attachment,
                        open_jaw_model_rad=.5, held_jaw_model_rad=.3)
    review = fixed_contact_review(arm, candidate)
    assert not review['accepted']
    assert abs(review['fixed_intrusion_m']) > .01
    assert abs(review['moving_contact_error_m']) > .01
