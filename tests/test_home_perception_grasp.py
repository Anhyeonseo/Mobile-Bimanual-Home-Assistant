import numpy as np
import pytest

from home_perception.geometry.plane import Plane
from home_perception.grasp.refine import (between_fingers, flat_object_grasp, nms, refine,
                                          tilt_deg, tip_heights)

# Table at z=0 with its normal +z, so "down" is -z in these tests.
TABLE = Plane(np.array([0., 0, 1]), np.zeros(3), 0., 1.)
DOWN = np.column_stack(([0, 0, -1.], [0, 1., 0], [1., 0, 0]))


def grasp(z, width=.06, depth=.02, score=.5, rotation=DOWN, x=0.):
    row = np.zeros(17)
    row[:4] = score, width, .02, depth
    row[4:13] = rotation.ravel()
    row[13:16] = x, 0, z
    return row


def bar():
    """Remote-like block: 14 x 4 cm, 1.2 cm tall, top and side surfaces."""
    x, y = np.meshgrid(np.linspace(-.07, .07, 60), np.linspace(-.02, .02, 20))
    return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, .012)))


def test_angles_and_tip_height():
    assert tilt_deg(DOWN[None], TABLE.normal)[0] == pytest.approx(0)
    assert tip_heights(grasp(.03)[None], TABLE)[0] == pytest.approx(.01)


def test_between_fingers_counts_only_held_points():
    counts, offsets = between_fingers(np.array([grasp(.02), grasp(.02, x=.3)]), bar())
    assert counts[0] > 50 and counts[1] == 0
    assert offsets[0] < .005


def test_refine_rejects_empty_and_table_piercing_candidates():
    array = np.array([grasp(.025), grasp(.025, x=.3), grasp(.01)])
    result, rejected = refine(array, bar(), TABLE, .012)
    assert len(result) == 1 and result[0, 15] == pytest.approx(.025)
    assert rejected['no_object_between_fingers'] == 1 and rejected['below_table'] == 1


def test_refine_limits_side_approach_only_for_flat_objects():
    side = np.column_stack(([1., 0, 0], [0, 1., 0], [0, 0, 1.]))
    array = grasp(.03, rotation=side, x=-.01)[None]
    points = np.column_stack((np.zeros(200), np.linspace(-.01, .01, 200), np.full(200, .03)))
    assert refine(array, points, TABLE, .012)[1]['approach_angle'] == 1
    assert refine(array, points, TABLE, .1)[1]['approach_angle'] == 0


def test_nms_keeps_one_of_two_close_candidates():
    assert len(nms(np.array([grasp(.03), grasp(.031), grasp(.03, x=.2)]))) == 2


def test_flat_rule_pinches_short_side_above_table():
    array, reason = flat_object_grasp(bar(), bar(), TABLE)
    assert reason is None
    assert array[0, 1] == pytest.approx(.06, abs=.003)
    assert abs(array[0, 4:13].reshape(3, 3)[:, 1] @ [1, 0, 0]) < .01
    assert tip_heights(array, TABLE)[0] > 0


def test_flat_rule_rejects_tall_wide_and_blocked_objects():
    tall = bar() + [0, 0, .05]
    assert flat_object_grasp(tall, tall, TABLE)[1] == 'not_flat'
    wide = bar() * [3, 3, 1]
    assert flat_object_grasp(wide, wide, TABLE)[1] == 'too_wide'
    neighbour = np.column_stack((np.zeros(50), np.full(50, .03), np.linspace(.01, .04, 50)))
    assert flat_object_grasp(bar(), np.vstack((bar(), neighbour)), TABLE)[1] == 'finger_collision'


def test_flat_rule_ignores_table_noise_beside_the_object():
    noise = np.column_stack((np.zeros(50), np.full(50, .03), np.linspace(.003, .0045, 50)))
    array, reason = flat_object_grasp(bar(), np.vstack((bar(), noise)), TABLE)
    assert reason is None and array is not None


def test_flat_rule_rejects_shapes_without_a_clear_long_axis():
    x, y = np.meshgrid(np.linspace(-.03, .03, 40), np.linspace(-.03, .03, 40))
    square = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, .012)))
    assert flat_object_grasp(square, square, TABLE)[1] == 'not_elongated'


def test_flat_rule_rejects_a_shape_with_an_empty_middle():
    x, y = np.meshgrid(np.r_[np.linspace(-.09, -.06, 20), np.linspace(.06, .09, 20)], np.linspace(-.015, .015, 20))
    ends = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, .012)))
    assert flat_object_grasp(ends, ends, TABLE)[1] == 'no_object_between_fingers'
