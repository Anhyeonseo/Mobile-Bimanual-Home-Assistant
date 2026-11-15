import pytest

from home_perception.contracts import AMBIGUOUS_TARGET, NOT_FOUND
from home_perception.language.target_selection import select_target

LEFT = {'box_xyxy': [0, 200, 100, 300]}
RIGHT = {'box_xyxy': [400, 100, 500, 200]}


def test_no_candidate_is_not_found():
    assert select_target([]) == (None, NOT_FOUND)


def test_single_candidate_needs_no_location():
    assert select_target([LEFT]) == (LEFT, None)


def test_several_candidates_without_location_are_not_guessed():
    assert select_target([LEFT, RIGHT]) == (None, AMBIGUOUS_TARGET)


@pytest.mark.parametrize('location,expected', [('left', LEFT), ('right', RIGHT), ('front', LEFT), ('back', RIGHT)])
def test_location_uses_geometry(location, expected):
    assert select_target([RIGHT, LEFT], location) == (expected, None)


def test_depth_decides_front_when_every_candidate_has_it():
    near = {'box_xyxy': [0, 0, 10, 10], 'center_xyz_m': [0, 0, .3]}
    far = {'box_xyxy': [0, 0, 10, 300], 'center_xyz_m': [0, 0, .5]}
    assert select_target([far, near], 'front') == (near, None)


def test_unknown_location_is_rejected():
    with pytest.raises(ValueError):
        select_target([LEFT, RIGHT], 'above')
