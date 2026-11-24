import pytest
from home_robot_tasks.pick_hardware import check_anchor_inventory


def fixture(raw):
    return ({'unpowered_observations':[{'devices':[{'bus':'right','requested_id':2,
        'feedback':{'position_raw':raw}}]}]}, {'arms':{'right':{'shoulder':{
        'minimum_unwrapped_raw':854,'maximum_unwrapped_raw':3208}}}})


def test_measured_right_shoulder_failure_is_specific_and_not_clamped():
    setup,ranges=fixture(848)
    with pytest.raises(ValueError,match='오른팔 어깨.*848.*854'):
        check_anchor_inventory(setup,ranges)
    assert setup['unpowered_observations'][0]['devices'][0]['feedback']['position_raw']==848
    assert ranges['arms']['right']['shoulder']['minimum_unwrapped_raw']==854


def test_inclusive_range_and_wrapped_profiles():
    for value in (854,2048,3208):check_anchor_inventory(*fixture(value))
    setup,ranges=fixture(10)
    ranges['arms']['right']['shoulder'].update(minimum_unwrapped_raw=4000,maximum_unwrapped_raw=4200)
    check_anchor_inventory(setup,ranges)
