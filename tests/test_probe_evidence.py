from dataclasses import replace
import pytest
from home_robot_tasks.probe_evidence import LiftSample, lift_relation


# The patch need not coincide with the TCP. Only synchronized displacement is compared.
START = LiftSample('base-1', 'remote', 'patch-a', 2., 2., (0., 0., .5), (.03, 0., .49), .0001)
END = replace(START, tool_time_s=2.2, vision_time_s=2.19,
              tool_xyz_m=(0., 0., .504), target_xyz_m=(.03, 0., .494))


def test_matching_visible_patch_past_the_noise_floor_can_confirm_test_lift():
    assert lift_relation(START, END, 2.2) == ('held', 'TARGET_FOLLOWED')
    assert lift_relation(START, replace(END, tool_xyz_m=(0., 0., .501),
                                        target_xyz_m=(.03, 0., .491)), 2.2)[0] == 'contact'


@pytest.mark.parametrize('change,now,reason', [
    ({'session': 'other'}, 2.2, 'TARGET_OR_REFERENCE_CHANGED'),
    ({'patch': 'table'}, 2.2, 'TARGET_OR_REFERENCE_CHANGED'),
    ({'target': 'cup'}, 2.2, 'TARGET_OR_REFERENCE_CHANGED'),
    ({'target_xyz_m': START.target_xyz_m}, 2.2, 'TARGET_DID_NOT_FOLLOW'),
    ({'target_xyz_m': (.035, 0., .494)}, 2.2, 'TARGET_DID_NOT_FOLLOW'),
    ({'position_error_bound_m': .0015}, 2.2, 'DISPLACEMENT_UNRESOLVED'),
    ({'target_xyz_m': (.03, 0., float('nan'))}, 2.2, 'INVALID_MEASUREMENT'),
    ({'target_xyz_m': None}, 2.2, 'INVALID_MEASUREMENT'),
    ({'position_error_bound_m': True}, 2.2, 'INVALID_MEASUREMENT'),
    ({'tool_time_s': 2.}, 2.2, 'UNSYNCHRONIZED_PAIR'),
    ({}, 2.5, 'STALE_PROBE'),
    ({'tool_time_s': 2., 'vision_time_s': 2.}, 2., 'STALE_PROBE'),
    ({'tool_xyz_m': (0., 0., .509)}, 2.2, 'OUTSIDE_PROBE_STROKE'),
    ({'tool_xyz_m': (.004, 0., .504)}, 2.2, 'OUTSIDE_PROBE_STROKE'),
])
def test_unobservable_or_unsupported_motion_cannot_confirm_payload(change, now, reason):
    assert lift_relation(START, replace(END, **change), now) == ('unknown', reason)
