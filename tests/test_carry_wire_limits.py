"""Quantization must narrow the usable endpoint interval, never widen limits."""
import numpy as np
import pytest

from home_robot_tasks.carry_program import _wire_interior


@pytest.mark.parametrize('sign',[-1,1])
def test_boundary_rounding_is_corrected_inward_before_retiming(sign):
    lo,hi=sorted([sign*(-.5711843642490718),sign*.712345234])
    for value in np.linspace(lo,hi,1001):
        corrected=_wire_interior(value,lo,hi)
        wire=round(corrected*1e6)/1e6
        assert lo<=wire<=hi
        assert abs(corrected-value)<=1e-6
    midpoint=(lo+hi)/2
    assert _wire_interior(midpoint,lo,hi)==midpoint


@pytest.mark.parametrize('value',[float('nan'),float('inf'),-.100001,.100001])
def test_real_limit_violations_are_not_clamped_away(value):
    with pytest.raises(ValueError):_wire_interior(value,-.1,.1)


def test_unrepresentable_interval_is_rejected():
    with pytest.raises(ValueError,match='representable'):_wire_interior(.0000003,.0000002,.0000004)
