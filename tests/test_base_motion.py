import math
import pytest
from home_robot_tasks.base_motion import Wheel, OmniKinematics, WheelOdometry
from home_robot_tasks.fetch import InvalidTask


def geometry():
    return OmniKinematics(tuple(Wheel(.2*math.cos(a),.2*math.sin(a),a+math.pi/2,.05)
                                for a in (0,2*math.pi/3,4*math.pi/3)))

@pytest.mark.parametrize('twist',[(0,0,0),(1,0,0),(0,1,0),(0,0,1),(-.5,.3,-.7)])
def test_forward_inverse_and_common_saturation(twist):
    g=geometry()
    assert g.body_twist(g.wheel_rates(twist))==pytest.approx(twist)
    rates=g.wheel_rates(twist,(2,3,4))
    assert all(abs(v)<=limit+1e-10 for v,limit in zip(rates,(2,3,4)))
    actual=g.body_twist(rates)
    factors=[a/b for a,b in zip(actual,twist) if b]
    if factors: assert factors==pytest.approx([factors[0]]*len(factors))


def test_independent_pure_rotation_wheel_rates():
    assert geometry().wheel_rates((0,0,1))==pytest.approx((4,4,4))


def test_singular_geometry_and_invalid_inputs_rejected():
    with pytest.raises(InvalidTask): OmniKinematics((Wheel(0,0,0,.05),)*3)
    for bad in ((True,0,0),(float('nan'),0,0),(0,0)):
        with pytest.raises(InvalidTask): geometry().wheel_rates(bad)


def test_exact_body_arc_integration_and_gap_does_not_invent_motion():
    g=geometry(); odom=WheelOdometry(g,max_gap_s=2)
    rates=g.wheel_rates((1,0,1))
    assert odom.observe(rates,0)==(0,0,0)
    assert odom.observe(rates,1)==pytest.approx((math.sin(1),1-math.cos(1),1))
    pose=odom.pose
    with pytest.raises(InvalidTask): odom.observe(rates,1)
    with pytest.raises(InvalidTask,match='feedback_gap'): odom.observe(rates,4)
    assert odom.pose==pose and odom.discontinuities==1
    assert odom.observe((0,0,0),5)==pose
    assert odom.observe((0,0,0),6)==pose
