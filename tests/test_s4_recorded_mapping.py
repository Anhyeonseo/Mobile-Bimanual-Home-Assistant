import copy
import json
import math
from pathlib import Path
import numpy as np
import pytest
from home_robot_tasks.s4_recorded_mapping import align, encoder_path, interpolate, pose, scan_points, transform
from test_s4_base_preparation import teleop_capture

ROOT=Path(__file__).resolve().parents[1]


def test_registration_direction_with_outliers():
    rng=np.random.default_rng(14)
    target=rng.uniform(-1,1,(300,2))
    expected=transform((.015,-.012,.008))
    source=(target-expected[:2,2])@expected[:2,:2]
    source=np.vstack((source,[[5.,4.],[7.,5.]]))
    matrix,quality=align(source,target)
    assert np.allclose(matrix,expected,atol=1e-5)
    assert quality['rmse_m']<1e-5
    with pytest.raises(ValueError,match='overlap'):align(source+100,target)


def test_pose_interpolation_refuses_unobserved_times_and_nonfinite_rows():
    rows=np.array([[1,0,0,3.1],[2,.2,-.1,3.2]])
    assert np.allclose(interpolate(rows,[1.5]),[[.1,-.05,3.15]])
    for t in ([.99],[2.01],[float('nan')]):
        with pytest.raises(ValueError):interpolate(rows,t)
    with pytest.raises(ValueError):interpolate(rows[::-1],[1.5])


def test_negative_ray_timing_deskews_each_ray_into_reference_base_frame():
    scan=dict(ranges=[1.,1.],angle_min=0.,angle_increment=math.pi/2,
              observed_s=2.,time_increment=-.1)
    odom=np.array([[1.,0,0,0],[2.,1.,0,0]])
    points=scan_points(scan,transform((.15,0,0)),odom)
    assert np.allclose(points,[[1.15,0],[.05,1]])
    assert np.allclose(scan_points(scan,transform((.15,0,0))),[[1.15,0],[.15,1]])


def test_encoder_uses_positions_even_when_reported_speed_has_stationary_bias():
    geometry=json.loads((ROOT/'config/home_assistant/s4_base.nominal.json').read_text())
    records=teleop_capture();biased=copy.deepcopy(records)
    for row in biased:
        if row['event']=='reply':
            for wheel in row['reply']['wheel_feedback']:wheel['speed_raw']=50
    a,_=encoder_path(records,geometry);b,_=encoder_path(biased,geometry)
    assert np.allclose(a,b)
    assert abs(a[-1,3])>0
    assert np.max(np.abs(np.diff(a[:,3])))<.02
    bad=copy.deepcopy(records);bad[-1]['error']='lost hold'
    with pytest.raises(ValueError):encoder_path(bad,geometry)


def test_localization_comparison_handles_angle_wrap_and_clock_offsets():
    from home_robot_tasks.s4_recorded_mapping import localization_comparison
    reference=[dict(stamp=100+t,xy_yaw=[.1*t,0.,3.1+.02*t]) for t in (1,2,3,4)]
    actual=[dict(stamp=200+t,xy_yaw=[.1*t+.01,0.,3.1+.02*t-2*math.pi]) for t in (1,2,3,4)]
    result=localization_comparison(actual,200,reference,100)
    assert result['tracking_sanity_passed']
    assert result['final_slam_difference_m']==pytest.approx(.01)
    assert result['final_slam_difference_deg']==pytest.approx(0,abs=1e-10)
    assert not result['physical_localization_qualified']
    assert not result['encoder_agreement_required']
    for row in actual:row['xy_yaw'][0]+=.5
    assert not localization_comparison(actual,200,reference,100)['tracking_sanity_passed']
    with pytest.raises(ValueError,match='shared'):localization_comparison(actual,300,reference,100)
