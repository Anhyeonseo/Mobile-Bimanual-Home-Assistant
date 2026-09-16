import math
import pytest
from home_robot_tasks.classical import route_for_arm
from home_robot_tasks.motion_quality import analyze_trace


def route(distance,**changes):
    kwargs=dict(current=(0.,)*12,opposite_hold=(0.,)*6,arm='left',
        steps=[{'kind':'arm','target_positions_rad':[distance,0,0,0,0]}],minimum_rad=(-1.,)*12,
        maximum_rad=(1.,)*12,max_speed_rad_s=(.5,)*12)
    kwargs.update(changes)
    return route_for_arm(**kwargs)

@pytest.mark.parametrize('distance',[0.,1e-6,.01,.2,.9,-.8])
def test_rest_to_rest_path_has_bounded_sampled_derivatives_and_exact_endpoint(distance):
    p=route(distance);q=[0.]*3+[x['positions_rad'][0] for x in p]+[distance]*3
    v=[(b-a)/.05 for a,b in zip(q,q[1:])]
    a=[(y-x)/.05 for x,y in zip(v,v[1:])]
    j=[(y-x)/.05 for x,y in zip(a,a[1:])]
    assert max(map(abs,v))<=.5+1e-8
    assert max(map(abs,a))<=1.+1e-8
    assert max(map(abs,j))<=8.+1e-8
    assert p[-1]['positions_rad'][0]==distance
    assert all(min(0,distance)-1e-10<=x<=max(0,distance)+1e-10 for x in q)
    assert all(x['positions_rad'][1:]==(0.,)*11 for x in p)


def test_smallest_grid_duration_within_quintic_bounds_is_used():
    p=route(.2);duration=(p[-1]['offset_ms']-p[0]['offset_ms'])/1000
    needed=max(1.875*.2/.5,math.sqrt(10/math.sqrt(3)*.2),(60*.2/8)**(1/3))
    assert needed<=duration<needed+.05
    assert abs(p[1]['positions_rad'][0])<.2/len(p)

@pytest.mark.parametrize('changes',[{'max_acceleration_rad_s2':(0.,)*12},
    {'max_jerk_rad_s3':(float('nan'),)*12},{'max_jerk_rad_s3':(1e-20,)*12}])
def test_invalid_or_unbounded_smoothing_rejected(changes):
    with pytest.raises(ValueError):route(.2,**changes)


def trace():
    return dict(joint_names=[f'joint_{n}' for n in range(12)],samples=[
        dict(time_s=i*.05,target_rad=[0.]*12,measured_rad=[.01*(-1)**i]+[0.]*11) for i in range(21)])

def test_trace_reports_hold_shake_without_claiming_tuning_or_settling_from_missing_samples():
    data=trace();r=analyze_trace(data)
    assert r['joints'][0]['hold_peak_to_peak_rad']==pytest.approx(.02)
    assert r['joints'][0]['rms_tracking_error_rad']==pytest.approx(.01)
    assert r['joints'][1]['settling_time_s']==0 and not r['automatic_tuning']
    assert analyze_trace(data,settling_tolerance_rad=.005)['joints'][0]['settling_time_s'] is None
    data['samples']=data['samples'][::4]
    assert analyze_trace(data)['joints'][0]['hold_peak_to_peak_rad'] is None

@pytest.mark.parametrize('fault',['time','nan','vector'])
def test_invalid_trace_fails_closed(fault):
    data=trace()
    if fault=='time':data['samples'][2]['time_s']=0
    elif fault=='nan':data['samples'][2]['measured_rad'][0]=float('nan')
    else:data['samples'][2]['target_rad']=[]
    with pytest.raises(ValueError):analyze_trace(data)
