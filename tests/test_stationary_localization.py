import math
import numpy as np
from home_robot_tasks.stationary_localization import StationaryScanWindow, fresh_stationary


def scan(t, ranges=None):
    return dict(observed_s=t,ranges=[1.]*360 if ranges is None else ranges)


def parked(t):
    return dict(observed_s=t,twist=[0.,0.,0.])


def test_fixed_anchor_median_and_original_obstacle_scan_preserved():
    w=StationaryScanWindow()
    for n in range(11):
        t=1+n*.2;r=[1.]*360;r[0]=float('nan');r[1]=.3 if n==10 else 1.
        source=scan(t,r)
        aggregated,seed=w.prepare(source,[n*.01,0,0],parked(t),execute=False,now=t)
        pose=w.observe([.005*(-1)**n,0,.002*(-1)**n],t)
        assert seed==[0.,0.,0.]  # never chain stationary ICP estimates
        assert source['ranges'][1]==r[1]
    assert aggregated['ranges'][1]==1. and source['ranges'][1]==.3
    assert math.isnan(aggregated['ranges'][0])
    assert w.report['ready'] and fresh_stationary(w.report,t)
    assert abs(pose[0])<=.005
    assert not fresh_stationary(w.report,t+.51)


def test_motion_or_stale_wheel_resets_and_does_not_smooth_moving_scan():
    for cause in ('execute','velocity','stale','missing'):
        w=StationaryScanWindow();w.prepare(scan(1),[0,0,0],parked(1),execute=False,now=1)
        wheel=parked(2)
        if cause=='velocity':wheel['twist']=[.01,0,0]
        if cause=='stale':wheel['observed_s']=1
        if cause=='missing':wheel=None
        original=scan(2)
        result,seed=w.prepare(original,[.1,0,0],wheel,execute=cause=='execute',now=2)
        assert result is original and seed==[.1,0,0] and w.anchor is None
        assert not w.report['ready']


def test_wraparound_and_stationary_drift_not_claimed_stable():
    w=StationaryScanWindow()
    for n in range(11):
        t=n*.2
        w.prepare(scan(t),[0,0,math.pi],parked(t),execute=False,now=t)
        w.observe([0,0,math.pi-.001 if n%2 else -math.pi+.001],t,revision='a')
    assert w.report['ready'] and abs(abs(w.report['pose'][2])-math.pi)<.002
    w.observe([0,0,math.pi],2.1,revision='b')
    assert not w.report['ready'] and w.anchor is None
    w=StationaryScanWindow()
    for n in range(11):
        t=n*.2;w.prepare(scan(t),[0,0,0],parked(t),execute=False,now=t)
        w.observe([n*.008,0,0],t)
    assert not w.report['ready']


def test_reordered_scan_or_gap_restarts_window():
    w=StationaryScanWindow()
    for t in (1.,1.2):w.prepare(scan(t),[0,0,0],parked(t),execute=False,now=t)
    w.prepare(scan(1.1),[.1,0,0],parked(1.1),execute=False,now=1.1)
    assert len(w.scans)==1 and w.anchor==[.1,0,0]
    w.prepare(scan(2),[.2,0,0],parked(2),execute=False,now=2)
    assert len(w.scans)==1 and w.anchor==[.2,0,0]


def test_stationary_evidence_must_match_current_furniture_map():
    report=dict(ready=True,observed_s=2,map_revision='old')
    assert fresh_stationary(report,2,revision='old')
    assert not fresh_stationary(report,2,revision='new')
    assert not fresh_stationary(dict(ready=True,observed_s=2),2,revision='new')


def test_inspection_encoder_window_handles_speed_flag_but_not_motion_or_power():
    from home_robot_tasks.stationary_localization import StationaryWheelInventory
    from types import SimpleNamespace
    def body_twist(speeds):
        assert all(type(v) is float for v in speeds)
        return tuple(speeds)
    geometry=SimpleNamespace(body_twist=body_twist)
    def rows(t,delta=0,torque=0):
        return [dict(complete=True,bus='left',requested_id=m,
            sample_started_tick_ms=int(t*1000),offset_and_mode=dict(operating_mode=1),
            command=dict(torque_enabled_raw=torque,goal_speed_raw=0),
            feedback=dict(position_raw=(4095+delta)%4096,speed_raw=32818,moving_raw=1)) for m in (7,8,9)]
    w=StationaryWheelInventory()
    for n in range(6):r=w.observe(rows(1+n*.1,n%2),1+n*.1,1+n*.1+.02,geometry)
    assert r['stationary_inventory'] and r['position_span_counts']==[1,1,1]
    assert w.observe(rows(1.6,5),1.6,1.62,geometry) is None
    assert w.observe(rows(1.7,torque=1),1.7,1.72,geometry) is None and not w.samples
    for n in range(6):w.observe(rows(2+n*.1),2+n*.1,2+n*.1+.02,geometry)
    assert w.observe(rows(2.5),2.6,2.62,geometry) is None  # repeated MCU timestamp
    assert w.observe(rows(2.7),2.7,3.,geometry) is None  # late query


def test_zero_command_window_uses_unfiltered_scans_and_never_claims_stop():
    w=StationaryScanWindow()
    for n in range(11):
        t=1+n*.2;wheel=dict(observed_s=t,twist=[.005,0,.01],zero_command_elapsed_s=.5+n*.2)
        source=scan(t,[1+n*.001]*360)
        registered,seed=w.prepare(source,[n*.03,0,0],wheel,execute=False,now=t)
        assert registered is source and seed==[0,0,0]
        w.observe([.002*(-1)**n,0,.001*(-1)**n],t)
    assert w.report['ready'] and w.report['source']=='raw_scan_zero_command_window'
    assert not w.report.get('stop_confirmed') and not w.report['surveyed_accuracy_verified']
    w.prepare(scan(t+.2),[0,0,0],dict(wheel,observed_s=t+.2,zero_command_elapsed_s=0),execute=False,now=t+.2)
    assert not w.report['ready'] and w.anchor is None


def test_zero_command_alone_does_not_validate_motion_or_stale_feedback():
    w=StationaryScanWindow()
    for n in range(11):
        t=1+n*.2;wheel=dict(observed_s=t,twist=[.005,0,0],zero_command_elapsed_s=1)
        w.prepare(scan(t),[0,0,0],wheel,execute=False,now=t)
        w.observe([n*.008,0,0],t)
    assert not w.report['ready']
    w.prepare(scan(t+.5),[0,0,0],wheel,execute=False,now=t+.5)
    assert w.anchor is None
