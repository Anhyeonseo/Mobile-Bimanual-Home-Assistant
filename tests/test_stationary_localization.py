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
