import numpy as np
import pytest
from home_robot_tasks.navigation_health import NavigationHealth,NavigationHealthProfile
from home_robot_tasks.fetch import InvalidTask


def setup():
    now=[1.];health=NavigationHealth(NavigationHealthProfile('laser'),lambda:now[0])
    return now,health


def scan(h,stamp):
    h.scan(observed_s=stamp,frame='laser',ranges=[1.]*360,minimum_range_m=.12,maximum_range_m=3.5)


def localize(h,stamp,cov=None):
    h.localization(observed_s=stamp,frame='map',covariance_xy_yaw=np.eye(3)*.001 if cov is None else cov,transform_valid=True)


def test_scan_cadence_gap_requires_new_consecutive_observations():
    now,h=setup()
    for t in (1.1,1.2,1.3):now[0]=t;scan(h,t)
    localize(h,1.3);assert h.ready()
    now[0]=1.71;scan(h,1.71);localize(h,1.71)
    assert not h.ready() and h.report()['maximum_scan_gap_s']>.4
    for t in (1.8,1.9):now[0]=t;scan(h,t)
    assert h.ready()
    now[0]=2.3;assert not h.ready()


@pytest.mark.parametrize('bad',['future','old','duplicate','empty','covariance','transform'])
def test_invalid_evidence_cannot_refresh_navigation(bad):
    now,h=setup();now[0]=1.2;scan(h,1.2);localize(h,1.2)
    with pytest.raises(InvalidTask):
        if bad=='future':scan(h,1.3)
        elif bad=='old':scan(h,.8)
        elif bad=='duplicate':scan(h,1.2)
        elif bad=='empty':h.scan(observed_s=1.2,frame='laser',ranges=[float('inf')]*360,minimum_range_m=.12,maximum_range_m=3.5)
        elif bad=='covariance':now[0]=1.3;localize(h,1.3,np.diag([.1,.1,.1]))
        else:h.localization(observed_s=1.2,frame='map',covariance_xy_yaw=np.eye(3)*.001,transform_valid=False)
    assert not h.ready()


def test_new_travel_epoch_rejects_old_but_recent_sensor_samples():
    now,h=setup();now[0]=1.2;scan(h,1.2);localize(h,1.2)
    now[0]=1.3;h.reset('second-travel')
    with pytest.raises(InvalidTask):scan(h,1.2)
    with pytest.raises(InvalidTask):localize(h,1.2)
    assert not h.ready()
