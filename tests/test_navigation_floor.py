import numpy as np
import pytest

from home_robot_tasks.navigation_depth import measured_floor_heights


def floor_points(offset=.005, slope=.03):
    x,y=np.meshgrid(np.linspace(.25,1.4,50),np.linspace(-.9,.9,60))
    return np.column_stack((x.ravel(),y.ravel(),offset+slope*y.ravel()))


def test_measured_floor_removes_tilt_without_removing_low_or_upper_obstacles():
    floor=floor_points(offset=.02)
    objects=np.array([[.7,-.7,.02+.03*(-.7)+h] for h in (.06,.16,.7)])
    heights,evidence=measured_floor_heights(np.vstack((floor,objects)),0.)
    assert evidence and np.max(np.abs(heights[:-3]))<.001
    assert np.all(heights[-3:]>.04)
    # The nominal reference incorrectly marks the far edge of this floor.
    assert np.any(floor[:,2]>.04)


@pytest.mark.parametrize('kind',['sparse','narrow','elevated','steep'])
def test_ambiguous_floor_keeps_nominal_obstacles(kind):
    p=floor_points()
    if kind=='sparse':p=p[:100]
    if kind=='narrow':p[:,1]*=.02
    if kind=='elevated':p[:,2]+=.055
    if kind=='steep':p[:,2]=p[:,1]*.2
    heights,evidence=measured_floor_heights(p,0.)
    assert evidence is None
    assert heights==pytest.approx(p[:,2])
