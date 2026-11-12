import numpy as np
import pytest
from tools.diagnostics.review_s4_sensor_capture import endpoint_obstacles


def test_elevated_surface_height_overlap_uses_full_voxel_not_center():
    voxels=np.array([[2,2,8],[3,3,13]])
    occupied,n=endpoint_obstacles(voxels,.1,(8,8),np.zeros(2),.1,bottom=.15,top=.81)
    assert n==1 and occupied[2,2] and not occupied[3,3]
    occupied,n=endpoint_obstacles(voxels,.1,(8,8),np.zeros(2),.1,bottom=.15,top=.8)
    assert n==0 and not occupied.any()


def test_unaligned_slam_grid_covers_voxel_extent_and_never_wraps_outside_points():
    occupied,n=endpoint_obstacles(np.array([[1,1,4],[-10,-10,4]]),.1,(4,4),np.array([.05,.05]),.1,bottom=.1,top=1.)
    assert n==2
    np.testing.assert_array_equal(occupied[:2,:2],np.ones((2,2),bool))
    assert occupied.sum()==4


@pytest.mark.parametrize('bottom,top',[(0.,float('nan')),(1.,.5),(-1.,1.),(0.,float('inf'))])
def test_invalid_projection_interval_refused(bottom,top):
    with pytest.raises(ValueError):endpoint_obstacles(np.zeros((1,3),int),.1,(3,3),np.zeros(2),.1,bottom=bottom,top=top)


def test_endpoint_review_does_not_infer_free_or_clear_missing_obstacles():
    occupied,n=endpoint_obstacles(np.empty((0,3),int),.1,(3,3),np.zeros(2),.1,bottom=0,top=1.)
    assert not occupied.any() and n==0  # no hit evidence, not free-space evidence
    with pytest.raises(ValueError):endpoint_obstacles(np.zeros((1,3),float),.1,(3,3),np.zeros(2),.1,bottom=0,top=1.)
