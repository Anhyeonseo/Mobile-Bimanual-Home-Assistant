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


def test_height_slab_clips_crossing_faces_instead_of_using_full_part_bounds():
    from tools.diagnostics.review_s4_sensor_capture import slab_footprints
    triangles=np.array([[[0,0,0],[2,0,2],[0,2,2]]],float)
    poly,=slab_footprints([triangles],.9,1.1)
    assert poly.max()==pytest.approx(1.1)
    assert np.min(poly.sum(1))==pytest.approx(.9)
    assert slab_footprints([triangles],3.,4.)==[]


def test_robot_forward_protrusion_blocks_centres_behind_obstacle_and_rotates():
    from tools.diagnostics.review_s4_sensor_capture import footprint_kernel
    poly=np.array([[.2,-.03],[.3,-.03],[.3,.03],[.2,.03]])
    kernel=footprint_kernel([poly],.05,0,0);cy,cx=np.array(kernel.shape)//2
    assert kernel[cy,cx-5] and not kernel[cy,cx+5]
    rotated=footprint_kernel([poly],.05,np.pi/2,0);cy,cx=np.array(rotated.shape)//2
    assert rotated[cy-5,cx] and not rotated[cy+5,cx]


def test_separate_parts_do_not_fill_empty_space_between_them():
    from tools.diagnostics.review_s4_sensor_capture import footprint_kernel
    poly=np.array([[.2,-.03],[.3,-.03],[.3,.03],[.2,.03]])
    kernel=footprint_kernel([poly,-poly],.05,0,0)
    cy,cx=np.array(kernel.shape)//2
    assert not kernel[cy,cx] and kernel[cy,cx-5] and kernel[cy,cx+5]


def test_low_base_width_is_not_applied_to_high_obstacle():
    from scipy.spatial import ConvexHull
    from tools.diagnostics.review_s4_sensor_capture import height_matched_candidates
    def box(x,y,z0,z1):
        vertices=np.array([[a,b,c] for a in [-x,x] for b in [-y,y] for c in [z0,z1]])
        return vertices[ConvexHull(vertices).simplices]
    surfaces=[box(.25,.2,0,.2),box(.04,.04,.2,1.)]
    occupied=np.zeros((25,25),bool)
    result,_,_=height_matched_candidates(occupied,np.array([[10,10,7]]),surfaces,
        resolution=.1,origin=np.zeros(2),voxel_size=.1,lidar_top=.21,height=1.,yaw=0,clearance=0)
    assert result[10,10] and not result[10,8]
    occupied[10,10]=True
    low,_,_=height_matched_candidates(occupied,np.empty((0,3),int),surfaces,
        resolution=.1,origin=np.zeros(2),voxel_size=.1,lidar_top=.21,height=1.,yaw=0,clearance=0)
    assert low[10,8]
