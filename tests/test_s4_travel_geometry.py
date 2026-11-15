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


def test_offset_depth_grid_does_not_pad_obstacles_twice():
    from tools.diagnostics.review_s4_sensor_capture import voxel_body_centres
    body=np.array([[-.06,-.06],[-.06,.06],[.06,.06],[.06,-.06]])
    result=voxel_body_centres(np.array([[1,1]]),[body],voxel_size=.1,
        grid_shape=(5,5),origin=np.array([.05,.05]),resolution=.1,yaw=0,clearance=0)
    # Original obstacle is [.1,.2]^2; centres .1 and .2 collide, .3 does not.
    assert result[0,0] and result[1,1]
    assert not result[2,1] and not result[1,2]
    padded=voxel_body_centres(np.array([[1,1]]),[body],voxel_size=.1,
        grid_shape=(5,5),origin=np.array([.05,.05]),resolution=.1,yaw=0,clearance=.05)
    assert padded[2,1]


def test_outside_grid_depth_obstacle_can_block_an_inside_robot_centre():
    from tools.diagnostics.review_s4_sensor_capture import voxel_body_centres
    body=np.array([[-.2,-.05],[-.2,.05],[.2,.05],[.2,-.05]])
    result=voxel_body_centres(np.array([[-1,1]]),[body],voxel_size=.1,
        grid_shape=(5,5),origin=np.zeros(2),resolution=.1,yaw=0,clearance=0)
    assert result[1,0] and result[1,1] and not result[1,3]


def test_complete_fixed_projection_includes_upper_lift_for_lidar_at_both_headings():
    from scipy.spatial import ConvexHull
    from tools.diagnostics.review_s4_sensor_capture import height_matched_candidates
    def box(x,y,z0,z1):
        v=np.array([[a,b,c] for a in [-x,x] for b in [-y,y] for c in [z0,z1]])
        return v[ConvexHull(v).simplices]
    surfaces=[box(.1,.1,0,.15),box(.1,.3,.2,.5)]
    fixed=np.array([[-.1,-.3],[.1,-.3],[.1,.3],[-.1,.3]])
    occupied=np.zeros((25,25),bool);occupied[12,12]=True
    for yaw,index in ((0,(10,12)),(np.pi/2,(12,10))):
        options=dict(resolution=.05,origin=np.zeros(2),voxel_size=.05,lidar_top=.15,height=.5,yaw=yaw,clearance=0)
        # At least 20 cm from the obstacle only the wider upper structure overlaps.
        index=(8,12) if yaw==0 else (12,8)
        old,_,_=height_matched_candidates(occupied,np.empty((0,3),int),surfaces,**options)
        full,_,_=height_matched_candidates(occupied,np.empty((0,3),int),surfaces,navigation_footprint=fixed,**options)
        assert not old[index] and full[index]
