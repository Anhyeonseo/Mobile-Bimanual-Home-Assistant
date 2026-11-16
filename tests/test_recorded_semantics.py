import copy
import numpy as np
import pytest
from home_robot_tasks.recorded_semantics import register_native_depth,RecordedMapPoses


def metadata():
    intr=dict(width=3,height=3,fx=1.,fy=1.,ppx=1.,ppy=1.,coeffs=[0.]*5)
    return dict(native_depth_intrinsics=intr,color_intrinsics=copy.deepcopy(intr),depth_scale_m=.001,
        depth_to_color=dict(rotation_column_major=np.eye(3).flatten(order='F').tolist(),translation_m=[0.,0.,0.]))


def test_registration_identity_holes_and_units():
    raw=np.array([[0,1000,0],[2000,1000,500],[0,1000,0]],dtype=np.uint16)
    np.testing.assert_allclose(register_native_depth(raw,metadata()),raw*.001)


def test_registration_zbuffer_chooses_nearest_measured_surface_without_fill():
    m=metadata();m['color_intrinsics'].update(fx=.1,fy=.1)
    raw=np.zeros((3,3),np.uint16);raw[1]=[2000,1000,500]
    registered=register_native_depth(raw,m)
    assert registered[1,1]==.5 and np.count_nonzero(registered)==1


def test_registration_uses_column_major_extrinsics_and_color_depth():
    m=metadata();r=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
    m['depth_to_color']['rotation_column_major']=r.flatten(order='F').tolist()
    raw=np.zeros((3,3),np.uint16);raw[1,2]=1000
    expected=np.zeros((3,3));expected[2,1]=1.
    np.testing.assert_allclose(register_native_depth(raw,m),expected)
    m=metadata();m['depth_to_color']['translation_m']=[0.,0.,.2]
    raw=np.zeros((3,3),np.uint16);raw[1,1]=1000
    assert register_native_depth(raw,m)[1,1]==pytest.approx(1.2)


@pytest.mark.parametrize('failure',['distortion','scale','intrinsics','rotation','dtype','shape'])
def test_registration_refuses_unmodelled_or_invalid_calibration(failure):
    m=metadata();raw=np.ones((3,3),np.uint16)*1000
    if failure=='distortion':m['color_intrinsics']['coeffs'][0]=.01
    if failure=='scale':m['depth_scale_m']=float('nan')
    if failure=='intrinsics':m['native_depth_intrinsics']['fx']=0.
    if failure=='rotation':m['depth_to_color']['rotation_column_major'][0]=2.
    if failure=='dtype':raw=raw.astype(float)
    if failure=='shape':raw=raw[:2]
    with pytest.raises(ValueError):register_native_depth(raw,m)


def test_map_pose_uses_encoder_motion_between_corrections_and_refuses_sparse_or_future():
    odom=np.array([[1.,0,0,0],[1.5,.2,0,0],[2.,.5,0,0],[4.,1.,0,0]])
    p=RecordedMapPoses(odom,[dict(stamp=11.,xy_yaw=[2.,3.,0]),dict(stamp=11.5,xy_yaw=[2.2,3.,0]),
                            dict(stamp=12.,xy_yaw=[2.5,3.,0]),dict(stamp=14.,xy_yaw=[3.,3.,0])],10.)
    matrix,gap=p.at(1.25);np.testing.assert_allclose(matrix[:2,2],[2.1,3.]);assert gap==.5
    for stamp in (.9,2.5,4.1):
        with pytest.raises(ValueError):p.at(stamp)


def test_final_graph_keyframes_use_dense_lidar_during_stationary_recovery():
    odom=np.array([[t,0.,0.,0.] for t in np.arange(1.,21.,.2)])
    saved=[dict(stamp=11.,xy_yaw=[2.,3.,0.]),dict(stamp=29.,xy_yaw=[2.01,3.,0.])]
    final=RecordedMapPoses(odom,saved,10.,final_graph=True)
    matrix,gap=final.at(10.)
    assert gap==18 and 2.<matrix[0,2]<2.01
    with pytest.raises(ValueError):RecordedMapPoses(odom,saved,10.).at(10.)
    sparse=odom[(odom[:,0]<5)|(odom[:,0]>15)]
    with pytest.raises(ValueError):RecordedMapPoses(sparse,saved,10.,final_graph=True).at(10.)


def test_final_graph_rotation_is_checked_at_robot_not_arbitrary_odom_origin():
    odom=np.array([[t,10.,0.,0.] for t in np.arange(1.,21.,.2)])
    saved=[dict(stamp=11.,xy_yaw=[10.,0.,0.]),dict(stamp=29.,xy_yaw=[10.,0.,.08])]
    matrix,gap=RecordedMapPoses(odom,saved,10.,final_graph=True).at(10.)
    assert gap==18 and np.linalg.norm(matrix[:2,2]-[10.,0.])<.02
    saved[-1]['xy_yaw']=[10.7,0.,0.]
    with pytest.raises(ValueError):RecordedMapPoses(odom,saved,10.,final_graph=True).at(10.)


def test_view_selection_covers_each_recording_before_extra_viewpoints():
    from home_robot_tasks.recorded_semantics import choose_recorded_views
    class Poses:
        def at(self,t):
            p=np.eye(3);p[0,2]=t;return p,0.
    rows=[dict(source_epoch=k,color_time_s=float(i)) for i,k in enumerate(['a']*30+['b']*2+['c']*30)]
    chosen=choose_recorded_views(rows,Poses(),6)
    assert len(chosen)==6 and {r['source_epoch'] for r in chosen}=={'a','b','c'}


def test_coarse_household_names_keep_unrelated_composites_unresolved():
    from home_robot_tasks.semantic_mapping import furniture_category
    from home_robot_tasks.assistant.furniture import FURNITURE_PROMPT
    assert furniture_category('desk table')=='table'
    assert furniture_category('storage basket.') is None
    assert furniture_category('handheld fan') is None
    assert furniture_category('appliance') is None
    assert furniture_category('fridge')=='refrigerator'
    assert furniture_category('shelf')=='bookshelf'
    assert furniture_category('bookshelf cabinet') is None
    assert furniture_category('chair table') is None
    assert furniture_category('person') is None
    assert 'refrigerator' in FURNITURE_PROMPT
    # Recognize distractors without making them semantic map landmarks.
    assert 'storage basket' in FURNITURE_PROMPT and 'appliance' in FURNITURE_PROMPT


def test_edge_model_roundoff_is_clipped_without_weakening_detector_contract():
    from home_robot_tasks.recorded_semantics import clip_recorded_boxes
    source=[dict(label='table',box_xyxy=[-.01,1.,8.,7.]),dict(label='chair',box_xyxy=[20.,1.,30.,7.])]
    clipped=clip_recorded_boxes(source,(6,8,3))
    assert len(clipped)==1 and clipped[0]['box_xyxy']==[0.,1.,8.,6.]
    assert clipped[0]['unclipped_box_xyxy']==source[0]['box_xyxy']
    assert source[0]['box_xyxy'][0]==-.01
    with pytest.raises(ValueError):clip_recorded_boxes([dict(box_xyxy=[0,0,float('nan'),5])],(6,8,3))
