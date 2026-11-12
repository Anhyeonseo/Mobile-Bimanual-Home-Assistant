import numpy as np
import pytest
from tools.lib.household_semantics import crop_boxes,review_scores
from home_robot_tasks.assistant.contracts import Context,Stamp
from home_robot_tasks.assistant.perception import TransformSample
from home_robot_tasks.assistant.furniture import masked_furniture
from home_robot_tasks.fetch import InvalidTask


def test_crop_agreement_is_not_reported_as_probability_and_ties_keep_original():
    result=review_scores('wardrobe refrigerator',[[1,3,0],[0,2,1]],['wardrobe','refrigerator','floor'])
    assert result['label']=='unclassified_furniture' and not result['resolved'] and not result['scores_are_probabilities']
    agree=review_scores('refrigerator',[[1,3,0],[0,2,1]],['wardrobe','refrigerator','floor'])
    assert agree['label']=='refrigerator' and agree['resolved']
    assert not review_scores('ambiguous',[[1,1.1],[1.1,1]],['a','b'])['resolved']
    assert not review_scores('ambiguous',[[1,1.1],[1,1.1]],['a','b'])['resolved']
    with pytest.raises(ValueError):review_scores('a',[[1,np.nan],[2,3]],['a','b'])


def test_context_crops_stay_in_source_frame():
    assert crop_boxes([0,0,10,10],(10,10,3))==[[0,0,10,10]]*2
    assert crop_boxes([3,3,7,7],(10,10,3))==[[3,3,7,7],[2,2,8,8]]
    with pytest.raises(ValueError):crop_boxes([-1,0,2,2],(10,10,3))


def observe(depth, floor):
    context=Context('home','map','cal','clock');stamp=Stamp(1.,context,'view')
    matrix=np.diag([1.,-1.,-1.,1.]);matrix[2,3]=1.
    transform=TransformSample(stamp,'color_camera_optical_frame','map',matrix,'simulation')
    return masked_furniture(category='table',detector_score=.8,detection_id='table',mask=np.ones((20,20),bool),
        depth_m=depth,intrinsics=(20,20,9.5,9.5),stamp=stamp,context=context,transform=transform,now_s=1.,floor_z_m=floor)


def test_floor_dominated_mask_is_not_saved_as_furniture_and_depth_is_unchanged():
    depth=np.ones((20,20));original=depth.copy()
    with pytest.raises(InvalidTask,match='floor_dominated'):observe(depth,0.)
    np.testing.assert_array_equal(depth,original)
    assert observe(depth,None)['category']=='table'  # historical policy is explicit


def test_floor_pixels_are_removed_without_removing_elevated_table():
    depth=np.full((20,20),.6);depth[10:]=1.
    observation=observe(depth,0.)
    assert min(c[2] for c in observation['observed_voxels'])>=7


def test_view_support_does_not_count_same_capture_or_near_identical_frames_twice():
    from home_robot_tasks.recorded_semantics import landmark_view_support
    from home_robot_tasks.assistant.world import WorldStore
    with pytest.raises(ValueError):crop_boxes([0,0,float('inf'),2],(10,10,3))
    store=WorldStore();instance=dict(instance_id='furniture-1',category='table')
    for i,angle in enumerate([0,0,.02,.3]):
        matrix=np.eye(4);matrix[:3,2]=[np.sin(angle),0,np.cos(angle)]
        key=f'capture-{i}'
        store.save_record('semantic_capture',key,dict(stamp=dict(capture_id=key),camera_to_map=matrix.tolist()))
        store.save_record('furniture_observation',f'observation-{i}',dict(observation=dict(stamp=dict(capture_id=key)),association=dict(instance_id='furniture-1')))
    result,=landmark_view_support(store,[instance]);assert result['distinct_captures']==4 and result['viewpoint_groups']==2
    assert not result['identity_verified'];store.close()


def test_one_floor_mask_does_not_discard_good_furniture_in_same_capture():
    from home_robot_tasks.assistant.world import WorldStore
    from home_robot_tasks.semantic_mapping import RGBDFurnitureMapper
    class Model:
        def detect(self,image,prompt):
            return [dict(label='table',score=.8,box_xyxy=[0,0,10,20]),
                    dict(label='table',score=.8,box_xyxy=[10,0,20,20])],None
    class Segmenter:
        def segment(self,image,boxes):
            left=np.zeros((20,20),bool);left[:,:10]=True
            return [(left,.9),(~left,.9)],None
    store=WorldStore();context=Context('home','map','cal','clock')
    mapper=RGBDFurnitureMapper(store,context,Model(),Segmenter(),model_revision='fixture',floor_z_m=0.)
    matrix=np.diag([1.,-1.,-1.,1.]);matrix[2,3]=1.
    stamp=Stamp(1.,context,'mixed-view');tf=TransformSample(stamp,'color_camera_optical_frame','map',matrix,'simulation')
    depth=np.full((20,20),.6);depth[:,:10]=1.
    kw=dict(stamp=stamp,transform=tf,now_s=1.)
    result=mapper.observe(np.zeros((20,20,3),np.uint8),depth,(20,20,9.5,9.5),**kw)
    assert len(result['provenance']['rejected_detections'])==1
    assert len(result['provenance']['accepted_detections'])==1
    assert len(mapper.memory.instances())==1
    assert mapper.observe(np.zeros((20,20,3),np.uint8),depth,(20,20,9.5,9.5),**kw)==result
    store.close()


def test_same_geometry_with_generic_storage_label_groups_but_a_bridge_stays_ambiguous():
    from home_robot_tasks.recorded_semantics import spatial_landmark_review
    def item(name,category,start):
        return dict(instance_id=name,category=category,voxel_size_m=.05,context={'map':'same'},
            observed_voxels=[[i,0,10] for i in range(start,start+10)])
    pair=spatial_landmark_review([item('a','wardrobe',0),item('b','unclassified_furniture',1)],[])
    assert len(pair)==1 and pair[0]['category']=='unclassified_furniture'
    assert pair[0]['label_state']=='NEEDS_REVIEW' and not pair[0]['identity_verified']
    broad=item('b','unclassified_furniture',0);broad['observed_voxels']+=item('c','cabinet',20)['observed_voxels']
    separate=spatial_landmark_review([item('a','wardrobe',0),broad,item('c','cabinet',20)],[])
    assert len(separate)==3 and any(x['ambiguous_neighbor_ids'] for x in separate)
