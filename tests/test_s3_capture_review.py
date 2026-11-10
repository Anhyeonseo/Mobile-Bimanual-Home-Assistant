"""Synthetic two-cup review. No devices, motor commands, model downloads or real grasp."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from home_robot_tasks import s2_commissioning as s2, s2_development as dev
from tools.diagnostics.review_s3_capture import review, save_frame

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def pair(tmp_path):
    Image=pytest.importorskip('PIL.Image')
    profile=json.loads((ROOT/'config/home_assistant/s2_coordinates.development.json').read_text())
    geometry=s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
    raw={side:profile['arms'][side]['reference_raw'] for side in s2.ARMS}
    ref=dict(calibration_id=profile['calibration_id'],session_id='synthetic',controller_boot_id='boot',
             origin_id='bottom',arm_anchor_id='home',clock_id='clock',lift_bottom_confirmed=True,
             arm_turns_anchored=True,lift_origin_raw_unwrapped=1234,arm_raw_unwrapped=raw,established_s=2.)
    snap={**{k:ref[k] for k in dev.ObservationSession.KEYS},**{k:profile[k] for k in ('calibration_id','assembly_revision','camera_device','source_frame')},
          'sequence':2,'capture_id':'rgbd-12','raw_encoding':'continuous_unwrapped_counts','camera_time_s':2.05,
          'lift_time_s':2.05,'received_s':2.07,'joint_times_s':{side:dict.fromkeys(s2.JOINTS,2.05) for side in s2.ARMS},
          'lift_raw_unwrapped':1234,'raw':raw}
    coordinates=dev.ObservationSession(profile,geometry,ref).ingest(snap,now_s=2.07,clock_id='clock')
    frame=dict(number=12,timestamp_domain='timestamp_domain.global_time',timestamp_ms=123456.)
    camera=dict(capture_id='rgbd-12',color_frame=frame,depth_frame=frame,color_time_s=2.05,
                acquisition_span_s=.04,camera_device=profile['camera_device'],source_frame=profile['source_frame'])
    meta=dict(captured_at_utc='2026-11-06T00:00:00Z',coordinate_frame=profile['source_frame'],
              color_frame=frame,depth_frame=frame,color_intrinsics=dict(width=100,height=24))
    row=dict(sequence=2,camera=camera,snapshot=snap,coordinates=coordinates,samples=[{'speed':0}]*13)
    report=dict(records=[row],reference_evidence=ref,status='motor_acquisition_verified',camera_synchronized=True,
                motion_authorized=False,motor_write_commands=0)
    color=np.zeros((24,100,3),np.uint8);color[0,0]=[7,11,23]
    depth=np.full((24,100),564,np.uint16)
    report['snapshot_export']=save_frame(tmp_path,(color,depth,meta),row)
    folder=tmp_path/snap['capture_id']
    assert Image.open(folder/'color.png').getpixel((0,0))==(23,11,7)
    assert Image.open(folder/'depth_aligned.png').getpixel((0,0))==564
    obs=dict(label='cup',capture_id=folder.name,captured_at_utc=meta['captured_at_utc'],frame_id=meta['coordinate_frame'],
             source_sha256=report['snapshot_export']['source_sha256'],clock_domain='global_time',calibration_id='unmeasured',
             status='accepted',surface_xyz_m=None,center_xyz_m=[-.04,.01,.56],box_xyxy=[5,3,30,20])
    other=deepcopy(obs);other.update(center_xyz_m=[.08,.01,.58],box_xyxy=[68,4,89,19])
    return folder,report,[other,obs],profile,geometry


def test_selects_image_left_and_converts_only_visible_center(pair):
    out=review(*pair,'cup','left');p=pair[3]
    wanted=(np.array(p['base_from_optical'])@[-.04,.01,.56,1])[:3]
    np.testing.assert_allclose(out['base_points_m']['center_xyz_m'],wanted)
    assert set(out['base_points_m'])=={'center_xyz_m'}
    assert out['grasp_pose'] is None and out['motion_authorized'] is False and not out['executable']
    assert len(out['model_joint_positions_si'])==13
    assert out['model_joint_positions_si']['right_model_gripper']==p['arms']['right']['reference_model_rad']['gripper']


@pytest.mark.parametrize('fault',[None,'drift','gap','epoch','torque','short'])
def test_noisy_lift_speed_requires_continuous_stationary_position_window(pair,fault):
    from so101_arm_bridge.observer_protocol import EXPECTED
    folder,report,items,p,g=pair
    middle=report['records'][0]
    records=[]
    for sequence in range(5):
        row=deepcopy(middle)
        row.update(epoch=7,sequence=sequence)
        row['samples']=[dict(bus=bus,id=motor,position=1000,speed=0,torque=0,acquired_s=sequence*.08) for bus,motor in EXPECTED]
        records.append(row)
    records[2]['samples'][6]['speed']=32818
    if fault=='drift':records[3]['samples'][6]['position']+=5
    if fault=='gap':records[3]['sequence']+=1
    if fault=='epoch':records[3]['epoch']+=1
    if fault=='torque':records[3]['samples'][6]['torque']=1
    # Remove coordinate metadata from surrounding records, as in real captures.
    for i,r in enumerate(records):
        if i!=2:
            for field in ('snapshot','coordinates','camera'):r.pop(field)
    report['records']=records[:4] if fault=='short' else records
    out=review(*pair,'cup','left')
    assert out['status']==('TARGET_DISPLAY_ONLY' if fault is None else 'REOBSERVE')
    if fault is None:
        assert out['stationarity_evidence']['span_counts']==0
        assert out['stationarity_evidence']['reported_lift_speed']==32818


@pytest.mark.parametrize('change,status',[('no_name','NOT_FOUND'),('no_side','AMBIGUOUS_TARGET'),
                                          ('overlap','AMBIGUOUS_TARGET'),('held','REOBSERVE'),('moving','REOBSERVE')])
def test_uncertain_target_never_becomes_a_grasp(pair,change,status):
    folder,r,items,p,g=pair
    if change=='overlap':items[0]['box_xyxy']=[10,4,40,19]
    if change=='held':items[1].update(status='held',reason='sparse_depth')
    if change=='moving':r['records'][0]['samples'][0]['speed']=50
    out=review(*pair,'pen' if change=='no_name' else 'cup',None if change=='no_side' else 'left')
    assert out['status']==status and 'base_points_m' not in out


@pytest.mark.parametrize('change',['wrong_device','other_frame','older_session','wrong_clock','other_images','file_changed'])
def test_mismatched_capture_rejected(pair,change):
    folder,r,items,p,g=pair;row=r['records'][0]
    if change=='wrong_device':row['camera']['camera_device']={}
    if change=='other_frame':items[0]['capture_id']='rgbd-13'
    if change=='older_session':row['coordinates']['session_id']='old'
    if change=='wrong_clock':items[0]['clock_domain']='unknown'
    if change=='other_images':items[0]['source_sha256']={}
    if change=='file_changed':(folder/'depth_aligned.png').write_bytes(b'other depth')
    with pytest.raises(ValueError):review(*pair,'cup','left')


def test_cli_produces_annotated_png_without_actuation(pair,tmp_path):
    import subprocess
    import sys
    from PIL import Image
    folder,r,items,p,g=pair
    for name,obj in [('report',r),('observations',{'observations':items}),('profile',p)]:
        (tmp_path/(name+'.json')).write_text(json.dumps(obj))
    cmd=[sys.executable,str(ROOT/'tools/diagnostics/review_s3_capture.py'),'--snapshot',str(folder)]
    for name in ('report','observations','profile','output'):
        cmd.extend(['--'+name,str(tmp_path/(name+'.json'))])
    preview=tmp_path/'annotated.png'
    subprocess.run(cmd+['--label','cup','--screen-side','left','--preview',str(preview)],check=True,capture_output=True)
    with Image.open(preview) as image:
        assert image.size==(100,24)
    output=json.loads((tmp_path/'output.json').read_text())
    assert output['status']=='TARGET_DISPLAY_ONLY' and output['physical_task_completed'] is False


def test_export_rejects_a_different_exposure_before_writing(pair,tmp_path):
    _,report,*_=pair
    row=report['records'][0]
    with pytest.raises(ValueError,match='exposure'):
        save_frame(tmp_path/'wrong',(None,None,{'color_frame':{},'depth_frame':{}}),row)
    assert not (tmp_path/'wrong').exists()


def test_farther_box_center_does_not_hide_overlap_with_selected_target(pair):
    _,_,items,_,_=pair
    items[0]['box_xyxy']=[31,4,35,19]
    third=deepcopy(items[0]);third['box_xyxy']=[25,3,90,19];items.append(third)
    assert review(*pair,'cup','left')['status']=='AMBIGUOUS_TARGET'


def test_rejected_depth_on_left_does_not_substitute_right_object(pair):
    _,_,items,_,_=pair
    items[1].update(status='rejected',failure='INVALID_DEPTH',reason='no depth',center_xyz_m=None)
    assert review(*pair,'cup','left')['status']=='REOBSERVE'


def test_height_only_replay_stops_before_grasp_and_arm_planning(pair, monkeypatch):
    from home_perception.geometry.plane import Plane
    from tools.diagnostics import replay_s3_grasp as replay
    folder, report, observations, profile, _ = pair
    cam_from_base = np.linalg.inv(profile['base_from_optical'])
    point = (cam_from_base @ [0., 0., profile['ground_z_m']+.75, 1.])[:3]
    plane = Plane(cam_from_base[:3,:3] @ [0.,0.,1.], point, 0., 1.)
    meta = json.loads((folder/'metadata.json').read_text())
    floor = Plane(plane.normal, point - .75 * plane.normal, 0., 1.)
    monkeypatch.setattr(replay, 'load_scene', lambda _: (meta, None, None, floor))
    def observed_on_table(*args, support_planes, up_in_optical):
        np.testing.assert_allclose(up_in_optical, plane.normal, atol=1e-12)
        support_planes.update({i: plane for i in range(len(observations))})
        return observations, []
    monkeypatch.setattr(replay, 'grounding_observations', observed_on_table)
    def unexpected(*args, **kwargs):
        pytest.fail('height-only mode must not generate grasps or plan an arm path')
    monkeypatch.setattr(replay.graspnet, 'candidates', unexpected)
    for name in ('flat_candidates','model','plan_pick'):
        monkeypatch.setattr(replay,name,unexpected)
    out = replay.replay(folder, 'detections.json', 'cup', 'left', robot_report=report, height_only=True, nominal_jaw=True)
    assert out['status'] == 'HEIGHT_SETTING' and 'path' not in out
    assert out['support_plane_source'] == 'target_neighborhood'
    assert out['lift_setting']['preset'] == '45cm'
    assert out['lift_setting']['target_travel_from_bottom_m'] == .45
    assert out['hardware_commands'] == 0 and not out['motion_authorized']


@pytest.mark.parametrize('measured,should_plan', [(0.,False), (.448,True), (.456,False)])
def test_arm_replay_only_plans_at_measured_selected_lift(pair, monkeypatch, measured, should_plan):
    from home_perception.geometry.plane import Plane
    from tools.diagnostics import replay_s3_grasp as replay
    folder, report, observations, profile, _ = pair
    report['records'][0]['coordinates']['model_joint_positions_si']['s2_lift_joint'] = measured
    cam_from_base = np.linalg.inv(profile['base_from_optical'])
    plane = Plane(cam_from_base[:3,:3] @ [0.,0.,1.], (cam_from_base @ [0.,0.,profile['ground_z_m']+.75,1.])[:3], 0., 1.)
    meta = json.loads((folder/'metadata.json').read_text())
    meta['aligned_depth_intrinsics'] = meta['color_intrinsics']
    monkeypatch.setattr(replay, 'load_scene', lambda _: (meta, np.ones((4,4)), None, plane))
    obs = [dict(o,mask_sha256='mask',support_height_m=.02) for o in observations]
    monkeypatch.setattr(replay, 'grounding_observations', lambda *args, **kwargs: (obs, [np.ones((4,4), bool)]*2))
    monkeypatch.setattr(replay, 'backproject', lambda *args: None)
    candidate = dict(source='graspnet',translation_m=[0.,0.,.5],rotation_matrix=np.eye(3).tolist(),depth_m=.02,width_m=.03)
    monkeypatch.setattr(replay.graspnet, 'candidates', lambda *args,**kwargs:dict(refined_grasps=[candidate]))
    called = []
    def plan_at_one_height(arm,*args,installed_left_jaw,observed_scene):
        assert installed_left_jaw is None and observed_scene is None
        called.append(arm.lift_m)
        return {'status':'CHECKED'}
    monkeypatch.setattr(replay, 'plan_pick', plan_at_one_height)
    scene = dict(capture_id=folder.name, session_id='synthetic', calibration_id=profile['calibration_id'],
                 frame='base_link', source_sha256=report['snapshot_export']['source_sha256'],
                 scene_boxes_m=[[[0., 0., 0.], [.1, .1, .1]]])
    out = replay.replay(folder,'detections.json','cup','left',robot_report=report,scene_boxes=scene,candidate_source='graspnet',nominal_jaw=True)
    assert called == ([measured] if should_plan else [])
    assert out['status'] == ('CHECKED' if should_plan else 'LIFT_ALIGNMENT_AND_NEW_CAPTURE_REQUIRED')


@pytest.mark.parametrize('network_width,first_fit', [(.03, 'graspnet'), (.09, 'rule'), (None, 'rule')])
def test_manual_stop_prioritizes_network_then_same_mask_rule(pair, monkeypatch, network_width, first_fit):
    from tools.diagnostics import replay_s3_grasp as replay
    folder, _, observations, _, _ = pair
    meta = json.loads((folder/'metadata.json').read_text())
    meta['aligned_depth_intrinsics'] = meta['color_intrinsics']
    from home_perception.geometry.plane import Plane
    plane = Plane(np.array([0., 0., -1.]), np.array([0., 0., .6]), 0., 1.)
    monkeypatch.setattr(replay, 'load_scene', lambda _: (meta, np.ones((4, 4)), None, plane))
    obs = [dict(o, mask_sha256='mask', support_height_m=.02) for o in observations]
    masks = [np.zeros((4, 4), bool), np.ones((4, 4), bool)]
    target_cloud = np.zeros((16, 3))
    def observations_with_clouds(snapshot,detections,meta,depth,plane,source,clouds,**kwargs):
        clouds[1] = target_cloud
        return obs,masks
    monkeypatch.setattr(replay, 'grounding_observations', observations_with_clouds)
    tcp = 'graspnet_origin_plus_depth_to_pad_tip'
    candidate = dict(translation_m=[0., 0., .5], rotation_matrix=np.eye(3).tolist(), depth_m=.02, width_m=.03)
    def network(depth, intr, supplied_masks, index, *args, section_points):
        assert index == 1 and supplied_masks is masks and section_points is target_cloud
        rows = [] if network_width is None else [dict(candidate, source='graspnet', width_m=network_width)]
        return dict(tcp_convention=tcp, candidate_reason='network', refined_grasps=rows)
    monkeypatch.setattr(replay.graspnet, 'candidates', network)
    monkeypatch.setattr(replay, 'flat_candidates', lambda *args, **kw: dict(tcp_convention=tcp, candidate_reason='flat',
                                                                         refined_grasps=[dict(candidate, source='rule')]))
    out = replay.replay(folder, 'detections.json', 'cup', 'left', nominal_jaw=True)
    assert out['candidate_source'] == 'graspnet-first' and out['first_width_fit_source'] == first_fit
    assert out['group']['mask_sha256'] == 'mask' and out['group']['capture_id'] == folder.name
    assert out['status'] == 'CAMERA_CANDIDATES_ONLY' and out['hardware_commands'] == 0



@pytest.mark.parametrize('changed', ['capture_id', 'session_id', 'source_sha256', 'calibration_id', 'frame'])
def test_relocated_base_cannot_reuse_previous_obstacles(pair, changed):
    from tools.diagnostics.replay_s3_grasp import capture_scene_boxes
    folder, report, _, profile, _ = pair
    current = dict(capture_id=folder.name, session_id='synthetic', calibration_id=profile['calibration_id'],
                   frame='base_link', source_sha256=report['snapshot_export']['source_sha256'])
    boxes = dict(current, scene_boxes_m=[[[0., 0., 0.], [.1, .1, .1]]])
    boxes[changed] = 'previous-stop'
    with pytest.raises(ValueError, match='belong to this capture'):
        capture_scene_boxes(boxes, current)


@pytest.mark.parametrize('surface_height,needs_lift', [(.20, False), (.75, True)])
def test_anchored_saved_depth_builds_scene_without_supplied_boxes(pair, tmp_path, surface_height, needs_lift):
    """Synthetic matched exposure with actual segmentation/point-cloud and arm-planner entry points."""
    pytest.importorskip('open3d')
    from PIL import Image
    import hashlib
    from tools.diagnostics import replay_s3_grasp as replay

    original, report, _, profile, geometry = pair
    row = report['records'][0]
    intr = replay.read(replay.CONFIG/'camera_device.observed.json')['sdk_calibration']['color_intrinsics']
    h, w = intr['height'], intr['width']
    v, u = np.indices((h, w))
    camera = np.asarray(profile['base_from_optical'])
    rays = np.stack(((u-intr['ppx'])/intr['fx'], (v-intr['ppy'])/intr['fy'], np.ones_like(u)), axis=-1)
    base_rays = rays @ camera[:3, :3].T
    mask = (abs(u-w//2) < 23) & (abs(v-h//2) < 7)
    table_z = profile['ground_z_m'] + surface_height
    depth = (np.where(mask, table_z+.012, table_z) - camera[2, 3]) / base_rays[:, :, 2]
    snap = row['snapshot']
    meta = json.loads((original/'metadata.json').read_text())
    meta.update(color_intrinsics=intr, aligned_depth_intrinsics=intr, depth_scale_m=.001, alignment='depth to color')
    dest = tmp_path/'rendered'
    report['snapshot_export'] = save_frame(dest, (np.zeros((h,w,3),np.uint8), np.rint(depth*1000).astype(np.uint16), meta), row)
    folder = dest/snap['capture_id']
    mask_path = folder/'mask_000.png'
    Image.fromarray(mask.astype('uint8')*255).save(mask_path)
    sha = hashlib.sha256(mask_path.read_bytes()).hexdigest()
    detection = dict(id=0, label='marker', box_xyxy=[w//2-23,h//2-7,w//2+23,h//2+7],
                     segmentation=dict(mask_file=mask_path.name, pixels=int(mask.sum()), sha256=sha, predicted_iou=.99))
    det_file = folder/'detections.json'
    det_file.write_text(json.dumps(dict(schema_version=2, segmentation='fixture', coordinate_frame=meta['coordinate_frame'],
                                        source_snapshot=str(folder), captured_at_utc=meta['captured_at_utc'],
                                        robot_transform=None, source_sha256=report['snapshot_export']['source_sha256'],
                                        detections=[detection])))
    out = replay.replay(folder, det_file, 'marker', candidate_source='flat-rule', robot_report=report,
                        left_pad_record=json.loads((ROOT/'config/home_assistant/left_pad_gap.observed.json').read_text()))
    assert out['observed_scene']['mask_sha256'] == sha and out['observed_scene']['session_id'] == 'synthetic'
    assert out['observed_scene']['source_sha256'] == report['snapshot_export']['source_sha256']
    assert out['observed_scene']['support_boxes_m']
    if needs_lift:
        assert out['status'] == 'LIFT_ALIGNMENT_AND_NEW_CAPTURE_REQUIRED' and 'path' not in out and 'group' not in out
    else:
        assert out['path']['attempts'][0]['checking'] == 'pick_pregrasp'
    assert not out['motion_authorized'] and out['hardware_commands'] == 0


def test_pre_home_capture_cannot_be_reinterpreted_after_physical_recalibration(pair):
    from tools.diagnostics import review_s3_capture as tool
    folder, report, observations, active, geometry = pair
    historical = json.loads((ROOT/tool.LEGACY_PROFILE).read_text())
    ref = report['reference_evidence']
    row = report['records'][0]
    snap = row['snapshot']
    ref['calibration_id'] = snap['calibration_id'] = historical['calibration_id']
    snap['assembly_revision'] = historical['assembly_revision']
    # Reconstruct the old observation with its own raw coordinate reference.
    for side in s2.ARMS:
        snap['raw'][side] = dict(historical['arms'][side]['reference_raw'])
        snap['raw'][side]['elbow'] += 48
    row['coordinates'] = dev.ObservationSession(historical,geometry,ref).ingest(snap,now_s=snap['received_s'],clock_id=snap['clock_id'])
    original = deepcopy(row['coordinates'])
    with pytest.raises(ValueError,match='unsupported capture calibration'):
        tool.review(folder,report,observations,active,geometry,'cup','left')
    assert row['coordinates'] == original
