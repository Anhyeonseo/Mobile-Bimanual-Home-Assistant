import hashlib
import json
from types import SimpleNamespace as NS
import threading

import numpy as np
import pytest

from home_robot_tasks.assistant.world import WorldStore
from home_robot_tasks.furniture_navigation import FurnitureNavigationLayer,visible_free_boxes,transformed_voxels
from test_furniture_geometry import scene


def fixture_layer(tmp_path,store=None):
    from tools.diagnostics.review_s4_sensor_capture import endpoint_obstacles
    points=np.array([[x,y,6] for x in range(8,13) for y in range(8,13)])
    voxels=np.vstack((points,[[8,8,10]]))  # another upper object shares a table column
    grid=np.zeros((30,30),int);grid[9,9]=100
    doc=dict(width=30,height=30,origin=[0.,0.],resolution=.05,data=grid.ravel().tolist())
    raw=json.dumps(doc).encode();(tmp_path/'occupancy.json').write_bytes(raw)
    poly=[[-.06,-.08],[.06,-.08],[.06,.08],[-.06,.08]]
    rgbd,_=endpoint_obstacles(voxels,.05,grid.shape,[0.,0.],.05,bottom=.1,top=1.1)
    np.savez_compressed(tmp_path/'baseline.npz',voxels=voxels,feasible_by_yaw=np.ones((4,30,30),bool),
        rgbd_obstacles=rgbd,uncertain_centers=np.zeros((30,30),bool),origin_xy_m=[0.,0.],resolution_m=.05,
        navigation_footprint_xy_m=poly,clearance_m=.03)
    config=dict(baseline_sha256=hashlib.sha256((tmp_path/'baseline.npz').read_bytes()).hexdigest(),map_sha256=hashlib.sha256(raw).hexdigest(),
        voxel_size_m=.05,reference_geometry=((points+.5)*.05).tolist(),reference_revision='r',furniture_id='table',
        body=dict(footprint_xy_m=poly,clearance_m=.03,height_above_floor_m=1.1),slabs={str(i):[poly] for i in range(22)})
    (tmp_path/'binding.json').write_text(json.dumps(config))
    return FurnitureNavigationLayer(tmp_path,store or WorldStore())


def exposure(t=1.,depth_mm=1000,scan=False,dx=.5):
    _,raw,meta,tf=scene();raw[:]=depth_mm;tf[:2,3]=[.5,.5]
    matrix=np.eye(4);matrix[0,3]=dx
    return dict(saved_location=dict(identity_verified=True,reference_revision='r',furniture_id='table',
        capture_id=str(t),observed_s=t,matrix=matrix.tolist()),native=raw,metadata=meta,camera_to_map=tf,
        scan=dict(observed_s=t,angle_min=0.,angle_increment=np.pi/180,ranges=[2.]*360) if scan else None,
        lidar_xy_yaw=[.1,.1,0.],capture_id=str(t),observed_s=t,now_s=t)


def test_depth_holes_occlusion_and_cropped_box_never_clear():
    _,_,meta,tf=scene();tf[:2,3]=[.5,.5]
    boxes=[([.45,.45,.3],[.5,.5,.35])]
    for depth in (np.zeros((240,320)),np.full((240,320),.4)):
        assert not visible_free_boxes(boxes,depth,meta['color_intrinsics'],tf)[0]
    assert visible_free_boxes(boxes,np.ones((240,320)),meta['color_intrinsics'],tf)[0]
    assert not visible_free_boxes([([8.,8.,.3],[8.05,8.05,.35])],np.ones((240,320)),meta['color_intrinsics'],tf)[0]


def test_absence_or_one_frame_cannot_erase_old_cells_and_other_objects_survive(tmp_path):
    layer=fixture_layer(tmp_path)
    layer.observe(**exposure(1))
    assert not layer.state['retired_voxels']
    layer.observe(**exposure(2,depth_mm=0))
    layer.observe(**exposure(3))
    assert not layer.state['retired_voxels']
    layer.observe(**exposure(4))
    assert len(layer.state['retired_voxels'])==len(layer.owned)
    b=layer.compile()
    assert b['area']['rgbd_obstacles'][8,8]  # upper object was not table-owned
    assert b['map']['data'][9*30+9]==100  # no laser evidence, keep its own layer
    assert not layer.state['retired_lidar_cells']


def test_laser_clear_requires_repeated_laser_evidence_and_persists_after_restart(tmp_path):
    store=WorldStore(tmp_path/'world.sqlite');layer=fixture_layer(tmp_path,store)
    layer.observe(**exposure(1,scan=True));layer.observe(**exposure(2,scan=True))
    assert [9,9] in layer.state['retired_lidar_cells']
    state=layer.state.copy();store.close()
    store=WorldStore(tmp_path/'world.sqlite');again=FurnitureNavigationLayer(tmp_path,store)
    assert again.state==state and not again.votes
    assert again.compile()['map']['data'][9*30+9]==0
    store.close()


def test_stale_replayed_wrong_identity_and_stale_laser_cannot_clear(tmp_path):
    layer=fixture_layer(tmp_path)
    e=exposure();e['now_s']=10
    with pytest.raises(ValueError):layer.observe(**e)
    e=exposure();e['saved_location']['identity_verified']=False
    with pytest.raises(ValueError):layer.observe(**e)
    for t in (1,2):
        e=exposure(t,scan=True);e['scan']['observed_s']=t-1;layer.observe(**e)
    assert not layer.state['retired_lidar_cells']
    with pytest.raises(ValueError):layer.observe(**exposure(2))


def test_intermediate_furniture_position_is_retained_until_seen_empty(tmp_path):
    layer=fixture_layer(tmp_path);layer.observe(**exposure(1,depth_mm=0,dx=.3))
    previous=set(map(tuple,layer.state['history_voxels']))
    layer.observe(**exposure(2,depth_mm=0,dx=.8))
    assert previous<=set(map(tuple,layer.state['history_voxels']))
    assert not layer.state['retired_voxels']


def test_identity_transform_does_not_enlarge_voxel():
    assert transformed_voxels({(-3,5,6)},np.eye(4),.05)=={(-3,5,6)}


def test_new_map_invalidates_active_nav2_path_and_zeros_cached_command():
    from home_robot_tasks.nav2_trial import Nav2SensorBridge
    b=Nav2SensorBridge.__new__(Nav2SensorBridge);b.map_revision='old';b.command_s=1;b.twist=(.1,0.,0.);b.active='move';b.cancelling=False
    b.global_grid_geometry=dict(shape=(2,2),resolution=.05,origin=(0.,0.))
    update=dict(revision='new',fused=np.zeros((2,2)),map=dict(resolution=.05,origin=[0.,0.]))
    b.state=NS(lock=threading.RLock(),navigation_update=update)
    calls=[];b.grid_message=lambda *args:'new-map';b.map_pub=NS(publish=lambda m:calls.append(m));b.follow=NS(cancel=lambda key:calls.append(key))
    b.apply_navigation_update()
    assert calls==['new-map','move'] and b.cancelling and b.map_revision=='new'
    assert b.command_s==-1 and b.twist==(0.,0.,0.) and b.path is None
    b.apply_navigation_update();assert len(calls)==2


def test_lidar_registration_preserves_capture_time_and_masked_rays():
    from navigation_trial_web import registered_lidar_sweep
    scan=dict(observed_s=123.45,angle_min=0.,angle_increment=.01,ranges=[1.,float('nan'),2.])
    result=registered_lidar_sweep(scan,[1.,2.,np.pi/2],dict(base_from_lidar_translation_m=[.2,.1,.16],yaw_rad=.3))
    assert result['observed_s']==123.45 and scan['observed_s']==123.45
    assert result['lidar_xy_yaw']==pytest.approx([.9,2.2,np.pi/2+.3])
    assert result['ranges']==[1.,None,2.] and np.isnan(scan['ranges'][1])


@pytest.mark.parametrize('origin', [[0.,0.],[-.23392979314933,-.1664437019647],[.03,.01]])
@pytest.mark.parametrize('yaw', [0.,.63,2.1])
def test_cached_projection_matches_original_at_grid_edges(origin,yaw):
    from tools.diagnostics.review_s4_sensor_capture import footprint_kernel,voxel_body_centres
    from home_robot_tasks.furniture_navigation import cached_voxel_centres
    cells=np.array([[-8,-4],[-5,-3],[-1,0],[0,0],[2,8],[14,17],[30,30]])
    polygons=[[[-.15,-.1],[.19,-.08],[.2,.11],[-.14,.13]]]
    res=.05;origin=np.asarray(origin);center=np.full(2,.5)*res
    phase=origin+(np.floor((center-origin)/res)+.5)*res-center
    kernel=footprint_kernel(polygons,res,yaw,.03,obstacle_size_m=res,lattice_offset_xy_m=phase)
    expected=voxel_body_centres(cells,polygons,voxel_size=res,grid_shape=(20,22),origin=origin,resolution=res,yaw=yaw,clearance=.03)
    assert np.array_equal(cached_voxel_centres(cells,kernel,(20,22),origin,res),expected)


def test_cached_compile_matches_uncached_and_rejects_changed_binding(tmp_path):
    from tools.diagnostics.prepare_furniture_navigation import prepare_geometry_cache
    store=WorldStore();layer=fixture_layer(tmp_path,store)
    layer.observe(**exposure(1,scan=True));layer.observe(**exposure(2,scan=True))
    original=layer.compile()
    prepare_geometry_cache(tmp_path)
    cached=FurnitureNavigationLayer(tmp_path,store).compile()
    assert original['map_bytes']==cached['map_bytes']
    assert np.array_equal(original['fused'],cached['fused'])
    assert all(np.array_equal(v,cached['area'][k]) for k,v in original['area'].items())
    manifest=json.loads((tmp_path/'geometry-cache.json').read_text());manifest['binding_sha256']='wrong'
    (tmp_path/'geometry-cache.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError,match='cache binding'):FurnitureNavigationLayer(tmp_path,store)


def verified_exposure(t=1.,dx=.5):
    e=exposure(t,depth_mm=0,dx=dx)
    e['saved_location'].update(method='appearance_geometry_reidentification',
        evidence_capture_ids=[str(t-2),str(t-1),str(t)])
    return e


def test_confirmed_relocation_replaces_only_owned_shape_and_persists(tmp_path):
    store=WorldStore(tmp_path/'world.sqlite');layer=fixture_layer(tmp_path,store)
    e=verified_exposure();store.save_record('furniture_last_location','table',e['saved_location'])
    layer.observe(**e)
    assert set(map(tuple,layer.state['retired_voxels']))==layer.owned
    before=layer.compile()
    assert before['area']['rgbd_obstacles'][8,8]  # separate object survives
    assert not before['area']['rgbd_obstacles'][10,10]  # old table gone despite cropped/invalid depth
    assert before['map']['data'][9*30+9]==0  # registered old table laser contribution retired
    store.close();store=WorldStore(tmp_path/'world.sqlite');again=FurnitureNavigationLayer(tmp_path,store)
    assert np.array_equal(before['fused'],again.compile()['fused'])
    # Move a second time: no historical duplicate remains, including intermediate pose.
    again.observe(**verified_exposure(3,dx=.8))
    active=transformed_voxels(again.owned,again.state['active_matrix'],again.size)
    assert set(map(tuple,again.state['history_voxels']))-set(map(tuple,again.state['retired_voxels']))==active
    store.close()


def test_legacy_saved_confirmed_identity_migrates_old_phantoms(tmp_path):
    store=WorldStore();layer=fixture_layer(tmp_path,store)
    layer.observe(**exposure(depth_mm=0))
    assert not layer.state['retired_voxels']
    store.save_record('furniture_last_location','table',verified_exposure()['saved_location'])
    again=FurnitureNavigationLayer(tmp_path,store)
    assert set(map(tuple,again.state['retired_voxels']))==again.owned
    revision=again.state['revision']
    assert FurnitureNavigationLayer(tmp_path,store).state['revision']==revision


def test_unconfirmed_or_duplicate_identity_cannot_retire_geometry(tmp_path):
    layer=fixture_layer(tmp_path)
    e=verified_exposure();e['saved_location']['evidence_capture_ids']=['x','x','x']
    layer.observe(**e);assert not layer.state['retired_voxels']


def test_repeated_laser_free_space_does_not_need_upper_camera_floor_view(tmp_path):
    layer=fixture_layer(tmp_path)
    layer.observe(**exposure(1,depth_mm=0,scan=True))
    assert not layer.state['retired_lidar_cells']
    layer.observe(**exposure(2,depth_mm=0,scan=True))
    assert [9,9] in layer.state['retired_lidar_cells']
    assert not layer.state['retired_voxels']  # laser cannot erase upper geometry


def test_relocation_does_not_remove_unowned_lidar_or_new_overlap(tmp_path):
    layer=fixture_layer(tmp_path)
    # Separate occupancy lies outside the registered table's source region.
    layer.grid[2,2]=100
    layer.observe(**verified_exposure(dx=.05))
    b=layer.compile()
    assert b['map']['data'][2*30+2]==100
    assert b['map']['data'][9*30+9]==0  # old contribution removed
    assert b['fused'][9,9]==100  # same cell still occupied by current table
    assert b['fused'][8,8]==100  # unrelated upper object preserved


def omitted_edge_layer(tmp_path,store):
    fixture_layer(tmp_path,store)
    archive=tmp_path/'baseline.npz';data=dict(np.load(archive))
    data['voxels']=np.vstack((data['voxels'],[[7,10,6]]))
    data['rgbd_obstacles'][10,7]=True
    np.savez_compressed(archive,**data)
    path=tmp_path/'binding.json';config=json.loads(path.read_text())
    config['baseline_sha256']=hashlib.sha256(archive.read_bytes()).hexdigest()
    path.write_text(json.dumps(config))
    return FurnitureNavigationLayer(tmp_path,store)


def ownership_evidence(tmp_path,cell=(7,10,6)):
    evidence=dict(method='original_rgbd_surface_membership',furniture_id='table',reference_revision='r',
        binding_sha256=hashlib.sha256((tmp_path/'binding.json').read_bytes()).hexdigest(),
        sources=[dict(view=7)],members=[dict(voxel=list(cell),views=[7])])
    (tmp_path/'ownership.json').write_text(json.dumps(evidence))


def test_original_surface_completion_removes_omitted_edge_and_moves_it_once(tmp_path):
    store=WorldStore(tmp_path/'world.sqlite');old=omitted_edge_layer(tmp_path,store)
    e=verified_exposure();store.save_record('furniture_last_location','table',e['saved_location'])
    old.observe(**e);assert old.compile()['fused'][10,7]==100
    ownership_evidence(tmp_path)
    fixed=FurnitureNavigationLayer(tmp_path,store);result=fixed.compile()
    assert result['fused'][10,7]==0  # previous edge disappears even outside current camera view
    assert result['fused'][10,17]==100  # completed shape exists at its latest location
    assert result['fused'][8,8]==100  # unrelated object at a different height survives
    assert fixed.state['ownership_revision']
    revision=fixed.state['revision'];store.close()
    store=WorldStore(tmp_path/'world.sqlite');again=FurnitureNavigationLayer(tmp_path,store)
    assert again.state['revision']==revision
    assert np.array_equal(result['fused'],again.compile()['fused'])
    again.observe(**verified_exposure(5,dx=.8))
    assert again.compile()['fused'][10,17]==0  # no duplicate at the intermediate location
    store.close()


def test_completed_ownership_without_identity_keeps_old_shape(tmp_path):
    store=WorldStore();layer=omitted_edge_layer(tmp_path,store)
    layer.observe(**exposure(depth_mm=0));ownership_evidence(tmp_path)
    again=FurnitureNavigationLayer(tmp_path,store)
    assert again.compile()['fused'][10,7]==100


def test_ownership_rejects_wrong_binding_or_invented_voxel(tmp_path):
    store=WorldStore();omitted_edge_layer(tmp_path,store);ownership_evidence(tmp_path)
    p=tmp_path/'ownership.json';d=json.loads(p.read_text());d['binding_sha256']='wrong';p.write_text(json.dumps(d))
    with pytest.raises(ValueError,match='binding'):FurnitureNavigationLayer(tmp_path,store)
    ownership_evidence(tmp_path,cell=(29,29,6))
    with pytest.raises(ValueError,match='original observed'):FurnitureNavigationLayer(tmp_path,store)


def test_surface_volume_membership_preserves_distant_or_other_height_cells():
    from tools.diagnostics.prepare_furniture_navigation import observed_surface_cells
    baseline={(0,0,6),(1,0,6),(2,0,6),(0,0,10),(20,0,6)}
    measured=np.array([[.049,.025,.325]])
    result=observed_surface_cells(baseline,measured,measured,.05)
    assert result=={(0,0,6),(1,0,6)}  # both recorded volumes border the measured edge


def test_missing_completion_cannot_resurrect_old_occupancy(tmp_path):
    store=WorldStore();omitted_edge_layer(tmp_path,store);ownership_evidence(tmp_path)
    FurnitureNavigationLayer(tmp_path,store)
    (tmp_path/'ownership.json').unlink()
    with pytest.raises(ValueError,match='ownership evidence missing'):FurnitureNavigationLayer(tmp_path,store)


def test_laser_membership_uses_overlapping_cell_volumes_with_shifted_lattice():
    from home_robot_tasks.furniture_navigation import furniture_laser_membership
    points=np.array([[.425,.425,0],[.625,.425,0],[.625,.625,0],[.425,.625,0]])
    centers=np.array([[.525,.385],[.525,.375],[.525,.30],[.525,.525],[.9,.525]])
    assert furniture_laser_membership(points,centers,.05,.05).tolist()==[True,False,False,True,False]
