from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import numpy as np
import pytest
from test_map_bundle import grid,convert
from home_robot_tasks.assistant.contracts import Context
from home_robot_tasks.assistant.world import WorldStore
from home_robot_tasks.navigation_depth import DepthProfile
from home_robot_tasks.navigation_fusion import TravelEnvelope
from home_robot_tasks.mapping_session import MappingSession
from home_robot_tasks.fetch import InvalidTask
from tools.run.replay_home_scan import replay


def fixture():
    msg=grid();msg.info.origin.position.x=.025
    doc=convert(msg)
    p=DepthProfile('depth','cal',.1,.1,3.,0.,1.,.3,.5,world_frame='map')
    c=Context(doc['map_id'],doc['revision'],'cal','clock')
    e=TravelEnvelope('body',.04,1.)
    pose=np.diag([1.,-1.,-1.,1.]);pose[:3,3]=[.25,.25,1.2]
    return doc,p,c,e,pose


def test_loop_correction_discards_previous_depth_and_archive_readiness():
    doc,p,c,e,pose=fixture();store=WorldStore()
    try:
        mapping=MappingSession(store,c,p,e,session_id='test',name='test',map_document=doc)
        mapping.depth(np.full((10,10),1.25),(10,10,4.5,4.5),pose,observed_s=1.,now_s=1.,
            frame='depth',calibration_id='cal',clock_id='clock',motion_qualified=True)
        assert mapping.volume.cells and mapping.frames==1
        old=mapping.volume.epoch
        mapping.replace_map({**doc,'revision':'corrected'})
        assert mapping.volume.epoch!=old and not mapping.volume.cells and mapping.frames==0
        with pytest.raises(InvalidTask):mapping.finish(now_s=1.,output=Path('/tmp/unused-map-output'))
    finally:store.close()


def test_registered_depth_replay_exports_hash_bound_map_and_archive(tmp_path):
    doc,p,c,e,pose=fixture()
    raw=json.dumps(doc).encode();(tmp_path/'map.json').write_bytes(raw)
    np.save(tmp_path/'depth.npy',np.full((10,10),1.25))
    frame=dict(depth_file='depth.npy',sha256=hashlib.sha256((tmp_path/'depth.npy').read_bytes()).hexdigest(),
        intrinsics=[10,10,4.5,4.5],camera_to_map=pose.tolist(),observed_s=1.,motion_qualified=True)
    manifest=dict(schema_version=1,mode='simulation',frames=[frame],map_file='map.json',map_sha256=hashlib.sha256(raw).hexdigest(),
        context=c.to_dict(),depth_profile=asdict(p),envelope=asdict(e),session_id='replay',name='Synthetic native depth')
    path=tmp_path/'replay.json';path.write_text(json.dumps(manifest))
    result=replay(path,tmp_path/'output')
    assert result['passed'] and result['hardware_commands']==0
    assert (tmp_path/'output/map/source_slam.json').is_file()
    saved=json.loads((tmp_path/'output/map/scan.json').read_text())
    assert saved['navigation_build']['accumulated_3d_volume_persisted']
    assert saved['map_document']['origin']['x']==0
    (tmp_path/'depth.npy').write_bytes(b'changed')
    with pytest.raises(ValueError,match='changed'):replay(path,tmp_path/'bad-output')


def test_rgb_native_depth_and_semantics_export_together(tmp_path):
    doc,p,c,e,pose=fixture()
    raw=json.dumps(doc).encode();(tmp_path/'map.json').write_bytes(raw)
    arrays={'depth':np.full((20,20),1.25),'rgb':np.zeros((20,20,3),dtype=np.uint8),
            'aligned_depth':np.full((20,20),1.25),'masks':np.ones((1,20,20),dtype=bool)}
    semantic=dict(model_revision='recorded-fixture-v1',detections=[{'label':'desk','score':.9,'box_xyxy':[1,1,19,19]}],
        mask_qualities=[.9],camera_to_map=pose.tolist(),intrinsics=[20,20,9.5,9.5],observation_xy_m=[.25,.25])
    for key,array in arrays.items():
        np.save(tmp_path/(key+'.npy'),array)
        if key!='depth':
            semantic[key+'_file']=key+'.npy'
            semantic[key+'_sha256']=hashlib.sha256((tmp_path/(key+'.npy')).read_bytes()).hexdigest()
    frame=dict(depth_file='depth.npy',sha256=hashlib.sha256((tmp_path/'depth.npy').read_bytes()).hexdigest(),
        intrinsics=[20,20,9.5,9.5],camera_to_map=pose.tolist(),observed_s=1.,motion_qualified=True,semantic=semantic)
    manifest=dict(schema_version=1,mode='simulation',frames=[frame],map_file='map.json',map_sha256=hashlib.sha256(raw).hexdigest(),
        context=c.to_dict(),depth_profile=asdict(p),envelope=asdict(e),session_id='rgbd',name='RGB-D scan')
    path=tmp_path/'replay.json';path.write_text(json.dumps(manifest))
    assert replay(path,tmp_path/'output')['passed']
    semantics=json.loads((tmp_path/'output/map/semantics.json').read_text())
    assert semantics['furniture'][0]['category']=='desk'
    assert semantics['rgbd_captures'][0]['model_revision']=='recorded-fixture-v1'
    assert semantics['rgbd_captures'][0]['rgb_sha256']==hashlib.sha256(arrays['rgb'].tobytes()).hexdigest()
