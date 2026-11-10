from dataclasses import asdict,replace
import json
from pathlib import Path
import numpy as np
from PIL import Image
import pytest

from home_robot_tasks.carry_bundle import sha,write_json,load_bundle
from home_robot_tasks.s2_geometry import digest
from test_carry_execution import program


def fixture(folder,*,hypothetical=False,simulation=True):
    Image.fromarray(np.full((12,16,3),120,np.uint8)).save(folder/'color.png')
    Image.fromarray(np.full((12,16),500,np.uint16)).save(folder/'depth_aligned.png')
    for name in ('mask.png','robot_sweep.png'):Image.fromarray(np.full((12,16),255,np.uint8)).save(folder/name)
    write_json(folder/'metadata.json',{'depth_scale_m':.001})
    p=replace(program(),hypothetical_lift=hypothetical,source_sha256={n:sha(folder/n) for n in ('color.png','depth_aligned.png','metadata.json')})
    profile=json.loads((Path(__file__).resolve().parents[1]/'config/home_assistant/s2_coordinates.development.json').read_text())
    profile['calibration_id']=p.calibration_id
    model={'profile':profile}
    data=asdict(p);write_json(folder/'program.json',data);write_json(folder/'model.json',model)
    write_json(folder/'proof.json',dict(program_sha256=digest(data),model_sha256=digest(model),
        native_geometry={'status':'PASSED'},simulation_passed=simulation))
    write_json(folder/'bundle.json',dict(kind='stationary_carry_bundle',schema_version=1,
        files_sha256={p.name:sha(p) for p in folder.iterdir()}))


def test_bundle_checks_source_and_geometry_proof_before_hardware(tmp_path):
    fixture(tmp_path);p,*_=load_bundle(tmp_path,for_execution=True)
    assert p.candidate_id=='grasp'
    (tmp_path/'color.png').write_bytes(b'changed')
    with pytest.raises(ValueError,match='bundle changed'):load_bundle(tmp_path,for_execution=True)


@pytest.mark.parametrize('kwargs',[{'hypothetical':True},{'simulation':False}])
def test_preview_or_failed_rehearsal_is_never_executable(tmp_path,kwargs):
    fixture(tmp_path,**kwargs)
    load_bundle(tmp_path)
    with pytest.raises(ValueError,match='native-checked'):load_bundle(tmp_path,for_execution=True)


def test_route_scoped_observation_bound_cannot_be_reused_for_another_program(tmp_path):
    fixture(tmp_path)
    model=json.loads((tmp_path/'model.json').read_text())
    model['scene_guard']={'kind':'checked_route_and_payload_depth_v1','program_sha256':'another-route'}
    write_json(tmp_path/'model.json',model)
    proof=json.loads((tmp_path/'proof.json').read_text());proof['model_sha256']=digest(model)
    write_json(tmp_path/'proof.json',proof)
    manifest=json.loads((tmp_path/'bundle.json').read_text())
    for name in ('model.json','proof.json'):manifest['files_sha256'][name]=sha(tmp_path/name)
    write_json(tmp_path/'bundle.json',manifest)
    with pytest.raises(ValueError,match='different route'):load_bundle(tmp_path,for_execution=True)
