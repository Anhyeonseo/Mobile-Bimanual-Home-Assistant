"""Portable checked plan. Hashes bind evidence; they do not grant execution."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
from .carry_program import CarryProgram, CarryStage, validate_program_winding
from .carry_trajectory import MotionBounds
from .s2_geometry import digest


def write_json(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_installed_binding(model, profile, range_sha256):
    """Old raw trajectories/jaw goals cannot survive a Home or model change.

    A self-consistent bundle hash only proves its own contents. Also bind it to
    the currently installed calibration, including unchanged-ID edits.
    """
    if model.get('range_sha256') != range_sha256:
        raise ValueError('installed ranges differ from plan; rebuild after calibration')
    key = 'config/home_assistant/arm_ranges.development.json'
    if profile.get('source_files_sha256', {}).get(key) != range_sha256:
        raise ValueError('installed coordinate profile uses stale arm ranges')
    if digest(model['profile']) != digest(profile):
        raise ValueError('installed coordinate calibration differs from plan; replan')


def load_bundle(folder,*,for_execution=False):
    folder=Path(folder);meta=json.loads((folder/'bundle.json').read_text())
    if meta.get('kind')!='stationary_carry_bundle' or meta.get('schema_version')!=1:
        raise ValueError('stationary bundle required')
    expected={'program.json','model.json','proof.json','color.png','depth_aligned.png','metadata.json','mask.png','robot_sweep.png'}
    if set(meta['files_sha256'])!=expected:raise ValueError('incomplete bundle')
    for name,checksum in meta['files_sha256'].items():
        if sha(folder/name)!=checksum:raise ValueError('bundle changed: '+name)
    data=json.loads((folder/'program.json').read_text());proof=json.loads((folder/'proof.json').read_text())
    model=json.loads((folder/'model.json').read_text())
    if proof['program_sha256']!=digest(data) or proof['model_sha256']!=digest(model):
        raise ValueError('native proof not bound to compiled program/model')
    guard=model.get('scene_guard',{})
    if guard.get('kind')=='checked_route_and_payload_depth_v1' and guard.get('program_sha256')!=digest(data):
        raise ValueError('scene depth scope belongs to a different route')
    data['bounds']=MotionBounds(**{k:tuple(v) for k,v in data['bounds'].items()})
    data['stages']=tuple(CarryStage(s['name'],tuple(TimedJointPoint(p['offset_ms'],tuple(p['positions_rad'])) for p in s['points']),s['timing'],s['attached']) for s in data['stages'])
    program=CarryProgram(**data)
    if [s.name for s in program.stages]!=['open','pregrasp','approach','close','probe','lift','carry','hold']:
        raise ValueError('stage order changed')
    for s in program.stages:
        q=np.asarray([p.positions_rad for p in s.points]);ticks=np.asarray([p.offset_ms for p in s.points])
        if (q.shape!=(len(s.points),12) or not 2<=len(q)<=20000 or not np.isfinite(q).all()
                or np.any(q<program.bounds.minimum) or np.any(q>program.bounds.maximum)
                or any(type(p.offset_ms)is not int or p.offset_ms%5 for p in s.points)
                or ticks[0]<20 or ticks[0]>250 or np.any(np.diff(ticks)<=0) or np.any(np.diff(ticks)>100)
                or not np.isclose(s.timing['duration_s'],ticks[-1]/1000)):
            raise ValueError('invalid compiled route')
    if for_execution and (program.hypothetical_lift or proof['native_geometry'].get('status')!='PASSED'
                          or proof.get('simulation_passed') is not True):
        raise ValueError('actual-height native-checked plan required; preview cannot execute')
    if program.calibration_id!=model['profile']['calibration_id']:raise ValueError('model profile changed')
    validate_program_winding(program.stages,model['profile'])
    for name,checksum in program.source_sha256.items():
        if name not in ('color.png','depth_aligned.png','metadata.json') or sha(folder/name)!=checksum:
            raise ValueError('capture identity changed')
    mask=np.asarray(Image.open(folder/'mask.png'))>0
    sweep=np.asarray(Image.open(folder/'robot_sweep.png'))>0
    color=np.asarray(Image.open(folder/'color.png').convert('RGB'))[:,:,::-1].copy()
    md=json.loads((folder/'metadata.json').read_text())
    depth=np.asarray(Image.open(folder/'depth_aligned.png'),dtype=np.uint16).astype(float)*md['depth_scale_m']
    if mask.shape!=depth.shape or sweep.shape!=depth.shape or color.shape[:2]!=depth.shape or not mask.any():raise ValueError('bundle image alignment mismatch')
    return program,model,proof,(color,depth),mask,sweep,md
