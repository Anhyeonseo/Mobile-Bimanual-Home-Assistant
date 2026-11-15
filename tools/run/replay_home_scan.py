#!/usr/bin/env python3
"""Import registered native depth captures; save raw, body-height and 3D maps.

Input poses must already belong to the supplied SLAM map revision. Does not
register frames, invent unknown space, open hardware, or execute navigation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks.assistant.contracts import Context,Stamp,document
from home_robot_tasks.assistant.perception import TransformSample
from home_robot_tasks.semantic_mapping import RGBDFurnitureMapper
from home_robot_tasks.assistant.world import WorldStore
from home_robot_tasks.navigation_depth import DepthProfile
from home_robot_tasks.navigation_fusion import TravelEnvelope
from home_robot_tasks.mapping_session import MappingSession


def replay(manifest_path,output):
    manifest_path=Path(manifest_path).resolve();root=manifest_path.parent
    if manifest_path.stat().st_size>2_000_000:raise ValueError('bounded scan manifest required')
    data=json.loads(manifest_path.read_text())
    if (data.get('schema_version')!=1 or data.get('mode') not in ('replay','simulation')
            or not isinstance(data.get('frames'),list) or not 1<=len(data['frames'])<=2048):
        raise ValueError('versioned bounded registered replay required')
    def path(name):
        result=(root/name).resolve()
        if not result.is_relative_to(root) or not result.is_file():raise ValueError('capture path outside replay directory')
        return result
    map_path=path(data['map_file'])
    if map_path.stat().st_size>2_000_000:raise ValueError('bounded map required')
    raw=map_path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=data['map_sha256']:raise ValueError('SLAM map changed since pose registration')
    context=Context.parse(data['context']);profile=DepthProfile(**data['depth_profile'])
    output=Path(output)
    if output.exists():raise ValueError('new output directory required')
    output.mkdir(parents=True)
    store=WorldStore(output/'world.db')
    try:
        session=MappingSession(store,context,profile,TravelEnvelope(**data['envelope']),
            session_id=data['session_id'],name=data['name'],map_document=json.loads(raw),mode=data['mode'])
        last=None
        for frame in data['frames']:
            source=path(frame['depth_file'])
            if source.stat().st_size>1280*720*8+65536:raise ValueError('bounded native depth file required')
            raw=source.read_bytes()
            if hashlib.sha256(raw).hexdigest()!=frame['sha256']:raise ValueError('depth capture changed')
            image=np.load(source,allow_pickle=False,mmap_mode="r")
            session.depth(image,frame['intrinsics'],frame['camera_to_map'],observed_s=frame['observed_s'],
                now_s=frame['observed_s'],frame=profile.frame,calibration_id=profile.calibration,
                clock_id=context.clock_id,motion_qualified=frame['motion_qualified'])
            if frame.get('semantic') is not None:
                semantic=frame['semantic']
                def array(key):
                    source=path(semantic[key+'_file'])
                    if source.stat().st_size>32_000_000:raise ValueError('bounded semantic image required')
                    if hashlib.sha256(source.read_bytes()).hexdigest()!=semantic[key+'_sha256']:
                        raise ValueError('semantic capture changed')
                    return np.load(source,allow_pickle=False,mmap_mode="r")
                rgb,aligned,masks=array('rgb'),array('aligned_depth'),array('masks')
                detections=semantic['detections'];qualities=semantic['mask_qualities']
                if (not isinstance(detections,list) or len(detections)>64 or
                        masks.shape!=(len(detections),*rgb.shape[:2]) or
                        not isinstance(qualities,list) or len(qualities)!=len(detections)):
                    raise ValueError('registered semantic detections required')
                # Replay recorded model outputs through the same checked adapter;
                # this does not claim that inference was run on this machine.
                class RecordedDetector:
                    def detect(self,image,prompt):return detections,None
                class RecordedMasks:
                    def segment(self,image,boxes):return list(zip(masks,qualities)),None
                c=Context(context.map_id,session.scan.get(session.session_id)['map_document']['revision'],
                          context.calibration_id,context.clock_id,context.frame_id)
                stamp=Stamp(frame['observed_s'],c,f'rgbd-{session.frames}')
                transform=TransformSample(stamp,'color_camera_optical_frame',c.frame_id,
                    semantic['camera_to_map'],'simulation' if data['mode']=='simulation' else 'measured')
                mapper=RGBDFurnitureMapper(store,c,RecordedDetector(),RecordedMasks(),
                    model_revision=semantic['model_revision'])
                session.rgbd(mapper,rgb,aligned,semantic['intrinsics'],stamp=stamp,
                    transform=transform,xy_m=semantic['observation_xy_m'],now_s=frame['observed_s'])
            last=frame['observed_s']
        result=session.finish(now_s=last,output=output/'map')
        report=dict(passed=True,frames=session.frames,mode=data['mode'],hardware_commands=0,
            physical_map_verified=False,whole_home_coverage_verified=False,map=result)
        (output/'report.json').write_text(document(report)+'\n')
        return report
    finally:store.close()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=replay(a.manifest,a.output);print(json.dumps({k:v for k,v in result.items() if k!='map'}))
if __name__=='__main__':main()
