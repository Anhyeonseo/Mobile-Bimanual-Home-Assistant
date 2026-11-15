#!/usr/bin/env python3
"""Offline directional base proposals from preserved cloud + stopped costmaps.

No serial, SSH, torque, motor or live planner access. Historical time is kept;
geometry and navigation previews never grant execution authority.
"""
import argparse,json,hashlib,math,sys,time
from pathlib import Path
import numpy as np
import cv2,yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'tools/diagnostics'),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from tools.diagnostics.run_s3_stationary_lab import model
from home_robot_tasks.assistant.object_alignment import FixedHeightAlignmentPlanner,AlignmentPolicy
from home_robot_tasks.assistant.alignment_contracts import TaskScene
from home_robot_tasks.assistant.contracts import Context,Stamp
from home_robot_tasks.assistant.placement import Check
from home_robot_tasks.assistant.view_geometry import view_check
from home_robot_tasks.alignment_navigation import AlignmentScreen
from home_robot_tasks.stationary_lift import LiftHeightRule
from home_robot_tasks.s4_sensor_frames import load_sensor_frames

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--prior-candidate',type=Path)
    ap.add_argument('--allow-stored-target-preview',action='store_true',help='Offline alternatives only; never enables blind grasp or operational view bypass')
    ap.add_argument('--cloud-review',type=Path,required=True);ap.add_argument('--stopped-review',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False)
    old=json.loads((args.cloud_review/'alignment-scene.json').read_text());st=json.loads((args.stopped_review/'final-status.json').read_text())
    assert st['stop_confirmed']
    pr=json.loads((args.cloud_review/'frame-provenance.json').read_text());meta=json.loads((args.cloud_review/'metadata.json').read_text())
    pts=np.asarray(old['points_map_m']);arm=model(0);start=np.asarray(st['pose'])
    ctx=Context('home',st['navigation_revision'],arm.profile['calibration_id'],pr['observation']['saved']['clock_id']);stamp=Stamp(pr['observation']['observed_s'],ctx,pr['frame']['capture_id'])
    scene=TaskScene('observed-object','observed-support',stamp,old.get('scene_revision',st['navigation_revision']),'stock-so101-release',stamp.capture_id,tuple(old['source_pose']),0.,pts,old['support_height_m'],old['source_sha256'])
    grids=np.load(args.stopped_review/'block-costmaps.npz');gm=json.loads((args.stopped_review/'block-costmaps.json').read_text());cfg=yaml.safe_load((args.stopped_review/'nav2-params.yaml').read_text());body=np.asarray(json.loads(cfg['/s4_nav2_trial/local_costmap/local_costmap']['ros__parameters']['footprint']))
    cache={}
    def free(pose,candidate=None):
        key=tuple(np.round(pose,6))
        if key in cache:return cache[key]
        x,y,yaw=pose;co,si=np.cos(yaw),np.sin(yaw);poly=body@np.array([[co,si],[-si,co]])+[x,y]
        for name in ('global','local'):
            grid=grids[name];m=gm[name];origin=np.array([m['origin']['position'][k] for k in ('x','y')]);uv=np.floor((poly-origin)/m['resolution']).astype(np.int32)
            if np.any(uv<0) or np.any(uv[:,0]>=grid.shape[1]) or np.any(uv[:,1]>=grid.shape[0]):cache[key]=False;return False
            mask=np.zeros(grid.shape,np.uint8);cv2.fillConvexPoly(mask,uv,1)
            if np.any(grid[mask.astype(bool)]>=254):cache[key]=False;return False
        cache[key]=True;return True
    def config(c):return Check(free(c['base_xy_yaw']), 'archived_full_fixed_body_costmap',mode='simulation')
    tfs={v['child']:np.array(v['matrix']) for v in load_sensor_frames(ROOT)['transforms']}
    def view(c):
        x,y,yaw=c['base_xy_yaw'];co,si=np.cos(yaw),np.sin(yaw);M=np.array([[co,-si,0,x],[si,co,0,y],[0,0,1,-arm.profile['ground_z_m']],[0,0,0,1.]])@tfs['color_camera_optical_frame']
        v=view_check(pts,M,meta['color_intrinsics'])
        return Check(v.valid,v.reason,cost=v.cost,mode='simulation')
    checker=AlignmentScreen(start=start,pose_free=free,configuration_check=config,view_check=view,mode='simulation',
        observation_strategy='stored_target' if args.allow_stored_target_preview else 'recapture_at_goal')
    evaluations=[]
    def screen(c):
        checks=checker(c);evaluations.append(dict(candidate=c,checks={k:vars(v) for k,v in checks.items()}));return checks
    seeds=[] if args.prior_candidate is None else [json.loads(args.prior_candidate.read_text())['candidate']]
    planner=FixedHeightAlignmentPlanner({'left':arm},AlignmentPolicy(joint_samples=2048,arm_samples=64,maximum_results=16))
    print('archived start clear',free(start),flush=True);t=time.monotonic()
    result=planner.plan_fixed(scene,current_xy_yaw=start.tolist(),current_lift_m=0.,height_rule=LiftHeightRule.from_profile(arm.profile),screen=screen,now_s=stamp.observed_s,maximum_screened=128,seed_candidates=seeds)
    result.update(elapsed_s=time.monotonic()-t,current_pose=start.tolist(),source_capture=stamp.capture_id,historical_replay=True,hardware_commands=0,motion_authorized=False,executable=False,requires_user_confirmation=True,requires_live_sensor_recheck=True,requires_fresh_capture_before_grasp=not args.allow_stored_target_preview,requires_target_tracking=args.allow_stored_target_preview,support_z_m=scene.support_z_m,source_files={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (args.cloud_review/'alignment-scene.json',args.stopped_review/'final-status.json',args.stopped_review/'block-costmaps.npz')})
    if args.allow_stored_target_preview:
        result['status']='OFFLINE_OBSERVATION_STRATEGY_PROPOSALS' if result['candidates'] else result['status']
        result['stored_target_preview_only']=True
        result['missing_execution_evidence']=['last-visible stopped capture', 'relative localization uncertainty bound', 'full arm and object collision path']
    (out/'evaluations.json').write_text(json.dumps(evaluations,indent=2)+'\n')
    (out/'candidates.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('candidates','source_files')},ensure_ascii=False),flush=True)
    for c in result['candidates'][:3]: print(json.dumps({k:c[k] for k in ('candidate_id','base_xy_yaw','lift_travel_m','contact_direction','cost')},ensure_ascii=False),flush=True)
if __name__=='__main__':main()
