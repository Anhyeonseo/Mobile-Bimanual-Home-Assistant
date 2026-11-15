#!/usr/bin/env python3
"""Replay saved RGB-D furniture observations into an isolated result database.

No device connection, live map change, or live memory update. Input capture
hashes and original camera-to-map observations are preserved and checked.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/home_robot_tasks'))


def replay(reference,trial,output):
    import numpy as np
    from home_robot_tasks.furniture_relocation import FurnitureRelocalizer,FurnitureLocationMemory
    from home_robot_tasks.assistant.world import WorldStore
    reference,trial,output=map(Path,(reference,trial,output))
    if output.exists():raise ValueError('new replay output directory required')
    frames=[json.loads(l) for l in (trial/'frames.jsonl').read_text().splitlines()]
    rows=[json.loads(l) for f in trial.glob('*Z-*.jsonl') for l in f.read_text().splitlines()]
    observations={r['capture_id']:r for r in rows if r.get('event')=='furniture_observation'}
    metadata=json.loads((trial/'metadata.json').read_text())
    matcher=FurnitureRelocalizer(reference);output.mkdir(parents=True)
    store=WorldStore(output/'replay-only.sqlite');memory=FurnitureLocationMemory(matcher.reference,store,'offline-replay')
    results=[]
    try:
        for frame in frames:
            file=trial/frame['file']
            if not file.exists():continue
            if hashlib.sha256(file.read_bytes()).hexdigest()!=frame['sha256']:
                raise ValueError('recorded capture hash mismatch')
            ob=observations[frame['capture_id']]
            with np.load(file,allow_pickle=False) as raw:
                start=time.monotonic()
                proposal=matcher.locate(raw['color_bgr'],raw['native_depth'],metadata,ob['camera_to_map'])
                elapsed=time.monotonic()-start
            stamp=ob['observed_s']
            # Replay time is explicit and cannot be passed off as live evidence.
            saved=memory.observe(proposal,capture_id=frame['capture_id'],observed_s=stamp,
                                 now_s=stamp,localization_qualified=True)
            results.append(dict(file=frame['file'],capture_id=frame['capture_id'],sha256=frame['sha256'],
                observed_s=stamp,camera_to_map=ob['camera_to_map'],proposal=proposal,
                saved=saved is not None,processing_s=elapsed,original_live_proposal=ob['proposal']))
        saved=memory.saved
    finally:store.close()
    store=WorldStore(output/'replay-only.sqlite')
    try:restored=FurnitureLocationMemory(matcher.reference,store,'offline-restart').saved
    finally:store.close()
    proposals=[r['proposal'] for r in results if r['proposal']]
    report=dict(mode='offline_recorded_replay',live_memory_updated=False,navigation_map_updated=False,
        captures=len(results),matched=len(proposals),original_live_matches=sum(bool(r['original_live_proposal']) for r in results),
        saved_in_isolated_replay=saved is not None,reloaded_identically=saved is not None and restored==saved,
        results=results)
    if proposals:
        centers=np.array([p['anchor_xyz_m'] for p in proposals]);report.update(
            anchor_median_m=np.median(centers,axis=0).tolist(),
            anchor_spread_m=float(np.max(np.linalg.norm(centers-np.median(centers,axis=0),axis=1))),
            inliers_range=[min(p['inliers'] for p in proposals),max(p['inliers'] for p in proposals)],
            fit_rms_range_m=[min(p['rms_m'] for p in proposals),max(p['rms_m'] for p in proposals)])
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return {k:v for k,v in report.items() if k!='results'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference',type=Path,required=True)
    p.add_argument('--trial',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(json.dumps(replay(a.reference,a.trial,a.output),ensure_ascii=False))
