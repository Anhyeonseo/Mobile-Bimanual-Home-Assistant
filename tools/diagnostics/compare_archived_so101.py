#!/usr/bin/env python3
"""Compare a hash-bound archived GraspNet result with fresh SO101 proposals.

This is an output replay comparison, never a GraspNet runtime benchmark.
Both sets of poses are remeasured against the same raw target cloud. Historical
GraspNet preprocessing/selection cannot be reproduced from final candidates.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.diagnostics.replay_s3_grasp import load_scene
from tools.diagnostics.run_s3_stationary_lab import stock_jaw
from home_perception.geometry.plane import Plane, backproject
from home_perception.grasp.so101 import candidates
from home_perception.grasp.handoff import observed_contact_line
from home_perception.grasp.trial_contact_offset import apply_trial_offset
from home_robot_tasks.jaw_closure import closure_plan


def evaluate(group, points, jaw):
    rows = []
    for original in group['refined_grasps']:
        c = copy.deepcopy(original)
        section = observed_contact_line(c, points)
        c['closing_section'] = section
        if section['status'] == 'VISIBLE_SPAN':
            c['width_m'] = section['high_m'] - section['low_m']
        plan = closure_plan(jaw, c)
        rotation = np.asarray(c['rotation_matrix'])
        tip = np.asarray(c['translation_m']) + c['depth_m']*rotation[:, 0]
        if section['status'] == 'VISIBLE_SPAN':
            c['fixed_contact_optical_m'] = (tip+section['low_m']*rotation[:, 1]).tolist()
            c['moving_contact_optical_m'] = (tip+section['high_m']*rotation[:, 1]).tolist()
        c['common_closure_check'] = plan
        rows.append(c)
    return rows


def panel(rgb, candidates, intr, title):
    canvas = Image.new('RGB', (rgb.width, rgb.height+55), '#20252c')
    canvas.paste(rgb, (0, 55)); draw = ImageDraw.Draw(canvas)
    draw.text((12, 8), title, fill='white')
    draw.text((12, 27), 'Top returned pose | red: fixed / cyan: moving | no IK', fill='white')
    if candidates and 'fixed_contact_optical_m' in candidates[0]:
        c = candidates[0]
        def pixel(p):
            x,y,z=p
            return (x/z*intr['fx']+intr['ppx'], y/z*intr['fy']+intr['ppy']+55)
        a,b = pixel(c['fixed_contact_optical_m']),pixel(c['moving_contact_optical_m'])
        draw.line([a,b],fill='yellow',width=3)
        for (x,y),color in ((a,'red'),(b,'cyan')):
            draw.ellipse((x-5,y-5,x+5,y+5),fill=color)
    return canvas


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot',type=Path,required=True)
    parser.add_argument('--archive',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    old=json.loads(args.archive.read_text()); group=old['group']
    if old['candidate_source'] != 'graspnet':
        raise ValueError('GraspNet-only archive required')
    for name,digest in old['source_sha256'].items():
        if Path(name).name != name:
            raise ValueError('invalid snapshot file name')
        assert hashlib.sha256((args.snapshot/name).read_bytes()).hexdigest()==digest
    assert hashlib.sha256((args.snapshot/'mask.png').read_bytes()).hexdigest()==group['mask_sha256']
    meta,depth,_,_=load_scene(args.snapshot)
    p=old['support_plane']
    plane=Plane(np.array(p['normal']),np.array(p['point_m']),p['rms_m'],p['inlier_fraction'])
    mask=np.asarray(Image.open(args.snapshot/'mask.png'))>0
    points=backproject(depth,meta['aligned_depth_intrinsics'],mask&(depth>0))
    points=points[plane.height(points)>.003]
    jaw=stock_jaw(); limits=(jaw.gaps[0],jaw.gaps[-1])
    times=[]
    for _ in range(30):
        start=perf_counter(); new=candidates(points,plane,limits,limit=32)
        times.append(1000*(perf_counter()-start))
    offset=group.get('trial_contact_offset',{}).get('requested_m',0.)
    transform=json.loads((ROOT/'config/home_assistant/s2_coordinates.development.json').read_text())['base_from_optical']
    adjusted=apply_trial_offset(new,points,plane,transform,offset,limits)
    groups={'graspnet_archive':evaluate(group,points,jaw), 'so101':evaluate(adjusted,points,jaw)}
    summary=dict(mode='archived_output_comparison',capture_id=old['capture_id'],input_hashes_verified=True,
        candidate_limit=32,trial_offset_m=offset,shared_evaluation_points=len(points),
        graspnet_rerun=False,graspnet_runtime_ms=None,so101_generation_ms_median=float(np.median(times)),
        so101_generation_ms_p95=float(np.percentile(times,95)),repeats=30,
        timing_scope='SO101 candidate generation only; excludes offset, shared evaluation and preprocessing',
        archived_counts=group['counts'],archived_selection=group['proposal_selection'],
        so101_counts=new['counts'],so101_selection=new['proposal_selection'],
        limitations=['archived GraspNet used cleaned points; fresh SO101 uses raw above-support points',
            'common raw-cloud contact measurement and closure check applied to both final pose sets',
            'historically rejected GraspNet candidates cannot be recovered',
            'scores are not comparable probabilities', 'no IK, full finger sweep, path or physical test'],
        hardware_commands=0,results={})
    for name,rows in groups.items():
        top=rows[0] if rows else None
        summary['results'][name]=dict(returned=len(rows),
            common_closure_pass=sum(c['common_closure_check']['status']=='BOUNDED_CLOSE' for c in rows),
            top_span_mm=None if top is None else 1000*top['width_m'],
            top_entry_gap_mm=None if top is None else 1000*top['common_closure_check'].get('entry_gap_m',0.))
    args.output_dir.mkdir(parents=True,exist_ok=False)
    (args.output_dir/'comparison.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
    (args.output_dir/'remeasured_candidates.json').write_text(json.dumps(groups,indent=2,allow_nan=False))
    (args.output_dir/'archive.json').write_bytes(args.archive.read_bytes())
    with Image.open(args.snapshot/'color.png') as image:
        rgb=image.convert('RGB')
    panels=[panel(rgb,groups[name],meta['aligned_depth_intrinsics'],title) for name,title in
        [('graspnet_archive','GraspNet archived + 30 mm trial offset'),('so101','SO101 new + 30 mm trial offset')]]
    combined=Image.new('RGB',(rgb.width*2,panels[0].height))
    for index,p in enumerate(panels):combined.paste(p,(index*rgb.width,0))
    combined.save(args.output_dir/'comparison.png')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
