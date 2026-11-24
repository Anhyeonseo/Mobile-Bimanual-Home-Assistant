#!/usr/bin/env python3
"""Real CUDA benchmark on saved RGB-D, with separate reload and cached-model runs."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from tools.diagnostics.replay_s3_grasp import load_scene
from tools.diagnostics.run_s3_stationary_lab import stock_jaw
from home_perception.geometry.plane import backproject
from home_perception.grasp import graspnet
from home_perception.grasp.so101 import candidates as so101_candidates


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts',type=Path,required=True)
    parser.add_argument('--inputs',type=Path,default=ROOT/'output/so101_photo_experiment')
    parser.add_argument('--output',type=Path,default=ROOT/'output/so101_photo_experiment/runtime-comparison.json')
    parser.add_argument('--repeats',type=int,default=5)
    args=parser.parse_args()
    if not 1<=args.repeats<=30:parser.error('repeats must be 1..30')
    try:
        import torch
    except ImportError:
        parser.exit(2,'Use the Python environment with PyTorch and GraspNet PointNet2 installed.\n')
    if not torch.cuda.is_available():
        parser.exit(2,'CUDA unavailable: run from a GPU-enabled terminal and check nvidia-smi.\n')
    if args.output.exists():parser.error('output already exists; select a new --output')
    graspnet.ARTIFACTS=args.artifacts.resolve()
    checkpoint=graspnet.ARTIFACTS/'models/checkpoint-rs.tar'
    jaw=stock_jaw();opening=(jaw.gaps[0],jaw.gaps[-1])
    results=[]

    def timed(fn):
        torch.cuda.synchronize();start=perf_counter()
        result=fn()
        torch.cuda.synchronize()
        return result,1000*(perf_counter()-start)

    original_infer=graspnet.infer
    for number,capture in [(1,'rgbd-99'),(2,'rgbd-103')]:
        snapshot=args.inputs/f'set-{number}'/capture
        meta,depth,scene,plane=load_scene(snapshot)
        mask=np.asarray(Image.open(snapshot/'mask.png'))>0
        intr=meta['aligned_depth_intrinsics']
        points=backproject(depth,intr,mask&(depth>0))
        points=points[plane.height(points)>.003]
        height=float(np.percentile(plane.height(points),95))
        def network():
            return graspnet.candidates(depth,intr,[mask],0,scene,plane,height,opening,
                                      checkpoint=checkpoint,section_points=points)
        def geometric():return so101_candidates(points,plane,opening,limit=32)
        print(f'Set {number}: existing GraspNet (reload each call)',flush=True)
        graspnet.infer=original_infer
        first,first_ms=timed(network)
        reload_times=[timed(network)[1] for _ in range(args.repeats)]

        def prepare_model():
            if hashlib.sha256(checkpoint.read_bytes()).hexdigest()!=graspnet.CHECKPOINT_SHA256:
                raise ValueError('checkpoint hash mismatch')
            model_type,decode,detector=graspnet._baseline()
            net=model_type(input_feature_dim=0,num_view=300,num_angle=12,num_depth=4,
                cylinder_radius=.05,hmin=-.02,hmax_list=[.01,.02,.03,.04],is_training=False)
            net.load_state_dict(torch.load(checkpoint,map_location='cpu',weights_only=True)['model_state_dict'],strict=True)
            return net.cuda().eval(),decode,detector
        (net,decode,detector),load_ms=timed(prepare_model)

        def cached_infer(cloud_points,unused_checkpoint):
            selected=np.random.default_rng(0).choice(len(cloud_points),20000,replace=len(cloud_points)<20000)
            torch.manual_seed(0)
            cloud=torch.as_tensor(cloud_points[selected][None],dtype=torch.float32,device='cuda')
            with torch.inference_mode():
                rows=decode(net({'point_clouds':cloud}))[0].cpu().numpy()
            return rows,detector
        graspnet.infer=cached_infer
        try:
            print(f'Set {number}: cached GraspNet and SO101',flush=True)
            network()  # Warmup excluded from repeat statistics.
            cached_times=[timed(network)[1] for _ in range(args.repeats)]
            geometric()
            geometric_runs=[timed(geometric) for _ in range(args.repeats)]
        finally:
            graspnet.infer=original_infer
        def stats(values):
            return dict(samples_ms=values,median_ms=float(np.median(values)),p95_ms=float(np.percentile(values,95)))
        record=dict(set=number,capture=capture,first_graspnet_call_ms=first_ms,
            cached_model_prepare_ms=load_ms,graspnet_reload=stats(reload_times),
            graspnet_cached=stats(cached_times),so101=stats([v for _,v in geometric_runs]),
            graspnet_counts=first['counts'],so101_counts=geometric_runs[-1][0]['counts'])
        results.append(record)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        report=dict(gpu=torch.cuda.get_device_name(0),torch_version=torch.__version__,repeats=args.repeats,
            timing_scope='backend candidates() calls; CUDA synchronized; same RGB-D, mask, plane and section points',
            excluded='RGB-D loading, plane fitting, detection/segmentation, IK, path planning, hardware',
            caveat='GraspNet includes existing legacy audit and proposal postprocessing; SO101 has no equivalent audit',
            results=results,complete=len(results)==2)
        args.output.write_text(json.dumps(report,indent=2,allow_nan=False))
        print(json.dumps(record,indent=2),flush=True)
        del net
    print(f'Saved: {args.output}')


if __name__=='__main__':main()
