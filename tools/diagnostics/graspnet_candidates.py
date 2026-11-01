#!/usr/bin/env python3
"""Official GraspNet baseline inference, scene collision checks, and target filtering."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import types


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--objects', type=Path, default=Path('tmp/qwen-object-cloud'))
    p.add_argument('--index', type=int, default=0)
    p.add_argument('--checkpoint', type=Path, default=Path('artifacts/models/checkpoint-rs.tar'))
    p.add_argument('--output', type=Path, default=Path('tmp/graspnet-candidates'))
    p.add_argument('--view', action='store_true')
    p.add_argument('--view-only', action='store_true', help='Display saved results without inference')
    p.add_argument('--top-k', type=int, default=20)
    p.add_argument('--focus-radius-m', type=float, help='Sample inputs near the target instead of the whole workspace')
    p.add_argument('--collision-voxel-m', type=float, default=0.01)
    p.add_argument('--stage', choices=['raw', 'target', 'filtered', 'final'], default='final',
                   help='Stage to display; raw=scene, target=SAM-associated, filtered=before NMS')
    args = p.parse_args()
    import numpy as np
    import open3d as o3d
    import torch
    from PIL import Image
    root = Path(__file__).resolve().parents[2]
    baseline = root/'artifacts/graspnet-baseline'
    for part in ['models', 'utils', 'pointnet2']:
        sys.path.insert(0, str(baseline/part))
    # The baseline imports this training-only dependency even during inference.
    # Provide equivalent 1-based nearest-neighbor indices without another CUDA extension.
    knn_module = types.ModuleType('knn_modules')
    def knn(ref, query, k=1):
        return (torch.cdist(query.transpose(1,2), ref.transpose(1,2)).topk(k, largest=False).indices+1).transpose(1,2).contiguous()
    knn_module.knn = knn
    sys.modules['knn_modules'] = knn_module
    # Load geometry submodule without importing dataset/evaluation-only dependencies.
    import importlib.util
    package_path = Path(importlib.util.find_spec('graspnetAPI').origin).parent
    api_package = types.ModuleType('graspnetAPI')
    api_package.__path__ = [str(package_path)]
    sys.modules['graspnetAPI'] = api_package
    from graspnetAPI.grasp import GraspGroup
    out = args.output
    if args.view_only:
        filename = 'grasps.npy' if args.stage == 'final' else f'grasps_{args.stage}.npy'
        grasps = GraspGroup(np.load(out/filename))
    else:
        if args.top_k < 1: raise ValueError('--top-k must be positive')
        from graspnet import GraspNet, pred_decode
        from collision_detector import ModelFreeCollisionDetector
        report = json.loads((args.objects/'objects.json').read_text())
        target = next((x for x in report['objects'] if x['index']==args.index and x['status']=='OK'), None)
        if target is None: raise ValueError('Target has no accepted SAM cloud')
        sample = Path(report['sample'])
        # Reject changed depth/calibration/masks before using their saved cloud.
        for name, sha in report['input_sha256'].items():
            if hashlib.sha256((sample/name).read_bytes()).hexdigest()!=sha:
                raise ValueError(f'Input changed: {name}')
        mask = np.asarray(Image.open(args.objects/target['mask']))>0
        k = np.asarray(report['intrinsic_matrix'])
        scene = o3d.io.read_point_cloud(str(args.objects/'scene.ply'))
        workspace = o3d.io.read_point_cloud(str(args.objects/'workspace.ply'))
        points = np.asarray(workspace.points)
        if args.focus_radius_m:
            focus = np.asarray(o3d.io.read_point_cloud(str(args.objects/target['cloud'])).points).mean(axis=0)
            points = points[np.linalg.norm(points-focus,axis=1)<=args.focus_radius_m]
        if not len(points): raise ValueError('Empty workspace cloud')
        rng = np.random.default_rng(0)
        selected = rng.choice(len(points),20000,replace=len(points)<20000)
        torch.manual_seed(0)
        net = GraspNet(input_feature_dim=0,num_view=300,num_angle=12,num_depth=4,
                       cylinder_radius=0.05,hmin=-0.02,hmax_list=[0.01,0.02,0.03,0.04],is_training=False).cuda().eval()
        checkpoint = torch.load(args.checkpoint,map_location='cpu',weights_only=True)
        net.load_state_dict(checkpoint['model_state_dict'],strict=True)
        start=time.perf_counter()
        with torch.inference_mode():
            end=net({'point_clouds':torch.from_numpy(points[selected].astype('float32')[None]).cuda()})
            raw=pred_decode(end)[0].cpu().numpy()
        seconds=time.perf_counter()-start
        raw=raw[np.isfinite(raw).all(1)]
        grasps=GraspGroup(raw)
        out.mkdir(parents=True,exist_ok=True)
        np.save(out/'grasps_raw.npy',grasps.grasp_group_array)
        generated=len(grasps)
        centers=grasps.translations
        # Association is approximate: center projection inside SAM and near a measured target surface.
        positive=centers[:,2]>0
        uv=np.zeros((len(centers),2),dtype=int)
        uv[positive]=np.rint(centers[positive,:2]/centers[positive,2:3]*[k[0,0],k[1,1]]+[k[0,2],k[1,2]]).astype(int)
        inside=positive & (uv[:,0]>=0)&(uv[:,0]<mask.shape[1])&(uv[:,1]>=0)&(uv[:,1]<mask.shape[0])
        ids=np.flatnonzero(inside); inside[ids]=mask[uv[ids,1],uv[ids,0]]
        from scipy.spatial import cKDTree
        target_points=np.asarray(o3d.io.read_point_cloud(str(args.objects/target['cloud'])).points)
        distances=cKDTree(target_points).query(centers)[0]
        grasps=grasps[inside & (distances<=0.03)]
        associated=len(grasps)
        np.save(out/'grasps_target.npy',grasps.grasp_group_array)
        detector=ModelFreeCollisionDetector(np.asarray(scene.points),voxel_size=args.collision_voxel_m)
        keep=[]
        for i in range(0,len(grasps),32):
            collision,empty=detector.detect(grasps[i:i+32],approach_dist=0.05,collision_thresh=0.01,return_empty_grasp=True)
            keep.extend((~collision & ~empty).tolist())
        if len(grasps):grasps=grasps[np.asarray(keep,dtype=bool)]
        survivors=len(grasps)
        np.save(out/'grasps_filtered.npy',grasps.grasp_group_array)
        if survivors:
            grasps=grasps.nms();grasps.sort_by_score()
        grasps=grasps[:args.top_k]
        out.mkdir(parents=True,exist_ok=True)
        np.save(out/'grasps.npy',grasps.grasp_group_array)
        result={'model':'official GraspNet baseline (RealSense)',
                'baseline_revision': '280c215129f759ed8649cb4e89fc5dfee55f4f80',
                'checkpoint_sha256':hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
                'objects_path':str(args.objects.resolve()),'target':target,'seed':0,
                'inference_seconds':seconds,'generated':generated,'target_associated':associated,
                'collision_and_empty_filtered':survivors,'saved':len(grasps),
                'nms':{'translation_m':0.03,'rotation_degrees':30}, 'top_k':args.top_k,
                'coordinate_frame':report['coordinate_frame'],
                'association':'center projects into SAM mask AND <= 0.03m from observed target',
                'focus_radius_m':args.focus_radius_m,
                'collision':{'scene':'all valid scene depth','voxel_m':args.collision_voxel_m,'threshold':0.01,'approach_m':0.05},
                'grasps':[{'score':float(g.score),'width_m':float(g.width),'depth_m':float(g.depth),
                           'translation_m':g.translation.tolist(),'rotation_matrix':g.rotation_matrix.tolist()} for g in grasps]}
        (out/'grasps.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        from PIL import ImageDraw
        image=Image.open(sample/'color.png').convert('RGBA')
        layer=Image.new('RGBA',image.size)
        draw=ImageDraw.Draw(layer)
        for mesh in grasps.to_open3d_geometry_list():
            vertices=np.asarray(mesh.vertices)
            pixels=vertices[:,:2]/vertices[:,2:3]*[k[0,0],k[1,1]]+[k[0,2],k[1,2]]
            for tri in np.asarray(mesh.triangles):
                if (vertices[tri,2]>0).all():
                    draw.polygon([tuple(x) for x in pixels[tri]],fill=(255,140,0,100),outline=(255,50,0,220))
        Image.alpha_composite(image,layer).convert('RGB').save(out/'grasps.jpg')
        print(json.dumps({k:v for k,v in result.items() if k!='grasps'},ensure_ascii=False,indent=2))
    if args.view or args.view_only:
        filename = 'grasps.npy' if args.stage == 'final' else f'grasps_{args.stage}.npy'
        grasps = GraspGroup(np.load(out/filename))
        print(f'Displaying {args.stage}: {len(grasps)} candidates (no display limit)', flush=True)
        report=json.loads((out/'grasps.json').read_text())
        source=Path(report['objects_path'])
        scene=o3d.io.read_point_cloud(str(source/'scene.ply'))
        target=o3d.io.read_point_cloud(str(source/report['target']['cloud']))
        scene.paint_uniform_color([0.5,0.5,0.5]);target.paint_uniform_color([0,1,0])
        o3d.visualization.draw_geometries([scene,target,*grasps.to_open3d_geometry_list()],window_name='GraspNet target candidates')


if __name__=='__main__': main()
