#!/usr/bin/env python3
"""Real cached models on saved RGB-D, twice; no camera, ROS node or controller."""
import argparse
import json
from pathlib import Path
import shutil
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('snapshot','artifacts-dir','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--camera-mode',choices=('wide','legacy','near'),default='near')
    parser.add_argument('--device',choices=('auto','cpu','cuda'),default='auto')
    parser.add_argument('--request', default='리모컨 집기', help='natural-language pick target in the snapshot')
    parser.add_argument('--candidate-source', choices=('graspnet','so101'), default='so101')
    args=parser.parse_args()
    from home_robot_tasks.pick_target import parse_target, translate_target
    target, prompt = parse_target(args.request)
    prompt = prompt or translate_target(target)
    args.output.mkdir(parents=True,exist_ok=False)
    from tools.lib.pick_compute import PickCompute
    runtime=PickCompute(dict(device=args.device,candidate_source=args.candidate_source,artifacts_dir=str(args.artifacts_dir)))
    try:
        start=time.monotonic();runtime.prepare();preparation=time.monotonic()-start
        def model_ids():
            return (id(runtime.detector.model), id(runtime.detector.segmenter.model),
                    id(runtime.grasp.net) if runtime.grasp is not None else None)
        identity = model_ids()
        results=[]
        for number in (1,2):
            folder=args.output/str(number)/args.snapshot.name
            folder.mkdir(parents=True)
            for name in ('color.png','depth_aligned.png','metadata.json'):
                shutil.copyfile(args.snapshot/name,folder/name)
            start=time.monotonic()
            detection=runtime.detect(str(folder),prompt)
            output = folder/'candidates.json'
            runtime.camera_candidates(str(folder), detection, prompt, args.camera_mode, str(output))
            candidates = json.loads(output.read_text())
            group = candidates.get('group', {})
            proposals = group.get('refined_grasps', [])
            if any(c['source'] != args.candidate_source for c in proposals):
                raise RuntimeError('unexpected candidate source')
            if identity != model_ids():
                raise RuntimeError('models were replaced between requests')
            results.append(dict(attempt=number, status=candidates['status'], reason=candidates.get('reason'),
                                candidates=len(proposals), counts=group.get('counts'),
                                seconds=time.monotonic()-start))
        report=dict(request=args.request,prompt=prompt,candidate_source=args.candidate_source,prepare_seconds=preparation,same_model_instances=True,hardware_commands=0,results=results,
                    includes_planning_or_motion=False)
        (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
        print(json.dumps(report,indent=2),flush=True)
        return 0 if all(r['status']=='CAMERA_CANDIDATES_ONLY' and r['candidates'] for r in results) else 2
    finally:runtime.close()


if __name__=='__main__':raise SystemExit(main())
