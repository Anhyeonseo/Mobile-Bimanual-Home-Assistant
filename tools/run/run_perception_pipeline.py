#!/usr/bin/env python3
"""Run local Qwen -> SAM -> GraspNet sequentially; no robot or downloads.

Each model runs in a separate process, releasing its CUDA memory before the
next model starts. Reports candidate generation only, never 5DoF feasibility.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.lib.run_manifest import provenance, sha256, write_json


def commands(sample, output, model, checkpoint, prompt, index):
    python = sys.executable
    return [
        ('qwen', [python, str(ROOT / 'tools/diagnostics/qwen3_vl_rgbd.py'),
                  '--image', str(sample / 'color.png'), '--model', str(model),
                  '--device', 'cuda', '--prompt', prompt, '--max-new-tokens', '1024',
                  '--output', str(output / 'qwen')]),
        ('sam', [python, str(ROOT / 'tools/diagnostics/qwen_object_cloud.py'),
                 str(output / 'qwen/detections.json'), '--sample', str(sample),
                 '--device', 'cuda', '--index', str(index), '--output', str(output / 'objects')]),
        ('graspnet', [python, str(ROOT / 'tools/diagnostics/graspnet_candidates.py'),
                      '--objects', str(output / 'objects'), '--checkpoint', str(checkpoint),
                      '--index', str(index), '--output', str(output / 'grasps')]),
    ]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group()
    inputs.add_argument('--sample', type=Path, help='color.png, raw depth.png, workspace_mask.png, meta.mat')
    inputs.add_argument('--recording', type=Path, help='Validated portable recording manifest')
    parser.add_argument('--frame-index', type=int, default=0)
    parser.add_argument('--model', type=Path, default=ROOT / 'artifacts/models/qwen3-vl-4b')
    parser.add_argument('--checkpoint', type=Path, default=ROOT / 'artifacts/models/checkpoint-rs.tar')
    parser.add_argument('--prompt', required=True)
    parser.add_argument('--index', type=int, default=0, help='Explicit zero-based Qwen detection to process')
    parser.add_argument('--output', type=Path, required=True, help='New run directory')
    parser.add_argument('--stage-timeout-s', type=int, default=180)
    parser.add_argument('--plan-only', action='store_true', help='Print commands without loading models/writing outputs')
    args = parser.parse_args(argv)
    if args.index < 0 or args.frame_index < 0 or not 1 <= args.stage_timeout_s <= 3600 or not args.prompt.strip():
        parser.error('nonnegative indices, nonempty prompt and timeout in 1..3600 required')
    if args.frame_index and not args.recording:
        parser.error('--frame-index requires --recording')
    output = args.output.resolve()
    sample = ((output / 'sample') if args.recording else
              (args.sample or ROOT / 'artifacts/datasets/graspnet-example').resolve())
    model, checkpoint = args.model.resolve(), args.checkpoint.resolve()
    stages = commands(sample, output, model, checkpoint, args.prompt, args.index)
    if args.plan_only:
        print(json.dumps({'mode': 'plan_only', 'recording_export': str(args.recording) if args.recording else None,
                          'network_downloads': False, 'hardware_commands': 0,
                          'stages': [{'name': name, 'command': command} for name, command in stages]},
                         ensure_ascii=False, indent=2))
        return 0
    try:
        output.mkdir(parents=True, exist_ok=False)
    except OSError as error:
        parser.exit(2, f'Cannot create new output directory: {error}\n')
    report = {'schema_version': 1, 'mode': 'offline_perception', 'status': 'RUNNING',
              'hardware_commands': 0, 'physical_task_completed': False,
              'robot_feasibility_checked': False, 'network_downloads': False,
              'prompt': args.prompt, 'selected_detection_index': args.index,
              'stages': [], 'provenance': provenance()}
    environment = os.environ.copy()
    environment.update(HF_HUB_OFFLINE='1', TRANSFORMERS_OFFLINE='1', HF_DATASETS_OFFLINE='1')
    started = time.monotonic()
    try:
        import torch
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA is not visible to this process; run on the GPU host outside the sandbox')
        report['gpu'] = torch.cuda.get_device_name(0)
        if args.recording:
            from tools.lib.rgbd_recording import export_graspnet, validate_recording
            report['recording'] = validate_recording(args.recording)
            export_graspnet(args.recording, sample, args.frame_index)
        files = [sample / name for name in ('color.png', 'depth.png', 'workspace_mask.png', 'meta.mat')]
        files += [model / 'model.safetensors', checkpoint]
        report['assets_sha256'] = {str(path): sha256(path) for path in files}
        for name, command in stages:
            entry = {'name': name, 'command': command, 'log': name + '.log'}
            report['stages'].append(entry)
            write_json(output / 'report.json', report)
            print(f'Running {name}; log: {output / entry["log"]}', flush=True)
            stage_start = time.monotonic()
            with (output / entry['log']).open('w', encoding='utf-8') as log:
                try:
                    completed = subprocess.run(command, cwd=ROOT, env=environment, stdout=log,
                                               stderr=subprocess.STDOUT, timeout=args.stage_timeout_s)
                finally:
                    entry['wall_seconds'] = time.monotonic() - stage_start
            entry['exit_code'] = completed.returncode
            if completed.returncode:
                raise RuntimeError(f'{name} failed with exit code {completed.returncode}; see {entry["log"]}')
            if name == 'qwen':
                detections = json.loads((output / 'qwen/detections.json').read_text(encoding='utf-8'))
                report['detections'] = detections['detections']
                if not detections['detections']:
                    report['status'] = 'NO_DETECTIONS'
                    break
                if args.index >= len(detections['detections']):
                    raise ValueError('selected detection index outside Qwen results')
            elif name == 'sam':
                objects = json.loads((output / 'objects/objects.json').read_text(encoding='utf-8'))
                report['objects'] = objects['objects']
                if not any(obj['status'] == 'OK' and obj['index'] == args.index for obj in objects['objects']):
                    report['status'] = 'MASK_REJECTED'
                    break
                for name_in_source, expected in objects['input_sha256'].items():
                    if sha256(sample / name_in_source) != expected:
                        raise ValueError('sample changed during SAM processing')
            else:
                grasp = json.loads((output / 'grasps/grasps.json').read_text(encoding='utf-8'))
                report['candidate_counts'] = {key: grasp[key] for key in
                                              ('generated', 'target_associated', 'collision_and_empty_filtered', 'saved')}
                report['status'] = 'CANDIDATES_READY' if grasp['saved'] else 'NO_GRASP_CANDIDATES'
        if any(sha256(path) != report['assets_sha256'][str(path)] for path in files):
            raise ValueError('input/model changed during inference')
    except KeyboardInterrupt:
        report['status'] = 'CANCELLED'
        report['error'] = 'interrupted_by_user'
    except Exception as error:
        report['status'] = 'FAILED'
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        report['wall_seconds'] = time.monotonic() - started
        write_json(output / 'report.json', report)
    print(json.dumps({'status': report['status'], 'report': str(output / 'report.json')}, ensure_ascii=False))
    return 130 if report['status'] == 'CANCELLED' else 1 if report['status'] == 'FAILED' else 0


if __name__ == '__main__':
    raise SystemExit(main())
