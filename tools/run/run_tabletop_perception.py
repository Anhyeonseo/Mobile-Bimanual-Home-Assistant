#!/usr/bin/env python3
"""Qwen -> SAM -> mask geometry on saved snapshots; no robot or downloads.

Unlabelled smoke run: reports counts and observations, never accuracy. Results
stay in the camera frame. Detected boxes must pass a crop check before use. Qwen runs in its own process per image, releasing its
CUDA memory before SAM starts.
"""
import argparse
import dataclasses
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from home_perception.contracts import NOT_FOUND, ObjectObservation, REJECTED
from home_perception.detect import qwen_vl
from home_perception.detect.verify import (crop_box, missing_objects, parse_answer, resize_for_model,
                                           split_verified)
from home_perception.io.snapshot import load_snapshot, snapshot_folders
from tools.lib.run_manifest import provenance, write_json

KOREAN = {'cup': '컵', 'remote control': '리모컨', 'calculator': '계산기',
          'marker pen': '마커펜', 'utility knife': '커터칼'}
ABSENT_TARGET = 'remote control'


def prompt_for(objects):
    return ', '.join(KOREAN[name] for name in objects) + '을(를) 각각 찾아서 물체마다 별도의 박스를 그려줘.'


def absent_prompt():
    return KOREAN[ABSENT_TARGET] + '을(를) 찾아줘.'


def observe(snapshot, detection, mask, score, plane):
    from home_perception.geometry.mask_cloud import measure
    result = measure(mask, snapshot.depth_m(), snapshot.intrinsics, plane, score)
    item = ObjectObservation(
        capture_id=snapshot.capture_id, captured_at_utc=snapshot.captured_at_utc,
        frame_id=snapshot.frame_id, label=detection.label, box_xyxy=detection.box_xyxy,
        detector=detection.detector, detector_revision=detection.revision, detector_score=detection.score,
        source_sha256=snapshot.source_sha256, status=result.status,
        mask_quality=score, mask_pixels=int(mask.sum()),
        valid_depth_fraction=result.valid_depth_fraction, point_count=result.point_count,
        cluster_fraction=result.cluster_fraction,
        center_xyz_m=result.center_xyz_m, obb=result.obb, support_height_m=result.support_height_m,
        point_semantics='centre of visible mask surface; not mass centre or grasp pose',
        failure=result.failure, reason=result.reason)
    return item, result.points


def verify(folder, entries, args, environment):
    """Crop every (snapshot, detection) and collect one answer per crop."""
    import cv2
    folder.mkdir(parents=True, exist_ok=True)
    jobs = []
    for number, (snapshot, detection) in enumerate(entries):
        crop = folder / f'crop_{number:03}.jpg'
        cv2.imwrite(str(crop), resize_for_model(crop_box(snapshot.bgr, detection.box_xyxy)))
        jobs.append({'crop': str(crop), 'name': detection.prompt_name})
    (folder / 'jobs.json').write_text(json.dumps(jobs, ensure_ascii=False), encoding='utf-8')
    with (folder / 'verify.log').open('w', encoding='utf-8') as log:
        subprocess.run([sys.executable, str(ROOT / 'tools/diagnostics/qwen_verify_crops.py'), str(folder / 'jobs.json'),
                        '--model', str(args.model.resolve()), '--device', args.device,
                        '--output', str(folder / 'answers.json')],
                       cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT,
                       timeout=args.stage_timeout_s, check=True)
    return json.loads((folder / 'answers.json').read_text(encoding='utf-8'))


def rejected_record(snapshot, detection):
    return dataclasses.asdict(ObjectObservation(
        capture_id=snapshot.capture_id, captured_at_utc=snapshot.captured_at_utc, frame_id=snapshot.frame_id,
        label=detection.label, box_xyxy=detection.box_xyxy, detector=detection.detector,
        detector_revision=detection.revision, source_sha256=snapshot.source_sha256, status=REJECTED,
        failure=NOT_FOUND, reason='verification_failed'))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=ROOT / 'data/tabletop_rgbd_20261101')
    parser.add_argument('--model', type=Path, default=ROOT / 'artifacts/models/qwen3-vl-4b')
    parser.add_argument('--output', type=Path, default=ROOT / 'tmp/tabletop-perception')
    parser.add_argument('--device', choices=['cuda', 'cpu'], default='cuda')
    parser.add_argument('--stage-timeout-s', type=int, default=180)
    parser.add_argument('--robustness', action='store_true', help='Repeat detection on perturbed images')
    parser.add_argument('--plan-only', action='store_true', help='Print prompts without loading models')
    args = parser.parse_args(argv)
    folders = snapshot_folders(args.dataset)
    if not folders:
        parser.error('no snapshots found')
    if args.plan_only:
        print(json.dumps({'mode': 'plan_only', 'hardware_commands': 0, 'captures': [
            {'capture': path.name, 'prompt': prompt_for(load_snapshot(path).objects)} for path in folders]},
            ensure_ascii=False, indent=2))
        return 0
    output = args.output.resolve()
    try:
        output.mkdir(parents=True, exist_ok=False)
    except OSError as error:
        parser.exit(2, f'Cannot create new output directory: {error}\n')
    environment = os.environ | {'HF_HUB_OFFLINE': '1', 'TRANSFORMERS_OFFLINE': '1', 'HF_DATASETS_OFFLINE': '1'}
    for name, value in environment.items():
        if name.endswith('_OFFLINE'):
            os.environ[name] = value
    report = {'schema_version': 1, 'mode': 'offline_perception_smoke', 'status': 'RUNNING',
              'labelled': False, 'hardware_commands': 0, 'physical_task_completed': False,
              'robot_transform': None, 'dataset': str(args.dataset), 'captures': [],
              'provenance': provenance()}
    started = time.monotonic()
    try:
        import torch
        report['device'] = args.device
        if args.device == 'cuda':
            if not torch.cuda.is_available():
                raise RuntimeError('CUDA is not visible to this process; run on the GPU host outside the sandbox')
            report['gpu'] = torch.cuda.get_device_name(0)
        report.update(_run(args, folders, output, environment, report))
        report['status'] = 'COMPLETED'
    except KeyboardInterrupt:
        report.update(status='CANCELLED', error='interrupted_by_user')
    except Exception as error:
        report.update(status='FAILED', error=f'{type(error).__name__}: {error}')
    finally:
        report['wall_seconds'] = time.monotonic() - started
        write_json(output / 'report.json', report)
    print(json.dumps({'status': report['status'], 'report': str(output / 'report.json')}, ensure_ascii=False))
    return 130 if report['status'] == 'CANCELLED' else 1 if report['status'] == 'FAILED' else 0


def _run(args, folders, output, environment, report):
    import cv2
    import numpy as np
    from home_perception.geometry.plane import backproject, fit_plane
    snapshots = [load_snapshot(path) for path in folders]
    by_id = {item.capture_id: item for item in snapshots}
    detect = lambda image, prompt, folder: qwen_vl.detect(
        image, prompt, folder, args.model.resolve(), environment, args.stage_timeout_s, args.device)
    detections = {item.capture_id: detect(item.folder / 'color.png', prompt_for(item.objects),
                                          output / item.capture_id / 'qwen') for item in snapshots}
    absent = []
    for item in snapshots:
        if ABSENT_TARGET not in item.objects:
            found = detect(item.folder / 'color.png', absent_prompt(), output / item.capture_id / 'qwen_absent')
            absent.append({'capture': item.capture_id, 'target': ABSENT_TARGET, 'found': found,
                           'boxes': [box.box_xyxy for box in found]})
    robustness = []
    if args.robustness:
        from home_perception.eval.robustness import PERTURBATIONS, compare, perturb
        for item in snapshots:
            for kind in PERTURBATIONS:
                folder = output / item.capture_id / kind
                folder.mkdir(parents=True)
                cv2.imwrite(str(folder / 'color.png'), perturb(item.bgr, kind))
                found = detect(folder / 'color.png', prompt_for(item.objects), folder / 'qwen')
                robustness.append({'capture': item.capture_id, 'perturbation': kind,
                                   **compare([box.box_xyxy for box in detections[item.capture_id]],
                                             [box.box_xyxy for box in found])})
    # Detection always returns something, so nothing below trusts a box before this.
    jobs = [(item, box) for item in snapshots for box in detections[item.capture_id]]
    for item, box in jobs:
        box.prompt_name = next((english for english, korean in KOREAN.items()
                                if korean in box.label or english in box.label.lower()), box.label)
    answers = verify(output / 'verify', jobs, args, environment)
    by_capture = {item.capture_id: [] for item in snapshots}
    for (item, _), answer in zip(jobs, answers):
        by_capture[item.capture_id].append(answer)
    rejected = {}
    for item in snapshots:
        found = detections[item.capture_id]
        detections[item.capture_id], rejected[item.capture_id] = split_verified(found, by_capture[item.capture_id])
    absent_jobs = [(row, box) for row in absent for box in row.pop('found')]
    for row, box in absent_jobs:
        box.prompt_name = row['target']
    absent_answers = verify(output / 'verify_absent', [(by_id[row['capture']], box) for row, box in absent_jobs],
                            args, environment) if absent_jobs else []
    for row in absent:
        replies = [answer for (owner, _), answer in zip(absent_jobs, absent_answers) if owner is row]
        row['verified'] = any(parse_answer(answer) for answer in replies)
        row['outcome'] = 'false_detection' if row['verified'] else NOT_FOUND
    from tools.lib.sam_rgbd import MODEL_ID, MODEL_REVISION, SAMMasks
    report.update(sam_model=MODEL_ID, sam_revision=MODEL_REVISION)
    segmenter = SAMMasks(args.device)
    entries = []
    for item in snapshots:
        folder = output / item.capture_id
        depth = item.depth_m()
        plane = fit_plane(backproject(depth, item.intrinsics, depth > 0))
        found = detections[item.capture_id]
        masks = segmenter.segment(item.bgr, [box.box_xyxy for box in found])[0] if found else []
        records, drawn, overlay = [], [], item.bgr.copy()
        for number, (detection, (mask, score)) in enumerate(zip(found, masks)):
            observation, points = observe(item, detection, mask, float(score), plane)
            cv2.imwrite(str(folder / f'mask_{number:03}.png'), mask.astype(np.uint8) * 255)
            color = {'accepted': (0, 220, 0), 'held': (0, 200, 255), 'rejected': (0, 0, 255)}[observation.status]
            overlay[mask] = (.5 * overlay[mask] + .5 * np.array(color)).astype(np.uint8)
            x1, y1 = int(detection.box_xyxy[0]), int(detection.box_xyxy[1])
            cv2.putText(overlay, f'{number} {observation.status}', (x1, max(12, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
            records.append(dataclasses.asdict(observation))
            drawn.append((observation, mask, points))
        cv2.imwrite(str(folder / 'masks.jpg'), overlay)
        write_json(folder / 'observations.json', {'plane': plane.to_dict(), 'observations': records})
        entries.append((item, drawn))
        report['captures'].append({'capture': item.capture_id, 'expected_objects': item.objects,
                                   'plane': plane.to_dict(), 'observations': records,
                                   'rejected_detections': [rejected_record(item, box) for box in rejected[item.capture_id]],
                                   'not_found': missing_objects(item.objects, found, KOREAN)})
    from home_perception.viz import write_recording
    write_recording(output / 'tabletop.rrd', entries)
    return {'absent_target': absent, 'robustness': robustness}


if __name__ == '__main__':
    raise SystemExit(main())
