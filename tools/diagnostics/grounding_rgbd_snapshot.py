#!/usr/bin/env python3
"""Run Grounding DINO and camera-coordinate conversion on a saved D415 capture."""
import argparse
import json
import hashlib
from datetime import datetime, timezone
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.lib.grounding_rgbd import GroundingRGBD


def recorded_snapshot(args):
    """Reuse native-depth registration and the existing snapshot detector."""
    import cv2
    import numpy as np
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root/'ros2_ws/src/home_robot_tasks'))
    from home_robot_tasks.recorded_semantics import register_native_depth
    report = json.loads(args.recorded_report.read_text())
    labels = json.loads((args.household_profile/'household-labels.json').read_text())
    if hashlib.sha256(args.recorded_report.read_bytes()).hexdigest()!=labels['source_report_sha256']:
        raise ValueError('recorded semantic report does not match household profile')
    if report['context']!=labels['context'] or report['map_sha256']!=labels['source_map_sha256']:
        raise ValueError('recorded map context mismatch')
    frame = report['frames'][args.view]
    capture = frame['capture']; source = Path(capture['source_path'])
    if hashlib.sha256(source.read_bytes()).hexdigest()!=capture['sha256']:
        raise ValueError('recorded RGB-D source hash mismatch')
    if args.support_id and args.support_id not in {x['id'] for x in labels['entities']}:
        raise ValueError('unknown household support')
    metadata = json.loads((source.parent/'metadata.json').read_text())
    with np.load(source, allow_pickle=False) as raw:
        rgb = raw['color_bgr'].copy()
        depth = register_native_depth(raw['native_depth'], metadata)
    args.output.mkdir(parents=True,exist_ok=False)
    if not cv2.imwrite(str(args.output/'color.png'),rgb):raise OSError('image write failed')
    if not cv2.imwrite(str(args.output/'depth_aligned.png'),np.rint(depth/metadata['depth_scale_m']).astype(np.uint16)):
        raise OSError('depth write failed')
    snapshot = dict(metadata,coordinate_frame='color_camera_optical_frame',
        alignment='depth to color: native pixel centres, no filling',
        aligned_depth_intrinsics=metadata['color_intrinsics'],
        captured_at_utc=datetime.fromtimestamp(capture['color_frame']['timestamp_ms']/1000,timezone.utc).isoformat(),
        color_frame=capture['color_frame'],depth_frame=capture['depth_frame'])
    (args.output/'metadata.json').write_text(json.dumps(snapshot,indent=2)+'\n')
    return args.output, frame, labels


def export_recorded_packet(args, detection, frame, labels):
    capture = frame['capture']
    stamp = dict(observed_s=capture['color_time_s'],context=labels['context'],
                 capture_id='recorded-'+capture['sha256'][:24])
    import cv2
    from tools.lib.household_semantics import ObjectCropReviewer
    reviewer=ObjectCropReviewer(Path(__file__).resolve().parents[2]/'artifacts/models/huggingface', device=args.device)
    detection['detections']=reviewer.verify(cv2.imread(str(args.output/'color.png')),detection['detections'])
    (args.output/'verified-detections.json').write_text(json.dumps(detection,indent=2)+'\n')
    packet = dict(request_id=args.request_id,mode='recorded',stamp=stamp,
        transform=dict(stamp=stamp,source_frame='color_camera_optical_frame',target_frame=labels['context']['frame_id'],
                       matrix=frame['camera_to_map'],source='recorded'),
        report={k:detection[k] for k in ('schema_version','coordinate_frame','detections')},
        evidence=dict(rgb_sha256=detection['source_sha256']['color.png'],depth_sha256=detection['source_sha256']['depth_aligned.png']),
        support_id=args.support_id,support_verified=False)
    (args.output/'object-capture.json').write_text(json.dumps(packet,ensure_ascii=False,indent=2)+'\n')
    (args.output/'provenance.json').write_text(json.dumps(dict(source=capture,
        semantic_report_sha256=labels['source_report_sha256'],map_sha256=labels['source_map_sha256'],
        mode='recorded',restamped_as_current=False,hardware_commands=0),indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path, nargs="?")
    parser.add_argument("--recorded-report", type=Path)
    parser.add_argument("--household-profile", type=Path)
    parser.add_argument("--view", type=int)
    parser.add_argument("--request-id")
    parser.add_argument("--support-id")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--prompt", required=True, help="English object description")
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--box-threshold", type=float, default=0.30)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument("--sam", action="store_true", help="Segment boxes with SAM 2.1 and sample depth inside masks")
    args = parser.parse_args()
    if args.recorded_report:
        if args.snapshot or any(v is None for v in (args.household_profile,args.view,args.request_id,args.output)):
            parser.error('recorded replay requires profile, view, request-id, output, and no snapshot')
        snapshot, frame, labels = recorded_snapshot(args)
    elif args.snapshot:
        snapshot = args.snapshot
    else:
        parser.error('snapshot or recorded report required')
    detector = GroundingRGBD(args.device, args.box_threshold, args.text_threshold, use_sam=args.sam)
    result, _, _ = detector.analyze_snapshot(snapshot, args.prompt)
    if args.recorded_report:
        export_recorded_packet(args,result,frame,labels)


if __name__ == "__main__":
    main()
