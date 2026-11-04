#!/usr/bin/env python3
"""Report figures for P0-P1 from saved runs: data, depth and plane, object cloud, detection, grasps.

Reads existing outputs only (no models). Every figure states what it is not:
camera-frame geometry, dev captures, no ground truth.
"""
import argparse
import json
from pathlib import Path
import sys

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from home_perception.eval.robustness import perturb
from home_perception.geometry.mask_cloud import clean_cloud, measure, oriented_box
from home_perception.geometry.plane import backproject, fit_plane
from home_perception.grasp.refine import parts
from home_perception.io.snapshot import load_snapshot, snapshot_folders

plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False, 'font.size': 10})


def rgb(bgr):
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def data_overview(dataset, out):
    folders = snapshot_folders(dataset)
    fig, axes = plt.subplots(2, 5, figsize=(16, 5.4))
    for ax, folder in zip(axes.ravel(), folders):
        snap = load_snapshot(folder)
        ax.imshow(rgb(snap.bgr))
        valid = (snap.depth_raw > 0).mean() * 100
        ax.set_title(f'{folder.name}\n유효 깊이 {valid:.1f}%', fontsize=9)
        ax.axis('off')
    fig.suptitle('촬영 데이터 10세트 (D415, 640×480, 고정 카메라)')
    fig.tight_layout()
    fig.savefig(out / 'p0_data_overview.png', dpi=130)
    plt.close(fig)


def depth_and_plane(snap, plane, out):
    depth = snap.depth_m()
    valid = depth > 0
    heights = np.full(depth.shape, np.nan)
    heights[valid] = plane.height(backproject(depth, snap.intrinsics, valid))
    on_plane = np.abs(heights) < .005
    overlay = rgb(snap.bgr).copy()
    overlay[on_plane] = (overlay[on_plane] * .45 + np.array([40, 200, 80]) * .55).astype(np.uint8)
    overlay[~valid] = (overlay[~valid] * .3 + np.array([255, 0, 0]) * .7).astype(np.uint8)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    axes[0].imshow(rgb(snap.bgr)); axes[0].set_title('컬러')
    shown = np.where(valid, depth, np.nan)
    im = axes[1].imshow(shown, cmap='turbo', vmin=.25, vmax=1.0)
    axes[1].set_title(f'정렬된 깊이 (m), 검정=측정 실패 {100 - valid.mean() * 100:.1f}%')
    axes[1].set_facecolor('black')
    fig.colorbar(im, ax=axes[1], fraction=.046)
    axes[2].imshow(overlay)
    axes[2].set_title(f'RANSAC 책상면 (초록, ±5mm)  RMS {plane.rms_m * 1000:.2f}mm\n'
                      f'빨강=깊이 없음  카메라–책상 {abs(plane.normal @ plane.point) * 1000:.0f}mm')
    for ax in axes:
        ax.axis('off')
    fig.tight_layout()
    fig.savefig(out / 'p1_depth_plane.png', dpi=130)
    plt.close(fig)


def box_edges(obb):
    centre, rotation, extent = (np.asarray(obb[k]) for k in ('center_m', 'rotation', 'extent_m'))
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    corners = centre + (signs * extent / 2) @ rotation.T
    return [(corners[a], corners[b]) for a in range(8) for b in range(a + 1, 8)
            if np.sum(signs[a] != signs[b]) == 1]


def equal_axes(ax, points, margin=1.3):
    """Same scale on all axes, sized by the 2-98 percentile so stray depth spikes do not shrink the object."""
    low, high = np.percentile(points, 2, axis=0), np.percentile(points, 98, axis=0)
    centre, span = (low + high) / 2, (high - low).max() / 2 * margin
    for setter, c in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), centre):
        setter(c - span, c + span)


def object_cloud(snap, plane, mask, label, out, name):
    depth = snap.depth_m()
    valid = mask & (depth > 0)
    raw = backproject(depth, snap.intrinsics, valid)
    above = raw[plane.height(raw) > .003]
    import open3d as o3d
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(above)).voxel_down_sample(.002)
    filtered, _ = cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.)
    filtered = np.asarray(filtered.points)
    kept, fraction = clean_cloud(above)
    result = measure(mask, depth, snap.intrinsics, plane)
    stages = [(raw, f'① 마스크 안 깊이 점\n{len(raw)}점 (깊이 {valid.sum() / mask.sum() * 100:.0f}%)'),
              (above, f'② 책상면 3mm 위만\n{len(above)}점'),
              (filtered, f'③ 2mm 복셀 + 이상점 제거\n{len(filtered)}점'),
              (kept, f'④ DBSCAN 최대 덩어리 + OBB\n{len(kept)}점 ({fraction * 100:.0f}% 유지)')]
    fig = plt.figure(figsize=(16, 4.6))
    for i, (points, title) in enumerate(stages, 1):
        ax = fig.add_subplot(1, 4, i, projection='3d')
        show = points[::max(1, len(points) // 4000)]
        ax.scatter(show[:, 0], show[:, 1], show[:, 2], s=1, c=show[:, 2], cmap='viridis', axlim_clip=True)
        if i == 4 and result.obb:
            for a, b in box_edges(result.obb):
                ax.plot(*np.stack([a, b]).T, color='crimson', lw=1.2)
        equal_axes(ax, kept)
        ax.view_init(elev=-60, azim=-90)
        ax.set_title(title, fontsize=9)
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
    extent = ' × '.join(f'{v * 100:.1f}' for v in sorted(result.obb['extent_m'], reverse=True)) if result.obb else '-'
    centre = ', '.join(f'{v:.3f}' for v in result.center_xyz_m) if result.center_xyz_m else '-'
    fig.suptitle(f'{label}: 판정 {result.status} ({result.reason or "근거 충분"})  '
                 f'중심(카메라 좌표, m) [{centre}]  OBB {extent} cm  높이 {(result.support_height_m or 0) * 1000:.0f}mm')
    fig.tight_layout()
    fig.savefig(out / f'p1_cloud_{name}.png', dpi=130)
    plt.close(fig)
    return result


def detection_panel(run, capture, out):
    folder = run / capture
    qwen = json.loads((folder / 'qwen/detections.json').read_text())
    report = json.loads((folder / 'observations.json').read_text())
    items = report['observations'] if isinstance(report, dict) else report
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    axes[0].imshow(plt.imread(str(folder / 'qwen/detections.jpg')))
    axes[0].set_title(f'① Qwen3-VL 검출: 박스 {len(qwen["detections"])}개')
    axes[1].imshow(plt.imread(str(folder / 'masks.jpg')))
    counts = {s: sum(o['status'] == s for o in items) for s in ('accepted', 'held', 'rejected')}
    axes[1].set_title(f'② 재확인 → SAM 마스크 → 판정: 수용 {counts["accepted"]}, 보류 {counts["held"]}, 거절 {counts["rejected"]}')
    for ax in axes:
        ax.axis('off')
    fig.tight_layout()
    fig.savefig(out / 'p1_detection_masks.png', dpi=130)
    plt.close(fig)


def gripper_lines(grasp):
    _, width, depth, rotation, translation = (v[0] for v in parts(grasp))
    approach, closing = rotation[:, 0], rotation[:, 1]
    left, right = translation + closing * width / 2, translation - closing * width / 2
    tail = translation - approach * .04
    return [(left, left + approach * depth), (right, right + approach * depth), (left, right), (translation, tail)]


def project(points, k):
    points = np.atleast_2d(points)
    return np.column_stack((k['fx'] * points[:, 0] / points[:, 2] + k['ppx'], k['fy'] * points[:, 1] / points[:, 2] + k['ppy']))


REJECT_NAMES = {'no_object_between_fingers': '집게 사이 물체 없음', 'off_centre': '중심 치우침',
                'below_table': '책상 관통', 'approach_angle': '접근 각도', 'too_wide': '폭 초과'}


def grasp_panel(grasps_dir, entry, snap, out, name):
    """entry: this object's row in the grasp run report.json (stage counts, refine rejections)."""
    import open3d as o3d
    refined = np.atleast_2d(np.load(grasps_dir / 'grasps_refined.npy'))
    candidates = np.atleast_2d(np.load(grasps_dir / 'grasps_target.npy'))
    objects = grasps_dir.parent / 'objects'
    target = np.asarray(o3d.io.read_point_cloud(str(objects / f'object_{entry["index"]:03}.ply')).points)
    scene = np.asarray(o3d.io.read_point_cloud(str(objects / 'scene.ply')).points)
    best = refined[:1]
    from_rule = entry.get('flat_rule') == 'added' and best[0, 0] == 0
    passed = entry['refined'] - (1 if entry.get('flat_rule') == 'added' else 0)
    fig = plt.figure(figsize=(18, 5))
    ax = fig.add_subplot(1, 3, 1)
    stages = [('GraspNet 후보', entry['generated']), ('대상 물체와 연관', entry['target_associated']),
              ('충돌·빈 파지 제거 후', entry['collision_and_empty_filtered']), ('규칙 필터 통과 (GraspNet)', passed)]
    if entry.get('flat_rule') == 'added':
        stages.append(('+ 납작한 물체 규칙 후보', 1))
    labels, values = [s for s, _ in stages][::-1], [max(n, .8) for _, n in stages][::-1]
    bars = ax.barh(labels, values, color='#66bb6a')
    ax.bar_label(bars, labels=[str(n) for _, n in stages][::-1], padding=3)
    ax.set_xscale('log')
    reasons = ', '.join(f'{REJECT_NAMES[k]} {v}' for k, v in entry['rejected_by_check'].items() if v)
    ax.set_title(f'파지 후보가 걸러지는 과정 (로그 축)\n규칙 필터 탈락: {reasons or "없음"}', fontsize=9)
    ax = fig.add_subplot(1, 3, 2)
    image = rgb(snap.bgr).copy()
    for row in candidates[:40]:
        for a, b in gripper_lines(row[None]):
            p = project(np.stack([a, b]), snap.intrinsics).astype(int)
            cv2.line(image, tuple(p[0]), tuple(p[1]), (255, 170, 60), 1, cv2.LINE_AA)
    for a, b in gripper_lines(best):
        p = project(np.stack([a, b]), snap.intrinsics).astype(int)
        cv2.line(image, tuple(p[0]), tuple(p[1]), (0, 200, 0), 3, cv2.LINE_AA)
    ax.imshow(image); ax.axis('off')
    source = '납작한 물체 규칙 후보(위에서 짧은 변 집기)' if from_rule else f'GraspNet 후보, 점수 {best[0, 0]:.3f}'
    ax.set_title(f'주황=대상 연관 후보(최대 40개), 초록=채택 1순위\n{source}, 폭 {best[0, 1] * 100:.1f}cm', fontsize=9)
    ax = fig.add_subplot(1, 3, 3, projection='3d')
    near = scene[np.linalg.norm(scene - target.mean(0), axis=1) < .15]
    near = near[::max(1, len(near) // 6000)]
    ax.scatter(near[:, 0], near[:, 1], near[:, 2], s=.5, c='lightgray', axlim_clip=True)
    ax.scatter(target[:, 0], target[:, 1], target[:, 2], s=1, c='tab:green', axlim_clip=True)
    for a, b in gripper_lines(best):
        ax.plot(*np.stack([a, b]).T, color='crimson', lw=2.5, axlim_clip=True)
    equal_axes(ax, target, margin=1.6)
    ax.view_init(elev=-55, azim=-80)
    ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
    ax.set_title('3D: 회색=주변 장면, 초록=대상, 빨강=채택 파지')
    fig.suptitle(f'{entry["label"]} (인식 판정 {entry["perception_status"]}): 평행 그리퍼 기준 후보. '
                 'SO101 집게·IK·실물 파지는 미검증', fontsize=10)
    fig.tight_layout()
    fig.savefig(out / f'p1_grasp_{name}.png', dpi=130)
    plt.close(fig)


def robustness_panel(snap, out):
    fig, axes = plt.subplots(1, 5, figsize=(18, 3.4))
    for ax, kind in zip(axes, ('original', 'gamma_dark', 'gamma_bright', 'blur', 'noise')):
        image = snap.bgr if kind == 'original' else perturb(snap.bgr, kind)
        ax.imshow(rgb(image)); ax.set_title(kind); ax.axis('off')
    fig.suptitle('평가용 강건성 변형 (학습 증강 아님). 27개 박스 모두 원본과 IoU ≥ 0.5')
    fig.tight_layout()
    fig.savefig(out / 'p1_robustness_inputs.png', dpi=110)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=ROOT / 'data/tabletop_rgbd_20261101')
    parser.add_argument('--perception', type=Path, default=ROOT / 'tmp/tabletop-perception-r4')
    parser.add_argument('--grasps', type=Path, default=ROOT / 'tmp/tabletop-grasps-r6')
    parser.add_argument('--capture', default='09_misc_objects')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=True)
    snap = load_snapshot(args.dataset / args.capture)
    depth = snap.depth_m()
    plane = fit_plane(backproject(depth, snap.intrinsics, depth > 0))
    data_overview(args.dataset, args.output)
    depth_and_plane(snap, plane, args.output)
    detection_panel(args.perception, args.capture, args.output)
    report = json.loads((args.perception / args.capture / 'observations.json').read_text())
    items = report['observations'] if isinstance(report, dict) else report
    english = {'리모컨': 'remote', '컵': 'cup', '마커펜': 'marker', '커터칼': 'knife', '계산기': 'calculator'}
    for index, item in enumerate(items):
        mask = cv2.imread(str(args.perception / args.capture / f'mask_{index:03}.png'), cv2.IMREAD_UNCHANGED) > 0
        object_cloud(snap, plane, mask, item['label'], args.output, english.get(item['label'], str(index)))
    grasp_report = json.loads((args.grasps / 'report.json').read_text())
    for entry in grasp_report['objects']:
        grasps_dir = args.grasps / args.capture / f'grasps_{entry["index"]:03}'
        if entry['capture'] == args.capture and (grasps_dir / 'grasps_refined.npy').exists() and entry.get('refined'):
            grasp_panel(grasps_dir, entry, snap, args.output, english.get(entry['label'], str(entry['index'])))
    robustness_panel(snap, args.output)
    for path in sorted(args.output.glob('*.png')):
        print(path)


if __name__ == '__main__':
    raise SystemExit(main())
