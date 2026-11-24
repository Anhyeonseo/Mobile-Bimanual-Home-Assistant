#!/usr/bin/env python3
"""Compare stationary and lateral pick routes from a single saved station capture."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.diagnostics.plan_capture_carry import plan
from tools.diagnostics.carry_moveit_session import PersistentSession
from tools.lib.pick_planning import station_plan_options
from home_robot_tasks.tabletop_alignment import preferred_offset


def review(prepared, meshes, output, distances, edge_angle_deg, *, compare=False, origin_report=None, planner=None):
    if not math.isfinite(edge_angle_deg):
        raise ValueError('finite table edge direction required')
    if not distances or len(distances) > 16 or any(not math.isfinite(d) or abs(d) > 150 for d in distances):
        raise ValueError('provide up to 16 signed offsets within 150 mm')
    angle = math.radians(edge_angle_deg)
    samples = {}
    rows = []
    output.mkdir(parents=True, exist_ok=False)
    session = planner or PersistentSession()
    try:
        for index, distance in enumerate(dict.fromkeys([0., *distances])):
            metres = distance / 1000
            delta = [metres * math.cos(angle), metres * math.sin(angle)]
            args = station_plan_options(prepared, output / f'pose-{index}', meshes)
            started = time.monotonic()
            path = args.output / 'report.json'
            if distance == 0 and origin_report is not None:
                path = Path(origin_report)
                report = json.loads(path.read_text())
                if (report.get('mode') != 'offline_capture_moveit' or
                        report.get('hypothetical_lift_m') is not None or
                        report.get('prepared_sha256') != hashlib.sha256(Path(prepared).read_bytes()).hexdigest()):
                    raise ValueError('origin report does not match the station capture')
            else:
                report = plan(args, session_factory=session, preview_base_xy=delta if distance else None)
            status = report['status']
            ok = status in ('OFFLINE_ROUTE_FOUND', 'HYPOTHETICAL_ROUTE_FOUND')
            rows.append(dict(offset_mm=distance, base_delta_xy_m=delta, status=status,
                             seconds=round(time.monotonic()-started, 2),
                             reason=report.get('reason'), observed_sweep_clear=report.get('observed_sweep_clear'),
                             within_camera_view=report.get('within_camera_view'), report=str(path.resolve())))
            samples[distance] = ok
            if status == 'PLANNER_ERROR' or (distance == 0 and not compare and
                    (ok or not report['recovery']['base_reposition_candidate'])):
                break
    finally:
        if planner is None:
            session.close()
    selected = None if rows[-1]['status'] == 'PLANNER_ERROR' else preferred_offset(samples)
    summary = dict(capture_id=report['capture_id'], samples=rows, preferred_tested_offset_mm=selected,
                   table_edge_angle_deg=edge_angle_deg, frame='start_base_link',
                   mode='offline_arm_route_comparison', base_corridor_checked=False,
                   post_move_visibility_verified=False, hardware_commands=0, motion_authorized=False)
    from home_robot_tasks.tabletop_alignment import draw_comparison
    points = json.loads(Path(prepared).read_bytes())['review']['base_points_m']
    center = points.get('center_xyz_m', points.get('surface_xyz_m'))
    draw_comparison(summary, center[:2], output/'positions.png')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared', required=True, type=Path)
    parser.add_argument('--mobile-mesh-dir', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--offsets-mm', nargs='+', type=float, default=[-100, -50, 50, 100])
    parser.add_argument('--edge-angle-deg', required=True, type=float,
                        help='table edge tangent in start base_link, from +X towards +Y')
    parser.add_argument('--compare', action='store_true', help='inspect offsets even if no relocation is needed')
    args = parser.parse_args()
    result = review(args.prepared, args.mobile_mesh_dir, args.output, args.offsets_mm,
                    args.edge_angle_deg, compare=args.compare)
    print(json.dumps(result, ensure_ascii=False))
    return 1 if any(s['status'] == 'PLANNER_ERROR' for s in result['samples']) else 0


if __name__ == '__main__':
    raise SystemExit(main())
