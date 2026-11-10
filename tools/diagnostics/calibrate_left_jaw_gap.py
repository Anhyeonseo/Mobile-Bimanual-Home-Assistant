#!/usr/bin/env python3
"""Pair hand measurements with stationary left ID6 observations; never open a device."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from statistics import median
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks.installed_jaw import fitted_curve, SECTION, INTERIOR_SECTION, MAX_RESIDUAL_MM
from home_robot_tasks.s2_geometry import model_angle_from_raw, control_from_model


def stationary_encoder(files, *, section=SECTION):
    if section not in (SECTION, INTERIOR_SECTION):
        raise ValueError('unknown jaw measurement section')
    if not files:
        raise ValueError('passive capture required')
    positions, reads, releases = [], [], set()
    for path in files:
        data = path.read_bytes()
        report = json.loads(data)
        if report.get('mode') != 'inspect' or report.get('error') or len(report['inspections']) < 3:
            raise ValueError('incomplete stationary jaw capture')
        tickets, ticks, writes = set(), [], set()
        for item in report['inspections']:
            first, row = item['identity'], item['before']
            if (first['firmware'] not in ('left-grip-bench-v3','left-grip-bench-v4','left-grip-bench-v5') or row['firmware'] != first['firmware']
                    or first['active'] or first['locked'] or row['active'] or row['locked']
                    or row['off_verified'] != 1 or row['torque'] != 0 or row['sample_valid'] != 1
                    or row['status_raw'] != 0 or row['sequence'] != 0 or (row['speed_raw'] & 32767) > 3
                    or row['reason'] not in ('READY','START_POSE_OR_TORQUE')
                    or not 0 < row['ticket'] <= 0xFFFFFFFF or row['ticket'] in tickets
                    or not 0 <= row['position'] <= 4095 or row['writes'] != first['writes']):
                raise ValueError('invalid or energized jaw observation')
            tickets.add(row['ticket'])
            ticks.append(row['tick_ms'])
            writes.add(row['writes'])
            positions.append(row['position'])
        if len(writes) != 1 or any(a >= b for a,b in zip(ticks,ticks[1:])):
            raise ValueError('write or clock discontinuity within capture')
        releases.add(report['release'])
        reads.append(dict(sha256=hashlib.sha256(data).hexdigest(), observed_at=report['observed_at'],
                          firmware_tick_ms=[ticks[0],ticks[-1]], writes=next(iter(writes))))
    if len(releases) != 1 or max(positions)-min(positions)>3:
        raise ValueError('jaw moved during measurement')
    if any(a['writes'] != b['writes'] or a['firmware_tick_ms'][1]>=b['firmware_tick_ms'][0]
           for a,b in zip(reads,reads[1:])):
        raise ValueError('jaw session changed between observations')
    return dict(section=section,raw=median(positions),raw_min=min(positions),raw_max=max(positions),
                samples=len(positions),release=releases.pop(),captures=reads)


def measured_point(files, gap_mm, *, section=SECTION):
    if not math.isfinite(gap_mm) or not 0 <= gap_mm <= 100:
        raise ValueError('measured pad gap in mm required')
    return dict(stationary_encoder(files,section=section),gap_mm=gap_mm)


def profile(points, check, coordinates):
    all_points = [*points,check]
    curve = fitted_curve(points,coordinates)
    if check.get('section') != SECTION:
        raise ValueError('independent check must measure the same front contact faces')
    if len({p['release'] for p in all_points})!=1:
        raise ValueError('mixed firmware releases')
    evidence = [c['sha256'] for p in all_points for c in p['captures']]
    if len(set(evidence))!=len(evidence) or any(
            check['raw_min']<=p['raw_max'] and p['raw_min']<=check['raw_max'] for p in points):
        raise ValueError('independent check must use a separate capture and aperture')
    joint=coordinates['arms']['left']['alignment']['gripper']
    predicted=curve.gap(control_from_model(model_angle_from_raw(check['raw'],joint,'gripper'),joint))
    residual=abs(predicted-check['gap_mm'])
    return dict(schema_version=1, arm='left', motor_id=6, section=SECTION,
                calibration_id=coordinates['calibration_id'],assembly_revision=coordinates['assembly_revision'],
                samples=sorted(points,key=lambda p:p['raw']), check=dict(measurement=check,
                    predicted_gap_mm=predicted, absolute_error_mm=residual, tolerance_mm=MAX_RESIDUAL_MM),
                status='CHECK_PASSED' if residual<=MAX_RESIDUAL_MM else 'CHECK_FAILED',
                motion_authorized=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='mode',required=True)
    point=sub.add_parser('point')
    point.add_argument('--inspect',nargs='+',type=Path,required=True)
    point.add_argument('--mm',type=float,required=True)
    point.add_argument('--section',choices=(SECTION,INTERIOR_SECTION),default=SECTION)
    fit=sub.add_parser('fit')
    fit.add_argument('--points',nargs='+',type=Path,required=True)
    fit.add_argument('--check',type=Path,required=True)
    for parser in (point,fit):
        parser.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    try:
        if args.mode=='point':
            result=measured_point(args.inspect,args.mm,section=args.section)
        else:
            coords=json.loads((ROOT/'config/home_assistant/s2_coordinates.development.json').read_text())
            result=profile([json.loads(path.read_text()) for path in args.points],
                           json.loads(args.check.read_text()),coords)
        with args.output.open('x') as dest:
            json.dump(result,dest,indent=2,allow_nan=False)
            dest.write('\n')
        print(json.dumps(result))
        return 2 if result.get('status')=='CHECK_FAILED' else 0
    except (OSError,TypeError,KeyError,ValueError) as exc:
        p.exit(2,str(exc)+'\n')


if __name__=='__main__':
    raise SystemExit(main())
