#!/usr/bin/env python3
"""Review a supported ID6 trial or compare two closures under matched conditions. No device I/O."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/so101_arm_bridge'))
from so101_arm_bridge.gripper_bench_protocol import (
    FIRMWARE, SUPPORTED_FIRMWARE, MEASURED_ENDINGS, check_hold_feedback, target_outcome, trial_limits,
)

TIP_OBSERVATIONS = ('unobserved', 'appeared_stationary', 'bilateral_unconfirmed', 'tip_contact', 'bilateral_contact')


def summarize(report, label):
    if (report.get('status') != 'trial_recorded' or report.get('operator_label') != label or
            report.get('grasp_verified') is not False):
        raise ValueError(f'{label}: completed, operator-labelled measurement required')
    first, begin, stop = (report[k] for k in ('before','start_reply','stop'))
    rows = report['samples']
    ticket = first['ticket']
    if (not rows or report['endpoint'] != rows[-1] or stop['active'] or stop['locked']
            or stop['off_verified'] != 1 or stop['torque'] != 0 or stop['ticket'] != ticket
            or not first['sample_valid'] or first['torque'] != 0 or begin['reason'] != 'ANCHORED'
            or begin['ticket'] != ticket or begin['active'] != 1):
        raise ValueError(f'{label}: missing anchored start or confirmed torque-off')
    if any(row['ticket'] != ticket or row['sequence'] != i or row['firmware'] != first['firmware'] or
           row['torque_limit_raw'] != begin['torque_limit_raw'] or row['sample_valid'] != 1
           for i,row in enumerate(rows,1)):
        raise ValueError(f'{label}: discontinuous feedback')
    end = rows[-1]
    if (end['active'] or not end['off_verified'] or end['locked']
            or stop['sequence'] != end['sequence']
            or end['reason'] not in MEASURED_ENDINGS):
        raise ValueError(f'{label}: interrupted trial')
    return dict(firmware=first['firmware'],start_raw=first['position'],end_raw=end['position'],
                travel_raw=first['position']-end['position'],torque_limit_raw=begin['torque_limit_raw'],
                peak_load_raw=max(row['load_raw'] & 1023 for row in rows),
                peak_current_raw=max(row['current_raw'] & 32767 for row in rows),
                end_reason=end['reason'],sample_count=len(rows))


def compare(empty_record, target_record):
    empty, target = summarize(empty_record,'empty'), summarize(target_record,'target')
    if (empty['firmware'] != target['firmware'] or empty['torque_limit_raw'] != target['torque_limit_raw']
            or abs(empty['start_raw']-target['start_raw'])>25):
        raise ValueError('different firmware, output cap or starting aperture; repeat under matched conditions')
    if empty['travel_raw'] < 12:
        raise ValueError('empty trial did not travel two increments; empty reference unavailable')
    return dict(empty=empty,target=target,endpoint_difference_raw=target['end_raw']-empty['end_raw'],
                peak_load_difference_raw=target['peak_load_raw']-empty['peak_load_raw'],
                peak_current_difference_raw=target['peak_current_raw']-empty['peak_current_raw'],
                grasp_verified=False,calibrated_contact_threshold=False)


def supported_trial(report, observation='unobserved', after=None):
    """Separate a bound encoder trace, operator witness, and automatic hold proof."""
    if observation not in TIP_OBSERVATIONS:
        raise ValueError('unknown front-tip observation')
    encoder = summarize(report, 'target')
    plan = report['target_plan']
    floor, entry = trial_limits('target', plan)
    start, end = report['start_reply'], report['endpoint']
    if start['firmware'] not in SUPPORTED_FIRMWARE or encoder['firmware'] != start['firmware']:
        raise ValueError('target-bound bench firmware required')
    previous_goal = start['goal']
    if not floor+6 <= previous_goal <= entry or previous_goal != start['position']:
        raise ValueError('invalid stationary seed')
    for row in (start, *report['samples']):
        if (row['floor_raw'] != floor or row['entry_raw'] != entry
                or not max(floor, previous_goal-6) <= row['goal'] <= previous_goal
                or not floor <= row['position'] <= entry):
            raise ValueError('goal or feedback outside bound')
        previous_goal = row['goal']
    automatic = target_outcome(end, plan)
    if report.get('contact_outcome') != automatic:
        raise ValueError('recorded automatic outcome disagrees with endpoint')
    duration=report.get('hold_requested_ms',0)
    hold=None
    if duration:
        if type(duration) is not int or not 500 <= duration <= 3000 or start['firmware'] != FIRMWARE:
            raise ValueError('unsupported hold request')
        previous=start
        for row in report['samples']:
            check_hold_feedback(previous,row,duration)
            previous=row
        rows=[row for row in report['samples'] if row['hold_goal']>=0]
        hold=dict(requested_ms=duration,complete=end['reason']=='HOLD_COMPLETE',
                  elapsed_ms=end['hold_elapsed_ms'],sample_count=len(rows),
                  goal_raw=end['hold_goal'],cause=end['hold_cause'],
                  position_span_raw=(max(r['position'] for r in rows)-min(r['position'] for r in rows)) if rows else None)
    elif end['reason']=='HOLD_COMPLETE':
        raise ValueError('completion without a requested hold')
    post_raw = None
    if after is not None:
        identity, reading = after['identity'], after['before']
        if (after['status'] != 'observed' or after['operator_label'] != 'inspect'
                or identity['ticket'] != end['ticket'] or identity['sequence'] != end['sequence']
                or identity['firmware'] != start['firmware'] or reading['firmware'] != start['firmware']
                or identity['active'] or identity['locked'] or reading['active'] or reading['locked']
                or reading['ticket'] <= 0 or reading['ticket'] == end['ticket'] or reading['sequence'] != 0
                or reading['status_raw'] != 0 or reading['speed_raw'] != 0
                or not identity['writes'] == reading['writes'] == report['stop']['writes']
                or not report['stop']['tick_ms'] <= identity['tick_ms'] < reading['tick_ms']
                or reading['sample_valid'] != 1 or reading['off_verified'] != 1 or reading['torque'] != 0):
            raise ValueError('post-stop reading does not confirm this trial')
        post_raw = reading['position']
    return dict(capture_id=plan['capture_id'], label=plan['label'], commanded_target_raw=floor,
                last_command_raw=end['goal'], encoder=encoder, post_stop_encoder_raw=post_raw,
                automatic_contact_outcome=automatic, operator_tip_observation=observation,
                bilateral_contact_observed=observation == 'bilateral_contact',
                goal_hold=hold,grasp_verified=False, lift_executed=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('first',type=Path,help='single target record, or first (empty) record of a comparison')
    p.add_argument('target',type=Path,nargs='?',help='target record when comparing a pair')
    p.add_argument('--observation',choices=TIP_OBSERVATIONS)
    p.add_argument('--after',type=Path,help='independent inspect record taken just after STOP')
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.target and (args.observation or args.after):
        p.error('--observation and --after apply to a single supported trial')
    try:
        first=json.loads(args.first.read_text())
        if args.target:
            result=compare(first,json.loads(args.target.read_text()))
        else:
            after=json.loads(args.after.read_text()) if args.after else None
            result=supported_trial(first,args.observation or 'unobserved',after)
        with args.output.open('x') as out:
            json.dump(result,out,indent=2)
            out.write('\n')
        print(json.dumps(result))
    except (OSError,TypeError,KeyError,ValueError) as exc:
        p.exit(2,str(exc)+'\n')
