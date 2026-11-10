"""Bench limits, finite goal retention and motor-only observations. No device I/O."""

FIRMWARE = 'left-grip-bench-v5'
SUPPORTED_FIRMWARE = ('left-grip-bench-v4', FIRMWARE)
MEASURED_ENDINGS = {'EFFORT_BOUND', 'EFFORT_PERSISTED', 'NO_PROGRESS', 'LOWER_GUARD', 'HOLD_COMPLETE'}


def check_hold_feedback(previous, row, duration_ms):
    """A supported rigid object's finite hold retains its preselected floor target."""
    if row.get('hold_ms') != duration_ms:
        raise ValueError('controller did not bind requested hold duration')
    held_goal = row.get('hold_goal')
    if type(held_goal) is not int:
        raise ValueError('missing hold feedback')
    if held_goal < 0:
        if previous.get('hold_goal', -1) >= 0 or row.get('hold_elapsed_ms') != 0:
            raise ValueError('hold state was lost')
        return
    elapsed = row.get('hold_elapsed_ms')
    if (duration_ms == 0 or held_goal != row['floor_raw'] or row['goal'] != held_goal
            or row.get('hold_cause') not in ('EFFORT_PERSISTED', 'NO_PROGRESS')
            or row.get('torque_limit_raw') != previous.get('torque_limit_raw')
            or type(elapsed) is not int or elapsed < previous.get('hold_elapsed_ms', 0)
            or row.get('sample_valid') != 1):
        raise ValueError('hold goal or feedback changed')
    if previous.get('hold_goal', -1) >= 0 and row['hold_cause'] != previous['hold_cause']:
        raise ValueError('hold cause changed')
    first=previous.get('hold_goal',-1)<0
    expected_writes=int(first and previous['goal']!=held_goal)
    if row['active']:
        if (row.get('torque') != 1 or row.get('off_verified') != 0
                or row['writes']-previous['writes'] != expected_writes):
            raise ValueError('hold changed motor output')
    elif row['writes'] - previous['writes'] not in (1, 2) or row.get('torque') != 0:
        raise ValueError('hold termination did not release torque')
    if row['reason'] == 'HOLD_COMPLETE' and elapsed < duration_ms:
        raise ValueError('hold ended before requested duration')


def trial_limits(mode, plan):
    if mode == 'empty' and plan is None:
        return 1950,3172
    if mode != 'target' or not isinstance(plan,dict) or plan.get('mode') != 'supported_left_jaw_contact':
        raise ValueError('supported target trial requires a capture-derived plan')
    floor, upper, entry = (plan[key] for key in ('floor_raw','upper_contact_raw','entry_raw'))
    target = plan.get('target_raw', floor)  # v4 plans originally used only floor_raw.
    if (type(target) is not int or target != floor or plan.get('arm','left') != 'left' or
            any(type(x) is not int for x in (floor,upper,entry)) or
            not 1950 <= floor <= upper < entry <= 3172 or entry-floor < 6 or
            not plan.get('capture_id') or not plan.get('calibration_id') or not plan.get('label')):
        raise ValueError('invalid contact limits or observation binding')
    return floor,entry


def target_outcome(end, plan):
    reason=end['reason']
    if reason == 'HOLD_COMPLETE':
        reason=end['hold_cause']
    if reason == 'EFFORT_PERSISTED':
        return ('CONTACT_CANDIDATE' if plan['floor_raw'] <= end['position'] <= plan['upper_contact_raw']
                else 'UNEXPECTED_RESISTANCE')
    if reason == 'LOWER_GUARD':
        return 'NO_CONFIRMED_CONTACT_AT_FLOOR'
    return 'EFFORT_OR_STALL_UNCONFIRMED'
