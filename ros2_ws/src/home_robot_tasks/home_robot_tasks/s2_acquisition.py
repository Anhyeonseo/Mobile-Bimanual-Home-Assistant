"""L2 continuous encoder state; legacy Reader import retained for callers."""
import math
from so101_arm_bridge.observer_protocol import EXPECTED, Reader
from so101_arm_bridge.encoder_units import observed_delta
from . import s2_geometry as s2


def axis_key(row):
    bus, motor = row['bus'], row['id']
    if type(bus) is not int or type(motor) is not int:
        raise ValueError('integer bus and motor ID required')
    return bus, motor


def measured(row):
    key = axis_key(row)
    pos, stamp = row['position'], row['acquired_s']
    if type(pos) is not int or not 0 <= pos < 4096 or type(stamp) not in (int, float) or not math.isfinite(stamp):
        raise ValueError(f'invalid encoder feedback: bus {key[0]} ID {key[1]}')
    return pos, stamp


def stationary_lift_window(records):
    """Check five continuous torque-off lift positions, not its noisy speed.

    The observed lift velocity register can alternate 0/+50/-50 while its
    encoder changes by one count. Five continuous position samples remain
    mandatory. This proves only a stationary observation, never an origin or motion permission.
    """
    if len(records) != 5:
        raise ValueError('five stationary bottom samples required')
    counts=[];previous=None;epoch=records[0]['epoch']
    for record in records:
        if record['epoch'] != epoch or [axis_key(r) for r in record['samples']] != EXPECTED:
            raise ValueError('bottom sample inventory or epoch changed')
        row=record['samples'][6];raw,stamp=measured(row)
        if row['torque'] != 0:
            raise ValueError('bottom reference requires torque-off lift')
        if previous is not None:
            last,old_stamp,sequence=previous
            if record['sequence'] != sequence+1 or not 0 < stamp-old_stamp <= .2:
                raise ValueError('bottom sample continuity lost')
            counts.append(counts[-1]+observed_delta(raw,last))
        else:
            counts.append(raw);started=stamp
        previous=raw,stamp,record['sequence']
    if not .15 <= stamp-started <= .6 or max(counts)-min(counts)>2:
        raise ValueError('lift moved before stationary bottom reference')
    # Larger counts correspond to lower height; keep all original readings and
    # the measured span in evidence rather than replacing them by a speed=0 claim.
    return dict(origin=max(counts),last=counts[-1],span_counts=max(counts)-min(counts),
                observed_duration_s=stamp-started,method='five_stationary_position_samples')


def stationary_bottom(records):
    """Caller must separately confirm the physical bottom before using the origin."""
    return dict(stationary_lift_window(records),method='five_position_samples_at_confirmed_bottom')


class EncoderTracker:
    """Track an explicit Home or range-resolved read-only anchor and lift bottom."""

    def __init__(self, profile, first, *, home_confirmed, bottom_confirmed,
                 range_reference=None, calibration=None, bottom_samples=None):
        if bottom_confirmed is not True or (home_confirmed is not True and range_reference is None):
            raise ValueError('operator reference and physical bottom required')
        if home_confirmed is True and range_reference is not None:
            raise ValueError('choose Home or current-pose reference')
        self.anchor_evidence = {'basis':'confirmed_home'}
        self.range_entries = None
        bottom=None
        if range_reference is not None:
            from so101_arm_bridge.development_ranges import read_development_ranges
            ranges, _, digest = read_development_ranges(range_reference)
            if profile['source_files_sha256'].get('config/home_assistant/arm_ranges.development.json') != digest:
                raise ValueError('range evidence differs from the coordinate profile')
            bottom=stationary_bottom(bottom_samples or [])
            if bottom_samples[-1] != first:
                raise ValueError('current-pose anchor must be the last stationary sample')
            if (calibration is None or calibration['epoch'] != first['epoch']
                    or calibration['sequence']+1 != bottom_samples[0]['sequence']
                    or calibration.get('motor_writes') != 0
                    or [axis_key(r) for r in calibration['samples']] != [k for k in EXPECTED if k[1]!=10]):
                raise ValueError('same-session twelve-axis calibration read required')
            for row in calibration['samples']:
                side = s2.ARMS[row['bus']]
                if (row['offset_raw'] != ranges['calibration'][side]['offsets_raw'][row['id']-1]
                        or row['mode'] != 0 or row['stored_id'] != row['id']
                        or (row['model_high'],row['model_low'],row['baud']) != (9,3,0)):
                    raise ValueError(f'{side} ID {row["id"]}: motor calibration or mode changed')
            gap=min(r['acquired_s'] for r in bottom_samples[0]['samples'])-max(r['acquired_s'] for r in calibration['samples'])
            if not 0 <= gap <= .2:
                raise ValueError('calibration evidence not immediately before the anchor')
            self.range_entries = ranges['arms']
            self.anchor_evidence = dict(basis='unique_branch_in_recorded_range',
                ranges_sha256=digest,calibration_sequence=calibration['sequence'],
                right_range_source='copied_from_left_by_operator_request',bottom=bottom,motion_authorized=False)
        self.profile = profile
        self.epoch = first['epoch']
        self.sequence = first['sequence']
        self.last, self.counts = {}, {}
        if [axis_key(r) for r in first['samples']] != EXPECTED:
            raise ValueError('all 13 ordered axes required to anchor')
        for row in first['samples']:
            key = bus, motor = axis_key(row)
            raw, stamp = measured(row)
            count = raw
            if self.range_entries is not None and (row['torque'] != 0 or (key != (0,10) and (row['speed'] & 0x7fff) > 5)):
                raise ValueError('current-pose anchor requires stationary torque-off motors')
            if key != (0, 10):
                side = s2.ARMS[bus]
                if self.range_entries is not None:
                    from so101_arm_bridge.development_ranges import unwrap_in_range
                    count = unwrap_in_range(self.range_entries[side][s2.JOINTS[motor-1]],raw)
                else:
                    ref = profile['arms'][side]['reference_raw'][s2.JOINTS[motor-1]]
                    delta = observed_delta(raw, ref)
                    if abs(delta) > 128:
                        raise ValueError(f'{side} ID {motor} no longer at captured reference pose')
                    count = ref + delta
            self.last[key], self.counts[key] = (raw, stamp), count
        if bottom is not None:
            self.counts[(0,10)]=bottom['last']
        self.origin = self.counts[(0, 10)] if bottom is None else bottom['origin']
        self.valid = True

    def update(self, record):
        try:
            if not self.valid or record['epoch'] != self.epoch or record['sequence'] != self.sequence+1:
                raise ValueError('encoder epoch/sequence changed')
            if [axis_key(r) for r in record['samples']] != EXPECTED:
                raise ValueError('axis inventory changed')
            for row in record['samples']:
                key = axis_key(row)
                old_pos, old_stamp = self.last[key]
                pos, stamp = measured(row)
                if not 0 < stamp-old_stamp <= .2:
                    raise ValueError(f'encoder sample gap or reset: bus {key[0]} ID {key[1]}')
                delta = observed_delta(pos, old_pos)
                if abs(delta) > 256:
                    raise ValueError(f'encoder discontinuity: bus {key[0]} ID {key[1]}')
                self.counts[key] += delta
                if self.range_entries is not None and key != (0,10):
                    entry=self.range_entries[s2.ARMS[key[0]]][s2.JOINTS[key[1]-1]]
                    if not entry['minimum_unwrapped_raw'] <= self.counts[key] <= entry['maximum_unwrapped_raw']:
                        raise ValueError('observed arm left the range used to resolve its turn')
                self.last[key] = pos, stamp
            self.sequence = record['sequence']
            return self.state()
        except (ValueError, KeyError, TypeError, OverflowError):
            self.valid = False
            raise

    def state(self):
        if not self.valid:
            raise ValueError('turn reference invalid')
        raw, times = {}, {}
        for bus, side in enumerate(s2.ARMS):
            raw[side] = {j: self.counts[(bus, i)] for i,j in enumerate(s2.JOINTS, 1)}
            times[side] = {j: self.last[(bus, i)][1] for i,j in enumerate(s2.JOINTS, 1)}
        lift = self.counts[(0, 10)]
        return {'raw': raw, 'joint_times_s': times, 'lift_raw_unwrapped': lift,
                'lift_time_s': self.last[(0, 10)][1],
                'lift_height_m': (lift-self.origin)*self.profile['lift']['meters_per_count']}
