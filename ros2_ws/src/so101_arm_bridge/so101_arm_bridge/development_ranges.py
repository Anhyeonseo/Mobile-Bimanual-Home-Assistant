"""Shared S1 range profile and offline point-target admission.

These ranges include operator-requested right-arm copies, not a certificate of
collision clearance. They cannot authorize a resident hardware stream.
"""
import hashlib
import json
from pathlib import Path

from .limit_manifest import JOINTS, DIRECTIONS, JOINT_NAMES, _object, _integer

FIRMWARE_VERSION = 0x0002490A
MAXIMUM_OUTPUT_STEP_RAW = 8


def _sha(value):
    if not isinstance(value, str) or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError('invalid evidence SHA256')


def read_development_ranges(path: Path):
    encoded = path.read_bytes()
    document = json.loads(encoded, object_pairs_hook=_object)
    entries = validate_development_ranges(document)
    digest = hashlib.sha256(encoded).hexdigest()
    return document, entries, digest


def validate_development_ranges(d):
    if not isinstance(d, dict) or (d.get('record_kind') != 'development_arm_ranges'
            or type(d.get('schema_version')) is not int or d['schema_version'] != 1
            or d.get('status') != 'LEFT_CAPTURED_RIGHT_COPIED_BY_OPERATOR_REQUEST'
            or d.get('hardware_enabled') is not False or d.get('firmware_limit_authorized') is not False):
        raise ValueError('expected non-executable S1 development profile')
    for name, value in (('zero_raw',2048),('raw_units_per_turn',4096),('turn_urad',6283185)):
        if _integer(d.get(name)) != value:
            raise ValueError('unsupported coordinate constants')
    if set(d.get('arms',{})) != {'left','right'} or set(d.get('calibration',{})) != {'left','right'}:
        raise ValueError('both arms required')
    entries=[]
    for arm in ('left','right'):
        calibration=d['calibration'][arm]
        _sha(calibration['centered_sha256']); _sha(calibration['persistence_sha256'])
        offsets=calibration['offsets_raw']
        if len(offsets)!=6 or any(not 0 <= _integer(x) < 4096 for x in offsets):
            raise ValueError('invalid six motor offsets')
        if set(d['arms'][arm]) != set(JOINTS):
            raise ValueError('six named joints required')
        for joint, direction in zip(JOINTS,DIRECTIONS,strict=True):
            e=d['arms'][arm][joint]
            lo,hi,ul,uh=(_integer(e[k]) for k in ('minimum_unwrapped_raw','maximum_unwrapped_raw','minimum_urad','maximum_urad'))
            if (e['coordinate']!='unwrapped_raw' or _integer(e['positive_raw_direction'])!=direction
                    or not -4096<=lo<=2048<=hi<=8191 or not 0<hi-lo<4096):
                raise ValueError('invalid or ambiguous continuous range')
            expected=sorted(direction*(x-2048)*6283185/4096 for x in (lo,hi))
            if ul>=uh or any(abs(a-b)>1 for a,b in zip((ul,uh),expected)):
                raise ValueError('raw/angle mismatch')
            measured=arm=='left'
            source='left_hand_guided_capture' if measured else 'copied_from_left_by_operator_request'
            if e.get('measured_on_this_arm') is not measured or e.get('range_source')!=source:
                raise ValueError('measurement/copy provenance mismatch')
            _sha(e['source_report_sha256']);_sha(e['source_samples_sha256'])
            if arm=='right':
                for key in ('minimum_unwrapped_raw','maximum_unwrapped_raw','positive_raw_direction','source_report_sha256','source_samples_sha256'):
                    if e[key]!=d['arms']['left'][joint][key]:
                        raise ValueError('right copy differs from left')
            if joint=='gripper' and (e.get('opening_raw_direction')!=1 or e.get('closing_raw_direction')!=-1 or e.get('maximum_open_raw')!=hi):
                raise ValueError('gripper opening semantics mismatch')
            entries.append(e)
    return tuple(entries)


def calibration_hash(digest):
    _sha(digest)
    return int(digest[:8],16)


def unwrap_in_range(entry, raw):
    if not 0<=_integer(raw)<4096:raise ValueError('invalid modulo encoder')
    matches=[raw+t*4096 for t in (-1,0,1)
             if entry['minimum_unwrapped_raw']<=raw+t*4096<=entry['maximum_unwrapped_raw']]
    if len(matches)!=1:raise ValueError('no unique encoder branch in range')
    return matches[0]


def shortest_target(entry, current_unwrapped, target_modulo, *, allow_seam=False):
    """Point-to-point target: never replace a rejected short path with a long one."""
    current=_integer(current_unwrapped);target=_integer(target_modulo)
    if not 0<=target<4096:raise ValueError('invalid target encoder')
    if not entry['minimum_unwrapped_raw']<=current<=entry['maximum_unwrapped_raw']:
        raise ValueError('current position outside profile')
    delta=(target-current%4096+2048)%4096-2048
    if delta==-2048:raise ValueError('ambiguous half turn')
    end=current+delta
    if not entry['minimum_unwrapped_raw']<=end<=entry['maximum_unwrapped_raw']:
        raise ValueError('nearest path outside profile; no long-way fallback')
    if not allow_seam and current//4096!=end//4096:
        raise ValueError('physical servo seam behavior is not verified')
    return end


def operational_limits_for_simulation(path):
    from .bimanual_stream_adapter import OperationalLimits
    _,entries,digest=read_development_ranges(path)
    return OperationalLimits(joint_names=JOINT_NAMES,
        minimum_urad=tuple(e['minimum_urad'] for e in entries),
        maximum_urad=tuple(e['maximum_urad'] for e in entries),source_sha256=digest,
        hardware_authorized=False)


def point_move_samples(entries, current_raw, target_raw, *, duration_ms=2000, allow_seam=False):
    """Offline 12-axis point move for the resident adapter, with explicit winding.

    Callers must supply current feedback; no saved Home is assumed as the start.
    This returns samples only and never acquires or enables a transport.
    """
    from .bimanual_stream_adapter import TimedJointPoint
    if len(entries)!=12 or len(current_raw)!=12 or len(target_raw)!=12:
        raise ValueError('exactly twelve joints required')
    if not 5<=_integer(duration_ms)<=60000 or duration_ms%5:
        raise ValueError('duration must be 5..60000 ms in control ticks')
    starts=[unwrap_in_range(e,r) for e,r in zip(entries,current_raw,strict=True)]
    ends=[shortest_target(e,start,target,allow_seam=allow_seam)
          for e,start,target in zip(entries,starts,target_raw,strict=True)]
    ticks=duration_ms//5
    # Four raw per 5 ms also fits the resident host's 9000 urad/tick policy.
    if any(abs(b-a)>4*ticks for a,b in zip(starts,ends)):
        raise ValueError('requested duration exceeds point-move speed bound')
    samples=[]
    for tick in range(ticks+1):
        raw=[a+round((b-a)*tick/ticks) for a,b in zip(starts,ends)]
        angles=tuple(round(e['positive_raw_direction']*(r-2048)*6283185/4096)/1e6
                     for e,r in zip(entries,raw,strict=True))
        samples.append(TimedJointPoint(100+5*tick,angles))
    return tuple(samples)
