"""L3A: compose verified motor and camera records without owning hardware.

The CLI, and later a recording service, may feed the same coordinate session.
Opening ports, choosing objects and generating motion are outside this layer.
"""
import uuid

from .s2_development import ObservationSession


def start_session(profile, geometry, tracker, first):
    identity = {key: str(uuid.uuid4()) for key in ('session_id','origin_id','arm_anchor_id')}
    evidence = {
        **identity, 'clock_id': f'host-monotonic-{uuid.uuid4()}',
        'controller_boot_id': f"observer-epoch-{first['epoch']}",
        'calibration_id': profile['calibration_id'],
        'lift_bottom_confirmed': True, 'arm_turns_anchored': True,
        'lift_origin_raw_unwrapped': tracker.origin, 'arm_raw_unwrapped': tracker.state()['raw'],
        'established_s': max(r['acquired_s'] for r in first['samples']),
        'arm_anchor_basis': tracker.anchor_evidence,
    }
    return ObservationSession(profile, geometry, evidence), evidence


def attach_coordinates(session, state, row, clock):
    camera = row['camera']
    snapshot = {
        **session.identity,
        **{k: session.profile[k] for k in ('calibration_id','assembly_revision')},
        **{k: camera[k] for k in ('camera_device','source_frame')},
        **{k: state[k] for k in ('raw','joint_times_s','lift_raw_unwrapped','lift_time_s')},
        'sequence': row['sequence'], 'capture_id': camera['capture_id'],
        'camera_time_s': camera['color_time_s'], 'received_s': clock(),
        'raw_encoding': 'continuous_unwrapped_counts',
    }
    row['snapshot'] = snapshot
    row['coordinates'] = session.ingest(snapshot, now_s=clock(), clock_id=session.identity['clock_id'])
