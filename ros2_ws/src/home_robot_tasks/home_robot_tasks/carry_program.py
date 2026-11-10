"""Compile a capture-bound MoveIt result into a reviewable stationary program.

No serial/ROS connection, authorization, automatic lift motion or device discovery.
Model geometry holds the observed object width; commands retain the smaller
rigid-close goal, including after measured contact prevents reaching that goal.
"""
from dataclasses import dataclass
import math

import numpy as np

from .carry_trajectory import MotionBounds, retime
from .s2_geometry import JOINTS, control_from_model, digest
from so101_arm_bridge.servo_position import validate_wire_route


def validate_program_winding(stages, profile):
    """Use the actual rounded wire coordinates across every stage boundary."""
    directions = tuple(profile['arms'][side]['alignment'][name]['raw_to_control_sign']
                       for side in ('left', 'right') for name in JOINTS)
    validate_wire_route((tuple(round(q*1e6) for q in point.positions_rad)
                         for stage in stages for point in stage.points), directions)


@dataclass(frozen=True)
class CarryStage:
    name: str
    points: tuple
    timing: dict
    attached: bool


@dataclass(frozen=True)
class CarryProgram:
    capture_id: str
    calibration_id: str
    source_sha256: dict
    report_sha256: str
    prepared_sha256: str
    candidate_id: str
    candidate_source: str
    planned_lift_m: float
    captured_lift_m: float
    hypothetical_lift: bool
    stages: tuple
    bounds: MotionBounds
    close_control_rad: float
    contact_control_interval: tuple
    contact_raw_interval: tuple
    close_raw: int
    observed_width_m: float


def _control(profile, side, name, value):
    return control_from_model(float(value), profile['arms'][side]['alignment'][name])


def _wire_interior(value, lower, upper):
    """Keep rounding to integer micro-radians inside both physical limits.

    A model-limit endpoint can round outside by half a wire unit. Move only
    that endpoint inward, before retiming and native collision revalidation.
    This does not admit an out-of-range model path or widen any limit.
    """
    if not all(math.isfinite(v) for v in (value,lower,upper)) or lower>upper:
        raise ValueError('finite ordered wire interval required')
    if value<lower-1e-12 or value>upper+1e-12:
        raise ValueError('model path outside control/model intersection')
    lo=math.ceil(lower*1e6);hi=math.floor(upper*1e6)
    if lo>hi:raise ValueError('no representable joint command inside limits')
    rounded=round(value*1e6)
    if rounded<lo:return lo/1e6
    if rounded>hi:return hi/1e6
    return value


def _split_probe(path, fk):
    """Split the checked 2 cm lift at 4 mm, preserving its original segments."""
    xyz = np.asarray([fk(q)[0][:3, 3] for q in path])
    rise = xyz[:, 2]-xyz[0, 2]
    if (not .018 <= rise[-1] <= .022 or np.any(np.diff(rise) < -.0002)
            or np.max(np.linalg.norm(xyz[:, :2]-xyz[0, :2], axis=1)) > .002):
        raise ValueError('probe requires a monotonic 2 cm lift within 2 mm lateral drift')
    hit = int(np.flatnonzero(rise >= .004)[0])
    if hit == 0:
        raise ValueError('invalid probe origin')
    u = (.004-rise[hit-1])/(rise[hit]-rise[hit-1])
    split = path[hit-1]+u*(path[hit]-path[hit-1])
    actual = fk(split)[0][:3, 3]-xyz[0]
    if not .0038 <= actual[2] <= .0042 or np.linalg.norm(actual[:2]) > .002:
        raise ValueError('joint interpolation did not preserve the bounded probe')
    return np.vstack([path[:hit], split]), np.vstack([split, path[hit:]])


def compile_program(report, profile, installed_jaw, arm, *, bounds):
    """Compile only the exact successful candidate and its checked stage order."""
    if (report.get('status') != 'OFFLINE_ROUTE_FOUND' or report.get('initial_state_valid') is not True
            or report.get('calibration_id') != profile['calibration_id']
            or arm.side != 'left' or not math.isclose(arm.lift_m, report['initial_model_joint_positions_si']['s2_lift_joint'], abs_tol=1e-9)):
        raise ValueError('successful bound left-arm plan required')
    plan = report['plan']
    stages = plan['stages']
    if [s['label'] for s in stages] != ['open_geometry', 'pregrasp', 'approach', 'close_geometry', 'lift', 'carry']:
        raise ValueError('incomplete or reordered carry plan')
    mode=report.get('approach_mode','tool_axis')
    if mode not in ('tool_axis','vertical'):raise ValueError('unknown approach mode')
    if mode=='vertical':
        from .carry_cartesian import validate_vertical_descent
        validate_vertical_descent([arm.fk(q)[0] for q in stages[2]['joints']])
    chosen = [c for c in report['candidates'] if c['candidate']['candidate_id'] == plan['candidate_id']]
    if len(chosen) != 1:
        raise ValueError('unique planned candidate required')
    candidate = chosen[0]
    jaw = candidate['jaw_preview']
    preview = installed_jaw.preview(jaw['closure'])
    saved_preview=jaw.get('measured_gap_preview',jaw.get('nominal_gap_preview'))
    if preview != saved_preview:
        raise ValueError('jaw calibration changed after planning')
    envelope = installed_jaw.contact_envelope(jaw['closure'])
    close_raw = preview['lower_search_raw']
    upper_raw = envelope['upper_contact_raw']
    # Initial raw sign is the calibrated controller convention; no 4096 wrap.
    sign = profile['arms']['left']['alignment']['gripper']['raw_to_control_sign']
    raw_control = lambda raw: sign*(raw-2048)*math.tau/4096
    open_control, close_control = raw_control(preview['entry_raw']), raw_control(close_raw)
    initial = report['initial_model_joint_positions_si']
    anchor = np.array([_control(profile, side, name, initial[side+'_model_'+name])
                       for side in ('left', 'right') for name in JOINTS])
    expected = np.array([initial['left_model_'+n] for n in JOINTS[:5]])
    model_paths = []
    for s in stages:
        path = np.asarray(s['joints'], dtype=float)
        if (path.ndim != 2 or path.shape[1] != 5 or not 1 <= len(path) <= 20000
                or not np.isfinite(path).all() or np.max(np.abs(path[0]-expected)) > .001):
            raise ValueError('broken model path continuity')
        # Preserve tiny numerical start residuals as an explicit checked segment.
        path = np.vstack([expected, path])
        model_paths.append(path)
        expected = path[-1]
    probe, lift = _split_probe(model_paths[4], arm.fk)
    compiled = []
    previous = anchor.copy()
    wire_intervals=[]
    for i,name in enumerate(JOINTS[:5]):
        a,b=sorted((_control(profile,'left',name,arm.lo[i]),_control(profile,'left',name,arm.hi[i])))
        wire_intervals.append((max(a,bounds.minimum[i]),min(b,bounds.maximum[i])))

    def add(name, path, jaw_control, attached=False, dwell=0.):
        nonlocal previous
        controls = []
        for row in path:
            state = previous.copy()
            state[:5] = [_wire_interior(_control(profile,'left',n,value),*interval)
                         for n,value,interval in zip(JOINTS[:5],row,wire_intervals)]
            state[5] = jaw_control
            controls.append(state)
        controls = np.vstack([previous, controls])
        points, timing = retime(controls, bounds, dwell_s=dwell)
        compiled.append(CarryStage(name, points, timing, attached))
        previous = np.asarray(points[-1].positions_rad)

    add('open', model_paths[0], open_control)
    add('pregrasp', model_paths[1], open_control)
    add('approach', model_paths[2], open_control)
    # A constant target, not increased squeeze in response to poor evidence.
    add('close', model_paths[3], close_control, dwell=2.)
    add('probe', probe, close_control, attached=True)
    add('lift', lift, close_control, attached=True)
    add('carry', model_paths[5], close_control, attached=True)
    add('hold', model_paths[5][-1:], close_control, attached=True, dwell=2.)
    validate_program_winding(compiled,profile)
    captured = report['captured_model_joint_positions_si']['s2_lift_joint']
    hypothetical = report.get('hypothetical_lift_m') is not None or abs(captured-arm.lift_m) > 1e-9
    return CarryProgram(report['capture_id'], report['calibration_id'], dict(report['source_sha256']),
                        digest(report), report['prepared_sha256'], plan['candidate_id'], candidate['source'],
                        arm.lift_m, captured, hypothetical, tuple(compiled), bounds, close_control,
                        tuple(sorted([raw_control(close_raw), raw_control(upper_raw)])),
                        (close_raw, upper_raw), close_raw, jaw['closure']['visible_span_m'])


def manifest(program):
    """Keep provenance/limits visible; this artifact cannot arm a controller."""
    return dict(schema_version=1, kind='stationary_pick_carry_preflight',
                capture_id=program.capture_id, calibration_id=program.calibration_id,
                source_sha256=program.source_sha256, report_sha256=program.report_sha256,
                prepared_sha256=program.prepared_sha256, candidate_id=program.candidate_id,
                candidate_source=program.candidate_source, planned_lift_m=program.planned_lift_m,
                captured_lift_m=program.captured_lift_m, hypothetical_lift=program.hypothetical_lift,
                close_raw=program.close_raw, contact_raw_interval=program.contact_raw_interval,
                observed_width_m=program.observed_width_m,
                stages=[dict(name=s.name, attached=s.attached, **s.timing) for s in program.stages],
                motion_authorized=False, physical_task_completed=False,
                required_before_live=['fresh_same_scene_and_joints', 'actual_fixed_lift',
                                      'resident_firmware_limits_and_gripper_cap_verified',
                                      'live_contact_and_independent_target_follow_monitor',
                                      'current_scene_revalidation'])
