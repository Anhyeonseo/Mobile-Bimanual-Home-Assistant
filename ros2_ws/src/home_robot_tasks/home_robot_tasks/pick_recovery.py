"""Classify a saved plan failure before considering a different base placement.

These are review outcomes, never base commands. A future relocation adapter must
prove a completed move and acquire a new capture before running the same pick
pipeline again. A failed search is not proof that every physical grasp is impossible.
"""
import ast
from collections import Counter


IK_PREFIX = 'pregrasp IK failed bounded multi-seed search:'
GEOMETRY_BINDINGS = {
    'OUTSIDE_NOMINAL_STL_RANGE', 'closing direction cannot slide along support',
    'CANDIDATE_OFF_TARGET', 'SO101_JAW_SECTION_LIMIT',
    'observed directed section and entry clearance exceed stock jaw',
}
# Only explicit geometry/search failures qualify. Unknown reasons fail closed;
# scene update, transport, calibration and malformed trajectories do not move a base.
GEOMETRY_ROUTES = {
    'CONTACT_TOO_FAR_FROM_PAYLOAD_CENTER', 'EXCESSIVE_TABLETOP_CONTACT_TILT',
    'actual contact too far from payload center',
    'actual contact exceeds tabletop posture envelope',
    'Cartesian endpoint pose mismatch', 'Cartesian orientation drift',
    'Cartesian path exceeds tabletop finger tilt envelope',
    'Cartesian lateral drift or backward motion',
    'large joint branch change; no encoder wrapping allowed',
    'no carry goal satisfies the requested orientation',
    'payload orientation violated along path',
    'OMPL error -1',  # MoveIt PLANNING_FAILED; other error codes need diagnosis.
}


def observed_approach_collision(failure):
    """An alternate base may clear a reported hand/obstacle collision on approach."""
    if failure.get('stage') not in ('pregrasp', 'approach'):
        return False
    prefix = 'invalid full robot state: '
    reason = failure.get('reason')
    if not isinstance(reason, str) or not reason.startswith(prefix) or len(reason) > 4096:
        return False
    try:
        pairs = ast.literal_eval(reason[len(prefix):])
    except (SyntaxError, ValueError):
        return False
    fingers = {'left_s2_gripper_link', 'left_s2_moving_jaw_so101_v1_link'}
    return isinstance(pairs, list) and bool(pairs) and all(
        isinstance(pair, tuple) and len(pair) == 2 and all(isinstance(link, str) for link in pair)
        and 'capture_obstacle' in pair and any(link in fingers for link in pair) for pair in pairs)


def planning_recovery(report):
    bindings = report.get('binding_failures') or []
    failures = (report.get('plan') or {}).get('failures') or []
    result = dict(
        schema_version=1, capture_id=report.get('capture_id'),
        action='CHECK_PLANNING_DIAGNOSTICS',
        reason='경로 계획 거절 근거를 확인해야 합니다.',
        base_reposition_candidate=False, automatic_base_motion=False,
        motion_authorized=False, requires_new_capture_after_move=True,
        reuse_previous_plan_after_move=False,
        retry_sequence=['confirm_base_stopped', 'capture', 'detect', 'candidates',
                        'plan', 'execute', 'verify'],
        binding_counts=dict(Counter('PREGRASP_IK' if str(f.get('reason', '')).startswith(IK_PREFIX)
                                    else f.get('reason', 'UNKNOWN') for f in bindings)),
        route_stage_counts=dict(Counter(f.get('stage', 'UNKNOWN') for f in failures)))
    if report.get('status') == 'OFFLINE_ROUTE_FOUND':
        result.update(action='NONE', reason='현재 위치에서 전체 경로를 찾았습니다.')
        return result
    if report.get('status') != 'NO_COMPLETE_ROUTE':
        return result
    if report.get('initial_state_valid') is not True:
        result.update(action='CHECK_START_STATE', reason='현재 팔 자세와 충돌 모델부터 확인해야 합니다.')
        return result
    if not bindings and not failures:
        result.update(action='REOBSERVE_TARGET', reason='검사할 파지 후보가 없습니다. 대상을 다시 관측해야 합니다.')
        return result
    if any(not (str(f.get('reason', '')).startswith(IK_PREFIX)
                or f.get('reason') in GEOMETRY_BINDINGS) for f in bindings):
        return result
    if any(f.get('reason') not in GEOMETRY_ROUTES and not observed_approach_collision(f) for f in failures):
        return result
    # Every bound candidate must have a recorded disposition before suggesting
    # a new base position. Otherwise continue the current search first.
    candidates = report.get('candidates') or []
    ids = {c.get('candidate', {}).get('candidate_id') for c in candidates}
    examined = {f.get('candidate') for f in failures}
    if candidates and (None in ids or not ids.issubset(examined)):
        result.update(action='CONTINUE_CANDIDATE_SEARCH', reason='현재 위치에서 아직 검사하지 않은 후보가 있습니다.')
        return result
    has_ik = any(str(f.get('reason', '')).startswith(IK_PREFIX) for f in bindings)
    has_route_search = any(f.get('stage') in ('pregrasp', 'approach', 'lift', 'carry', 'carry_goals')
                           for f in failures)
    if has_ik or has_route_search:
        result.update(action='REPOSITION_BASE_AND_REOBSERVE', base_reposition_candidate=True,
            reason='현재 후보로 전체 파지 경로를 찾지 못했습니다. 베이스 재배치 가능성을 검토해야 합니다. 자동 이동은 연결되지 않았습니다.')
    else:
        result.update(action='REVIEW_GRASP_GEOMETRY',
            reason='물체 영역·집게 폭·접촉 방향을 확인해야 합니다. 베이스 이동으로 해결된다고 판단할 근거는 부족합니다.')
    return result
