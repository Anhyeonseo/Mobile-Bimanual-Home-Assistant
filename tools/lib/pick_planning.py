"""One planning policy for web requests and saved-capture regression checks.

This only constructs offline planner inputs. It never opens a device or
authorizes execution, and preserves the original capture and start evidence.
"""
import json
from collections import Counter
from pathlib import Path
from types import SimpleNamespace


def planning_failure_message(report):
    """Expose candidate rejection evidence, including failures before MoveIt."""
    bindings = report.get('binding_failures', [])
    parts = ['no executable route: '+report['status']]
    if report.get('reason'):
        parts.append(str(report['reason']))
    parts.append(f"파지 자세 변환 거절 {len(bindings)}회, 경로 검사 입력 후보 {len(report.get('candidates', []))}개")
    reasons = Counter()
    ik_fits = []
    for failure in bindings:
        reason = failure.get('reason') or '사유 미기록'
        if reason.startswith('pregrasp IK failed bounded multi-seed search:'):
            reason = '사전 접근 역기구학(IK) 조건 미충족'
            fit = failure.get('diagnostics') or {}
            if 'position_error_m' in fit and 'orientation_error_deg' in fit:
                ik_fits.append(fit)
        reasons[reason] += 1
    parts.extend(f'{reason}: {count}회' for reason, count in reasons.most_common(4))
    if len(reasons) > 4:
        parts.append(f'그 외 거절 사유 {len(reasons)-4}종 (경로 계획 거절 근거 참조)')
    if ik_fits:
        # Report one actual fit's residuals; never combine minima from different fits.
        fit = min(ik_fits, key=lambda f: (f['position_error_m']*1000)**2+f['orientation_error_deg']**2)
        parts.append(f"IK 잔차 최소 시도: 위치 {fit['position_error_m']*1000:.1f} mm, 자세 {fit['orientation_error_deg']:.1f}°")
    failures = report.get('plan', {}).get('failures', [])
    labels = {'calibration_domain': '보정 실험 자세 범위', 'pregrasp': '사전 접근',
              'table_clearance':'손가락 상판 여유', 'approach': '접근', 'lift': '들어 올리기'}
    parts.extend(f'{labels.get(stage, stage)} {count}회'
                 for stage, count in Counter(f['stage'] for f in failures).items())
    clearance = [f for f in failures if f.get('stage') == 'table_clearance']
    if clearance:
        first = clearance[0]
        parts.append(f"상판 여유 탈락 예시 {first['candidate']} · {first['reason']}")
    names = {'position': '접근 위치 조건', 'orientation': '접근 자세 조건',
             'joint_step': '관절 변화량', 'solver': 'IK 수렴'}
    violations = Counter(v for f in failures for v in (f.get('diagnostics') or {}).get('violations', []))
    parts.extend(f'{names.get(key, key)} {count}개' for key, count in violations.items())
    return ' · '.join(parts)


def station_plan_options(prepared, output, mobile_mesh_dir, *, maximum_grasps=8):
    from home_robot_tasks.pick_calibration import TOP_BASES

    if type(maximum_grasps) is not int or not 1 <= maximum_grasps <= 32:
        raise ValueError('maximum_grasps must be an integer within 1..32')
    prepared = Path(prepared)
    data = json.loads(prepared.read_text())
    start = data.get('start_revalidation', data['review'])
    top = start.get('lift_reference_basis') in TOP_BASES
    return SimpleNamespace(
        prepared=prepared, output=Path(output), mobile_mesh_dir=Path(mobile_mesh_dir),
        exhaust_candidates=True, screen_table_clearance=top, maximum_grasps=maximum_grasps, compare_all_routes=False,
        approach_mode='vertical' if top else 'tool_axis', close_at_pregrasp=top, nominal_jaw=True,
        approach_distance_m=0. if top else .02, grasp_base_x_offset_m=-.005 if top else 0.,
        grasp_posture='tabletop', entry_clearance_mm=10, contact_clearance_mm=6,
        close_allowance_mm=5, close_extra_raw=50, fixed_finger_clearance_mm=0,
        preview_lift=None, nominal_obstacles=None, obstacle_padding_mm=1)


def save_plan_options(args, path):
    """Retain the exact requested settings alongside the planner's findings."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    values = {key: str(value) if isinstance(value, Path) else value
              for key, value in vars(args).items()}
    path.write_text(json.dumps(values, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
