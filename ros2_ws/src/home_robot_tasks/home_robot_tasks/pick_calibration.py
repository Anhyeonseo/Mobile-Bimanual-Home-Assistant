"""Installed calibration, lift reference checks and historical posture evidence."""
from hashlib import sha256


REVISION = '05b1ae603e6b1fbe57a99992db17ba85ac86e7fb'
TOP_BOARD = (0x2492E, 2696088964)
RISE_BOARD = (0x24930, 3399362976)
RISE_BOARDS = ((0x2492E, 3399362976), (0x2492F, 3399362976), RISE_BOARD)
TOP_BOARDS = (TOP_BOARD, *RISE_BOARDS)
# Recorded full travel; independent of the old floor-height conversion.
# config/home_assistant/lift_geometry.measured.json: descent_encoder_delta_raw.
TOP_TRAVEL_COUNTS = 30717


def rise_target_um(board):
    from so101_arm_bridge.unified_commissioning import pick_board_scale
    return TOP_TRAVEL_COUNTS * pick_board_scale(*board) // 4096


def top_reference_um(profile, board):
    return rise_target_um(board) if tuple(board) in RISE_BOARDS else round(profile['lift']['travel_m']*1e6)

TOP_BASIS = 'operator_supported_top_straight_arm_20261118_v2'
RISE_TOP_BASIS = 'measured_bottom_to_profile_top_20261119'
RESUME_HEIGHT_BASIS = 'resumed_measured_height_encoder_delta'
TOP_BASES = (TOP_BASIS, RISE_TOP_BASIS, RESUME_HEIGHT_BASIS)
TOP_MODEL_M = .630
HEIGHT_MODELS = ('field_effective', 'encoder_no_sag')
STATION_HEIGHT_MODEL = 'encoder_no_sag'


def planning_lift_limit(profile):
    """Model extent for the already recorded travel, not a new motion limit."""
    if profile.get('base_profile_id', profile.get('calibration_id')) == 's2-dev-4683680db9f74ce58184eced':
        return max(profile['lift']['travel_m'], TOP_TRAVEL_COUNTS * abs(profile['lift']['meters_per_count']))
    return profile['lift']['travel_m']

# Historical observations, not an absolute-position sensor or a calibrated tolerance.
TOP_RAW_OBSERVED = (1727, 1882)
# The successful -40 contact shoulder was 0.758657 rad (rounded to 0.76 in the handoff).
CONTACT_RANGES = ((.758, 1.10), (-.30, 0.), (.10, .50))
# Includes the recorded -40 vertical descent and lift, rounded outward to 0.001 rad.
VERTICAL_RANGES = ((.447, 1.10), (-.30, .018), (.10, .702))
# Handoff section 4-4 used this extended-arm reference with W_REF=3.
# It is a preference, not a joint limit or an experimentally proven boundary.
TOP_POSTURE_REFERENCE = (1.032, -.239, .127)
PROFILE_FILES = {
    'measured_calibration.development.json': 'f6eec82fe6d17d9aa0abc5a63c5347fbfa3ffd665d25fca8abc3952109167c0f',
    's2_coordinates.development.json': '60d70e1e92400d792dd2e9ad51445a76793378020059425c5ebcf66df3385aad',
    's2_coordinates.wide.development.json': '45df3857455f865bfb67d918a25e41807eeaa3f6fff94c88034208d0bcfde201',
    'camera_height.calibrated.json': '5895aec095df3fd1fd52f1cccd92893d8432b333b35f6cd781befb8367200884',
    'left_pad_gap.observed.json': '5d6346fc71259e992daaf9f07ad84a8149d5ade23ff3c89cd71f4b4cb0f72922',
}


def verify_installed(root):
    for filename, expected in PROFILE_FILES.items():
        if sha256((root/'config/home_assistant'/filename).read_bytes()).hexdigest() != expected:
            raise ValueError(f'보정 파일이 현장 기준 {REVISION[:7]}과 다릅니다. {filename}')
    return REVISION


def check_top_encoder(position_raw):
    # The servo reports one revolution. Physical top is established by the
    # operator and reference_supported_top(), never inferred from this modulo.
    if type(position_raw) is not int or not 0 <= position_raw < 4096:
        raise ValueError(f'리프트 엔코더 측정값 {position_raw}가 0–4095 범위를 벗어났습니다.')
    return position_raw


def check_top_sample(status, *, anchor_raw=None, board=None):
    # LS flag 8 means an encoder sample exists. The reference procedure
    # independently requires a window of fresh, interlocked stationary samples.
    age_ms = (status.mcu_tick_ms - status.observed_ms) & 0xffffffff
    if status.flags & 8 and age_ms < 200:
        check_top_encoder(status.position_raw)
        if anchor_raw is not None:
            _check_top_drift(status.position_raw, anchor_raw, board)


def _check_top_drift(position_raw, anchor_raw, board):
    from so101_arm_bridge.unified_commissioning import pick_board_scale
    anchor = check_top_encoder(anchor_raw)
    delta = (check_top_encoder(position_raw) - anchor + 2048) % 4096 - 2048
    if abs(delta) * pick_board_scale(*board) / 4096 > 2000:
        raise ValueError('확인한 최상단 기준에서 리프트 엔코더가 2 mm 넘게 변했습니다.')


def check_top_inventory(inventory):
    positions = [d['feedback']['position_raw'] for d in inventory['devices']
                 if d['bus'] == 'left' and d['requested_id'] == 10]
    if len(positions) != 1:
        raise ValueError('리프트 엔코더 측정값을 확인할 수 없습니다.')
    return check_top_encoder(positions[0])


def make_top_reference(status, profile, board, *, operator_confirmed=False, height_model="field_effective", fixed_base_station=False, stationary_mobile=None):
    """Bind the freshly registered, physically confirmed top to this MCU session."""
    if height_model not in HEIGHT_MODELS:raise ValueError("unknown lift height model")
    if operator_confirmed is not True:
        raise ValueError('실제 기계적 최상단 확인이 필요합니다.')
    _check_registered_top(status, profile, board, stationary_mobile=stationary_mobile,
                          fixed_base_station=fixed_base_station)
    return dict(basis=TOP_BASIS, height_model=height_model, operator_confirmed=True, boot_id=status.boot_id,
                lift_session=status.session, calibration_id=profile['calibration_id'],
                controller_board=list(board), position_raw=status.position_raw,
                height_um=status.height_um, target_um=status.target_um,
                **({'base_reference_basis':'operator_fixed_station'} if fixed_base_station else {}))


def _check_registered_top(status, profile, board, *, stationary_mobile=None, fixed_base_station=False):
    target = top_reference_um(profile,board)
    check_top_encoder(status.position_raw)
    if tuple(board or ()) not in TOP_BOARDS:
        raise ValueError('상단 등록에 지원하지 않는 제어기입니다.')
    holding = status.holding(200)
    if not holding and stationary_mobile is not None:
        from .platform_preparation import stationary_platform_is_holding
        holding = stationary_platform_is_holding(stationary_mobile, status, board, fixed_base_station=fixed_base_station)
    if (tuple(board or ()) not in TOP_BOARDS or status.target_um != target
            or abs(status.height_um - target) > 2000
            or not status.boot_id or not status.session or not holding
            or status.flags & 2 or status.command_raw != 0):
        raise ValueError('상단 등록 높이 또는 현재 유지 상태를 확인할 수 없습니다.')


def top_model_height(status, profile, board, reference=None, *, stationary_mobile=None):
    _check_registered_top(status, profile, board, stationary_mobile=stationary_mobile,
        fixed_base_station=isinstance(reference,dict) and reference.get("base_reference_basis")=="operator_fixed_station")
    if (not isinstance(reference, dict) or reference.get('basis') not in TOP_BASES
            or not (reference.get('operator_confirmed') is True if reference.get('basis') == TOP_BASIS
                    else reference.get('operator_height_unchanged') is True if reference.get('basis') == RESUME_HEIGHT_BASIS
                    else reference.get('arrival_verified') is True and reference.get('bottom_referenced') is True)
            or reference.get('boot_id') != status.boot_id
            or reference.get('lift_session') != status.session
            or reference.get('calibration_id') != profile['calibration_id']
            or reference.get('controller_board') != list(board)
            or reference.get('target_um') != status.target_um
            or type(reference.get('height_um')) is not int
            or abs(reference['height_um'] - status.target_um) > 2000):
        raise ValueError('현재 제어기 세션의 최상단 등록 기록이 필요합니다.')
    _check_top_drift(status.position_raw, reference.get('position_raw'), board)
    mode=reference.get('height_model','field_effective')
    if mode not in HEIGHT_MODELS:raise ValueError('unknown lift height model')
    from .unified_carry_inputs import controller_lift_height_m
    if reference['basis'] == RESUME_HEIGHT_BASIS:
        from so101_arm_bridge.lift_wire import LiftStatus
        from so101_arm_bridge.unified_commissioning import pick_board_scale
        source = reference['source_platform']
        if source['controller_board'] != list(board) or source['top_reference']['basis'] != RISE_TOP_BASIS:
            raise ValueError('이전 상승 완료 기록과 현재 제어기 보정이 다릅니다.')
        previous = LiftStatus(**source['lift'])
        # Validate the archived arrival under its original model before using its encoder distance.
        measured = top_model_height(previous, profile, board, source['top_reference'])
        if mode=='encoder_no_sag':
            measured=controller_lift_height_m(previous.height_um,profile,board=board)
        _check_top_drift(reference['position_raw'], previous.position_raw, board)
        delta = (reference['position_raw'] - previous.position_raw + 2048) % 4096 - 2048
        # Installed pick boards count downward positively. Keep the same local
        # distance scale as the existing measured-top model, including drift.
        if mode=='encoder_no_sag':
            return measured + delta*profile['lift']['meters_per_count'] + controller_lift_height_m(
                status.height_um-reference['height_um'],profile,board=board)
        return measured - delta * pick_board_scale(*board) / 4096e6 + (status.height_um-reference['height_um'])/1e6
    if mode=='encoder_no_sag':
        return controller_lift_height_m(status.height_um,profile,board=board)
    target = status.target_um
    # The controller retains its actual registered height and any measured drift.
    return TOP_MODEL_M + (status.height_um - target) / 1e6


def straight_top_contact(joints, *, vertical_path=False):
    """Whether a pose falls in the historical test box; never a reachability test."""
    limits = VERTICAL_RANGES if vertical_path else CONTACT_RANGES
    return len(joints) >= 4 and all(lo <= q <= hi for q, (lo, hi) in
                                     zip(joints[1:4], limits, strict=True))


def top_posture_preference(joints):
    return dict(cost=3*sum((q-ref)**2 for q,ref in zip(joints[1:4],TOP_POSTURE_REFERENCE,strict=True)),
                reference_model_rad=list(TOP_POSTURE_REFERENCE),hard_limit=False,
                basis='20261118_field_extended_arm_preference')


def review_top_path(stages):
    """Keep extrapolation visible without turning a few trials into a workspace fence."""
    outside=[]
    for name,joints in stages:
        if any(not straight_top_contact(q,vertical_path=name not in ('close','close_geometry')) for q in joints):
            outside.append(name)
    return dict(policy='historical_posture_advisory',outside_recorded_stages=outside,
                within_recorded_ranges=not outside,hard_limit=False,
                physical_accuracy_verified=False,
                field_record='folded-arm trial reported 15–20 mm height error; prefer extended postures')


def check_top_surface(surface, target_base, profile):
    import numpy as np
    transform = np.asarray(profile['base_from_optical'])
    point = transform[:3, :3] @ surface['point_m'] + transform[:3, 3]
    normal = transform[:3, :3] @ surface['normal']
    xy = np.asarray(target_base)[:2]
    if not all(np.isfinite(v).all() for v in (point, normal, xy)) or np.linalg.norm(normal) < 1e-9:
        raise ValueError('상판 평면과 대상 위치에 유효한 3차원 측정값이 필요합니다.')
    normal = normal / np.linalg.norm(normal)
    if normal[2] < np.cos(np.deg2rad(20)):
        raise ValueError('수평 상판을 확인하지 못했습니다.')
    z = point[2] - normal[:2] @ (xy - point[:2]) / normal[2]
    # The old 0.7308..0.7399 m interval described one cabinet observation,
    # not robot reach or calibration uncertainty. Keep the observed plane;
    # IK and full-path collision checks determine geometric feasibility.
    # The few recorded posture ranges are advisory, not physical joint limits.
    return float(z)


def audit_top_program(program, profile, *, expected_lift_m=TOP_MODEL_M):
    from .s2_geometry import JOINTS, model_from_control
    if (not 0 <= program.planned_lift_m <= planning_lift_limit(profile)
            or abs(program.planned_lift_m - expected_lift_m) > .002001 or program.hypothetical_lift):
        raise ValueError('상단 계획 높이와 등록 기준이 다릅니다.')
    contact_stages = {'approach', 'close', 'probe', 'lift'}
    seen = set(); reviewed = []
    for stage in program.stages:
        if stage.name in contact_stages:
            seen.add(stage.name)
            if not stage.points:
                raise ValueError('상단 접촉 구간 경로가 비어 있습니다.')
            models=[]
            for point in stage.points:
                model = [model_from_control(float(round(q, 6)), profile['arms']['left']['alignment'][name])
                         for name, q in zip(JOINTS[:5], point.positions_rad[:5], strict=True)]
                models.append(model)
            reviewed.append((stage.name,models))
    if seen != contact_stages:
        raise ValueError('상단 접근·닫기·들기 경로가 필요합니다.')
    return review_top_path(reviewed)


def make_resumed_height_reference(status, profile, board, source_platform, *, operator_height_unchanged=False, height_model="field_effective"):
    """Preserve the measured arrival height across reset; never infer an absolute height from modulo."""
    from copy import deepcopy
    if height_model not in HEIGHT_MODELS:raise ValueError("unknown lift height model")
    if operator_height_unchanged is not True:
        raise ValueError('이전 상승 완료 위치에서 리프트 높이가 유지됐는지 확인이 필요합니다.')
    _check_registered_top(status, profile, board)
    reference = dict(basis=RESUME_HEIGHT_BASIS, height_model=height_model, operator_confirmed=False,
                     operator_height_unchanged=True, source_platform=deepcopy(source_platform),
                     boot_id=status.boot_id, lift_session=status.session,
                     calibration_id=profile['calibration_id'], controller_board=list(board),
                     position_raw=status.position_raw, height_um=status.height_um, target_um=status.target_um)
    top_model_height(status, profile, board, reference)
    return reference
