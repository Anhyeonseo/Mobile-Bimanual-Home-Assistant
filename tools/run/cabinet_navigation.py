"""Drive to the archived cabinet view using the pick station's existing MCU lease."""
import multiprocessing
import hashlib
import json
import math
import time
import sys
import os
import subprocess
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from home_robot_tasks.base_alignment import settle_wheels
from home_robot_tasks.carry_live_inputs import ObservationNotFresh
from home_robot_tasks.navigation_trial import NearGoal, validate_navigation_body
from home_robot_tasks.stationary_localization import fresh_stationary
from home_robot_tasks.platform_preparation import arm_observed_stationary, supported_lift_is_holding

from tools.run.navigation_trial_web import ROOT, SessionRecord, State, make_trial, sensor_bridge_worker

CONFIG = ROOT / 'config/home_assistant'
ARCHIVE = Path('/home/hyper/robot/shared/diagnostics')
DRIVER = Path('/home/hyper/robot/shared/runtime/s4-lidar-timing/install/hls_lfcd_lds_driver/lib/hls_lfcd_lds_driver/hlds_laser_publisher')


def recorded_file(route, role, archive):
    source = route['files'][role]
    path = archive / source['path']
    if hashlib.sha256(path.read_bytes()).hexdigest() != source['sha256']:
        raise ValueError(f'수납장 관측 위치와 지도 파일이 일치하지 않습니다 ({role})')
    return path


def map_preview():
    route = json.loads((CONFIG / 'cabinet_navigation.json').read_text())
    saved = json.loads(recorded_file(route, 'map', ARCHIVE).read_text())
    return dict(grid={k:saved[k] for k in ('width','height','resolution','origin','data')},
                view=route['view_xy_yaw'])


def load_route(seed, output, archive=ARCHIVE):
    """Bind the recorded viewpoint to the exact occupancy, visibility and clearance grids."""
    route = json.loads((CONFIG / 'cabinet_navigation.json').read_text())
    files = {role: recorded_file(route, role, archive) for role in ('map', 'visibility', 'area')}
    args = SimpleNamespace(**files, initial_pose=seed, output=output, driver=DRIVER,
        lidar_port='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
        station_navigation=True, nav2=True, nav2_distance=1.2, nav2_goal=route['view_xy_yaw'], camera_mode='wide',
        manipulation_alignment=False, fine_alignment=False, furniture_navigation=None)
    with np.load(args.area, allow_pickle=False) as saved:
        area = dict(saved)
    body = validate_navigation_body(json.loads((CONFIG/'navigation_body.fixed.json').read_text()), area, args.map)
    geometry = json.loads((CONFIG/'s4_base.nominal.json').read_text())
    trial = make_trial(args, geometry, area, body)
    if not trial.map_free(args.nav2_goal):
        raise ValueError('수납장 관측 위치에 차체를 정차할 공간이 없습니다.')
    return args, trial


def cabinet_front(points, pose, anchor, view_yaw):
    """Match contiguous installed-LDS returns near the mapped cabinet landmark."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 10:
        return None
    world = points @ np.array([[c, s], [-s, c]]) + pose[:2]
    direction = np.array([math.cos(view_yaw), math.sin(view_yaw)])
    tangent = np.array([-direction[1], direction[0]])
    # The historical anchor labels the cabinet, not a precise edge or a stop.
    anchor = np.asarray(anchor, dtype=float)
    relative = world - anchor
    indices = np.flatnonzero((abs(relative @ direction) <= .14) & (abs(relative @ tangent) <= .40))
    matches = []
    for run in np.split(indices, np.flatnonzero((np.diff(indices) != 1) |
                        (np.linalg.norm(np.diff(world[indices], axis=0), axis=1) > .055)) + 1):
        if len(run) < 10:
            continue
        cloud = world[run]
        center = cloud.mean(axis=0)
        _, _, axes = np.linalg.svd(cloud-center, full_matrices=False)
        residual = (cloud-center) @ axes[1]
        # A few mixed rays at a drawer edge should not tilt the whole face.
        inliers = abs(residual-np.median(residual)) <= .02
        if inliers.mean() < .85:
            continue
        cloud = cloud[inliers]
        center = cloud.mean(axis=0)
        _, _, axes = np.linalg.svd(cloud-center, full_matrices=False)
        normal = axes[1] if axes[1] @ direction > 0 else -axes[1]
        side = np.array([-normal[1], normal[0]])
        lateral = (cloud-anchor) @ side
        offset = float((center-anchor) @ normal)
        residual = np.abs((cloud-center) @ normal)
        if (normal @ direction < math.cos(math.radians(25)) or np.max(residual) > .025
                or np.ptp(lateral) < .20 or lateral.min() > .05 or lateral.max() < -.05
                or np.dot(center-pose[:2], normal) <= 0):
            continue
        matches.append(dict(lateral_m=[float(lateral.min()), float(lateral.max())], point=(anchor+offset*normal).tolist(), yaw=math.atan2(normal[1], normal[0]),
                            span_m=float(np.ptp(lateral)), rays=len(cloud)))
    return matches[0] if len(matches) == 1 else None


CABINET_PARK_GAP_M = .06
CABINET_GAP_TOLERANCE_M = .02


def base_outline(body):
    if 'base_footprint_xy_m' in body:
        return np.asarray(body['base_footprint_xy_m'])
    # Earlier upper-travel captures stored the fixed body apart from the arms.
    if body.get('mode') == 'measured_upper_travel':
        return np.asarray(body['self_mask_xy_m'])
    return np.asarray(body['footprint_xy_m'])


def cabinet_gap(pose, face, body):
    """Normal distance from the front support of the unpadded base to the lidar face."""
    yaw = face['yaw']
    normal = np.array([math.cos(yaw), math.sin(yaw)])
    relative = yaw - pose[2]
    support = base_outline(body) @ [math.cos(relative), math.sin(relative)]
    return float((np.asarray(face['point'])-pose[:2]) @ normal - support.max())


def cabinet_gap_reached(pose, face, body):
    return abs(cabinet_gap(pose, face, body)-CABINET_PARK_GAP_M) <= CABINET_GAP_TOLERANCE_M + 1e-9


def front_view_goals(trial, pose, face, route, *, geometric_only=False):
    """Park where the near camera can see the top before choosing a pick pose."""
    from home_robot_tasks.camera_evidence import load_capture_profile
    profile, metadata = load_capture_profile(ROOT, 'near')
    tf = np.asarray(profile['base_from_optical'])
    intr = metadata['sdk_calibration']['color_intrinsics']
    yaw = face['yaw']
    normal = np.array([math.cos(yaw), math.sin(yaw)])
    side = np.array([-normal[1], normal[0]])
    point = np.asarray(face['point'])
    low, high = face['lateral_m']
    resolution = float(trial.area['resolution_m'])
    # Use the observed cabinet frontage to frame a patch just inside its edge.
    patch_offsets = np.array([-resolution, 0., resolution])
    z = route['table_height_m'] + profile['ground_z_m']
    body = trial.navigation_body
    outline = np.asarray(body['padded_outline_xy_m'])
    # The solid cabinet face ends at the tabletop. Keep lower components and
    # their clearance in this plane check; the map and live sweep still check
    # all components against obstacles along the actual route.
    if 'component_z_base_m' in body:
        low_parts = [poly for poly, (bottom, _) in zip(body['component_footprints'], body['component_z_base_m'], strict=True)
                     if bottom <= z + body['clearance_m']]
        if low_parts:
            from home_robot_tasks.nav2_trial import offset_convex_footprint
            outline = np.vstack([offset_convex_footprint(poly, body['clearance_m']) for poly in low_parts])
    def can_observe(goal):
        xy = np.asarray(goal[:2])
        distance = (point-xy) @ normal
        angle = goal[2]
        rot = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        if (not low <= (xy-point) @ side <= high or not cabinet_gap_reached(goal, face, trial.navigation_body)
                or distance <= max(outline @ (rot.T @ normal)) or (not geometric_only and not trial.map_free(goal))):
            return False
        # A side slot observes the real top directly ahead of it. The
        # historical landmark names the cabinet, not the eraser position.
        center = np.clip((xy-point) @ side, low+resolution, high-resolution)
        patch = point + resolution*normal + (center+patch_offsets)[:, None]*side
        body_points = np.column_stack(((patch-xy) @ rot, np.full(len(patch), z)))
        optical = (body_points-tf[:3,3]) @ tf[:3,:3]
        if (optical[:,2] <= 0).any():
            return False
        u = intr['fx']*optical[:,0]/optical[:,2]+intr['ppx']
        v = intr['fy']*optical[:,1]/optical[:,2]+intr['ppy']
        return bool(((0 <= u) & (u < intr['width']) & (0 <= v) & (v < intr['height'])).all())

    if abs(math.remainder(yaw-pose[2], math.tau)) > trial.maximum_turn:
        return
    step = min(resolution/2, CABINET_GAP_TOLERANCE_M/2)
    hull = base_outline(trial.navigation_body)
    lateral = np.clip((np.asarray(pose[:2])-point) @ side, low, high)
    options = []
    headings = [yaw]
    if abs(math.remainder(yaw-pose[2], math.tau)) <= trial.arrival_yaw:
        if can_observe(pose):
            yield list(pose)
        headings.append(pose[2])
    for heading in set(headings):
        relative = yaw-heading
        preferred = max(hull @ [math.cos(relative), math.sin(relative)]) + CABINET_PARK_GAP_M
        for offset_m in (0., step, -step, CABINET_GAP_TOLERANCE_M, -CABINET_GAP_TOLERANCE_M):
            for offset in np.unique(np.r_[lateral, 0., np.arange(low,high+step/2,step)]):
                xy = point - (preferred+offset_m)*normal + offset*side
                travel = math.dist(pose[:2], xy)
                if travel <= trial.distance:
                    options.append((abs(offset_m), travel,[*xy.tolist(), heading]))
    seen = set()
    detours = []
    for _, _, goal in sorted(options):
        cell = tuple(np.floor((np.asarray(goal[:2])-trial.area['origin_xy_m'])/resolution).astype(int))
        cell = (*cell,goal[2])
        if cell in seen or not can_observe(goal):
            continue
        if geometric_only or all(trial.map_free(p) for p in NearGoal.path(trial, pose, goal)):
            seen.add(cell)
            yield goal
        else:
            detours.append((cell, goal))
    # A blocked straight segment can still have a Nav2 route around it.
    # These are planner goals only. The real path and every commanded
    # velocity still go through the normal lidar/depth checks.
    for cell, goal in detours:
        if cell not in seen:
            seen.add(cell)
            yield goal


def archived_front_goals(area_path, body, limits, pose, face, route):
    # Only map geometry crosses the process boundary. The Jetson serial owner
    # must continue refreshing interlocks even during Python/NumPy imports.
    with np.load(area_path, allow_pickle=False) as saved:
        trial = SimpleNamespace(area=dict(saved), navigation_body=body, **limits)
    trial.map_free = lambda candidate: NearGoal.map_free(trial, candidate)
    return list(front_view_goals(trial, pose, face, route))


def navigate_upper_front(ctx, hardware, args, trial):
    route = json.loads((CONFIG/'cabinet_navigation.json').read_text())
    origin = args.initial_pose
    if math.dist(origin[:2], route['view_xy_yaw'][:2]) > .35:
        origin = drive_to_view(ctx, hardware, args, trial)
    options = SimpleNamespace(**vars(args))
    options.output = args.output/'cabinet-front'
    options.output.mkdir()
    options.initial_pose = origin
    options.nav2_goal = origin
    options.manipulation_alignment = True
    options.cabinet_route = route
    options.cabinet_front_prior = [route['support_anchor_xy_m'], route['view_xy_yaw'][2]]
    options.nav2_distance = .65
    geometry = json.loads((CONFIG/'s4_base.nominal.json').read_text())
    ctx.progress('라이다로 수납장 앞면 확인 · 베이스 앞끝에서 6cm 정차')
    for attempt in range(2):
        options.allow_gap_correction = attempt == 0
        options.correct_park_gap = False
        align = make_trial(options, geometry, trial.area, trial.navigation_body)
        align.arrival_xy = min(align.arrival_xy, CABINET_GAP_TOLERANCE_M)
        pose = drive_to_view(ctx, hardware, options, align)
        if not options.correct_park_gap:
            args.observed_cabinet_face = options.observed_cabinet_face
            return pose
        options.initial_pose = options.nav2_goal = pose
        options.output = args.output/'cabinet-gap-correction'
        options.output.mkdir()
        ctx.progress('정차 후 측정한 수납장 간격으로 접근 위치 보정')
    raise RuntimeError('베이스 앞끝 기준 수납장 6cm 정차 확인 실패')


def drive_to_view(ctx, hardware, args, trial):
    """LDS/D415/Nav2 run in the original sensor process. Only this thread writes wheels."""
    mobile = hardware.mobile
    upper = getattr(args, 'navigation_body', None) is not None
    if hardware.guard.supported_top != upper or (not upper and abs(hardware.guard.current_height) > .003):
        raise ValueError('현재 리프트 기준과 경로를 검사한 자세가 다릅니다.')
    capabilities = mobile.status().capabilities
    if not capabilities & 8:
        raise ValueError('주행 후 같은 세션에서 주차를 확인할 수 없는 제어기입니다.')
    scales = json.loads((CONFIG/'s4_commissioning.pending.json').read_text())[
        'measurement_inputs']['wheel_geometry_and_raw_speed_scale']['candidate_raw_per_encoder_rad_s']
    if len(scales) != 3 or not all(math.isfinite(v) and v > 0 for v in scales):
        raise ValueError('바퀴 속도 환산값을 확인할 수 없습니다.')
    record = SessionRecord(args.output)
    state = State(trial, record)
    state.async_corridor = state.controller_ready = True
    def observe():
        ctx.check()
        lift = hardware.observe_preparation()
        target = hardware.guard.top_reference['target_um'] if upper else 0
        if not supported_lift_is_holding(lift) or abs(lift.height_um-target) > 2000:
            raise RuntimeError('주행 중 리프트 높이 유지 확인 실패')
        if upper:
            from home_robot_tasks.pick_calibration import check_top_sample
            ref = hardware.guard.top_reference
            if lift.boot_id != ref['boot_id'] or lift.session != ref['lift_session'] or lift.target_um != target:
                raise RuntimeError('주행 중 상단 등록 세션이 바뀌었습니다.')
            check_top_sample(lift, anchor_raw=ref['position_raw'], board=hardware.guard.board)
        return lift

    zero_since = None
    motion_since = None
    last_encoder_s = -math.inf
    previous_positions = None
    displacement = [0, 0, 0]
    def record_wheels(reply, sent, command):
        nonlocal motion_since, last_encoder_s, previous_positions
        if any(command) and motion_since is None:
            motion_since = sent
        elapsed = None if motion_since is None else round((sent - motion_since) * 1000)
        if not any(command):
            motion_since = None
        starting = any(command) and sent == motion_since
        stopping = not any(command) and elapsed is not None
        if sent - last_encoder_s < .10 and not (starting or stopping):
            return
        last_encoder_s = sent
        positions = None
        if capabilities & 16:
            snapshot = hardware.transport.stationary_platform()
            if snapshot['present_mask'] & 7 == 7 and max(snapshot['sample_age_ms'][:3]) <= 250:
                positions = snapshot['positions_raw'][:3]
                if previous_positions is not None:
                    for i, position in enumerate(positions):
                        displacement[i] += (position - previous_positions[i] + 2048) % 4096 - 2048
            previous_positions = positions
        record.write('wheel_sample', command=command, feedback=reply.velocity_raw[:3],
                     sample_tick_ms=reply.feedback_tick_ms, continuous_command_ms=elapsed,
                     positions_raw=positions, displacement_raw=displacement if positions is not None else None,
                     encoder_tick_ms=snapshot['firmware_tick_ms'] if positions is not None else None,
                     encoder_sample_age_ms=snapshot['sample_age_ms'][:3] if positions is not None else None)

        return snapshot if positions is not None else None

    def wheels(reply, sent, command=None):
        nonlocal zero_since
        if not reply.feedback_is_fresh(250):
            return
        # A fresh disabled mobile session cannot issue a wheel target. Its
        # quiet interval lets the lidar collect raw scan matches even when a
        # servo speed byte flickers. Final park admission still requires all flags.
        unarmed = (upper and command is None and reply.state == 0 and reply.session == 0
                   and reply.reason == 0 and reply.feedback_is_fresh(200)
                   and max(map(abs, reply.velocity_raw[:3])) <= 50)
        zero_target = unarmed or (command is not None and not any(command))
        zero_since = (zero_since or sent) if zero_target else None
        age = ((reply.mcu_tick_ms - reply.feedback_tick_ms) & 0xffffffff) / 1000
        twist = trial.geometry.body_twist(tuple(v / s for v, s in zip(reply.velocity_raw[:3], scales)))
        with state.lock:
            state.wheel_observation = dict(observed_s=sent-age, twist=twist,
                zero_command_elapsed_s=sent-zero_since if zero_since else 0.,
                feedback_tick_ms=reply.feedback_tick_ms, boot_id=reply.boot_id)

    def wait_at_zero(now):
        ctx.check()
        zero = (0, 0, 0, 0)
        reply = mobile.velocity(zero, lifetime_ms=150)
        if not reply.ready_for_motion(mobile.session, 250):
            raise RuntimeError('관측 재확인 중 바퀴 정지 명령 거부')
        wheels(reply, now, zero)
        record_wheels(reply, now, zero)
        return reply

    with (ThreadPoolExecutor(max_workers=1) as pool,
          ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn')) as geometry_pool):
        bridge = pool.submit(sensor_bridge_worker, state, args)
        try:
            deadline = time.monotonic() + 90
            recheck_start = False
            checking_arrival = False
            measure_front = bool(getattr(args, 'cabinet_front_prior', None))
            front_required = measure_front and not getattr(args, 'follow_cabinet_face', False)
            face = getattr(args, 'observed_cabinet_face', None)
            front_bound = False
            last_face = None
            goals = None
            goal_search = None
            candidate_count = 0
            while True:
                ctx.service()
                if bridge.done():
                    bridge.result()
                    raise RuntimeError('지도 주행 센서가 종료됐습니다.')
                sent = time.monotonic()
                parked_status = mobile.status()
                wheels(parked_status, sent)
                with state.lock:
                    # A single boundary-cell match can change between stationary
                    # scans. Use the existing preparation window for a new fix.
                    blocked_start = (upper and state.pose is not None and
                                     not trial.map_free(state.pose))
                    recheck_start |= blocked_start
                    stable = not recheck_start or fresh_stationary(state.stationary_localization, sent,
                        revision=trial.required_map_revision)
                    stationary = fresh_stationary(state.stationary_localization, sent, revision=trial.required_map_revision)
                    if goal_search is not None and goal_search.done():
                        goals = iter(goal_search.result())
                        goal_search = None
                    if (front_required and not front_bound and not state.review_cabinet
                            and goals is None and goal_search is None):
                        sample = state.cabinet_face
                        if (stationary and sample and sample['stamp'] == state.pose_stamp == state.scan_stamp
                                and 0 <= sent-sample['stamp'] <= .35 and state.sensors_ready(sent)):
                            face = sample['face']
                            if (last_face and sample['stamp'] > last_face['stamp']
                                    and sample['stamp']-last_face['stamp'] <= .5
                                    and math.dist(face['point'], last_face['face']['point']) <= trial.arrival_xy
                                    and abs(math.remainder(face['yaw']-last_face['face']['yaw'], math.tau)) <= trial.arrival_yaw):
                                record.write('cabinet_front_observed', scan=sample, pose=state.pose,
                                             gap_m=cabinet_gap(state.pose, face, trial.navigation_body))
                                # Exhausting even an empty generator can exceed the MCU service period.
                                limits = {key: getattr(trial, key) for key in ('distance','maximum_turn','arrival_yaw')}
                                goal_search = geometry_pool.submit(archived_front_goals, args.area,
                                    trial.navigation_body, limits, tuple(state.pose), face, args.cabinet_route)
                            last_face = sample
                        if goals is None and sent >= deadline:
                            raise ValueError('정지 라이다 관측에서 수납장 앞면을 구분하지 못했습니다.')
                    if goal_search is not None and sent >= deadline:
                        raise TimeoutError('수납장 정차 후보 계산 시간 초과')
                    if state.review_cabinet and not front_bound:
                        report = state.cabinet_review
                        if report is None or not stationary:
                            if sent >= deadline:
                                raise ValueError('정지 상태에서 최신 라이다·깊이 경로 재검사를 마치지 못했습니다.')
                        elif report['goal'] is None:
                            raise ValueError('6cm 접근 경로 재검사 · 통과 후보 없음 · '+str(report['rejected']))
                        elif state.navigation_update and state.sensors_ready(sent):
                            sample = state.cabinet_face
                            if (sample and sample['stamp'] == state.pose_stamp == state.scan_stamp
                                    and cabinet_gap_reached(report['goal'], sample['face'], trial.navigation_body)):
                                last_face = sample
                                goals = iter([report['goal']])
                    rejected = state.nav2_feedback.get('rejected_goal')
                    if goals is not None and (not front_bound or rejected == args.nav2_goal):
                        if front_bound:
                            record.write('cabinet_front_unreachable', goal=rejected,
                                         reason=state.nav2_feedback.get('wait_reason'))
                        goal = next(goals, None)
                        if goal is None:
                            if candidate_count == 0 and not state.review_cabinet:
                                state.review_cabinet = True
                                goals = None
                                record.write('cabinet_live_path_review_requested', pose=state.pose)
                                continue
                            raise ValueError(f'베이스 앞끝 기준 수납장 6cm 정차 범위에서 통과 경로를 찾지 못했습니다 (검토 후보 {candidate_count}곳).')
                        candidate_count += 1
                        trial.target_pose = tuple(goal)
                        state.path = trial.plan(state.pose)
                        args.nav2_goal = goal
                        checking_arrival = False
                        front_bound = True
                        record.write('cabinet_front_goal', scan=last_face, goal=goal, pose=state.pose,
                                     candidate=candidate_count)
                    at_view = (upper and (not args.manipulation_alignment or front_bound) and state.pose is not None
                               and math.dist(state.pose[:2], args.nav2_goal[:2]) <= trial.arrival_xy
                               and abs(math.remainder(state.pose[2]-args.nav2_goal[2], math.tau)) <= trial.arrival_yaw)
                    if at_view and measure_front:
                        live = state.cabinet_face
                        at_view = bool(live and live['stamp'] == state.pose_stamp == state.scan_stamp
                                       and state.sensors_ready(sent)
                                       and cabinet_gap_reached(state.pose, live['face'], trial.navigation_body))
                    if at_view and not checking_arrival:
                        record.write('checking_observation_pose', pose=list(state.pose), goal=args.nav2_goal)
                        checking_arrival = True
                    if checking_arrival and stationary and not at_view:
                        checking_arrival = False
                    if (at_view and not blocked_start and stationary and parked_status.state == 0
                            and parked_status.session == 0 and parked_status.reason == 0
                            and parked_status.flags == 31 and parked_status.feedback_is_fresh(200)):
                        hardware.guard.require_fresh(sent)
                        parked_pose = list(state.pose)
                        if measure_front:
                            args.observed_cabinet_face = live['face']
                            record.write('cabinet_gap_confirmed', gap_m=cabinet_gap(parked_pose, live['face'], trial.navigation_body))
                        record.write('already_parked', pose=parked_pose, goal=args.nav2_goal,
                                     boot_id=parked_status.boot_id, nonzero_commands=0)
                        return parked_pose
                    if (not front_required or front_bound) and not checking_arrival and not blocked_start and stable and state.snapshot()['ready']:
                        if getattr(args, 'manipulation_alignment', False) and not front_required and (
                                math.dist(state.pose[:2], args.initial_pose[:2]) > trial.arrival_xy or
                                abs(math.remainder(state.pose[2]-args.initial_pose[2], math.tau)) > trial.arrival_yaw):
                            raise ValueError('재측정한 출발 자세와 물체를 촬영한 자세가 일치하지 않습니다.')
                        trial.start(sent, state.pose, operator_confirmed=True)
                        record.write('started', pose=state.pose, goal=trial.goal)
                        break
                    if sent >= deadline:
                        if checking_arrival:
                            raise ValueError('관측점 부근에서 정지 위치를 확인하지 못했습니다. 바퀴 명령 0회.')
                        if blocked_start or not stable:
                            raise ValueError('현재 라이다 위치에서 상단 양팔 주행 여유가 지도와 겹칩니다.')
                        raise RuntimeError('지도 주행 준비 실패 · ' + str(state.error or state.scan_error or state.depth_error or state.nav2_feedback.get('error') or 'Nav2 경로 대기'))
                ctx.pause(.03)
            hardware.base_moving = True
            arm_observed_stationary(observe, mobile, session=mobile.sequence+1, timeout_s=15.,
                on_sample=lambda **row: record.write('departure_sample', **row),
                on_deferred=lambda reply: record.write('departure_arm_deferred', reply=reply))
            record.write('departure_armed', session=mobile.session)
            previous = time.monotonic()
            nonzero_commands = 0
            previous_command = None
            hold_wait = None
            while True:
                try:
                    observe()
                except ObservationNotFresh as error:
                    now = time.monotonic()
                    # Only a timing rejection can enter this branch. A rejected
                    # zero, a posture change or a controller fault still aborts.
                    if hold_wait is None:
                        hold_wait = now
                        record.write('arm_observation_wait', reason=str(error))
                        with state.lock:
                            if trial.resume_required or trial.state not in ('RUNNING', 'WAITING_SENSOR'):
                                raise
                            trial.pause(now, 'WAITING_SENSOR')
                            trial.sensor_wait_scan = state.scan_stamp
                    elif now-hold_wait >= .25:
                        raise
                    wait_at_zero(now)
                    previous_command = (0, 0, 0, 0)
                    previous = now
                    continue
                if hold_wait is not None:
                    now = time.monotonic()
                    if now-hold_wait >= .25:
                        raise TimeoutError('주행 재개 전 팔·바퀴 정지 관측 시간 초과')
                    reply = wait_at_zero(now)
                    previous = now
                    age = ((reply.mcu_tick_ms-reply.feedback_tick_ms) & 0xffffffff)/1000
                    if now-age <= hold_wait or any(reply.velocity_raw):
                        continue
                    record.write('arm_observation_recovered', elapsed_ms=(now-hold_wait)*1000)
                    hold_wait = None
                if bridge.done():
                    bridge.result()
                    raise RuntimeError('주행 센서 연결 종료')
                ctx.check()
                with state.lock:
                    now = time.monotonic()
                    if now-previous > .2:
                        raise RuntimeError('바퀴 명령 주기 지연')
                    previous = now
                    previous_state = trial.state
                    hits = state.corridor_hits()
                    corridor = state.corridor_observation or {}
                    checked_twist = corridor.get('nav_twist')
                    proposal = corridor.get('proposal')
                    command = trial.command(now, state.pose, state.pose_stamp, scan_stamp=state.scan_stamp,
                        sensors_fresh=hits is not None, corridor_clear=hits is not None and not any(hits.values()),
                        checked_twist=checked_twist, checked_proposal=proposal,
                        lidar_preview_only=corridor.get('lidar_preview_only', False))
                    if previous_state != trial.state and 'WAITING_SENSOR' in (previous_state, trial.state):
                        record.write('sensor_wait_transition', previous=previous_state, state=trial.state,
                                     scan_age_s=now-state.scan_stamp, depth_age_s=now-state.depth_stamp,
                                     nav_depth_age_s=now-trial.nav.get('depth_s', -1),
                                     nav_odom_age_s=now-trial.nav.get('odom_s', -1),
                                     command_age_s=now-trial.nav.get('command_s', -1),
                                     corridor=hits, checked_twist=checked_twist)
                    if trial.state == 'SETTLING':
                        break
                    if trial.resume_required or trial.state not in ('RUNNING', *trial.WAITING):
                        record.write('motion_interrupted', state=trial.state, pose=state.pose,
                                     pose_age_s=now-state.pose_stamp, scan_age_s=now-state.scan_stamp,
                                     depth_age_s=now-state.depth_stamp, corridor=state.corridor_hits(),
                                     nav_wait=state.nav2_feedback.get('wait_reason'),
                                     nonzero_commands=nonzero_commands)
                        raise RuntimeError('수납장 주행 중단 · '+trial.state)
                    sent = time.monotonic()
                    if any(command) and state.review_cabinet:
                        from tools.run.cabinet_path_review import command_in_path
                        review = state.cabinet_review
                        if (not proposal or not review or not review['goal']
                                or proposal['map_revision'] != trial.required_map_revision
                                or not command_in_path(review, state.pose, trial.sweep(proposal['twist']))):
                            record.write('reviewed_approach_deviation', pose=state.pose)
                            raise ValueError('주행 명령이 실측으로 확인한 접근 경로를 벗어납니다.')
                    if any(command) and proposal is not None:
                        command, reply = mobile.velocity_checked(command, math.floor(proposal['valid_until_s'] * 1000))
                    else:
                        command = (0, 0, 0, 0)
                        reply = mobile.velocity(command, lifetime_ms=150)
                if not reply.ready_for_motion(mobile.session, 250):
                    raise RuntimeError('바퀴 명령 상태 확인 실패')
                wheels(reply, sent, command)
                record_wheels(reply, sent, command)
                nonzero_commands += bool(any(command))
                if command != previous_command:
                    record.write('wheel_command_accepted', command=command, feedback=reply.velocity_raw,
                                 nonzero_commands=nonzero_commands)
                    previous_command = command
                time.sleep(.02)
            from home_robot_tasks.base_alignment import WheelRestWindow
            rest = WheelRestWindow()
            def settling_sample(event, *, raw, reply):
                sent = time.monotonic()
                wheels(reply, sent, raw)
                snapshot = record_wheels(reply, sent, raw)
                rest.observe(reply, snapshot)
            settle_wheels(mobile, observe, check=ctx.check, log=settling_sample, encoder_rest=rest)
            observe()
            parked = mobile.park()
            hardware.history.clear()
            hardware.base_moving = False
            if measure_front:
                # Park acknowledgment is wheel evidence. Localize again before
                # handing a camera-relative object to the placement planner.
                parked_at = time.monotonic()
                while True:
                    ctx.service()
                    now = time.monotonic()
                    wheels(mobile.status(), now)
                    with state.lock:
                        sample = state.cabinet_face
                        if (sample and parked_at < sample['stamp'] == state.scan_stamp == state.pose_stamp
                                and state.sensors_ready(now)
                                and fresh_stationary(state.stationary_localization, now, revision=trial.required_map_revision)
                                and math.dist(sample['face']['point'], face['point']) <= float(trial.area['resolution_m'])
                                and abs(math.remainder(sample['face']['yaw']-face['yaw'], math.tau)) <= trial.arrival_yaw
                                and math.dist(state.pose[:2], args.nav2_goal[:2]) <= trial.arrival_xy
                                and abs(math.remainder(state.pose[2]-args.nav2_goal[2], math.tau)) <= trial.arrival_yaw):
                            gap = cabinet_gap(state.pose, sample['face'], trial.navigation_body)
                            record.write('cabinet_gap_observed', gap_m=gap, target_m=CABINET_PARK_GAP_M)
                            if cabinet_gap_reached(state.pose, sample['face'], trial.navigation_body):
                                args.observed_cabinet_face = sample['face']
                                record.write('cabinet_front_parked', scan=sample, pose=state.pose, gap_m=gap)
                                break
                            if getattr(args, 'allow_gap_correction', False) and now-parked_at >= .6:
                                args.correct_park_gap = True
                                args.observed_cabinet_face = sample['face']
                                record.write('cabinet_gap_correction', gap_m=gap, pose=state.pose)
                                break
                            if now-parked_at >= 7.9:
                                raise ValueError(f'수납장 실측 간격 {gap*100:.1f}cm · 베이스 앞끝 기준 목표 6cm')
                    if now-parked_at >= 8 or bridge.done():
                        raise ValueError('주차 후 라이다에서 수납장 정면 정렬을 재확인하지 못했습니다.')
                    ctx.pause(.04)
            with state.lock:
                trial.finish(True)
                parked_pose = list(state.pose)
                record.write('parked', pose=parked_pose, goal=trial.goal, boot_id=parked.boot_id,
                             nonzero_commands=nonzero_commands)
            ctx.service()
            hardware.guard.require_fresh(time.monotonic())
        except BaseException:
            # Stop before waiting for sensor processes or releasing the camera.
            hardware.abort()
            raise
        finally:
            state.closed.set()
            hardware.base_moving = False
            try:
                if not bridge.done() and not hardware.stopped:
                    ctx.wait(bridge)
                else:
                    bridge.result()
            finally:
                record.close()
    return parked_pose


def raw_arms(inventory):
    if not inventory['all_snapshots_complete']:
        raise ValueError('양팔의 전체 엔코더 관측이 필요합니다.')
    devices = {(d['bus'], d['requested_id']): d['feedback']['position_raw'] for d in inventory['devices']}
    return {side: [devices[side, motor] for motor in range(1, 7)] for side in ('left', 'right')}


def prepare_upper_route(ctx, serial_port, args, trial):
    """Inspect the current pose and reject blocked map seeds before commissioning."""
    import serial
    from home_robot_tasks.camera_evidence import load_capture_profile
    from home_robot_tasks.pick_calibration import top_reference_um
    from home_robot_tasks.unified_carry_inputs import controller_lift_height_m
    from so101_arm_bridge.backend_lease import acquire_backend_lease
    from so101_arm_bridge.device_inventory import collect_inventory
    from so101_arm_bridge.serial_port import open_exclusive_serial
    from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2
    from so101_arm_bridge.unified_commissioning import query_pick_controller

    from home_robot_tasks.pick_service import Rejected

    profile, _ = load_capture_profile(ROOT, 'near')
    with acquire_backend_lease('stm32', 189), open_exclusive_serial(serial, serial_port, 921600, .02) as port:
        transport = StreamValidationTransportV2(port, response_timeout_s=3)
        board, _ = query_pick_controller(transport, profile)
        raw = raw_arms(collect_inventory(transport))
    height = controller_lift_height_m(top_reference_um(profile, board), profile, board=board)
    args.output.mkdir(parents=True, exist_ok=True)
    observation = dict(raw=raw, height=height, calibration_id=profile['calibration_id'])
    (args.output/'upper-posture.json').write_text(json.dumps(observation))
    ctx.progress('최상단 고정 · 현재 양팔 자세로 지도 주행 공간 검사')
    # Geometry compilation cannot delay the serial owner or compete with camera capture.
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    env['PYTHONPATH'] = os.pathsep.join([str(ROOT), str(ROOT/'ros2_ws/src/home_robot_tasks'),
                                         str(ROOT/'ros2_ws/src/so101_arm_bridge'), env.get('PYTHONPATH','')])
    with subprocess.Popen([sys.executable, '-m', 'tools.run.cabinet_upper_body', str(args.output.resolve()),
                           str(ARCHIVE)], cwd=ROOT, env=env) as process:
        try:
            while process.poll() is None:
                ctx.pause(.05)
        finally:
            if process.poll() is None:
                process.terminate()
        if process.wait() != 0:
            raise ValueError('상단 차체·팔 주행 공간 계산에 실패했습니다.')
    body = json.loads((args.output/'upper-body.json').read_text())
    args.area = args.output/'upper-area.npz'
    with np.load(args.area, allow_pickle=False) as data:
        trial.area = dict(data)
    args.navigation_body = trial.navigation_body = body
    args.depth_ceiling_m = max(1.12, body['height_above_floor_m'] + body['clearance_m'])
    ctx.artifact(args.output/'upper-body.json', '출발 양팔·리프트 주행 형상')
    if not trial.map_free(args.initial_pose):
        raise Rejected('상단 양팔을 포함하면 입력한 출발 위치의 지도 여유가 부족합니다. 바퀴 명령 0회.')
    if math.dist(args.initial_pose[:2], args.nav2_goal[:2]) > .35 and not trial.map_free(args.nav2_goal):
        raise Rejected('상단 양팔을 포함하면 수납장 관측 지점에 정차할 공간이 부족합니다.')
    return args, trial


def bind_upper_start(hardware, body):
    raw = raw_arms(hardware.travel_inventory)
    if (not hardware.guard.supported_top or hardware.profile['calibration_id'] != body['calibration_id']
            or abs(hardware.guard.current_height-body['lift_position_m']) > .002):
        raise ValueError('상단 실제 높이와 경로를 계산한 기준이 다릅니다.')
    for side in raw:
        for current, previous in zip(raw[side], body['raw'][side], strict=True):
            if abs((current-previous+2048)%4096-2048) > 3:
                raise ValueError('준비 중 팔 자세가 바뀌어 상단 주행 공간을 다시 계산해야 합니다.')


def placement_candidates(trial, pose, object_xyz, reference_xy, face_yaw):
    """Rank swept, map-clear placements by error from the measured grasp reference."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    desired = np.asarray(object_xyz[:2]) - reference_xy
    if not np.isfinite(desired).all():
        raise ValueError('invalid cabinet object position')
    normal = np.array([math.cos(face_yaw-yaw), math.sin(face_yaw-yaw)])
    side = np.array([-normal[1], normal[0]])
    forward, lateral = desired @ normal, desired @ side
    steps = np.arange(-30, 31, 5) / 100.
    # Keep the measured object offset, including offsets between grid samples.
    lateral_steps = np.unique(np.r_[np.clip(lateral, -.30, .30), steps])
    rows = []
    for along in lateral_steps:
        for depth in np.unique(np.r_[forward, steps]):
            dx, dy = along*side + depth*normal
            length = math.hypot(dx, dy)
            if not .03 < length <= .30:
                continue
            wx, wy = c*dx - s*dy, s*dx + c*dy
            if all(trial.map_free((x+t*wx, y+t*wy, yaw))
                   for t in np.linspace(0, 1, math.ceil(length/.01)+1)):
                # A pure strafe should reduce the error to the observed target.
                if depth == 0 and abs(lateral-along) >= abs(lateral):
                    continue
                rows.append(dict(delta=[float(dx),float(dy)], goal=[x+wx,y+wy,yaw],
                                 lateral_only=bool(depth == 0)))
    rows.sort(key=lambda row: (math.dist(row['delta'], desired), not row['lateral_only'],
                              math.hypot(*row['delta'])))
    return rows


def search_views(face, pose):
    """Sample the observed cabinet frontage on either side of the first view."""
    side = np.array([-math.sin(face['yaw']), math.cos(face['yaw'])])
    origin = np.asarray(pose[:2])
    offset = (origin - face['point']) @ side
    views = []
    for limit in sorted(face['lateral_m'], key=lambda limit: -abs(limit-offset)):
        # Two distinct views within a single checked 30 cm transfer of each other.
        shift = float(np.clip((limit-offset)*.7, -.14, .14))
        if abs(shift) >= .08:
            views.append([*(origin + side*shift).tolist(), pose[2]])
    return views


def reviewed_view_shift(trial, pose, goal):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    wx, wy = goal[0]-pose[0], goal[1]-pose[1]
    if (.03 < math.hypot(wx, wy) <= .30 and
            all(trial.map_free(p) for p in NearGoal.path(trial, pose, goal))):
        return dict(goal=goal, delta=[c*wx+s*wy, -s*wx+c*wy])
    return None


def drive_to_pick_pose(ctx, hardware, navigation, selected):
    args, previous, pose = navigation
    delta = selected['delta']
    if not all(math.isfinite(v) for v in delta) or not .03 < math.hypot(*delta) <= .30:
        raise ValueError('invalid reviewed cabinet offset')
    folder = ctx.folder/'pick-navigation'
    folder.mkdir()
    options = SimpleNamespace(**vars(args))
    options.initial_pose, options.nav2_goal, options.output = pose, selected['goal'], folder
    options.manipulation_alignment = True
    options.nav2_distance = .35
    options.cabinet_front_prior = [args.observed_cabinet_face['point'], args.observed_cabinet_face['yaw']]
    options.follow_cabinet_face = True
    geometry = json.loads((CONFIG/'s4_base.nominal.json').read_text())
    trial = make_trial(options, geometry, previous.area, previous.navigation_body)
    trial.arrival_xy = min(trial.arrival_xy, CABINET_GAP_TOLERANCE_M)
    if not cabinet_gap_reached(options.nav2_goal, args.observed_cabinet_face, previous.navigation_body):
        raise ValueError('정렬 후보가 베이스 앞끝 기준 수납장 6cm 정차 범위를 벗어납니다.')
    if not trial.map_free(options.nav2_goal):
        raise ValueError('정렬 위치가 상단 차체 주행 가능 영역 밖입니다.')
    camera = hardware.camera
    hardware.camera = None
    ctx.wait(hardware.jobs.submit(camera.__exit__, None, None, None))
    with hardware.lock:
        hardware.latest = None
    arrived = drive_to_view(ctx, hardware, options, trial)
    args.observed_cabinet_face = options.observed_cabinet_face
    hardware.start_camera(ctx)
    return dict(parked=True, pose=arrived, goal=options.nav2_goal, delta=delta)
