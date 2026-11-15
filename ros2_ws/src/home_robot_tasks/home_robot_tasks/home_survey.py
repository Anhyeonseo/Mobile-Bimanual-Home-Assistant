"""Observed-map frontier selection and the application's local survey rehearsal.

The planner only sees observations. HiddenSyntheticHouse is a sensor fixture,
not an input map for planning and never a hardware adapter. Navigation uses the
existing application's executor, lease, cancellation and stop confirmation.
"""
from collections import deque
from copy import deepcopy
import math
import hashlib
import uuid

from .mapping_checkpoint import MappingCheckpoints, ResumeEvidence

from .fetch import InvalidTask
from .navigation import NavigationMap


def next_view(map_document, xy, visited=(), *, view_radius_cells=4):
    nav = NavigationMap(map_document)
    start = nav._cell(*xy)
    queue, distance = deque([start]), {start: 0}
    while queue:
        x, y = queue.popleft()
        for p in ((x-1,y), (x+1,y), (x,y-1), (x,y+1)):
            if (0 <= p[0] < nav.width and 0 <= p[1] < nav.height and
                    p not in nav.blocked and p not in distance):
                distance[p] = distance[(x,y)] + 1
                queue.append(p)
    choices = []
    for (x, y), cost in distance.items():
        if (x, y) in visited or cost == 0:
            continue
        # Unknown information gain is a proposal; occlusion is resolved by the
        # next real observation, never by marking these cells free in advance.
        gain = sum(nav.rows[b][a] == '?' for b in range(max(0,y-view_radius_cells), min(nav.height,y+view_radius_cells+1))
                   for a in range(max(0,x-view_radius_cells), min(nav.width,x+view_radius_cells+1))
                   if (a-x)**2 + (b-y)**2 <= view_radius_cells**2)
        if gain:
            choices.append((gain/(1+.3*cost), -cost, -y, -x, (x,y)))
    if not choices:
        return None
    cell = max(choices)[-1]
    return dict(cell=cell, xy=(nav.ox+(cell[0]+.5)*nav.resolution,
                              nav.oy+(cell[1]+.5)*nav.resolution))


class HiddenSyntheticHouse:
    """Occluding ray fixture; truth is never handed to next_view()."""
    def __init__(self, map_document, *, elevated_cells=(), radius_cells=4):
        self.truth = NavigationMap(map_document)
        self.elevated = set(elevated_cells)
        self.radius = radius_cells
        self.rows = [['?']*self.truth.width for _ in range(self.truth.height)]
        self.sequence = 0

    def capture(self, xy):
        n = self.truth
        cx, cy = math.floor((xy[0]-n.ox)/n.resolution), math.floor((xy[1]-n.oy)/n.resolution)
        visible = set()
        for i in range(360):
            a = i*math.pi/180
            for step in range(self.radius*5+1):
                x, y = int(math.floor(cx+.5+math.cos(a)*step/5)), int(math.floor(cy+.5+math.sin(a)*step/5))
                if not (0 <= x < n.width and 0 <= y < n.height):
                    break
                char = '#' if (x,y) in self.elevated else n.rows[y][x]
                self.rows[y][x] = char
                visible.add((x,y))
                if char != '.':
                    break
        self.sequence += 1
        value = deepcopy(n._document)
        rows = [''.join(r) for r in self.rows]
        value.update(rows=rows, revision='survey-'+hashlib.sha256('\n'.join(rows).encode()).hexdigest()[:24])
        return value, visible


class SimulatedHomeSurvey:
    """One-button local rehearsal. Never selected for physical adapters.

    The persisted scan is the workflow reservation; each leg takes the existing
    robot lease. A server restart pauses the workflow; it never replays motion.
    """
    def __init__(self, service):
        self.service = service
        self.reference_document = deepcopy(service.app.map._document)
        self.sensor = None
        self.session_id = None
        self.visited = set()
        self.child_id = None
        self.pending = None
        self.start_xy = None
        self.started_s = None
        self.submitting = False
        self.returning = False
        self.boot_id = 'boot-' + uuid.uuid4().hex
        self.checkpoints = MappingCheckpoints(service.store)
        self.readiness_provider = self._simulation_readiness
        self.resume_requested_s = None
        for scan in service.scan.sessions():
            if scan.get('autonomous') and scan['status'] not in {'SAVED', 'DISCARDED'}:
                try:
                    self.checkpoints.restore_scan(scan['session_id'], service.context)
                except InvalidTask as error:
                    scan.update(status='PAUSED', survey_state='RECOVERY_REQUIRED',
                        power_disconnect_allowed=False, simulated_shutdown_ready=False,
                        recovery_error=str(error), resume_checks_required=True)
                    service.store.save_record('scan', scan['session_id'], scan)

    def start(self, session_id, name, now_s):
        s, app = self.service, self.service.app
        previous = s.store.record('scan', session_id)
        if previous:
            if previous['name'] != name or not previous.get('autonomous'):
                raise InvalidTask('scan_id_conflict')
            return previous
        if app.adapter.mode != 'simulation' or not app.exact_goal_simulation:
            raise InvalidTask('survey_sensor_adapter_not_commissioned')
        if not app.pose_known:
            raise InvalidTask('localization_required')
        scan = s.scan.start(session_id, name, mode='simulation')
        self.session_id = session_id
        self.sensor = HiddenSyntheticHouse(self.reference_document)
        self.start_xy = tuple(app.xy)
        self.started_s = now_s
        self.visited, self.child_id, self.pending, self.returning = set(), None, None, False
        scan.update(autonomous=True, survey_state='OBSERVING', survey_legs=0,
                    start_xy=list(self.start_xy), hardware_enabled=False,
                    hints=['주변을 확인하며 다음 관측 위치를 고릅니다. 현재는 가상 집 체험입니다.'])
        s.store.save_record('scan', session_id, scan)
        return scan

    def _save(self, **values):
        scan = self.service.scan.get(self.session_id)
        scan.update(values)
        self.service.store.save_record('scan', self.session_id, scan)
        return scan

    def _simulation_readiness(self, now_s, revision):
        app = self.service.app
        # Read the CURRENT simulation plant, never the checkpoint pose. A
        # commissioned implementation must inject measured readiness instead.
        from .fake_robot import FakeRobot
        if not isinstance(app.adapter, FakeRobot) or not app.exact_goal_simulation:
            raise InvalidTask('survey_sensor_adapter_not_commissioned')
        failed = app.adapter.false_conditions
        return ResumeEvidence(self.boot_id, now_s-app.adapter.stale_by_s, revision,
            self.service.context.calibration_id, tuple(getattr(app.adapter, 'xy', app.xy)),
            'localized' not in failed, 'hardware_ready' not in failed,
            'arm_in_transport_pose' not in failed, 'lift_in_transport_position' not in failed,
            app.lease.owner is None and 'base_stopped' not in failed and
            'lift_stopped' not in failed and abs(getattr(app.adapter, 'velocity', 0.)) < 1e-8,
            'simulation')

    def _attach(self, session_id):
        scan = self.service.scan.get(session_id)
        if scan.get('recovery_error'):
            raise InvalidTask(scan['recovery_error'])
        self.session_id = session_id
        self.sensor = HiddenSyntheticHouse(self.reference_document)
        self.start_xy = tuple(scan['start_xy'])
        self.visited = {tuple(c) for c in scan.get('visited_cells', [])}
        if scan.get('map_document'):
            self.sensor.rows = [list(r) for r in scan['map_document']['rows']]
            self.sensor.sequence = scan.get('sensor_sequence', 0)
        self.child_id, self.pending, self.returning = None, None, False

    def _resume(self, now_s):
        app, scan = self.service.app, self.service.scan.get(self.session_id)
        if app.lease.owner or app.adapter.mode != 'simulation':
            raise InvalidTask('scan_cannot_resume')
        current_map = scan.get('map_document') or self.reference_document
        try:
            evidence = self.readiness_provider(now_s, current_map['revision'])
            if not isinstance(evidence, ResumeEvidence):
                raise InvalidTask('typed_resume_evidence_required')
            xy = evidence.check(boot_id=self.boot_id, map_revision=current_map['revision'],
                calibration_id=self.service.context.calibration_id,
                requested_s=self.resume_requested_s, now_s=now_s, mode='simulation')
            nav = NavigationMap(current_map)
            nav._cell(*xy)
        except InvalidTask:
            return self._save(survey_state='RECOVERY_REQUIRED', resume_checks_required=True,
                hints=['현재 위치와 센서·팔·리프트 상태를 확인하지 못했습니다. 이동하지 않고 기다립니다.'])
        app.replace_map(nav, now_s)
        app.localize_simulated(xy, now_s)
        old_context = scan['context']
        self._save(context=self.service.context.to_dict(), restored_from_context=old_context,
            captures=[], surface_views=[], navigation_build=None, navigation_map_document=None,
            navigation_ready=False, power_disconnect_allowed=False, simulated_shutdown_ready=False,
            resume_checks_required=False, exit_requested=False, recovery_error=None)
        self.service.scan.resume(self.session_id)
        self.started_s, self.resume_requested_s = now_s, None
        return self._save(survey_state='OBSERVING', hints=['현재 상태를 확인했습니다. 주변을 새로 관측하며 이어갑니다.'])

    def action(self, session_id, action, now_s):
        app, scan = self.service.app, self.service.scan.get(session_id)
        if not scan.get('autonomous') or scan['status'] not in {'SCANNING', 'PAUSED', 'REVIEW'}:
            raise InvalidTask('active_survey_required')
        if self.pending == 'save_exit' and action != 'save_exit':
            raise InvalidTask('save_exit_in_progress')
        if session_id != self.session_id or self.sensor is None:
            if app.lease.owner:
                raise InvalidTask('robot_busy')
            self._attach(session_id)
        if action == 'resume':
            if scan['status'] != 'PAUSED' or app.lease.owner:
                raise InvalidTask('scan_cannot_resume')
            self.resume_requested_s = now_s
            self._save(power_disconnect_allowed=False, simulated_shutdown_ready=False)
            return self._resume(now_s)
        if action not in {'pause', 'review', 'discard', 'save_exit'}:
            raise InvalidTask('unsupported_survey_action')
        self.resume_requested_s = None
        if action == 'save_exit' and scan.get('simulated_shutdown_ready'):
            return scan
        self.pending = action
        if action == 'save_exit':
            self._save(exit_requested=True, simulated_shutdown_ready=False, power_disconnect_allowed=False)
        if self.child_id and app.lease.owner == self.child_id:
            app.cancel(self.child_id, now_s)
            return self._save(survey_state='STOPPING', hints=['멈춘 것을 확인한 뒤 지도를 저장합니다.' if action == 'save_exit'
                                                           else '멈춘 것을 확인한 뒤 요청을 반영합니다.'])
        if app.lease.owner:
            raise InvalidTask('robot_busy')
        return self._finish_action()

    def _checkpoint(self):
        result = self.checkpoints.save(self.session_id, stopped=self.service.app.lease.owner is None)
        # Read back all assets; disk errors never result in a ready indication.
        verified = self.checkpoints.load(self.session_id)
        if verified['checkpoint_id'] != result['checkpoint_id']:
            raise InvalidTask('checkpoint_readback_failed')
        self._save(checkpoint_id=result['checkpoint_id'], checkpoint_fallback_used=False)
        return result

    def _finish_action(self):
        action, self.pending = self.pending, None
        self.child_id = None
        if action == 'save_exit':
            try:
                scan = self.service.scan.get(self.session_id)
                if not scan.get('map_document'):
                    observed, _ = self.sensor.capture(self.service.app.xy)
                    self.service.scan.ingest_map(self.session_id, observed)
                self._save(status='PAUSED', survey_state='SAVING', power_disconnect_allowed=False)
                saved = self._checkpoint()
                if not saved['durable']:
                    raise InvalidTask('persistent_storage_required_for_exit')
            except Exception:
                self._save(status='PAUSED', survey_state='SAVE_FAILED', simulated_shutdown_ready=False,
                    power_disconnect_allowed=False, hints=['저장을 완료하지 못했습니다. 전원을 유지하고 저장을 다시 시도해 주세요.'])
                return self.service.scan.get(self.session_id)
            return self._save(status='PAUSED', survey_state='SAVED_FOR_EXIT', exit_requested=False,
                simulated_shutdown_ready=True, power_disconnect_allowed=False,
                hints=['이전 지도 이어 만들기로 재개할 수 있도록 저장했습니다.',
                       '가상 실행 종료 준비가 끝났습니다. 실제 로봇의 전원 차단 허가는 아닙니다.'])
        scan = getattr(self.service.scan, action)(self.session_id)
        return self._save(survey_state=scan['status'], hints=[
            '현재까지의 지도는 보관했습니다. 이어하기로 다시 둘러볼 수 있습니다.'
            if action == 'pause' else '현재까지 확인한 지도를 검토할 수 있습니다.'])

    def tick(self, now_s):
        if not self.session_id:
            return
        s, app = self.service, self.service.app
        scan = s.scan.get(self.session_id)
        if scan['status'] == 'PAUSED' and self.resume_requested_s is not None:
            if now_s-self.resume_requested_s > 5:
                self.resume_requested_s = None
                self._save(hints=['현재 상태 확인을 마치지 못했습니다. 준비되면 이전 지도 이어 만들기를 다시 눌러 주세요.'])
            else:
                self._resume(now_s)
            return
        if scan['status'] != 'SCANNING':
            return
        if self.child_id:
            result = app.status(self.child_id)
            state = result['status']
            if state not in {'SUCCEEDED', 'FAILED', 'CANCELLED'}:
                return
            self.child_id = None
            if self.pending:
                self._finish_action()
                return
            if state != 'SUCCEEDED':
                s.scan.pause(self.session_id)
                self._save(survey_state='PAUSED', hints=['이동을 완료하지 못해 멈췄습니다. 주변을 확인한 뒤 이어갈 수 있습니다.'])
                return
            if self.returning:
                s.scan.review(self.session_id)
                self._save(survey_state='REVIEW', hints=['돌아볼 수 있는 구역의 탐색을 마쳤습니다. 닫힌 문 뒤와 가려진 곳은 미확인으로 남습니다.'])
                return
        if app.lease.owner:
            raise InvalidTask('unexpected_task_during_survey')
        observed, visible = self.sensor.capture(app.xy)
        s.scan.ingest_map(self.session_id, observed)
        app.replace_map(NavigationMap(observed), now_s)
        app.localize_simulated(app.xy, now_s)
        nav = app.map
        self.visited.add(nav._cell(*app.xy))
        self._save(last_confirmed_xy=list(app.xy), visited_cells=[list(c) for c in sorted(self.visited)],
                   sensor_sequence=self.sensor.sequence)
        self._checkpoint()
        view = next_view(observed, app.xy, self.visited)
        scan = s.scan.get(self.session_id)
        limit = scan['survey_legs'] >= 64 or now_s-self.started_s >= 1200
        if view is None or limit:
            self.returning = True
            goal = self.start_xy
            if math.dist(goal, app.xy) < nav.resolution/2:
                s.scan.review(self.session_id)
                self._save(survey_state='REVIEW', hints=['접근 가능한 구역을 확인했습니다. 미관측 구역은 지도에 남겨 두었습니다.'])
                return
        else:
            goal = view['xy']
        leg = scan['survey_legs']+1
        rid = f'{self.session_id[:48]}-{uuid.uuid4().hex[:12]}-leg-{leg}'
        # Durable workflow identity precedes any executor effect.
        self._save(survey_state='RETURNING' if self.returning else 'EXPLORING',
                   survey_legs=leg, child_request_id=rid,
                   hints=['출발한 곳으로 돌아갑니다.' if self.returning else '아직 확인하지 못한 곳을 향해 이동합니다.',
                          '가상 센서로 동작 흐름을 확인 중입니다. 실제 센서 검증은 별도입니다.'])
        request = dict(schema_version=1, operation='navigate_to', request_id=rid,
            map_id=nav.map_id, map_revision=nav.revision, floor_id=nav.floor_id,
            frame_id=nav.frame_id, point=dict(x=goal[0],y=goal[1],z=nav.floor_z), yaw_rad=0.)
        self.submitting = True
        try:
            app.submit(request, now_s)
            self.child_id = rid
        except InvalidTask:
            s.scan.pause(self.session_id)
            self._save(survey_state='PAUSED', hints=['다음 위치로 갈 수 없어 멈췄습니다. 현재까지의 지도는 보관했습니다.'])
        finally:
            self.submitting = False
