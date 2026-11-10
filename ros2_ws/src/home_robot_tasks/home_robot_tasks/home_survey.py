"""Observed-map frontier selection and the application's local survey rehearsal.

The planner only sees observations. HiddenSyntheticHouse is a sensor fixture,
not an input map for planning and never a hardware adapter. Navigation uses the
existing application's executor, lease, cancellation and stop confirmation.
"""
from collections import deque
from copy import deepcopy
import math
import hashlib

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

    def action(self, session_id, action, now_s):
        if session_id != self.session_id or self.sensor is None:
            scan = self.service.scan.get(session_id)
            app = self.service.app
            if not scan.get('autonomous') or scan['status'] != 'PAUSED' or app.lease.owner:
                raise InvalidTask('paused_survey_required')
            if action in {'review', 'discard', 'pause'}:
                return getattr(self.service.scan, action)(session_id)
            if action != 'resume' or app.adapter.mode != 'simulation' or not app.exact_goal_simulation:
                raise InvalidTask('survey_sensor_adapter_not_commissioned')
            self.session_id = session_id
            self.sensor = HiddenSyntheticHouse(self.reference_document)
            self.start_xy = tuple(scan['start_xy'])
            self.started_s = now_s
            self.visited = {tuple(c) for c in scan.get('visited_cells', [])}
            if scan.get('map_document'):
                self.sensor.rows = [list(r) for r in scan['map_document']['rows']]
                self.sensor.sequence = scan.get('sensor_sequence', 0)
                app.replace_map(NavigationMap(scan['map_document']), now_s)
                app.localize_simulated(tuple(scan.get('last_confirmed_xy', self.start_xy)), now_s)
            self.child_id, self.pending, self.returning = None, None, False
        app, scan = self.service.app, self.service.scan.get(session_id)
        if action == 'resume':
            if scan['status'] != 'PAUSED' or app.lease.owner:
                raise InvalidTask('scan_cannot_resume')
            # The synthetic plant stops at the last confirmed pose; real
            # integration must obtain measured pose, never copy this shortcut.
            app.localize_simulated(app.xy, now_s)
            self.pending = None
            self.service.scan.resume(session_id)
            return self._save(survey_state='OBSERVING')
        if action not in {'pause', 'review', 'discard'}:
            raise InvalidTask('unsupported_survey_action')
        self.pending = action
        if self.child_id and app.lease.owner == self.child_id:
            app.cancel(self.child_id, now_s)
            return self._save(survey_state='STOPPING', hints=['멈춘 것을 확인한 뒤 요청을 반영합니다.'])
        return self._finish_action()

    def _finish_action(self):
        action, self.pending = self.pending, None
        self.child_id = None
        scan = getattr(self.service.scan, action)(self.session_id)
        return self._save(survey_state=scan['status'], hints=[
            '현재까지의 지도는 보관했습니다. 이어하기로 다시 둘러볼 수 있습니다.'
            if action == 'pause' else '현재까지 확인한 지도를 검토할 수 있습니다.'])

    def tick(self, now_s):
        if not self.session_id:
            return
        s, app = self.service, self.service.app
        scan = s.scan.get(self.session_id)
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
        rid = f'{self.session_id[:64]}-leg-{leg}'
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
