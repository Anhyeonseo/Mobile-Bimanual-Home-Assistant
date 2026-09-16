"""Production coordinators over synthetic ports/images/measurements, no devices."""
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from home_robot_tasks.application import RobotApplication
from home_robot_tasks.capture_port import CapturePort, ImageDetection, ImagePair, LatestRGBDSource
from home_robot_tasks.execution import Feedback, TaskLease, invariants
from home_robot_tasks.fetch import FetchRequest, plan_fetch
from home_robot_tasks.fetch_composition import compose_fetch_adapter, ResolvedPort
from home_robot_tasks.manipulation_port import ManipulationPort, PlanningContext
from home_robot_tasks.navigation import NavigationMap
from home_robot_tasks.payload_monitor import PayloadMonitor, PayloadTaskMonitor, GripMeasurement, PayloadObservation
from home_robot_tasks.perception import CameraIntrinsics
from home_robot_tasks.search import SearchSession, Viewpoint
from home_robot_tasks.skill_ports import PortResult
from home_robot_tasks.stationary_rgbd import StationaryRGBD, CapturePolicy, MotionEvidence, RGBDFrame
from home_robot_tasks.system_stop_backend import SystemStopBackend
from home_robot_tasks.workflow_ports import SequencePort, Phase, SearchSkillPort

ROOT = Path(__file__).resolve().parents[1]

def read(name):
    return json.loads((ROOT / 'config' / name).read_text())


class Leaf:
    def __init__(self, rig, role):
        self.rig, self.role, self.entries = rig, role, {}
        self.block = self.cancel_error = False
        self.starts = []

    def ready(self):
        return not any(e['state'] == 'RUNNING' for e in self.entries.values())

    def start(self, g, p):
        assert self.ready()
        self.entries[g] = dict(state='RUNNING', polls=0)
        self.starts.append((g, p))
        if self.role == 'lift':
            self.rig.conditions['lift_stopped'] = False
        if self.role in ('lift', 'navigation', 'alignment'):
            self.rig.pose += 1
        if self.role == 'execute':
            self.rig.held = g.endswith('_pick')

    def poll(self, g):
        e = self.entries[g]
        if e['state'] == 'RUNNING' and not self.block:
            e['polls'] += 1
            if e['polls'] >= 2:
                e['state'] = 'SUCCEEDED'
                if self.role in ('lift', 'navigation', 'alignment'):
                    self.rig.since = self.rig.now
                if self.role == 'lift':
                    self.rig.conditions['lift_stopped'] = True
        return PortResult(e['state'])

    def cancel(self, g):
        if self.cancel_error:
            raise RuntimeError('cancel ACK lost')
        self.entries[g]['state'] = 'CANCELLED'
        if self.role == 'lift':
            self.rig.conditions['lift_stopped'] = True

    def result(self, g):
        if self.role == 'detector':
            return [ImageDetection('remote_control', .9, ((20, 20), (40, 20), (40, 30), (20, 30)))]
        return ('synthetic-trajectory',)


class StopClient:
    def __init__(self, rig):
        self.rig, self.requests, self.confirmed = rig, 0, False

    def request(self):
        self.requests += 1

    def status(self):
        return SimpleNamespace(confirmed=self.confirmed, flags=255), self.rig.now


class Rig:
    def __init__(self):
        self.now, self.sequence, self.held, self.sensor_enabled = 1., 0, False, True
        self.since, self.pose = 0., 0
        self.clock = lambda: self.now
        self.plan = plan_fetch(FetchRequest.from_dict(read('fetch_remote.example.json')), read('home.example.json'))
        keys = {k for s in self.plan['steps'] for k in (*s['start_requires'], *s['success_requires'], *invariants(s['skill']))}
        keys.add('arms_holding')
        self.conditions = dict.fromkeys(keys, True)  # Explicit synthetic environment, not action-generated proof.
        self.targets, self.captures, self.resolved = [], [], []
        self.camera = StationaryRGBD(CapturePolicy(width=64, height=48))
        self.motion = lambda: MotionEvidence(self.now, self.since, f'pose-{self.pose}', self.conditions['base_stopped'], self.conditions['lift_stopped'])
        self.source = LatestRGBDSource()
        self.leaves = {r: Leaf(self, r) for r in ('arm', 'lift', 'navigation', 'alignment', 'detector', 'planner', 'execute')}
        self.capture = CapturePort(self.source, self.leaves['detector'], self.camera, self.motion, self.clock)
        self.lease = TaskLease()
        self.client = StopClient(self)
        self.stop = SystemStopBackend(self.client, self.lease)
        self.payload = PayloadMonitor(gap_min_mm=5, gap_max_mm=30, open_gap_mm=40,
            effort_min_raw=20, effort_max_raw=100, empty_effort_max_raw=5,
            max_age_s=.25, max_skew_s=.05, dwell_s=.1)
        self.evidence = PayloadTaskMonitor(self.environment, self.payload)
        move = SequencePort(lambda p: [Phase('view', self.leaves['navigation'], p,
            ('base_stopped', 'lift_stopped'), ('lift_stopped',), ('base_stopped',), 5)], self.evidence, self.clock)
        self.search = SearchSkillPort(
            lambda g, p, t: SearchSession(g, p['object_id'], 'sofa', 'scene',
                [Viewpoint('front', 'sofa', ('seat',))], self.camera, now_s=t),
            {'front': {'synthetic_view': 'front'}}, move, self.capture, self.motion,
            self.stop, self.clock, self.targets.append)
        self.manipulation = ManipulationPort(self.leaves['planner'], self.leaves['execute'],
            lambda: PlanningContext(self.now, self.captures[-1][1].capture_id, f'pose-{self.pose}', 'scene',
                'simulation-only', 'base_link', (0.,)*12, True, True), self.clock)
        backends = {r: self.leaves[r] for r in ('arm', 'lift', 'navigation', 'alignment')}
        backends.update(search=self.search, capture=self.capture, manipulation=self.manipulation)
        self.adapter = compose_fetch_adapter(backends=backends, resolve=self.resolve,
            monitor=self.evidence, phase_monitor=self.evidence, stop_backend=self.stop,
            clock=self.clock, capture_sink=self.capture_sink, before_start=self.evidence.before_start)
        self.app = RobotApplication(read('home.example.json'), NavigationMap(read('navigation_map.simulation.json')),
            self.adapter, lease=self.lease, exact_goal_simulation=False)
        self.app.submit({'schema_version': 1, 'operation': 'fetch_object', 'request_id': 'fetch',
            'request': read('fetch_remote.example.json')}, self.now)
        self.task = self.app.tasks['fetch']

    def environment(self, g, step, t):
        conditions = dict(self.conditions)
        for k in ('target_observed', 'target_pose_resolved', 'fresh_target_pose'):
            conditions[k] = bool(self.targets)
        return Feedback(g, t, 'READY', conditions)

    def resolve(self, skill, p, now):
        self.resolved.append(skill)
        if skill.startswith('prepare_'):
            return {'arm': {'pose': 'synthetic-transport'}, 'height_um': 10000}
        if skill.startswith('adjust_lift_'):
            return {'height_um': 20000}
        if skill in ('reobserve_target', 'inspect_destination', 'reobserve_destination', 'verify_delivery'):
            return {'requested_s': now, 'object_id': 'remote_control'}
        return p

    def capture_sink(self, g, skill, parameters, observation):
        frame, kwargs = observation
        assert kwargs['detections'] and frame.observed_s <= self.now
        self.captures.append((skill, frame))

    def tick(self):
        self.now = round(self.now + .05, 6)
        self.sequence += 1
        frame = RGBDFrame(f'frame-{self.sequence}', self.now, f'pose-{self.pose}', 'camera_color_optical_frame',
            'simulation-only', 64, 48, CameraIntrinsics(50, 50, 32, 24), 'base_link', self.now,
            ((1,0,0,0), (0,1,0,0), (0,0,1,0), (0,0,0,1)), True)
        self.source.publish(ImagePair(frame, np.zeros((48,64,3), np.uint8), np.ones((48,64), np.float32), self.now, ('seat',), ()))
        if self.sensor_enabled:
            self.payload.observe_grip(GripMeasurement(self.sequence, self.now, 15 if self.held else 45, 40 if self.held else 0, True), self.now)
            self.payload.observe_vision(PayloadObservation(self.sequence, self.now, 'remote_control',
                'held' if self.held else 'empty', 'bedroom_drop_zone', not self.held), self.now)
        return self.app.tick(self.now)

    def reach(self, skill):
        for _ in range(300):
            self.tick()
            if self.task.step['skill'] == skill and self.task.state == 'RUNNING':
                return
            assert self.task.state not in self.task.TERMINAL, self.task.result()
        pytest.fail(str(self.task.result()))


def test_whole_fetch_uses_real_search_capture_manipulation_and_independent_payload():
    r = Rig()
    saw_moving_lift = False
    for _ in range(300):
        r.tick()
        saw_moving_lift |= not r.conditions['lift_stopped']
        if r.task.state in r.task.TERMINAL:
            break
    result = r.task.result()
    assert result['status'] == 'SUCCEEDED', result
    assert result['completed_steps'] == 15 and saw_moving_lift
    assert r.resolved == [s['skill'] for s in r.plan['steps']]
    assert len(r.targets) == 1 and len(r.captures) == 4
    assert len(r.leaves['planner'].starts) == len(r.leaves['execute'].starts) == 2
    assert r.payload.proof(r.now).conditions['release_verified'] and not r.payload.carrying
    assert r.client.requests == 0 and r.lease.owner is None
    assert not result['physical_task_completed'] and result['hardware_commands'] == 0


@pytest.mark.parametrize('skill', [s['skill'] for s in plan_fetch(FetchRequest.from_dict(read('fetch_remote.example.json')), read('home.example.json'))['steps']])
def test_composed_cancel_waits_for_whole_stop_and_rejects_late_child(skill):
    r = Rig(); r.reach(skill); r.tick()
    assert r.app.cancel('fetch', r.now)['status'] == 'STOPPING'
    r.tick()
    assert r.lease.owner == 'fetch' and r.client.requests == 1
    r.client.confirmed = True
    for _ in range(10):
        r.tick()
        if r.task.state == 'CANCELLED': break
    assert r.task.state == 'CANCELLED', r.task.result()
    assert r.lease.owner is None
    with pytest.raises(RuntimeError): r.stop.request('fetch/03_search:99', True, r.now)


def test_successful_pick_without_new_sensor_measurements_cannot_start_transport():
    r = Rig(); r.reach('pick'); r.sensor_enabled = False
    for _ in range(12): r.tick()
    assert 'prepare_transport' not in r.resolved and r.lease.owner == 'fetch'
    assert not r.payload.proof(r.now).conditions['load_retained']
    assert r.task.state in {'STOPPING', 'STOP_UNCONFIRMED', 'RUNNING'}


def test_load_loss_during_transport_requests_stop_and_keeps_lease():
    r = Rig(); r.reach('navigate_with_load')
    r.leaves['navigation'].block = True
    r.held = False
    r.tick()
    assert r.task.state == 'STOPPING' and r.client.requests == 1
    assert r.lease.owner == 'fetch'


def test_capture_sink_failure_is_terminal_and_not_replayed_during_stop():
    r = Rig(); calls = []
    leaf = r.leaves['navigation']
    def sink(*args):
        calls.append(args)
        raise RuntimeError('invalid target frame')
    p = ResolvedPort('capture', leaf, lambda *a: {}, r.clock, result_sink=sink)
    p.start('g', {}); p.poll('g')
    assert p.poll('g').state == 'FAILED'
    p.cancel('g')
    assert p.poll('g').state == 'FAILED' and len(calls) == 1 and p.owner is None


def test_phase_safety_cannot_be_forged_by_top_level_ready_flags():
    r = Rig()
    for _ in range(20):
        r.tick()
        if not r.conditions['lift_stopped']: break
    assert not r.conditions['lift_stopped']
    r.conditions['arms_safe_for_lift'] = False
    r.conditions['preparation_phase_safe'] = True
    r.tick()
    assert r.task.state == 'STOPPING' and r.client.requests == 1
    assert r.lease.owner == 'fetch'


def test_search_cancel_failure_does_not_prevent_whole_stop_or_release_early():
    r = Rig(); r.reach('search'); r.tick(); r.tick()
    nav = r.leaves['navigation']
    nav.block = nav.cancel_error = True
    r.app.cancel('fetch', r.now)
    r.client.confirmed = True
    r.tick()
    assert r.client.requests == 1 and r.lease.owner == 'fetch'
    assert r.task.state == 'STOPPING'
    # Late terminal acknowledgement, independently of the MCU stop receipt.
    for e in nav.entries.values(): e['state'] = 'CANCELLED'
    r.tick()
    assert r.task.state == 'CANCELLED' and r.lease.owner is None
