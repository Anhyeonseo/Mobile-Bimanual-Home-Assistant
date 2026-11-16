"""Bounded background geometry with control-thread snapshots and dispatch.

The worker only plans. Cancellation quarantines its result; ownership stays
busy until the worker returns, so repeated clicks cannot accumulate work.
"""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from .fetch_composition import ResolvedPort
from .fetch import InvalidTask
from .skill_ports import PortResult


class PlanningWorker:
    """One host planning owner across alignment, grasp, loaded carry and place."""
    def __init__(self):
        self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='task-geometry')
        self.pending=None
        self.closed=False
    def ready(self):return not self.closed and (self.pending is None or self.pending.done())
    def submit(self,function,request):
        if not self.ready():raise InvalidTask('shared geometry worker busy')
        self.pending=self.executor.submit(function,request)
        return self.pending
    def close(self):
        self.closed=True
        self.executor.shutdown(wait=False,cancel_futures=True)


class AsyncResolvedPort(ResolvedPort):
    def __init__(self, skill, port, policy, catalog, clock, *, before=None, timeout_s=120, worker=None):
        super().__init__(skill, port, catalog.resolve, clock, before=before)
        if not 0 < timeout_s <= 180: raise InvalidTask('bounded planning timeout required')
        self.policy, self.catalog, self.timeout = policy, catalog, timeout_s
        self.owns_worker=worker is None
        self.worker = worker or PlanningWorker()
        self.pending = None

    def ready(self):
        return super().ready() and self.worker.ready()

    def start(self, goal_id, parameters):
        if not self.ready() or goal_id in self.entries or len(self.entries) >= 128:
            raise InvalidTask('planning owned or duplicate')
        request = self.policy.snapshot(self.skill, deepcopy(parameters), self.catalog, self.clock())
        self.pending = self.worker.submit(self.policy.compute, request)
        self.entries[goal_id] = dict(status='RUNNING', reason='', attempted=False, delivered=False,
            parameters=deepcopy(parameters), request=request, future=self.pending,
            started_s=self.clock(), cancelled=False)
        self.owner = goal_id

    def cancel(self, goal_id):
        e = self.entries[goal_id]
        if e['status'] != 'RUNNING': return
        e['cancelled'] = True
        if e['attempted']: self.port.cancel(goal_id)
        else:
            e['future'].cancel()
            e['status'], self.owner = 'CANCELLED', None

    def poll(self, goal_id):
        e = self.entries[goal_id]
        if e['status'] != 'RUNNING': return PortResult(e['status'], e['reason'])
        if e['attempted']:
            result=super().poll(goal_id)
            if e.get('dispatch_failed') and result.state in {'SUCCEEDED','FAILED','CANCELLED'}:
                e['status']='FAILED'
                return PortResult('FAILED',e['reason'])
            return result
        try:
            # Cheap current-state check runs on every tick, not on the worker.
            self.policy.check_pending(e['request'], self.catalog, self.clock())
            if self.clock()-e['started_s'] > self.timeout:
                raise InvalidTask('planning timeout')
            if not e['future'].done() or not self.port.ready(): return PortResult('RUNNING')
            resolved = self.policy.commit(e['request'], e['future'].result(), self.catalog, self.clock())
            if self.before: self.before(self.skill, deepcopy(e['parameters']), self.clock())
            e['attempted'] = True
            self.port.start(goal_id, resolved)
            return PortResult('RUNNING')
        except Exception as error:
            e['reason'] = str(error)
            if e['attempted']:
                # Potentially partial dispatch remains owned until cancellation
                # is acknowledged by the shared child.
                self.port.cancel(goal_id)
                e['dispatch_failed'] = True
            else:
                e['future'].cancel(); e['status'], self.owner = 'FAILED', None
            return PortResult(e['status'], e['reason'])

    def phase_safe(self, goal_id):
        e = self.entries[goal_id]
        if e['attempted']: return self.port.phase_safe(goal_id)
        try:
            self.policy.check_pending(e['request'], self.catalog, self.clock())
            return e['status'] == 'RUNNING'
        except Exception:
            return False

    def close(self):
        if self.owner is not None: self.cancel(self.owner)
        if self.owns_worker:self.worker.close()
