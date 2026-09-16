from copy import deepcopy
import pytest

from home_robot_tasks.execution import Feedback
from home_robot_tasks.pick_place import PickPlacePort
from home_robot_tasks.skill_ports import PortResult


class Child:
    def __init__(self, role, trace):
        self.role, self.trace = role, trace
        self.states = {}
        self.cancel_completes = True

    def ready(self):
        return 'RUNNING' not in self.states.values()

    def start(self, goal, p):
        self.trace.append((self.role, goal, deepcopy(p)))
        self.states[goal] = 'RUNNING'

    def poll(self, goal):
        return PortResult(self.states[goal])

    def cancel(self, goal):
        if self.cancel_completes:
            self.states[goal] = 'CANCELLED'


def fixture(operation='pick'):
    now, trace, resolves = [1.], [], []
    ports = {name:Child(name,trace) for name in ('motion','gripper','scene')}
    conditions = dict.fromkeys(('hardware_ready','base_stopped','lift_stopped','gripper_empty',
        'gripper_open','grasp_verified','load_retained','object_attached','object_detached',
        'release_verified','arm_clear'), True)
    def resolve(stage, p, stamp):
        resolves.append(stage)
        return {'stage':stage,'object_id':p['object_id'],'resolved_at':stamp}
    port=PickPlacePort(**ports,resolve_stage=resolve,
        monitor=lambda g,p,t:Feedback(g,now[0],'READY',dict(conditions)),clock=lambda:now[0])
    port.start('task',dict(operation=operation,arm='left',object_id='remote'))
    return port,ports,conditions,now,trace,resolves


def tick(f, complete=False):
    port,ports,_,now,_,_=f
    now[0]+=.05
    if complete:
        for child in ports.values():
            for g,s in child.states.items():
                if s=='RUNNING':child.states[g]='SUCCEEDED'
    return port.poll('task')


@pytest.mark.parametrize('operation,expected',[
    ('pick',['pick_open','pick_pregrasp','pick_approach','pick_close','pick_attach','pick_retreat']),
    ('place',['place_preplace','place_approach','place_open','place_detach','place_retreat'])])
def test_exact_stage_order_and_lazy_resolution(operation,expected):
    f=fixture(operation)
    assert not f[-1]
    for _ in range(30):
        if tick(f,True).state=='SUCCEEDED':break
    assert f[-1]==expected
    assert [p['stage'] for _,_,p in f[-2]]==expected
    assert [p['phase'] for p in f[0].history('task')]==expected
    assert f[0].owner is None


def advance_to(f,stage):
    for _ in range(30):
        tick(f,True)
        if f[-1] and f[-1][-1]==stage:return
    raise AssertionError(stage)


def test_gripper_ack_does_not_prove_grasp_and_timeout_never_attaches():
    f=fixture();advance_to(f,'pick_close');f[2]['grasp_verified']=False
    tick(f,True)
    assert f[0].poll('task').state=='RUNNING' and not f[1]['scene'].states
    f[3][0]+=5
    assert f[0].poll('task').state=='FAILED'
    assert 'pick_attach' not in f[-1]


def test_release_proof_required_before_detaching_and_retreating():
    f=fixture('place');advance_to(f,'place_open');f[2]['release_verified']=False
    for _ in range(5):assert tick(f,True).state=='RUNNING'
    assert 'place_detach' not in f[-1]
    f[2]['release_verified']=True
    for _ in range(10):tick(f,True)
    assert f[0].poll('task').state=='SUCCEEDED'


def test_attach_readback_and_load_required_before_retreat():
    f=fixture();advance_to(f,'pick_attach');f[2]['object_attached']=False
    for _ in range(4):tick(f,True)
    assert 'pick_retreat' not in f[-1]
    f[2]['object_attached']=True;tick(f,True)
    f[2]['load_retained']=False
    for _ in range(5):tick(f,True)
    assert 'pick_retreat' not in f[-1]


@pytest.mark.parametrize('operation,stage',[(op,stage) for op,stages in (
    ('pick',['open','pregrasp','approach','close','attach','retreat']),
    ('place',['preplace','approach','open','detach','retreat'])) for stage in stages])
def test_cancel_each_stage_waits_for_child_without_cleanup_motion(operation,stage):
    f=fixture(operation);advance_to(f,operation+'_'+stage)
    for c in f[1].values():c.cancel_completes=False
    count=len(f[-2]);f[0].cancel('task')
    assert tick(f).state=='RUNNING' and f[0].owner=='task'
    assert len(f[-2])==count
    assert tick(f,True).state=='CANCELLED' and f[0].owner is None
    assert len(f[-2])==count


@pytest.mark.parametrize('condition',['hardware_ready','base_stopped','lift_stopped','load_retained','object_attached'])
def test_condition_loss_during_loaded_retreat_cancels(condition):
    f=fixture();advance_to(f,'pick_retreat');f[2][condition]=False
    assert tick(f).state=='FAILED'


def test_stale_monitor_cannot_start_motion():
    f=fixture();f[0].sequence.monitor=lambda g,p,t:Feedback(g,0,'READY',f[2])
    assert tick(f).state=='FAILED' and not f[-2]
