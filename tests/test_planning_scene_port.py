from copy import deepcopy
from concurrent.futures import Future
import pytest
from home_robot_tasks.planning_scene_port import SceneUpdatePort


def change(**kw):
    return dict(operation='attach',object_id='remote',arm='left',link_name='left_hand',
        frame_id='left_hand',position_m=[0.,0.,.03],quaternion_xyzw=[0.,0.,0.,1.],
        size_m=[.02,.04,.1],touch_links=['left_hand','left_finger'],observed_s=1.,**kw)


class Client:
    def __init__(self):
        self.write=Future();self.query=Future();self.sent=[];self.reads=0
    def ready(self):return True
    def apply(self,p):self.sent.append(deepcopy(p));return self.write
    def read(self):self.reads+=1;return self.query
    def matches(self,p,value):return p==value


def fixture():
    c=Client();now=[1.]
    port=SceneUpdatePort(c,lambda:now[0],gripper_links={
        'left':['left_hand','left_finger'],'right':['right_hand','right_finger']})
    return port,c,now


def test_write_ack_alone_cannot_complete_until_matching_readback():
    p,c,_=fixture();p.start('attach',change());c.write.set_result(True)
    assert p.poll('attach').state=='RUNNING' and c.reads==1 and not p.ready()
    c.query.set_result(change())
    assert p.poll('attach').state=='SUCCEEDED' and p.revision==1 and p.ready()
    assert len(c.sent)==1


@pytest.mark.parametrize('bad',[
    {'touch_links':['left_hand','base_link']},{'touch_links':['right_hand']},
    {'link_name':'right_hand'},{'frame_id':'base_link'},
    {'size_m':[0,.1,.1]},{'size_m':[2,.1,.1]},{'position_m':[float('nan'),0,0]},
    {'quaternion_xyzw':[0,0,0,0]},{'observed_s':0},{'observed_s':2},
    {'operation':'detach'},{'object_id':''},
])
def test_rejects_unobserved_pose_or_broadened_touch_permissions(bad):
    p,c,_=fixture()
    with pytest.raises(ValueError):p.start('bad',{**change(),**bad})
    assert not c.sent and p.owner is None


@pytest.mark.parametrize('mismatch',[{'size_m':[.2,.04,.1]}, {'object_id':'other'},
                                    {'link_name':'right_hand'}, {'operation':'detach'}])
def test_mismatched_readback_faults_without_retry(mismatch):
    p,c,_=fixture();p.start('a',change());c.write.set_result(True);p.poll('a')
    c.query.set_result({**change(),**mismatch})
    assert p.poll('a').state=='FAILED' and not p.ready() and p.revision==0
    assert len(c.sent)==1


@pytest.mark.parametrize('phase',['apply','read'])
def test_cancel_waits_for_outstanding_service_and_reads_actual_scene(phase):
    p,c,_=fixture();p.start('a',change())
    if phase=='read':c.write.set_result(True);p.poll('a')
    p.cancel('a')
    assert p.poll('a').state=='RUNNING' and not p.ready()
    if phase=='apply':c.write.set_result(True);p.poll('a')
    assert p.poll('a').state=='RUNNING'
    c.query.set_result(change())
    assert p.poll('a').state=='CANCELLED' and p.revision==1
    assert len(c.sent)==1  # No compensating detach on cancellation.


@pytest.mark.parametrize('phase',['dispatch','apply','read'])
def test_ambiguous_service_failure_retains_ownership_until_explicit_recovery(phase):
    p,c,_=fixture()
    if phase=='dispatch':
        def broken(_):raise RuntimeError('transport failed after send')
        c.apply=broken
    p.start('a',change())
    if phase=='apply':c.write.set_exception(RuntimeError('lost ACK'))
    if phase=='read':
        c.write.set_result(True);p.poll('a');c.query.set_exception(RuntimeError('lost read'))
    p.poll('a');p.cancel('a')
    assert p.poll('a').state=='RUNNING' and p.owner=='a' and p.fault and not p.ready()


def test_negative_ack_is_not_success_even_if_scene_already_matches():
    p,c,_=fixture();p.start('a',change());c.write.set_result(False);p.poll('a')
    c.query.set_result(change())
    assert p.poll('a').state=='FAILED' and p.revision==0 and not p.ready()
