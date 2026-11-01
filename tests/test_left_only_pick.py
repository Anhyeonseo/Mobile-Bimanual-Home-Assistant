"""Left-only software scenarios: six measured axes, no phantom feedback."""
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import sys
import pytest
from test_fixed_pick_demo import cli, recipe_for
from test_filming_workflows import FilmingTransport, Clock
from so101_arm_bridge.left_only_adapter import (
    LeftOnlyAdapter, left_limits, LEFT_FIRMWARE_VERSION, LEFT_CAPABILITIES,
)
from so101_arm_bridge.bimanual_stream_adapter import (
    load_operational_limits, ResidentBimanualStreamAdapter, BimanualStreamError,
)
from so101_arm_bridge.fixed_pick_demo import (
    new_recipe, limits_digest, check_recipe, compose_targets, execute_leg,
    capture_pose, OWNER, load_json,
)
from so101_arm_bridge.stream_protocol_v2 import BimanualFeedbackSnapshotV2
ROOT=Path(__file__).resolve().parents[1]
FULL=load_operational_limits(ROOT/'config/bimanual_operational_limits.json')
LIMITS=left_limits(FULL)
DIGEST=limits_digest(ROOT/'config/bimanual_operational_limits.json')

def recipe():
    value=new_recipe('left',DIGEST,left_only=True)
    value['poses']=recipe_for('left')['poses']
    for p in value['poses'].values(): p['positions_rad']=p['positions_rad'][:6]
    return value

class LeftTransport(FilmingTransport):
    def __init__(self, clock, home): super().__init__(clock,tuple(home)+(0.,)*6)
    def enter_binary_mode(self):
        h=super().enter_binary_mode()
        h.joint_count=6;h.firmware_version=LEFT_FIRMWARE_VERSION;h.capabilities=LEFT_CAPABILITIES
        return h
    def prepare_shadow(self):
        s=super().prepare_shadow();s.joint_count=6;s.right_present_mask=0
        s.positions_raw=s.positions_raw[:6]+(0,)*6
        s.unwrapped_positions_raw=s.unwrapped_positions_raw[:6]+(0,)*6
        return s
    @staticmethod
    def accepted(epoch=0):
        s=FilmingTransport.accepted(epoch);s.arm_mask=1;return s
    def open_stream(self,policy):
        assert policy.arm_mask==1
        return super().open_stream(policy)
    def append(self,batch):
        assert batch.arm_mask==1
        assert all(s.positions_urad[6:]==(0,)*6 for s in batch.samples)
        return super().append(batch)
    def get_feedback_snapshot(self):
        s=super().get_feedback_snapshot()
        return BimanualFeedbackSnapshotV2(0,6,0x3f,0,0,s.firmware_tick_ms,
            s.completed_pairs,s.positions_urad,s.sample_age_ms[:6]+(0xffffffff,)*6)


def setup():
    rec=recipe();clock=Clock();home=rec['poses']['home']['positions_rad']
    tr=LeftTransport(clock,home)
    a=LeftOnlyAdapter(tr,LIMITS,home_reference=home,motion_authorized=True)
    return rec,clock,tr,a


def test_left_recipe_contains_only_six_real_joints_and_rejects_old_files():
    rec,clock,tr,a=setup()
    assert check_recipe(rec,LIMITS,DIGEST)['arm']=='left'
    a.prepare()
    record=capture_pose(a,LIMITS,sleep=clock.advance)
    assert len(record['positions_rad'])==6
    assert len(rec['joint_names'])==6
    with pytest.raises(ValueError): check_recipe(recipe_for('left'),LIMITS,DIGEST)
    with pytest.raises(ValueError): check_recipe(rec,FULL,DIGEST)
    with pytest.raises(ValueError): new_recipe('right',DIGEST,left_only=True)


def test_bimanual_adapter_does_not_accept_left_firmware():
    _,_,tr,_=setup()
    with pytest.raises(BimanualStreamError): ResidentBimanualStreamAdapter(tr,FULL).prepare()
    assert 'enable' not in tr.calls


@pytest.mark.parametrize('defect',['old_firmware','missing_left','right_present','fake_right_position'])
def test_invalid_physical_identity_or_feedback_blocks_arming(defect):
    _,_,tr,a=setup()
    if defect=='old_firmware':
        original=tr.enter_binary_mode
        def bad(): h=original();h.firmware_version=0x24908;return h
        tr.enter_binary_mode=bad
    else:
        original=tr.prepare_shadow
        def bad():
            s=original()
            if defect=='missing_left': s.left_present_mask=0x1f
            if defect=='right_present': s.right_present_mask=0x3f
            if defect=='fake_right_position': s.anchor_positions_urad=s.anchor_positions_urad[:6]+(1,)*6
            return s
        tr.prepare_shadow=bad
    with pytest.raises(BimanualStreamError): a.prepare()
    assert 'enable' not in tr.calls


def test_left_home_shift_blocks_first_motion():
    rec,clock,tr,a=setup();a.prepare()
    tr.shadow_positions_urad=(100000,)+tr.shadow_positions_urad[1:]
    with pytest.raises(ValueError): execute_leg(a,rec['poses']['home']['positions_rad'],LIMITS,clock=clock,sleep=clock.advance)
    assert 'enable' not in tr.calls


def test_hold_60_seconds_uses_only_left_feedback_and_exact_jaw_target(monkeypatch):
    rec,clock,tr,a=setup();monkeypatch.setattr(cli.time,'monotonic',clock);a.prepare()
    targets=compose_targets(rec,rec['poses']['home']['positions_rad'],LIMITS,DIGEST)
    prev=None
    for stage in ('home','approach','grasp','closed','lift'):
        execute_leg(a,targets[stage],LIMITS,commanded_start=prev,clock=clock,sleep=clock.advance)
        prev=targets[stage]
    began=clock(); before=len(tr.appended)
    class Commands:
        def read(self,timeout=0):
            clock.advance(max(timeout,.01));return ['return'] if clock()-began>=60 else []
    assert cli.wait_command(a,LIMITS,targets['lift'],Commands(),clock=clock,sleep=clock.advance)=='return'
    assert clock()-began>=60
    expected=tuple(round(q*1e6) for q in targets['lift'])+(0,)*6
    assert all(s.positions_urad==expected for b in tr.appended[before:] for s in b.samples)
    assert tr.safe_stop_count==0
    assert 'shadow' not in tr.calls[tr.calls.index('enable')+1:]


@pytest.mark.parametrize('exit_kind',['q','eof','interrupt','feedback_loss','stop_failure'])
def test_left_shoot_return_repeat_and_stop(tmp_path,monkeypatch,exit_kind):
    rec,clock,tr,_=setup();path=tmp_path/'poses.json';cli.save_json(path,rec)
    monkeypatch.setattr(cli.time,'monotonic',clock)
    @contextmanager
    def connection(_): yield tr
    monkeypatch.setattr(cli,'connection',connection)
    monkeypatch.setattr(sys,'stdin',SimpleNamespace(isatty=lambda:True))
    class Commands:
        started=None;action=None
        def read(self,timeout=0):
            clock.advance(max(timeout,.01))
            if self.started is None or clock()-self.started<2:return []
            action,self.started=self.action,None
            if action=='eof':raise EOFError('SSH closed')
            if action=='interrupt':raise KeyboardInterrupt()
            if action=='feedback_loss':tr.feedback_transport_errors_remaining=100;return []
            if action=='stop_failure':
                def fail():tr.safe_stop_count+=1;raise OSError('USB lost')
                tr.safe_stop=fail;return ['q']
            return [action]
        def during_motion(self): assert self.started is None
    commands=Commands();monkeypatch.setattr(cli,'TerminalCommands',lambda _:commands)
    real_leg,real_wait=cli.execute_leg,cli.wait_command
    def leg(*a,**k):
        k.update(clock=clock,sleep=clock.advance);return real_leg(*a,**k)
    actions=iter(['return','run',exit_kind])
    def wait(*a,**k):
        commands.action,commands.started=next(actions),clock()
        return real_wait(*a,**k,clock=clock,sleep=clock.advance)
    monkeypatch.setattr(cli,'execute_leg',leg);monkeypatch.setattr(cli,'wait_command',wait)
    args=SimpleNamespace(file=path,execute=True,log=None,left_only=True)
    if exit_kind=='q':cli.play(args,LIMITS,DIGEST)
    else:
        with pytest.raises((EOFError,KeyboardInterrupt,RuntimeError)):cli.play(args,LIMITS,DIGEST)
    log=load_json(path.with_name('poses.last-run.json'))
    assert len(log['events'])==13
    assert tr.safe_stop_count>=1
    assert log['stop_latched'] is (exit_kind!='stop_failure')
    assert tr.calls.count('enable')==1
    assert not log['physical_pick_verified']


def test_dedicated_left_cli_help_and_offline_check(tmp_path):
    import subprocess
    path=tmp_path/'left.json';cli.save_json(path,recipe())
    script=ROOT/'tools/run/fixed_pick_left.py'
    result=subprocess.run([sys.executable,str(script),'teach','--help'],capture_output=True,text=True,check=True)
    assert '--arm {left}' in result.stdout and '{left,right}' not in result.stdout
    subprocess.run([sys.executable,str(script),'check','--file',str(path)],capture_output=True,text=True,check=True)
    result=subprocess.run([sys.executable,str(script),'play','--file',str(path)],capture_output=True,text=True,check=True)
    assert '장치는 열지 않았습니다' in result.stdout
