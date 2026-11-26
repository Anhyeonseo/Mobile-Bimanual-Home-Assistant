from dataclasses import asdict, dataclass, replace
from types import SimpleNamespace as NS
import pytest

from tools.run.unified_lift import (
    OperatorStop, held, main, rise_after_probe, stop_and_confirm, validate_target,
)


@dataclass
class Status:
    boot_id: int = 87
    session: int = 2
    state: int = 5
    fault: int = 0
    flags: int = 29
    command_raw: int = 0
    height_um: int = 10000
    target_um: int = 10000
    mcu_tick_ms: int = 100
    observed_ms: int = 99


class Clock:
    value = 0.
    def __call__(self): return self.value
    def sleep(self, seconds): self.value += seconds


class Lift:
    mobile = NS(boot_id=87)
    def __init__(self, ack_error=False):
        self.moves=[];self.renewals=0;self.ack_error=ack_error
    def move(self, session, target):
        self.moves.append((session,target))
        if self.ack_error: raise TimeoutError('lost movement ACK')
    def keepalive(self): self.renewals += 1


def probe(): return dict(target_um=10000,final=asdict(Status()))


@pytest.mark.parametrize('value',[-1,0,9,551,1000,True,550.0])
def test_reject_bad_console_target(value):
    with pytest.raises(ValueError):validate_target(value)


@pytest.mark.parametrize('value',[10,50,550])
def test_target_is_millimetres_above_bottom(value):
    assert validate_target(value)==value*1000


@pytest.mark.parametrize('flags',[0,1,25,31])
def test_new_move_requires_fresh_hold_and_interlock(flags):
    assert not held(replace(Status(),flags=flags))
    assert not held(replace(Status(),observed_ms=0,mcu_tick_ms=250))


def test_long_rise_is_single_command_after_probe_and_renews_owned_lease():
    c=Clock();lift=Lift();samples=iter([
        Status(),
        Status(session=3,state=3,flags=15,command_raw=30,height_um=12000,target_um=550000),
        Status(session=3,state=4,flags=15,height_um=549000,target_um=550000),
        Status(session=3,height_um=549500,target_um=550000),
    ])
    result=rise_after_probe(lift,lambda:next(samples),550000,probe(),clock=c,sleep=c.sleep)
    assert lift.moves==[(3,550000)] and lift.renewals==2
    assert result['final']['height_um']==549500


@pytest.mark.parametrize('initial',[
    Status(boot_id=88),Status(session=3),Status(flags=25),Status(flags=0),
    Status(height_um=50000),Status(target_um=0),Status(command_raw=10),
    Status(session=0xffffffff),Status(mcu_tick_ms=500,observed_ms=1),
])
def test_changed_checkpoint_never_starts_a_move(initial):
    lift=Lift()
    with pytest.raises(RuntimeError):rise_after_probe(lift,lambda:initial,550000,probe())
    assert not lift.moves


@pytest.mark.parametrize('p',[None,{},dict(target_um=10000),
                             dict(target_um=10000,final=asdict(Status(boot_id=99)))])
def test_old_or_missing_probe_does_not_authorize_a_long_move(p):
    lift=Lift()
    with pytest.raises((RuntimeError,ValueError)):
        rise_after_probe(lift,Status,550000,p)
    assert not lift.moves


@pytest.mark.parametrize('fault',[
    dict(boot_id=88),dict(session=4),dict(fault=1),dict(flags=14),
    dict(target_um=400000),dict(mcu_tick_ms=500,observed_ms=1),
    dict(height_um=554000),dict(state=5,flags=29,command_raw=0,height_um=30000),
    dict(flags=11),dict(state=0),dict(height_um=6000),
])
def test_bad_feedback_stops_the_loop_without_retry(fault):
    c=Clock();lift=Lift()
    moving=Status(session=3,state=3,flags=15,command_raw=30,height_um=12000,target_um=550000)
    samples=iter([Status(),replace(moving,**fault)])
    with pytest.raises(RuntimeError):
        rise_after_probe(lift,lambda:next(samples),550000,probe(),clock=c,sleep=c.sleep)
    assert lift.moves==[(3,550000)] and lift.renewals==0


def test_missing_move_ack_never_retries_or_renews_unknown_session():
    lift=Lift(ack_error=True)
    with pytest.raises(TimeoutError):rise_after_probe(lift,Status,550000,probe())
    assert lift.moves==[(3,550000)] and lift.renewals==0


@pytest.mark.parametrize('problem',['backward_clock','observation_gap','operator_stop'])
def test_control_loop_interruption_does_not_continue(problem):
    c=Clock();lift=Lift();calls=0
    def observe():
        nonlocal calls
        calls+=1
        if calls==1:return Status()
        if problem=='operator_stop':raise OperatorStop('terminal closed')
        c.value=-1 if problem=='backward_clock' else .26
        return Status(session=3,state=3,flags=15,command_raw=30,target_um=550000)
    with pytest.raises(RuntimeError):
        rise_after_probe(lift,observe,550000,probe(),clock=c,sleep=c.sleep)
    assert len(lift.moves)==1 and lift.renewals==0


def test_motion_deadline_prevents_infinite_lease_renewal():
    c=Clock();lift=Lift()
    def observe():
        if not lift.moves:return Status()
        return Status(session=3,state=3,flags=15,command_raw=30,target_um=20000)
    with pytest.raises(TimeoutError):
        rise_after_probe(lift,observe,20000,probe(),clock=c,sleep=c.sleep)
    assert len(lift.moves)==1 and c.value<41


@dataclass
class Ack:
    accepted: bool=True


@dataclass
class Proof:
    confirmed: bool=True


@pytest.mark.parametrize('mode',['normal','lost_ack','unconfirmed','status_error'])
def test_stop_proof_is_independent_of_lift_ack_and_never_releases(mode):
    c=Clock();events=[];report={}
    class Stop:
        def request(self):
            events.append('STOP')
            if mode=='lost_ack':raise TimeoutError('lost STOP ACK')
            return Ack()
        def status(self):
            events.append('query')
            if mode=='status_error':raise TimeoutError('no proof')
            return Proof(confirmed=mode!='unconfirmed'),c()
    stop_and_confirm(Stop(),report,clock=c,sleep=c.sleep)
    assert events.count('STOP')==1
    assert report['stop_confirmed']==(mode in ('normal','lost_ack'))
    if mode in ('unconfirmed','status_error'):assert 'stop_error' in report
    assert set(events)<= {'STOP','query'}


def test_execute_without_physical_invariants_exits_before_importing_serial():
    with pytest.raises(SystemExit) as exc:main(['--execute'])
    assert exc.value.code==2


@pytest.mark.parametrize('scenario',['continue','early_g_then_stop','lost_activation_ack','read_only'])
def test_console_session_gates_motion_and_always_stops_after_activation(monkeypatch,tmp_path,scenario):
    """Full console with injected devices: no serial/terminal/robot access."""
    from contextlib import nullcontext
    from pathlib import Path
    import importlib
    import json
    import sys
    from tools.run import unified_lift as tool
    import prepare_unified_controller as prep
    from home_robot_tasks import platform_preparation as platform
    root=Path(__file__).resolve().parents[1]
    baseline=tmp_path/'baseline.json';baseline.write_text('{}')
    out=tmp_path/'log';events=[];c=Clock()
    monkeypatch.setattr(tool.time,'monotonic',c)
    monkeypatch.setattr(tool.time,'sleep',c.sleep)
    monkeypatch.setattr(tool.signal,'signal',lambda *_:None)
    monkeypatch.setattr(tool.sys,'stdin',NS(isatty=lambda:True,fileno=lambda:42))
    monkeypatch.setattr(tool.termios,'tcgetattr',lambda _:['original'])
    monkeypatch.setattr(tool.termios,'tcsetattr',lambda *_:events.append('terminal_restored'))
    monkeypatch.setattr(tool.tty,'setcbreak',lambda *_:None)
    config=json.loads((root/'config/home_assistant/arm_ranges.development.json').read_text())
    devices=[dict(bus=side,requested_id=i,complete=True,
                  command=dict(torque_enabled_raw=0),feedback=dict(position_raw=2048))
             for side,ids in (('left',range(1,11)),('right',range(1,7))) for i in ids]
    def inspect(*_,report,**__):
        events.append('inspect')
        report.update(status='CONNECTED_NOT_ARMED',inventory_before=dict(devices=devices),
                      commission_final=dict(flags=17,boot_id=87,profile_crc32=tool.EXPECTED_PROFILE_CRC32))
    monkeypatch.setattr(prep,'inspect',inspect)
    def patch(module,name,value):
        monkeypatch.setattr(importlib.import_module('so101_arm_bridge.'+module),name,value)
    patch('backend_lease','acquire_backend_lease',lambda *_:nullcontext())
    patch('serial_port','open_exclusive_serial',lambda *_:nullcontext(object()))
    patch('stream_transport_v2','StreamValidationTransportV2',
          lambda *_,**__:NS(heartbeat=lambda:NS(stop_latched=False,status_code=0)))
    patch('stream_transport_v2','MobileV2Exchange',lambda x:x)
    @dataclass
    class Limits:
        hardware_authorized:bool=False
    @dataclass
    class Snapshot:
        positions_urad:tuple=(0,)*12
    class Arm:
        motion_limits=Limits();state=NS(value='ready');prepared_state=Snapshot()
        def __init__(self,*_,**__):pass
        def prepare(self):events.append('arm_prepare');return self.prepared_state
        def start_current_pose_hold(self,*_):events.append('arm_hold')
        def feedback_snapshot(self):return Snapshot()
    patch('unified_arm_adapter','UnifiedArmAdapter',Arm)
    class Mobile:
        boot_id=87
        def __init__(self,*_):pass
        def synchronize(self):events.append('clock_sync')
    patch('mobile_client','MobileClient',Mobile)
    ls=[Status(height_um=0,target_um=0,session=1)]
    class FakeLift:
        def __init__(self,mobile):self.mobile=mobile
        def status(self):return ls[0]
    patch('lift_wire','LiftClient',FakeLift)
    class Evidence:
        def __init__(self,*_,**__):pass
        def publish_mcu_snapshot(self,*_,**__):events.append('observation')
    patch('robot_evidence','RobotEvidenceClient',Evidence)
    class Commission:
        def __init__(self,*_):pass
        def activate_zero(self,**_):
            events.append('activate')
            if scenario=='lost_activation_ack':raise TimeoutError('lost activation ACK')
            return Ack()
    patch('unified_commissioning','CommissionClient',Commission)
    class Stop:
        def __init__(self,*_):pass
        def request(self):events.append('STOP');return Ack()
        def status(self):return Proof(),c()
    patch('system_stop','SystemStopClient',Stop)
    monkeypatch.setattr(platform,'measured_hold_source',lambda *_:None)
    def reference(observe,*_):events.append('reference');observe()
    monkeypatch.setattr(platform,'reference_supported_bottom',reference)
    def first_probe(lift,observe,stop,target):
        assert target==10000
        events.append('probe');ls[0]=Status();observe()
        return probe()
    monkeypatch.setattr(platform,'move_observed_lift',first_probe)
    def long_rise(lift,observe,target,p):
        assert 'fresh_g' in events and p==probe()
        events.append('long_rise');ls[0]=Status(session=3,height_um=target,target_um=target)
        observe()
        return dict(target_um=target,final=asdict(ls[0]))
    monkeypatch.setattr(tool,'rise_after_probe',long_rise)
    pending=[];early_sent=False;fresh_sent=False
    def input_ready(*_):
        nonlocal early_sent,fresh_sent
        state_path=out/'state.json'
        state=json.loads(state_path.read_text())['status'] if state_path.exists() else ''
        if state=='PREPARING_HOLD' and not early_sent:
            pending.append(b'g');early_sent=True;events.append('early_g')
        elif state=='WAITING_UP_CHECK' and not fresh_sent:
            pending.append(b'g' if scenario=='continue' else b'x');fresh_sent=True
            events.append('fresh_g' if scenario=='continue' else 'operator_stop')
        elif state=='TARGET_HOLDING':pending.append(b'x')
        return ([42],[],[]) if pending else ([],[],[])
    monkeypatch.setattr(tool.select,'select',input_ready)
    monkeypatch.setattr(tool.os,'read',lambda *_:pending.pop(0))
    args=['--root',str(root),'--baseline',str(baseline),'--output',str(out)]
    if scenario!='read_only':args+=['--execute','--supported-empty-bottom','--swept-clearance-confirmed']
    code=tool.main(args)
    if scenario=='read_only':
        assert code==0 and events==['inspect']
    else:
        assert events.count('activate')==events.count('STOP')==1
        assert 'terminal_restored' in events
        assert ('long_rise' in events)==(scenario=='continue')
        if scenario=='lost_activation_ack':
            assert code==2 and 'reference' not in events and 'probe' not in events
        else:assert code==0 and events.index('STOP')>events.index('probe')
        assert json.loads((out/'state.json').read_text())['stop_confirmed']
