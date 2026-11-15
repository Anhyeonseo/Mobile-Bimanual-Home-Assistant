from dataclasses import dataclass
from types import SimpleNamespace as NS
import pytest
from tools.run.prepare_unified_controller import move_observed_lift

@dataclass
class Status:
    session:int=1
    flags:int=29
    state:int=5
    fault:int=0
    height_um:int=0
    target_um:int=0
    command_raw:int=0
    mcu_tick_ms:int=100
    observed_ms:int=99
    def holding(self,*args):return self.state==5 and self.fault==0 and self.flags&29==29

@pytest.mark.parametrize('mode',['success','fault','wrong_arrival','lost_ownership','timeout'])
def test_rise_reuses_owned_lift_and_stop_on_failure(mode):
    clock=[0.];events=[]
    class Lift:
        mobile=NS(boot_id=53)
        active=False
        def status(self):
            if not self.active:return Status()
            if mode=='fault':return Status(state=6,fault=1)
            if mode=='lost_ownership':return Status(state=3,flags=1)
            if mode=='timeout' or clock[0]<.1:return Status(state=3,flags=7,target_um=10000,command_raw=30)
            return Status(height_um=20000 if mode=='wrong_arrival' else 10000,target_um=10000)
        def move(self,session,target):
            assert (session,target)==(2,10000);self.active=True;events.append('move')
        def keepalive(self):events.append('renew')
    class Stop:
        def request(self):events.append('STOP')
        def status(self):return NS(confirmed=True,flags=255),clock[0]
    lift=Lift()
    def observe():
        r=lift.status()
        if r.fault:raise RuntimeError('feedback fault')
        return r
    if mode=='success':
        result=move_observed_lift(lift,observe,Stop(),10000,clock=lambda:clock[0],sleep=lambda n:clock.__setitem__(0,clock[0]+n))
        assert result['final']['height_um']==10000 and events.count('move')==1 and 'renew' in events
    else:
        with pytest.raises((RuntimeError,TimeoutError)):
            move_observed_lift(lift,observe,Stop(),10000,clock=lambda:clock[0],sleep=lambda n:clock.__setitem__(0,clock[0]+n))
        assert events.count('move')==1

@pytest.mark.parametrize('height',[0,-1,10001,True,10000.])
def test_bad_target_never_opens_a_motion(height):
    with pytest.raises(ValueError):move_observed_lift(None,None,None,height)
