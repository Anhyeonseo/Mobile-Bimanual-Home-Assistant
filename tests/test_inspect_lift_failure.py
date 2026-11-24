from dataclasses import dataclass
from types import SimpleNamespace as NS
import pytest
from tools.diagnostics.inspect_lift_failure import collect_controller


@dataclass
class Hello:
    firmware_version:int=0x2492d
    stop_latched:bool=True


@dataclass
class Commission:
    boot_id:int=87
    flags:int=127


@dataclass
class Proof:
    confirmed:bool=False


@dataclass
class Lift:
    state:int=6
    fault:int=1
    height_um:int=8100


@pytest.mark.parametrize('runtime_error',[False,True])
def test_latched_stop_collection_only_queries_and_preserves_partial_failures(monkeypatch,runtime_error):
    from so101_arm_bridge import unified_commissioning,system_stop,lift_wire
    events=[]
    class Transport:
        def enter_binary_mode(self):events.append('hello');return Hello()
        def stationary_runtime(self):
            events.append('cached_runtime')
            if runtime_error:raise TimeoutError('missing cached reply')
            return dict(stop_latched=True)
        def stationary_feedback_failures(self):events.append('cached_bus_errors');return {}
        def exchange_mobile(self,packet):
            command=lift_wire.LiftCommand.decode(packet)
            assert command.kind==4 and command.valid_until_ms==0 and command.target_um==0
            events.append('lift_status');return b'example_status'
    class Client:
        def __init__(self,*_):pass
        def query(self):events.append('commission_query');return Commission()
    class Stop:
        def __init__(self,*_):pass
        def status(self):events.append('stop_query');return Proof(),1.
        def request(self):raise AssertionError('STOP request is forbidden during diagnosis')
    monkeypatch.setattr(unified_commissioning,'CommissionClient',Client)
    monkeypatch.setattr(system_stop,'SystemStopClient',Stop)
    monkeypatch.setattr(lift_wire.LiftStatus,'decode',lambda _:Lift())
    r=collect_controller(Transport())
    assert events==['hello','cached_runtime','cached_bus_errors','commission_query','stop_query','lift_status']
    assert r['controller']['stop_latched'] and not r['stop_proof']['confirmed']
    assert r['lift']['height_um']==8100
    assert r['motion_commands']==r['torque_commands']==r['reset_requests']==r['reference_requests']==r['stop_requests']==0
    assert ('runtime' in r['errors'])==runtime_error


def test_wrong_firmware_stops_after_identity_query():
    r=collect_controller(NS(enter_binary_mode=lambda:Hello(firmware_version=0x24911)))
    assert 'identity' in r['errors'] and 'runtime' not in r
