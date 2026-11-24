"""Request-scoped, single supported release before any arm activation."""
from copy import deepcopy
from dataclasses import dataclass
from types import SimpleNamespace as NS
import pytest
from so101_arm_bridge.unified_commissioning import CommissionStatus
from tools.run import prepare_unified_controller as prep

BOARD=(0x2492E,3399362976)

@dataclass
class Hello:
    firmware_version:int=BOARD[0]
    protocol_version:int=2
    joint_count:int=12
    stop_latched:bool=False
    left_calibration_hash:int=10
    right_calibration_hash:int=20

@dataclass
class Lift:
    flags:int=0

def rig(monkeypatch,*,powered=True,fault=None):
    from test_unified_commissioning import inventory
    original=inventory()
    for d in original['devices']:
        d['sample_started_tick_ms']=0
        d['feedback']['position_raw']=2000
        d['command']['goal_speed_raw']=0
        if d['bus']=='left' and d['requested_id']>=7:d['offset_and_mode']['operating_mode']=1
    state=dict(powered=powered,tick=0,flags=17 if fault=='configured' else 0)
    events=[]
    def sample(_):
        row=deepcopy(original);state['tick']+=200
        for d in row['devices']:
            d['sample_started_tick_ms']=state['tick']
            d['command']['torque_enabled_raw']=int(state['powered'])
        if fault=='calibration':row['devices'][0]['offset_and_mode']['offset_raw']+=1
        if fault=='missing':row['all_snapshots_complete']=False
        return row
    def status(kind=1):
        return CommissionStatus(kind,1,state['tick'],42 if state['flags'] else 0,7,0,
            state['flags'],0,123 if fault=='crc' else BOARD[1])
    class Client:
        def __init__(self,*_):pass
        def query(self):return status()
        def release_boot_supported(self,**kw):
            assert kw==dict(supported=True,empty_grippers=True)
            events.append('release')
            if fault=='release_ack':raise TimeoutError('release reply lost')
            if fault!='still_on':state['powered']=False
            return status(5)
        def prepare(self,**kw):
            assert not state['powered']
            events.append('prepare');state['flags']=17;return status(2)
    monkeypatch.setattr(prep,'CommissionClient',Client)
    monkeypatch.setattr(prep,'collect_inventory',sample)
    monkeypatch.setattr(prep.time,'sleep',lambda _:None)
    monkeypatch.setattr(prep,'inspect_mobile',lambda _:dict(mobile_available=True))
    monkeypatch.setattr(prep,'LiftStatus',NS(decode=lambda _:Lift()))
    transport=NS(enter_binary_mode=lambda:Hello(),exchange_mobile=lambda _:b'')
    baseline=dict(hello=dict(left_calibration_hash=10,right_calibration_hash=20),inventory=original)
    return transport,baseline,events


@pytest.mark.parametrize('powered,confirmed,expected',[(True,True,['release','prepare']),
    (False,True,['prepare']),(False,False,['prepare'])])
def test_prepare_releases_only_retained_torque_with_explicit_confirmation(monkeypatch,powered,confirmed,expected):
    t,b,events=rig(monkeypatch,powered=powered)
    result=prep.inspect(t,prepare=True,supported=True,empty=True,baseline=b,pick_board=BOARD,
                        release_retained=confirmed)
    assert events==expected
    assert result['unpowered_stationarity']['proven']
    assert all(d['command']['torque_enabled_raw']==0 for s in result['unpowered_observations'] for d in s['devices'])
    assert result['commission_final']['flags']==17
    assert result['reset_requested'] is False


def test_existing_clients_never_release_without_new_confirmation(monkeypatch):
    t,b,events=rig(monkeypatch)
    with pytest.raises(ValueError,match='actual torque=1, lock=1, speed=0, moving=0'):
        prep.inspect(t,prepare=True,supported=True,empty=True,baseline=b,pick_board=BOARD)
    assert events==[]


@pytest.mark.parametrize('fault,expected',[
    ('calibration',[]),('missing',[]),('configured',[]),('crc',[]),
    ('release_ack',['release']),('still_on',['release'])])
def test_failed_release_or_readback_never_prepares_or_retries(monkeypatch,fault,expected):
    t,b,events=rig(monkeypatch,fault=fault)
    with pytest.raises((ValueError,RuntimeError,TimeoutError)):
        prep.inspect(t,prepare=True,supported=True,empty=True,baseline=b,pick_board=BOARD,release_retained=True)
    assert events==expected


@pytest.mark.parametrize('kw',[dict(prepare=False,supported=True,empty=True),
    dict(prepare=True,supported=False,empty=True),dict(prepare=True,supported=True,empty=False)])
def test_release_permission_is_checked_before_transport_access(kw):
    with pytest.raises(ValueError,match='supported empty preparation required'):
        prep.inspect(object(),release_retained=True,**kw)


@pytest.mark.parametrize('confirmation',[False,True])
def test_request_retains_explicit_release_confirmation(tmp_path,confirmation):
    from home_robot_tasks.pick_service import PickStation
    backend=NS(mode='station',mode_detail='',kit_id='test',options={},check_operation=lambda op,*_:op)
    station=PickStation(backend,tmp_path)
    station.data['accepting']=True
    station.submit('지우개',operation='bottom-pick',supported_empty_bottom=True,
                   supported_startup_release=confirmation)
    assert station.pending['supported_startup_release'] is confirmation


@pytest.mark.parametrize('confirmation',[True,'true',1])
def test_release_request_requires_boolean_and_supported_endpoint(tmp_path,confirmation):
    from home_robot_tasks.pick_service import PickStation,Rejected
    backend=NS(mode='station',mode_detail='',kit_id='test',options={},check_operation=lambda op,*_:op)
    station=PickStation(backend,tmp_path)
    station.data['accepting']=True
    with pytest.raises(Rejected):station.submit('지우개',operation='top-inspect',supported_startup_release=confirmation)
    assert station.pending is None


@pytest.mark.parametrize('confirmation',[False,True])
def test_preparation_passes_request_confirmation_to_hardware(monkeypatch,confirmation):
    from home_robot_tasks.pick_backend import PipelineBackend
    from home_robot_tasks.pick_web_backend import WebPickBackend
    backend=WebPickBackend(dict(mode='station',baseline='baseline.json'))
    options=[]
    monkeypatch.setattr(backend,'_prepare_hardware',lambda ctx,opts:options.append(opts))
    for method in ('capture','detect','candidates'):
        monkeypatch.setattr(PipelineBackend,method,lambda self,ctx,value:value)
    value=dict(preview_only=True,operation='bottom-pick',camera_mode='wide',supported_startup_release=confirmation)
    result=backend.prepare_pick(NS(progress=lambda _:None),value)
    assert options[0]['supported_startup_release'] is confirmation
    assert result['preview_only'] is False
