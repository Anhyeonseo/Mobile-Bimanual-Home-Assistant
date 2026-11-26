import struct
from dataclasses import replace
import pytest
from so101_arm_bridge.unified_commissioning import request,CommissionStatus,CommissionClient,validate_request
from so101_arm_bridge.mobile_wire import crc32c
from so101_arm_bridge.lift_wire import LiftClient
from tools.run.prepare_unified_controller import compare_calibration
from copy import deepcopy

def status(kind=1,seq=1,flags=0,reason=0):
    b=bytearray(64);b[:4]=b'CS\x01'+bytes([kind])
    struct.pack_into('<IIIII',b,4,seq,100,42 if flags&1 else 0,7 if not flags else 0,100)
    b[24]=flags;b[25]=reason;struct.pack_into('<I',b,28,123)
    struct.pack_into('<I',b,60,crc32c(b[:60]));return bytes(b)

def test_query_never_carries_preparation_flags():
    q=request(1,1)
    assert len(q)==36 and not any(q[8:32])
    for kw in ({'ticket':1},{'supported':True}):
        with pytest.raises(ValueError):request(1,1,**kw)

def test_boot_release_requires_support_and_never_resets_or_retries():
    class BootTransport:
        def __init__(self,fail=False,flags=0):self.sent=[];self.fail=fail;self.flags=flags
        def exchange_mobile(self,q):
            self.sent.append(q);kind=q[3];seq=struct.unpack_from('<I',q,4)[0]
            if kind==5 and self.fail:raise TimeoutError('uncertain release')
            return status(kind,seq,self.flags)
    t=BootTransport();c=CommissionClient(t)
    with pytest.raises(ValueError):c.release_boot_supported(supported=True)
    assert not t.sent
    assert c.release_boot_supported(supported=True,empty_grippers=True).kind==5
    assert [q[3] for q in t.sent]==[1,5]
    t=BootTransport(fail=True);c=CommissionClient(t)
    with pytest.raises(TimeoutError):c.release_boot_supported(supported=True,empty_grippers=True)
    assert [q[3] for q in t.sent]==[1,5]
    for flags in (17,113,127):
        t=BootTransport(flags=flags);c=CommissionClient(t)
        with pytest.raises(RuntimeError):c.release_boot_supported(supported=True,empty_grippers=True)
        assert [q[3] for q in t.sent]==[1]

@pytest.mark.parametrize('seq', [0,True,-1,2**32])
def test_invalid_sequence(seq):
    with pytest.raises(ValueError):request(1,seq)

def test_status_crc_reserved_fields_and_rejections():
    assert CommissionStatus.decode(status()).ticket==7
    assert CommissionStatus.decode(status(130,flags=16,reason=3)).reason==3
    for i in range(64):
        b=bytearray(status());b[i]^=1
        with pytest.raises(ValueError):CommissionStatus.decode(bytes(b))
    for i in (26,32,59):
        b=bytearray(status());b[i]=1;struct.pack_into('<I',b,60,crc32c(b[:60]))
        with pytest.raises(ValueError):CommissionStatus.decode(bytes(b))

class Transport:
    def __init__(self,fail=False):self.sent=[];self.fail=fail
    def exchange_mobile(self,q):
        self.sent.append(q);seq=struct.unpack_from('<I',q,4)[0]
        if q[3]==2:
            if self.fail:raise TimeoutError('lost prepare reply')
            return status(2,seq,17)
        return status(1,seq)

def test_prepare_requires_confirmation_and_has_no_automatic_retry():
    t=Transport();c=CommissionClient(t)
    with pytest.raises(ValueError):c.prepare()
    assert not t.sent
    result=c.prepare(supported=True,empty_grippers=True)
    assert result.flags==17 and [q[3] for q in t.sent]==[1,2]
    t=Transport(True);c=CommissionClient(t)
    with pytest.raises(TimeoutError):c.prepare(supported=True,empty_grippers=True)
    assert [q[3] for q in t.sent]==[1,2]

def test_lift_reference_confirmation_precedes_any_transport_access():
    c=LiftClient(object())
    with pytest.raises(ValueError):c.reference_bottom(1)
    with pytest.raises(ValueError):c.reference_bottom(1,operator_confirmed=1)

def inventory():
    return dict(all_snapshots_complete=True,devices=[dict(bus=bus,requested_id=i,
        identity=dict(reported_id=i,baud_code=0,model_raw=777),limits=dict(minimum_raw=0,maximum_raw=4095),
        offset_and_mode=dict(offset_raw=100+i,operating_mode=0),command=dict(torque_enabled_raw=0),
        feedback=dict(lock_raw=1,moving_raw=0,speed_raw=0))
        for bus,ids in (('left',range(1,11)),('right',range(1,7))) for i in ids])

def test_calibration_comparison_is_identity_based_and_only_allows_mobile_modes():
    before=inventory();after=deepcopy(before);after['devices'].reverse()
    compare_calibration(before,after)
    for d in after['devices']:
        if d['bus']=='left' and d['requested_id']>=7:d['offset_and_mode']['operating_mode']=1
    compare_calibration(before,after,allow_mobile_mode_change=True)
    with pytest.raises(ValueError):compare_calibration(before,after)

def test_boot_release_preflight_keeps_configuration_checks_with_retained_torque():
    from tools.run.prepare_unified_controller import release_boot_supported
    before=inventory();after=deepcopy(before)
    for d in after['devices']:
        d['command']['torque_enabled_raw']=1
        if d['bus']=='left' and d['requested_id']>=7:d['offset_and_mode']['operating_mode']=1
    compare_calibration(before,after,allow_mobile_mode_change=True,configuration_only=True)
    with pytest.raises(ValueError):compare_calibration(before,after,allow_mobile_mode_change=True)
    after['devices'][0]['offset_and_mode']['offset_raw']+=1
    with pytest.raises(ValueError):compare_calibration(before,after,allow_mobile_mode_change=True,configuration_only=True)
    # Missing physical support confirmation must not even touch the transport.
    with pytest.raises(ValueError):release_boot_supported(object(),baseline=before,supported_empty=False)

@pytest.mark.parametrize('field,key,value',[
    ('offset_and_mode','offset_raw',999),('offset_and_mode','operating_mode',1),
    ('limits','maximum_raw',4000),('command','torque_enabled_raw',1),
    ('feedback','lock_raw',0),('feedback','moving_raw',1),('feedback','speed_raw',1),
    ('identity','model_raw',778)])
def test_changed_calibration_or_nonquiet_state_blocks_preparation(field,key,value):
    before=inventory();after=deepcopy(before);after['devices'][0][field][key]=value
    with pytest.raises(ValueError):compare_calibration(before,after)

def test_duplicate_inventory_cannot_hide_missing_motor():
    before=inventory();after=deepcopy(before);after['devices'][1]=after['devices'][0]
    with pytest.raises(ValueError):compare_calibration(before,after)

def test_commissioning_uses_production_framing_transport():
    from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2
    from so101_arm_bridge.stream_protocol_v2 import FrameV2,StreamMessageTypeV2,decode_frame_v2,encode_frame_v2
    class Port:
        reply=b''
        def __init__(self):self.sent=[]
        def write(self,data):
            frame=decode_frame_v2(data);self.sent.append(frame)
            assert frame.message_type==StreamMessageTypeV2.MOBILE_REQUEST
            validate_request(frame.payload)
            kind=frame.payload[3];seq=struct.unpack_from('<I',frame.payload,4)[0]
            self.reply=encode_frame_v2(FrameV2(message_type=StreamMessageTypeV2.MOBILE_RESPONSE,
                sequence=frame.sequence,payload=b'\0'+status(kind,seq,17 if kind==2 else 0)))
            return len(data)
        def read_until(self,delimiter):
            data,self.reply=self.reply,b'';return data
    port=Port();transport=StreamValidationTransportV2(port)
    assert CommissionClient(transport).prepare(supported=True,empty_grippers=True).flags==17
    assert len(port.sent)==2
    for index in (3,8,12,16,31):
        bad=bytearray(request(1,1));bad[index]=99
        struct.pack_into('<I',bad,32,crc32c(bad[:32]))
        with pytest.raises(ValueError):transport.exchange_mobile(bytes(bad))
    assert len(port.sent)==2

class PowerTransport:
    def __init__(self,fail_kind=None):self.flags=17;self.sent=[];self.fail_kind=fail_kind
    def exchange_mobile(self,q):
        validate_request(q);kind=q[3];self.sent.append(kind)
        seq=struct.unpack_from('<I',q,4)[0]
        if kind==3:self.flags=113
        if kind==4:self.flags=17
        if kind==self.fail_kind:raise TimeoutError('reply lost')
        b=bytearray(status(kind,seq,self.flags))
        if kind==1:struct.pack_into('<I',b,16,7)
        struct.pack_into('<I',b,60,crc32c(b[:60]));return bytes(b)

def test_power_activation_release_is_explicit_and_not_automatically_retried():
    t=PowerTransport();c=CommissionClient(t)
    with pytest.raises(ValueError):c.activate_zero()
    with pytest.raises(ValueError):c.release_supported()
    assert not t.sent
    assert c.activate_zero(supported=True,empty_grippers=True).flags==113
    assert c.release_supported(supported=True,empty_grippers=True).flags==17
    assert t.sent==[1,3,1,4]
    t=PowerTransport(3)
    with pytest.raises(TimeoutError):CommissionClient(t).activate_zero(supported=True,empty_grippers=True)
    assert t.sent==[1,3]

@pytest.mark.parametrize('fault',[None,'drift','torque','goal','calibration','activation_ack','release_ack',
                                 'wheel_transient','wheel_persistent','wheel_duplicate'])
def test_bounded_power_check_releases_only_through_guarded_command(monkeypatch,fault):
    from tools.run import prepare_unified_controller as tool
    t=PowerTransport(3 if fault=='activation_ack' else 4 if fault=='release_ack' else None)
    before=inventory()
    for d in before['devices']:
        d['feedback']['position_raw']=100;d['command']['goal_speed_raw']=0
        if d['bus']=='left' and d['requested_id']>=7:d['offset_and_mode']['operating_mode']=1
    clock=[0.];count=[0]
    monkeypatch.setattr(tool.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(tool.time,'sleep',lambda seconds:clock.__setitem__(0,clock[0]+seconds))
    def inspected(transport,**kw):
        r=kw.get('report');r={} if r is None else r
        r.update(inventory_before=deepcopy(before),commission_final=dict(flags=17));return r
    def collected(transport):
        r=deepcopy(before);clock[0]+=.15;count[0]+=1
        for d in r['devices']:d['sample_started_tick_ms']=round(clock[0]*1000)
        if t.flags==113:
            for d in r['devices']:
                if d['bus']=='left' and d['requested_id']>=7:d['command']['torque_enabled_raw']=1
            d=r['devices'][9]
            if fault=='drift':d['feedback']['position_raw']+=count[0]*3
            if fault=='torque':d['command']['torque_enabled_raw']=0
            if fault=='goal':d['command']['goal_speed_raw']=1
            if fault=='calibration':d['offset_and_mode']['offset_raw']+=1
            wheel=r['devices'][8]
            if fault=='wheel_persistent' or (fault=='wheel_transient' and count[0]==5):
                wheel['feedback'].update(speed_raw=32818,moving_raw=1)
            if fault=='wheel_duplicate':wheel['sample_started_tick_ms']=100
        return r
    monkeypatch.setattr(tool,'inspect',inspected);monkeypatch.setattr(tool,'collect_inventory',collected)
    r=tool.check_zero_power(t,baseline={'inventory':before},supported_empty=True,seconds=1)
    assert (r['status']=='ZERO_POWER_CHECK_PASSED_RELEASED')==(fault in (None,'wheel_transient'))
    assert t.sent==[1,3,1,4]
    assert r['released_confirmed']==(fault!='release_ack')
    assert r['nonzero_velocity_commands']==0 and not r['arms_enabled']
    if fault=='wheel_transient':
        assert len(r['powered_observation']['events'])==1
        assert r['powered_observation']['proven']
        assert r['samples'][-1]['elapsed_s']>1.4
    if fault=='wheel_persistent':assert clock[0]<=2.3

def powered_observation_fixture():
    from tools.run.prepare_unified_controller import ZeroPowerObservation
    before=inventory()
    for d in before['devices']:
        d['feedback']['position_raw']=100;d['command']['goal_speed_raw']=0
        if d['bus']=='left' and d['requested_id']>=7:d['offset_and_mode']['operating_mode']=1
    def sample(stamp,**feedback):
        r=deepcopy(before)
        for d in r['devices']:
            d['sample_started_tick_ms']=stamp%(2**32)
            if d['bus']=='left' and d['requested_id']>=7:d['command']['torque_enabled_raw']=1
        r['devices'][8]['feedback'].update(feedback)
        return r
    return ZeroPowerObservation(before),sample

def test_small_wheel_report_requires_new_powered_quiet_window_and_retains_raw_evidence():
    o,s=powered_observation_fixture()
    assert not o.observe(s(100))
    assert not o.observe(s(300))
    assert o.observe(s(500))
    assert not o.observe(s(700,speed_raw=32818,moving_raw=1,position_raw=99))
    assert not o.observe(s(900))
    assert not o.observe(s(1100))
    assert o.observe(s(1300))
    assert o.events==[dict(motor_id=9,sample_started_tick_ms=700,position_raw=99,speed_raw=32818,moving_raw=1)]

@pytest.mark.parametrize('fault',['jump_on_activation','gradual_drift','span','large_speed','duplicate','old','gap','off','goal','lift','arm','mode','incomplete'])
def test_powered_position_proof_rejects_actual_drift_stale_reads_and_wrong_power(fault):
    o,s=powered_observation_fixture()
    if fault=='jump_on_activation':
        with pytest.raises(ValueError):o.observe(s(100,position_raw=103))
        return
    o.observe(s(100));bad=s(300)
    if fault=='gradual_drift':
        o.observe(s(300,position_raw=101));o.observe(s(500,position_raw=102));bad=s(700,position_raw=103)
    elif fault=='span':
        o.observe(s(300,position_raw=98));bad=s(500,position_raw=102)
    elif fault=='large_speed':bad=s(300,speed_raw=51)
    elif fault=='duplicate':bad=s(100)
    elif fault=='old':bad=s(99)
    elif fault=='gap':bad=s(351)
    elif fault=='off':bad['devices'][8]['command']['torque_enabled_raw']=0
    elif fault=='goal':bad['devices'][8]['command']['goal_speed_raw']=1
    elif fault=='lift':bad['devices'][9]['feedback']['speed_raw']=50
    elif fault=='arm':bad['devices'][0]['feedback']['moving_raw']=1
    elif fault=='mode':bad['devices'][8]['offset_and_mode']['operating_mode']=0
    elif fault=='incomplete':bad['all_snapshots_complete']=False
    with pytest.raises(ValueError):o.observe(bad)
    assert not o.proven
    with pytest.raises(ValueError):o.observe(s(900))

def test_repeated_small_reports_never_establish_quiet_proof():
    o,s=powered_observation_fixture()
    for stamp in range(100,4100,200):assert not o.observe(s(stamp,speed_raw=50,moving_raw=1))


@pytest.mark.parametrize('defect',[None,'drift','goal','torque','missing','speed','stale'])
def test_unpowered_lift_reports_need_complete_stationary_position_window(defect):
    from tools.run.prepare_unified_controller import ZeroPowerObservation
    original,sample=powered_observation_fixture()
    o=ZeroPowerObservation(original.before,powered=False)
    def observed(stamp,index):
        s=sample(stamp)
        for d in s['devices']:d['command']['torque_enabled_raw']=0
        lift=s['devices'][9];lift['feedback'].update(speed_raw=32818,moving_raw=1,position_raw=100+index%2)
        if index==2:
            if defect=='drift':lift['feedback']['position_raw']=103
            if defect=='goal':lift['command']['goal_speed_raw']=1
            if defect=='torque':lift['command']['torque_enabled_raw']=1
            if defect=='missing':s['all_snapshots_complete']=False
            if defect=='speed':lift['feedback']['speed_raw']=51
            if defect=='stale':lift['sample_started_tick_ms']=300
        return s
    assert not o.observe(observed(100,0))
    assert not o.observe(observed(300,1))
    if defect:
        with pytest.raises(ValueError):o.observe(observed(500,2))
        assert not o.proven
    else:
        assert o.observe(observed(500,2))
        assert len(o.events)==3

def test_wrapped_encoder_and_mcu_clock_do_not_appear_as_motion_or_stale_data():
    o,s=powered_observation_fixture();o.axes[9]['anchor']=4095
    for index in range(3):
        proven=o.observe(s(2**32-300+index*200,position_raw=index%2 and 0 or 4095))
    assert proven

@pytest.mark.parametrize('defect',[None,'missing','stale','drift','outside_limits','roundtrip','negative_age'])
def test_supported_hold_evidence_uses_measured_pose_and_original_age(defect):
    from types import SimpleNamespace as NS
    from tools.run.prepare_unified_controller import measured_hold_source
    s=NS(status_code=0,joint_count=12,present_mask=4095,positions_urad=(0,)*12,sample_age_ms=(50,)*12)
    limits=NS(minimum_urad=(-100000,)*12,maximum_urad=(100000,)*12)
    end=10.02
    if defect=='missing':s.present_mask=2047
    if defect=='stale':s.sample_age_ms=(240,)*12
    if defect=='drift':s.positions_urad=(49089,)+(0,)*11
    if defect=='outside_limits':limits.maximum_urad=(-1,)*12
    if defect=='roundtrip':end=10.11
    if defect=='negative_age':s.sample_age_ms=(-1,)*12
    if defect:
        with pytest.raises(ValueError):measured_hold_source(s,(0,)*12,limits,10.,end)
    else:assert measured_hold_source(s,(0,)*12,limits,10.,end)==9.95

@pytest.mark.parametrize('failure',[None,'interlock_only','arm','lift','stale','stop','final_stop','evidence'])
def test_hold_reference_stop_orchestration_retains_torque_and_never_fake_confirms(monkeypatch,failure):
    from dataclasses import dataclass
    from types import SimpleNamespace as NS
    from tools.run import prepare_unified_controller as tool
    import so101_arm_bridge.unified_arm_adapter as am
    import so101_arm_bridge.mobile_client as mm
    import so101_arm_bridge.lift_wire as lm
    import so101_arm_bridge.robot_evidence as em
    import so101_arm_bridge.system_stop as sm
    clock=[1.];events=[];state={'stop':False,'reference':False,'proofs':0}
    monkeypatch.setattr(tool.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(tool.time,'sleep',lambda v:clock.__setitem__(0,clock[0]+v))
    @dataclass
    class Limits:
        minimum_urad:tuple=(-100000,)*12
        maximum_urad:tuple=(100000,)*12
        hardware_authorized:bool=False
    @dataclass
    class Pose:positions_urad:tuple=(0,)*12
    @dataclass
    class Heartbeat:
        stop_latched:bool=False
        status_code:int=0
    @dataclass
    class Snapshot:
        status_code:int=0
        joint_count:int=12
        present_mask:int=4095
        positions_urad:tuple=(0,)*12
        sample_age_ms:tuple=(50,)*12
    @dataclass
    class LS:
        flags:int=4
        fault:int=0
        state:int=5
        command_raw:int=0
        height_um:int=0
        target_um:int=0
        mcu_tick_ms:int=100
        observed_ms:int=99
        def holding(self):return self.flags==29 and not self.fault
    @dataclass
    class Proof:confirmed:bool=True
    class T(PowerTransport):
        def heartbeat(self):clock[0]+=.01;return Heartbeat(state['stop'])
        def get_feedback_snapshot(self):
            clock[0]+=.01
            return Snapshot(sample_age_ms=(999 if failure=='stale' else 50,)*12)
    class Adapter:
        def __init__(self,*a,**kw):self.motion_limits=kw.get('authorized_limits',Limits());self.state=NS(value='new');self.prepared_state=Pose()
        def prepare(self):events.append('prepare');self.state.value='ready';return self.prepared_state
        def current_pose_hold_points(self):return (Pose(),)*3
        def start_current_pose_hold(self,*a,**kw):
            events.append('arm');self.state.value='active'
            if failure=='arm':raise RuntimeError('arm failed')
        def poll(self,*a):clock[0]+=.1;self.state.value='ready'
    class Mobile:
        def __init__(self,*a):self.boot_id=42
        def synchronize(self):pass
    class Lift:
        def __init__(self,*a):pass
        def status(self):
            flags=29 if state['reference'] else 4
            if state['reference'] and (state['stop'] or (failure=='interlock_only' and clock[0]>1.5)):flags=25
            return LS(flags,1 if failure=='lift' else 0)
        def reference_bottom(self,*a,operator_confirmed=False):
            assert operator_confirmed;events.append('reference');state['reference']=True;return LS(29)
    class Evidence:
        def __init__(self,*a,**kw):pass
        def publish_mcu_snapshot(self,snapshot,sent,received,**kw):
            assert sent<received<=clock[0] and kw==dict(arms_safe=True,lift_hold=False,payload_safe=True)
            if failure=='evidence':raise RuntimeError('evidence rejected')
    class Stop:
        def __init__(self,*a):pass
        def request(self):events.append('STOP');state['stop']=True;return Heartbeat(True)
        def status(self):
            state['proofs']+=1
            return Proof(not (failure=='stop' or (failure=='final_stop' and state['proofs']>1))),clock[0]
    monkeypatch.setattr(am,'UnifiedArmAdapter',Adapter);monkeypatch.setattr(mm,'MobileClient',Mobile)
    monkeypatch.setattr(lm,'LiftClient',Lift);monkeypatch.setattr(em,'RobotEvidenceClient',Evidence)
    monkeypatch.setattr(sm,'SystemStopClient',Stop)
    def inspect(*a,**kw):
        r=kw.get('report');r={} if r is None else r
        r.update(commission_final=dict(flags=17,boot_id=42));return r
    monkeypatch.setattr(tool,'inspect',inspect)
    t=T();r=tool.check_hold_stop(t,baseline={},supported_empty=True,bottom_confirmed=True)
    assert (r['status']=='SUPPORTED_HOLD_REFERENCE_STOP_CONFIRMED_TORQUE_RETAINED')==(failure in (None,'interlock_only'))
    assert 'STOP' in events and t.sent==[1,3] # Never generic DISABLE / maintenance release after arm attempt.
    assert not r['torque_release_requested'] and not r['lift_move_commands']
    if failure is None:assert events==['prepare','arm','reference','STOP']

@pytest.mark.parametrize('change',[{}, {'flags':29}, {'flags':9}, {'flags':27}, {'state':4},
    {'fault':1}, {'command_raw':1}, {'height_um':2001}, {'observed_ms':750}, {'observed_ms':800}])
def test_supported_lift_hold_is_not_a_movement_permission(change):
    from types import SimpleNamespace
    from tools.run.prepare_unified_controller import supported_lift_is_holding
    fields=dict(state=5,fault=0,flags=25,command_raw=0,height_um=0,target_um=0,mcu_tick_ms=1000,observed_ms=999)
    fields.update(change)
    assert supported_lift_is_holding(SimpleNamespace(**fields))==(change in ({},{'flags':29},{'observed_ms':800}))


def test_hold_stop_confirmation_gate_precedes_all_hardware_access():
    from tools.run.prepare_unified_controller import check_hold_stop
    with pytest.raises(ValueError):check_hold_stop(object(),baseline={},supported_empty=True,bottom_confirmed=False)
    with pytest.raises(ValueError):check_hold_stop(object(),baseline={},supported_empty=True,bottom_confirmed=True,base_pulse=True)

@pytest.mark.parametrize('failure',[None,'feedback','hold','settle'])
def test_first_base_pulse_is_bounded_and_always_requests_zero(monkeypatch,failure):
    from dataclasses import dataclass
    from types import SimpleNamespace
    from tools.run import prepare_unified_controller as tool
    clock=[0.];commands=[]
    monkeypatch.setattr(tool.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(tool.time,'sleep',lambda v:clock.__setitem__(0,clock[0]+v))
    @dataclass
    class Status:
        velocity_raw:tuple=(0,0,0,0)
        def ready_for_motion(self,session):return session==1 and failure!='feedback'
    class Mobile:
        def arm(self,session):assert session==1
        def velocity(self,values,lifetime_ms):
            assert lifetime_ms==150
            commands.append((clock[0],values));return Status((50,0,0,0) if failure=='settle' else (0,0,0,0))
    def observe():
        return SimpleNamespace(state=5,fault=int(failure=='hold'),flags=25,command_raw=0,
            height_um=0,target_um=0,mcu_tick_ms=1000,observed_ms=999)
    r={'nonzero_mobile_velocity_commands':0}
    if failure:
        with pytest.raises((RuntimeError,TimeoutError)):tool.supported_base_pulse(Mobile(),observe,r)
    else:
        tool.supported_base_pulse(Mobile(),observe,r)
        assert r['base_pulse']['zero_feedback_verified']
    assert all(values in ((-50,50,0,0),(0,0,0,0)) for _,values in commands)
    assert all(t<.2 for t,values in commands if any(values))
    assert commands[-1][1]==(0,0,0,0)

def test_torque_off_window_reuses_teleop_bounds_without_changing_powered_policy():
    from tools.run.prepare_unified_controller import ZeroPowerObservation
    powered,sample=powered_observation_fixture()
    off=ZeroPowerObservation(powered.before,powered=False)
    def read(stamp,position=100):
        r=sample(stamp,speed_raw=32818,moving_raw=1,position_raw=position)
        for d in r['devices']:d['command']['torque_enabled_raw']=0
        return r
    assert not off.observe(read(100))
    assert not off.observe(read(300,108))
    assert off.observe(read(500,101))
    assert len(off.events)==3
    with pytest.raises(ValueError):off.observe(read(700,109))
    assert not off.proven
    # The same recurring speed/moving report still never proves torque-ON hold.
    for stamp in (100,300,500):assert not powered.observe(sample(stamp,speed_raw=32818,moving_raw=1))

@pytest.mark.parametrize('field,value',[('torque',1),('goal',1),('mode',0),('speed',51)])
def test_off_window_does_not_admit_nonzero_drive_or_unknown_mode(field,value):
    from tools.run.prepare_unified_controller import ZeroPowerObservation
    o,s=powered_observation_fixture();off=ZeroPowerObservation(o.before,powered=False)
    r=s(100)
    for d in r['devices']:d['command']['torque_enabled_raw']=0
    d=r['devices'][8]
    if field=='torque':d['command']['torque_enabled_raw']=value
    elif field=='goal':d['command']['goal_speed_raw']=value
    elif field=='mode':d['offset_and_mode']['operating_mode']=value
    else:d['feedback']['speed_raw']=value
    with pytest.raises(ValueError):off.observe(r)


@pytest.mark.parametrize('changes,reason', [
    ({'stop_latched': True}, 'stop_latched=true'),
    ({'firmware_version': 123}, '0x7b'),
    ({'protocol_version': 1}, '통신 프로토콜 2 필요 (현재 1)'),
    ({'joint_count': 6}, '양팔 12개 관절 필요 (현재 6)'),
])
def test_pick_rejection_identifies_actual_failed_condition(changes, reason):
    from types import SimpleNamespace
    from so101_arm_bridge.unified_commissioning import query_pick_controller
    hello = SimpleNamespace(**dict(dict(firmware_version=0x2492E, protocol_version=2,
                                       joint_count=12, stop_latched=False), **changes))
    transport = SimpleNamespace(enter_binary_mode=lambda:hello,
        exchange_mobile=lambda _:pytest.fail('commissioning after rejected controller'))
    with pytest.raises(ValueError) as error:
        query_pick_controller(transport, {})
    assert reason in str(error.value)
    assert hello.stop_latched == changes.get('stop_latched', False)
