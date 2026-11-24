"""The probe exits on unlocked state; its sample path has only query clients."""
from dataclasses import dataclass
from types import SimpleNamespace as NS
import pytest
from tools.diagnostics.probe_pick_serial_timing import initialize_probe, read_sample, read_unprepared_sample, summarize


@dataclass
class State:
    stop_latched: bool
    status_code: int=0

@dataclass
class Status:
    boot_id: int


def test_unlatched_state_does_not_query_other_devices_or_try_to_stop():
    events=[]
    def forbidden():raise AssertionError('must not query after unlocked state')
    transport=NS(get_state=lambda:events.append('get_state') or State(False))
    with pytest.raises(RuntimeError,match='STOP'):
        read_sample(transport,NS(status=forbidden),NS(status=forbidden))
    assert events==['get_state']


@pytest.mark.parametrize('lift_boot',[162,163])
def test_stopped_probe_uses_queries_only_and_rejects_boot_change(lift_boot):
    events=[]
    transport=NS(get_state=lambda:events.append('get_state') or State(True))
    base=NS(status=lambda:events.append('base_status') or Status(162))
    lift=NS(status=lambda:events.append('lift_status') or Status(lift_boot))
    if lift_boot!=162:
        with pytest.raises(RuntimeError,match='boot changed'):read_sample(transport,base,lift)
    else:assert read_sample(transport,base,lift)['state']['stop_latched'] is True
    assert events==['get_state','base_status','lift_status']


def test_summary_keeps_slow_request_details_separate():
    rows=[dict(operation='exchange_mobile',inner_request_hex='414c0104',duration_ms=149),
          dict(operation='exchange_mobile',inner_request_hex='414c0104',duration_ms=2),
          dict(operation='get_state',duration_ms=3)]
    result=summarize(rows)
    assert result['414c0104']['maximum_ms']==149 and result['414c0104']['over_50ms']==1
    assert result['get_state']['count']==1


@pytest.mark.parametrize('already_binary',[False,True])
@pytest.mark.parametrize('latched',[False,True])
def test_probe_negotiates_mode_and_checks_stop_without_activation(already_binary,latched):
    from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2
    from so101_arm_bridge.stream_protocol_v2 import (
        FrameV2, StreamMessageTypeV2 as Msg, HELLO_V2, encode_frame_v2, decode_frame_v2,
    )
    class Port:
        def __init__(self):self.binary=already_binary;self.writes=[];self.reply=b'';self.ack=b''
        def reset_input_buffer(self):self.reply=b''
        def write(self,data):
            self.writes.append(data)
            if data==b'P':
                if not self.binary:self.ack=b'BINARY_PROTOCOL_READY_RESET_TO_EXIT\r\n'
                self.binary=True
            elif data!=b'\0':
                assert self.binary,'binary query before negotiation'
                frame=decode_frame_v2(data)
                assert frame.message_type==Msg.HELLO_REQUEST,'no activation/heartbeat/reset permitted'
                self.reply=encode_frame_v2(FrameV2(message_type=Msg.HELLO_RESPONSE,
                    sequence=frame.sequence,sender_time_ms=100,
                    payload=HELLO_V2.pack(2,12,int(latched),0,0x24930,1,1,0,0)))
            return len(data)
        def readline(self):line,self.ack=self.ack,b'';return line
        def read_until(self,_):reply,self.reply=self.reply,b'';return reply
    port=Port();transport=StreamValidationTransportV2(port)
    if latched:assert initialize_probe(transport).stop_latched
    else:
        with pytest.raises(RuntimeError,match='STOP 잠금이 없습니다'):initialize_probe(transport)
    assert port.writes[0]==b'P'
    assert len(port.writes)==(3 if already_binary else 2)


def test_unknown_firmware_does_not_start_probe():
    hello=NS(protocol_version=2,joint_count=12,firmware_version=0,stop_latched=True)
    with pytest.raises(RuntimeError,match='unsupported controller'):
        initialize_probe(NS(enter_binary_mode=lambda:hello))


@dataclass
class Commission:
    flags: int=0
    boot_id: int=0
    reason: int=0


@pytest.mark.parametrize('flags,boot,active,settling,accepted',[
    (0,0,False,False,True), (17,162,False,False,False),
    (113,162,False,False,False), (0,0,True,False,False),
    (0,0,False,True,False), (0,0,None,False,False),
])
def test_unprepared_probe_requires_zero_commission_and_inactive_executor(flags,boot,active,settling,accepted):
    t=NS(get_state=lambda:State(False),
         stationary_runtime=lambda:dict(executor_active=active,settling=settling))
    c=NS(query=lambda:Commission(flags,boot))
    if accepted:
        result=read_unprepared_sample(t,c)
        assert result['commission']['flags']==0 and 'lift' not in result
    else:
        with pytest.raises(RuntimeError):read_unprepared_sample(t,c)


def test_unprepared_probe_rejects_new_stop_before_other_queries():
    with pytest.raises(RuntimeError,match='state changed'):
        read_unprepared_sample(NS(get_state=lambda:State(True)),None)
