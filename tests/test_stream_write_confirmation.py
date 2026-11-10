"""A timed-out local write is uncertain delivery, not a reason to resend.

Exercise the installed pyserial POSIX write function with a fake kernel/clock:
all bytes can be queued before its post-write timeout check raises. Only a
CRC-valid response matching both sequence and message type confirms delivery.
"""
import inspect
import textwrap
from types import SimpleNamespace as NS

import pytest
import serial.serialposix as posix

import so101_arm_bridge.stream_transport_v2 as transport_module
from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2, StreamResponseTimeoutError
from so101_arm_bridge.stream_protocol_v2 import FrameV2, StreamMessageTypeV2 as Msg, STATE_V2, STREAM_STATUS_V2, StreamBatchV2, StreamSampleV2, encode_frame_v2, decode_frame_v2


class Clock:
    def __init__(self):self.now=10.
    def __call__(self):return self.now


class Port:
    def __init__(self, clock, reply='valid'):
        self.clock=clock;self.reply=reply;self.writes=[];self.pending=b''
        self.is_open=True;self._write_timeout=.02;self.fd=123
        self.pipe_abort_write_r=456
        outer=self
        class Timeout:
            is_non_blocking=False;is_infinite=False
            def __init__(self, duration):self.deadline=clock()+duration
            def expired(self):return clock()>self.deadline
            def time_left(self):return max(0,self.deadline-clock())
        def kernel_write(fd, data):
            assert fd==123
            outer.writes.append(data)
            frame=decode_frame_v2(data)
            payload=STATE_V2.pack(int(reply=='latched'),0,12,2,1,0,1,1,100)
            kind=Msg.STATE_FEEDBACK if reply!='wrong_type' else Msg.MOBILE_RESPONSE
            if frame.message_type==Msg.SETPOINT_BATCH:
                kind=Msg.STREAM_STATUS
                payload=STREAM_STATUS_V2.pack(0,0,2,3,frame.sequence,frame.sender_time_ms,1,200,140,2,2,0)
            sequence=frame.sequence+1 if reply=='wrong_sequence' else frame.sequence
            encoded=encode_frame_v2(FrameV2(message_type=kind,sequence=sequence,sender_time_ms=100,payload=payload))
            if reply=='corrupt':encoded=bytes([encoded[0]^1])+encoded[1:]
            outer.pending=b'' if reply=='missing' else encoded
            # Successful os.write, followed by host scheduling delay. The
            # real pyserial method still raises before accounting n bytes.
            clock.now+=.021
            return len(data)
        namespace=dict(vars(posix),os=NS(write=kernel_write),Timeout=Timeout)
        exec(textwrap.dedent(inspect.getsource(posix.Serial.write)),namespace)
        self.production_write=namespace['write']
    def write(self, data):return self.production_write(self,data)
    def read_until(self, delimiter):
        assert delimiter==b'\x00';self.clock.now+=.005
        result,self.pending=self.pending,b''
        return result


def test_actual_pyserial_can_raise_after_the_complete_packet_was_queued():
    clock=Clock();port=Port(clock)
    packet=encode_frame_v2(FrameV2(message_type=Msg.HEARTBEAT,sequence=1,sender_time_ms=10,payload=b''))
    with pytest.raises(posix.SerialTimeoutException,match='Write timeout'):port.write(packet)
    assert port.writes==[packet] and port.pending


def test_matching_response_resolves_write_uncertainty_without_resending(monkeypatch):
    clock=Clock();monkeypatch.setattr(transport_module,'time',NS(monotonic=clock))
    port=Port(clock);t=StreamValidationTransportV2(port,response_timeout_s=.12)
    assert not t.heartbeat().stop_latched
    assert len(port.writes)==1  # Never repeat even a possibly delivered command.
    d=t.write_diagnostics()
    assert d['timeout_count']==1 and d['events'][0]['resolution']=='matching_response'
    assert d['events'][0]['message_type']=='HEARTBEAT'


def test_trajectory_append_is_never_replayed_after_write_timeout(monkeypatch):
    clock=Clock();monkeypatch.setattr(transport_module,'time',NS(monotonic=clock))
    port=Port(clock);t=StreamValidationTransportV2(port,response_timeout_s=.12)
    batch=StreamBatchV2(200,1,(StreamSampleV2(120,(0,)*12),StreamSampleV2(140,(0,)*12)))
    result=t.append(batch)
    assert result.accepted_samples==2 and result.request_sequence==1
    assert len(port.writes)==1 and t.write_diagnostics()['events'][0]['resolution']=='matching_response'


@pytest.mark.parametrize('reply',['wrong_type','wrong_sequence','corrupt','missing'])
def test_uncertain_delivery_without_matching_response_still_fails_bounded(monkeypatch,reply):
    clock=Clock();monkeypatch.setattr(transport_module,'time',NS(monotonic=clock))
    port=Port(clock,reply);t=StreamValidationTransportV2(port,response_timeout_s=.12)
    with pytest.raises(StreamResponseTimeoutError):t.heartbeat()
    assert len(port.writes)==1 and clock()<10.15
    assert t.write_diagnostics()['events'][0]['resolution']=='response_timeout'


def test_received_stop_latch_is_not_silently_treated_as_a_healthy_enable(monkeypatch):
    clock=Clock();monkeypatch.setattr(transport_module,'time',NS(monotonic=clock))
    port=Port(clock,'latched');t=StreamValidationTransportV2(port,response_timeout_s=.12)
    with pytest.raises(transport_module.StreamTransportV2Error,match='ENABLE rejected'):t.enable()
    assert len(port.writes)==1


@pytest.mark.parametrize('failure',['short','disconnect'])
def test_partial_write_and_real_io_error_do_not_enter_confirmation_wait(failure):
    class BadPort:
        def write(self, data):
            if failure=='disconnect':raise OSError('USB disconnected')
            return len(data)-1
        def read_until(self,*args):raise AssertionError('must not wait after real I/O failure')
    with pytest.raises((OSError,transport_module.StreamTransportV2Error)):
        StreamValidationTransportV2(BadPort()).heartbeat()


def test_repeated_resolved_timeouts_have_bounded_diagnostic_history(monkeypatch):
    clock=Clock();monkeypatch.setattr(transport_module,'time',NS(monotonic=clock))
    port=Port(clock);t=StreamValidationTransportV2(port)
    for _ in range(30):assert not t.heartbeat().stop_latched
    d=t.write_diagnostics()
    assert d['timeout_count']==30 and len(d['events'])==20 and len(port.writes)==30


def test_usb_disconnect_while_confirming_is_recorded_and_not_retried(monkeypatch):
    clock=Clock();monkeypatch.setattr(transport_module,'time',NS(monotonic=clock))
    port=Port(clock)
    def disconnected(*_):raise OSError('USB disconnected')
    port.read_until=disconnected;t=StreamValidationTransportV2(port)
    with pytest.raises(OSError,match='disconnected'):t.heartbeat()
    assert len(port.writes)==1 and t.write_diagnostics()['events'][0]['resolution']=='receive_error'
