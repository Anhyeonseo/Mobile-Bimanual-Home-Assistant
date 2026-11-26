"""Injected delays identify a transport phase without changing wire behavior."""
import pytest
from so101_arm_bridge import stream_transport_v2 as module
from so101_arm_bridge.stream_protocol_v2 import (
    FrameV2, StreamMessageTypeV2 as Msg, STATE_V2, encode_frame_v2, decode_frame_v2,
)


@pytest.mark.parametrize('slow_phase', ['write', 'read', 'decode'])
def test_phase_timing_preserves_reply_and_never_resends(monkeypatch,slow_phase):
    wall=[10.];cpu=[2.]
    monkeypatch.setattr(module,'_perf_counter',lambda:wall[0])
    monkeypatch.setattr(module,'_thread_time',lambda:cpu[0])
    def spend(phase):
        wall[0]+=.149 if phase==slow_phase else .001
        cpu[0]+=.148 if phase==slow_phase=='decode' else .0001
    class Port:
        writes=[]
        def write(self,data):
            self.writes.append(data);spend('write')
            request=decode_frame_v2(data)
            self.reply=encode_frame_v2(FrameV2(message_type=Msg.STATE_FEEDBACK,
                sequence=request.sequence,sender_time_ms=89323,
                payload=STATE_V2.pack(0,0,12,2,496,3,1,1,89323)))
            return len(data)
        def read_until(self,_):
            spend('read');return self.reply
    decode=module.decode_frame_v2
    def delayed_decode(data):spend('decode');return decode(data)
    monkeypatch.setattr(module,'decode_frame_v2',delayed_decode)
    port=Port();transport=module.StreamValidationTransportV2(port)
    state=transport.heartbeat()
    assert not state.stop_latched and state.last_heartbeat_ms==89323
    assert len(port.writes)==1
    result=transport.timing_diagnostics()['transactions'][0]
    assert result['message_type']=='HEARTBEAT' and result['sequence']==1
    assert result['duration_ms']==pytest.approx(151)
    for name,phase in result['phases'].items():
        assert phase['wall_ms']==pytest.approx(149 if name==slow_phase else 1)
    assert result['phases'][slow_phase]['thread_cpu_ms']==pytest.approx(148 if slow_phase=='decode' else .1)
    assert 'error' not in result
    result['phases'].clear()
    assert transport.timing_diagnostics()['transactions'][0]['phases']  # Detached snapshot.


def test_failure_is_preserved_and_diagnostic_ring_is_bounded():
    class Port:
        def write(self,_):raise OSError('disconnected')
    transport=module.StreamValidationTransportV2(Port())
    for _ in range(260):
        with pytest.raises(OSError,match='disconnected'):transport.heartbeat()
    rows=transport.timing_diagnostics()['transactions']
    assert len(rows)==256 and rows[0]['sequence']==5
    assert rows[-1]['error']=='OSError: disconnected'
    assert rows[-1]['phases']['write']['calls']==1
