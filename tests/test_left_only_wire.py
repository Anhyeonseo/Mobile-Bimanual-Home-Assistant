"""Exercise actual framing and parsing, rather than pre-decoded mock responses."""
import pytest
from so101_arm_bridge import stream_protocol_v2 as p
from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2
from so101_arm_bridge.left_only_adapter import LEFT_FIRMWARE_VERSION, LEFT_CAPABILITIES

class WirePort:
    def __init__(self, count=6):
        self.count=count
        self.pending=b''
        self.sent=[]
    def reset_input_buffer(self): self.pending=b''
    def readline(self): return b'BINARY_PROTOCOL_READY_RESET_TO_EXIT\n'
    def read_until(self, delimiter):
        value=self.pending;self.pending=b'';return value
    def write(self, data):
        if data==b'P': return len(data)
        request=p.decode_frame_v2(data);self.sent.append(request.message_type)
        kind=request.message_type;count=self.count
        if kind==p.StreamMessageTypeV2.HELLO_REQUEST:
            response=p.StreamMessageTypeV2.HELLO_RESPONSE
            payload=p.HELLO_V2.pack(2,count,0,0,LEFT_FIRMWARE_VERSION,0x2D90167E,0x2D90167E,LEFT_CAPABILITIES,0)
        elif kind==p.StreamMessageTypeV2.PREPARE_SHADOW:
            response=p.StreamMessageTypeV2.SHADOW_SNAPSHOT
            payload=p.UNWRAPPED_SHADOW_SNAPSHOT_V2.pack(0,count,0x3f,0,*([100]*6+[0]*6),*([100]*6+[0]*6),*([1000]*6+[0]*6))
        elif kind==p.StreamMessageTypeV2.GET_FEEDBACK_SNAPSHOT:
            response=p.StreamMessageTypeV2.FEEDBACK_SNAPSHOT
            payload=p.FEEDBACK_SNAPSHOT_V2.pack(0,count,0x3f,request.sequence,request.sender_time_ms,123,1,*([1000]*6+[0]*6),*([1]*6+[0xffffffff]*6))
        else:
            response=p.StreamMessageTypeV2.STATE_FEEDBACK
            payload=p.STATE_V2.pack(int(kind==p.StreamMessageTypeV2.SAFE_STOP),0,count,2,1,0,0x2D90167E,0x2D90167E,123)
        self.pending=p.encode_frame_v2(p.FrameV2(message_type=response,sequence=request.sequence,sender_time_ms=123,payload=payload))
        return len(data)

METHODS=['enter_binary_mode','get_state','heartbeat','enable','safe_stop','prepare_shadow','get_feedback_snapshot']

@pytest.mark.parametrize('method',METHODS)
def test_left_only_wire_all_identity_responses(method):
    tr=StreamValidationTransportV2(WirePort(),physical_joint_count=6)
    result=getattr(tr,method)()
    assert result.joint_count==6
    if method=='get_feedback_snapshot':
        assert result.present_mask==0x3f
        assert result.sample_age_ms[6:]==(0xffffffff,)*6
    if method=='prepare_shadow':
        assert result.right_present_mask==0
        assert result.anchor_positions_urad[6:]==(0,)*6

@pytest.mark.parametrize('method',METHODS)
@pytest.mark.parametrize('expected,actual',[(12,6),(6,12)])
def test_cross_profile_response_rejected(method,expected,actual):
    tr=StreamValidationTransportV2(WirePort(actual),physical_joint_count=expected)
    with pytest.raises(p.StreamContractError, match="identity|joint count"):getattr(tr,method)()

@pytest.mark.parametrize('count',[0,1,7,True,6.0,'6',None])
def test_invalid_physical_count_rejected(count):
    with pytest.raises(ValueError):StreamValidationTransportV2(WirePort(),physical_joint_count=count)

@pytest.mark.parametrize('left_only,expected',[(False,12),(True,6)])
def test_cli_connection_selects_explicit_physical_profile(monkeypatch,left_only,expected):
    from contextlib import nullcontext
    from types import SimpleNamespace
    import sys
    from test_fixed_pick_demo import cli
    module=cli
    monkeypatch.setitem(sys.modules,'serial',SimpleNamespace())
    monkeypatch.setattr(module,'resolve_serial_device',lambda _: '/test/serial')
    monkeypatch.setattr(module,'acquire_backend_lease',lambda *_: nullcontext())
    monkeypatch.setattr(module,'open_exclusive_serial',lambda *_: nullcontext(WirePort(expected)))
    args=SimpleNamespace(port='test',baud=921600,left_only=left_only)
    with module.connection(args) as tr:
        assert tr.enter_binary_mode().joint_count==expected
