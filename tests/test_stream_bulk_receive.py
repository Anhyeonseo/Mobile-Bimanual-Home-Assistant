from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2

class BufferedPort:
    def __init__(self, data):self.data=bytearray(data);self.reads=[]
    @property
    def in_waiting(self):return len(self.data)
    def read(self,size):
        self.reads.append(size);result=bytes(self.data[:size]);del self.data[:size];return result
    def read_until(self,*_):raise AssertionError('do not use per-byte reader on serial')


def test_multiple_responses_are_drained_once_and_preserved_in_order():
    p=BufferedPort(b'first\0second\0third');t=StreamValidationTransportV2(p)
    assert t._read_packet()==b'first\0'
    assert t._read_packet()==b'second\0' and len(p.reads)==1
    assert t._read_packet() is None
    p.data.extend(b'-tail\0')
    assert t._read_packet()==b'third-tail\0'


def test_partial_response_survives_idle_reads_without_relabeling():
    p=BufferedPort(b'partial');t=StreamValidationTransportV2(p)
    assert t._read_packet() is None
    assert t._read_packet() is None and p.reads[-1]==1
    p.data.extend(b'\0')
    assert t._read_packet()==b'partial\0'


def test_noise_is_bounded_and_later_complete_packet_can_be_read():
    p=BufferedPort(b'x'*5000);t=StreamValidationTransportV2(p)
    assert t._read_packet() is None and p.reads[0]==4096
    assert t._read_packet() is None and len(t._rx_residual)==4096
    p.data.extend(b'\0valid\0')
    assert t._read_packet().endswith(b'\0')
    assert t._read_packet()==b'valid\0'
