from dataclasses import replace
import struct
from so101_arm_bridge.mobile_inspection import inspect_mobile
from so101_arm_bridge.mobile_wire import crc32c
from so101_arm_bridge.stream_protocol_v2 import HelloV2

class Transport:
    def __init__(self,error=None):self.error=error;self.queries=[];self.hello=HelloV2(2,12,False,0x24908,1,1,0,0)
    def enter_binary_mode(self):return self.hello
    def exchange_mobile(self,p):
        self.queries.append(p)
        assert p[:3]==b'AM\x01' and p[3] in (4,5)
        if self.error:raise RuntimeError(self.error)
        # Same canonical 64-byte mobile status as the firmware wire schema.
        data=bytearray(64);data[:4]=b'AS\x01'+bytes([p[3]])
        struct.pack_into('<III',data,4,p[3],100,42)
        struct.pack_into('<I',data,60,crc32c(data[:60]))
        return bytes(data)


def test_inspection_sends_only_queries_and_never_claims_verified_motor_ids():
    t=Transport();r=inspect_mobile(t)
    assert r['mobile_available'] and len(t.queries)==2
    assert r['motor_commands_sent']==0 and not r['motion_authorized'] and not r['motor_ids_verified']


def test_missing_endpoint_is_reported_without_arm_or_retry():
    t=Transport('mobile endpoint is not attached on this firmware');r=inspect_mobile(t)
    assert not r['mobile_available'] and 'not attached' in r['error'] and len(t.queries)==1


def test_other_protocol_is_not_probed_with_mobile_commands():
    t=Transport();t.hello=replace(t.hello,protocol_version=1)
    assert not inspect_mobile(t)['mobile_available'] and not t.queries
