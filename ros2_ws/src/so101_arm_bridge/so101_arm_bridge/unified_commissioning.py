"""S4 AC/CS commissioning over the existing locked protocol-v2 transport.

Queries never prepare, arm, home, or release anything. Explicit preparation
changes only mobile mode/lock registers while all sixteen torques are OFF.
"""
from dataclasses import dataclass
import struct
from .mobile_wire import crc32c

FIRMWARE = 0x0002492D

def request(kind, sequence, ticket=0, *, supported=False):
    if type(kind) is not int or kind not in (1, 2, 3, 4, 5) or type(sequence) is not int or not 0<sequence<2**32:
        raise ValueError('commissioning operation/sequence required')
    if type(ticket) is not int or not 0<=ticket<2**32:
        raise ValueError('invalid commissioning ticket')
    if kind==1 and (ticket or supported):raise ValueError('query cannot carry preparation flags')
    if kind!=1 and (not ticket or supported is not True):raise ValueError('fresh ticket and supported mechanism required')
    value=struct.pack('<2sBBIII16x',b'AC',1,kind,sequence,ticket,0x53454355 if supported else 0)
    return value+struct.pack('<I',crc32c(value))

def validate_request(packet):
    if not isinstance(packet,bytes) or len(packet)!=36 or packet[:3]!=b'AC\x01':
        raise ValueError('invalid commissioning request')
    sequence,ticket,confirmation=struct.unpack_from('<III',packet,4)
    if confirmation not in (0,0x53454355):raise ValueError('invalid support confirmation')
    if request(packet[3],sequence,ticket,supported=confirmation==0x53454355)!=packet:
        raise ValueError('invalid commissioning request fields or CRC')

@dataclass(frozen=True)
class CommissionStatus:
    kind:int
    sequence:int
    mcu_tick_ms:int
    boot_id:int
    ticket:int
    issued_ms:int
    flags:int
    reason:int
    profile_crc32:int

    @classmethod
    def decode(cls,data):
        if (not isinstance(data,bytes) or len(data)!=64 or data[:3]!=b'CS\x01' or
            data[3] not in (1,2,3,4,5,129,130,131,132,133) or any(data[26:28]) or any(data[32:60]) or
            data[24]&~127 or data[25]>7 or struct.unpack_from('<I',data,60)[0]!=crc32c(data[:60])):
            raise ValueError('invalid commissioning response')
        result=cls(data[3],*struct.unpack_from('<IIIII',data,4),data[24],data[25],struct.unpack_from('<I',data,28)[0])
        if bool(result.flags&1)!=bool(result.boot_id):raise ValueError('inconsistent prepared boot')
        return result

class CommissionClient:
    def __init__(self,transport):self.transport=transport;self.sequence=0
    def _exchange(self,kind,ticket=0,*,supported=False):
        self.sequence+=1
        reply=CommissionStatus.decode(self.transport.exchange_mobile(request(kind,self.sequence,ticket,supported=supported)))
        if reply.kind!=kind or reply.sequence!=self.sequence:
            raise RuntimeError(f'commissioning rejected; no retry: {reply}')
        return reply
    def query(self):return self._exchange(1)
    def prepare(self,*,supported=False,empty_grippers=False):
        if supported is not True or empty_grippers is not True:raise ValueError('current support and empty-gripper confirmation required')
        initial=self.query()
        if initial.flags or not initial.ticket:raise RuntimeError('fresh unprepared controller required')
        result=self._exchange(2,initial.ticket,supported=True)
        if result.flags!=17 or result.reason or not result.boot_id:raise RuntimeError('unexpected preparation state')
        return result

    def activate_zero(self,*,supported=False,empty_grippers=False):
        if supported is not True or empty_grippers is not True:
            raise ValueError('supported empty robot confirmation required before torque enable')
        initial=self.query()
        if initial.flags!=17 or initial.reason or not initial.ticket:
            raise RuntimeError('fresh prepared, torque-off maintenance state required')
        result=self._exchange(3,initial.ticket,supported=True)
        if (result.flags!=113 or result.reason or result.boot_id!=initial.boot_id or
                result.profile_crc32!=initial.profile_crc32):raise RuntimeError('zero-power activation incomplete')
        return result

    def release_supported(self,*,supported=False,empty_grippers=False):
        if supported is not True or empty_grippers is not True:
            raise ValueError('support and empty-gripper confirmation required before torque release')
        initial=self.query()
        if not initial.flags&64 or initial.flags&14 or not initial.ticket:
            raise RuntimeError('mobile-only maintenance power session required')
        result=self._exchange(4,initial.ticket,supported=True)
        if result.flags!=17 or result.reason or result.boot_id!=initial.boot_id:
            raise RuntimeError('torque release unconfirmed')
        return result

    def release_boot_supported(self,*,supported=False,empty_grippers=False):
        """Explicit supported release after reset, before any registration.

        No reset/retry here; a partial failure needs a new operator decision.
        This command never enables torque or changes calibration/modes.
        """
        if supported is not True or empty_grippers is not True:
            raise ValueError('supported empty mechanism required before boot release')
        initial=self.query()
        if initial.flags or initial.reason or not initial.ticket:
            raise RuntimeError('fresh unconfigured boot required for supported release')
        result=self._exchange(5,initial.ticket,supported=True)
        if result.flags or result.reason or result.boot_id or result.profile_crc32!=initial.profile_crc32:
            raise RuntimeError('supported boot release unconfirmed')
        return result
