"""Age-preserving independent robot-state observations on the shared v2 link."""
import math
import struct
from .mobile_wire import crc32c


def validate_request(packet):
    if (len(packet) != 36 or packet[:4] not in (b'AE\x01\x01', b'AE\x01\x02')
            or not struct.unpack_from('<I', packet, 4)[0]
            or not struct.unpack_from('<I', packet, 8)[0]
            or packet[20] & ~7 or any(packet[21:32])
            or struct.unpack_from('<I', packet, 32)[0] != crc32c(packet[:32])):
        raise ValueError('invalid robot evidence')


class RobotEvidenceClient:
    def __init__(self, transport, mobile, clock, *, maximum_age_ms):
        if type(maximum_age_ms) is not int or not 1 <= maximum_age_ms <= 1000:
            raise ValueError('explicit evidence age required')
        self.transport, self.mobile, self.clock = transport, mobile, clock
        self.age, self.sequence, self.last_observed = maximum_age_ms, 0, None
        self.last_source_s = None
        self.source_clock = None
        self.last_packet = None
        self.transport_only = False

    def publish_mcu_snapshot(self, snapshot, sent_s, received_s, *, arms_safe, lift_hold, payload_safe):
        """Preserve the oldest joint's native MCU timestamp on this shared link.

        A joint snapshot can legitimately repeat its oldest sample while other
        joints advance. Skip that duplicate without renewing its lease. This
        API carries no pose/safety inference; the caller validates measured
        conditions before publishing them.
        """
        now=self.clock()
        if self.transport_only:raise RuntimeError("transport session requires fresh recommissioning before arm/lift use")
        if (self.source_clock not in (None,'mcu') or self.mobile.boot_id is None or
            any(type(v) is not bool for v in (arms_safe,lift_hold,payload_safe)) or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in (sent_s,received_s,now)) or
            not 0<=sent_s<=received_s<=now or received_s-sent_s>=.1 or
            snapshot.status_code or snapshot.joint_count!=12 or snapshot.present_mask!=4095 or
            len(snapshot.sample_age_ms)!=12 or type(snapshot.firmware_tick_ms) is not int or
            not 0<=snapshot.firmware_tick_ms<2**32 or
            any(type(a) is not int or not 0<=a<self.age for a in snapshot.sample_age_ms)):
            raise ValueError('fresh complete same-MCU snapshot required')
        oldest=max(snapshot.sample_age_ms)
        if (now-sent_s)*1000+oldest>=self.age:raise ValueError('snapshot evidence expired in transit')
        observed=(snapshot.firmware_tick_ms-oldest)&0xffffffff
        if self.last_observed is not None:
            delta=(observed-self.last_observed)&0xffffffff
            if delta==0:return False
            if delta>=0x80000000:raise ValueError('regressed MCU sample time')
        # This timestamp is already in the MCU clock domain. AE acceptance
        # checks the current boot and age on-device; no host-clock conversion
        # or periodic synchronization transaction is needed for this proof.
        if not self.mobile.clock.sample or self.mobile.clock.sample[2].boot_id!=self.mobile.boot_id:
            raise RuntimeError('evidence boot mismatch')
        self.source_clock='mcu'
        self._publish_observed(observed,arms_safe,lift_hold,payload_safe)
        if (self.clock()-sent_s)*1000+oldest>=self.age:raise RuntimeError('snapshot evidence expired in transit')
        return True

    def publish(self, observed_s, *, arms_safe, lift_hold, payload_safe):
        if self.transport_only:raise RuntimeError('transport session is wheel-only')
        if self.source_clock not in (None,'host'):raise ValueError('cannot mix evidence source clocks')
        now = self.clock()
        if (type(observed_s) not in (float, int) or not math.isfinite(observed_s)
                or not 0 <= observed_s <= now or now - observed_s >= self.age / 1000
                or any(type(v) is not bool for v in (arms_safe, lift_hold, payload_safe))):
            raise ValueError('fresh independent evidence required')
        if self.last_source_s is not None and observed_s <= self.last_source_s:
            raise ValueError("repeated source observation")
        host_now = self.mobile.now()
        if not self.mobile.clock.sample or host_now - self.mobile.clock.sample[1] >= 500:
            self.mobile.refresh_clock()
            host_now = self.mobile.now()
        self.mobile.clock.deadline(host_now, self.age)
        sent, received, sync = self.mobile.clock.sample
        if sync.boot_id != self.mobile.boot_id:
            raise RuntimeError('evidence boot mismatch')
        # Preserve sensor age and subtract the full synchronization uncertainty.
        source_age = math.ceil((self.clock() - observed_s) * 1000)
        observed = (sync.mcu_tick_ms + host_now - received - (received-sent) - source_age) & 0xFFFFFFFF
        if self.last_observed is not None and not 0 < ((observed-self.last_observed)&0xFFFFFFFF) < 0x80000000:
            raise ValueError('repeated or regressed evidence time')
        self.source_clock='host'
        self.last_source_s = observed_s
        self._publish_observed(observed,arms_safe,lift_hold,payload_safe)
        if (self.clock()-observed_s)*1000 >= self.age:
            raise RuntimeError('evidence expired in transit')

    def _publish_observed(self,observed,arms_safe,lift_hold,payload_safe):
        if self.sequence == 0xFFFFFFFF:raise RuntimeError('new boot and evidence client required')
        self.sequence += 1
        packet = bytearray(36);packet[:4] = b'AE\x01\x01'
        struct.pack_into('<4I', packet, 4, self.sequence, self.mobile.boot_id, observed, (observed+self.age)&0xFFFFFFFF)
        packet[20] = int(arms_safe) | (int(lift_hold)<<1) | (int(payload_safe)<<2)
        struct.pack_into('<I', packet, 32, crc32c(packet[:32]))
        reply = self.transport.exchange_mobile(bytes(packet))
        if (len(reply) != 64 or reply[:4] != b'AF\x01\x01'
                or struct.unpack_from('<2I', reply, 4) != (self.sequence, self.mobile.boot_id)
                or struct.unpack_from('<I', reply, 16)[0] != observed or reply[20] != packet[20]
                or any(reply[21:60]) or struct.unpack_from('<I', reply, 60)[0] != crc32c(reply[:60])):
            raise RuntimeError('evidence acknowledgement mismatch')
        self.last_observed = observed
        self.last_packet = bytes(packet)

    def enter_transport(self):
        """One-way switch referencing the last real initial posture sample.

        False is a correlated UART-busy rejection with no switch; all malformed,
        expired or missing acknowledgements remain errors. Never renew evidence.
        """
        if self.transport_only:return True
        if self.last_packet is None or self.source_clock != 'mcu':
            raise RuntimeError('initial measured posture required')
        packet=bytearray(self.last_packet);packet[3]=2
        struct.pack_into('<I',packet,32,crc32c(packet[:32]))
        reply=self.transport.exchange_mobile(bytes(packet))
        if (len(reply)!=64 or reply[:3]!=b'AF\x01' or reply[3] not in (2,130)
                or reply[4:12]!=packet[4:12] or reply[16:21]!=packet[12:16]+packet[20:21]
                or any(reply[21:60]) or struct.unpack_from('<I',reply,60)[0]!=crc32c(reply[:60])):
            raise RuntimeError('transport admission acknowledgement mismatch')
        self.transport_only=reply[3]==2
        return self.transport_only
