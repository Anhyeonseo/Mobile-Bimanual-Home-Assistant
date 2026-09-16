"""Age-preserving independent robot-state observations on the shared v2 link."""
import math
import struct
from .mobile_wire import crc32c


def validate_request(packet):
    if (len(packet) != 36 or packet[:4] != b'AE\x01\x01'
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

    def publish(self, observed_s, *, arms_safe, lift_hold, payload_safe):
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
        if self.sequence == 0xFFFFFFFF:
            raise RuntimeError('new boot and evidence client required')
        self.sequence += 1
        packet = bytearray(36);packet[:4] = b'AE\x01\x01'
        struct.pack_into('<4I', packet, 4, self.sequence, sync.boot_id, observed, (observed+self.age)&0xFFFFFFFF)
        packet[20] = int(arms_safe) | (int(lift_hold)<<1) | (int(payload_safe)<<2)
        struct.pack_into('<I', packet, 32, crc32c(packet[:32]))
        self.last_source_s = observed_s
        reply = self.transport.exchange_mobile(bytes(packet))
        if (len(reply) != 64 or reply[:4] != b'AF\x01\x01'
                or struct.unpack_from('<2I', reply, 4) != (self.sequence, sync.boot_id)
                or struct.unpack_from('<I', reply, 16)[0] != observed or reply[20] != packet[20]
                or any(reply[21:60]) or struct.unpack_from('<I', reply, 60)[0] != crc32c(reply[:60])):
            raise RuntimeError('evidence acknowledgement mismatch')
        if (self.clock()-observed_s)*1000 >= self.age:
            raise RuntimeError('evidence expired in transit')
        self.last_observed = observed
