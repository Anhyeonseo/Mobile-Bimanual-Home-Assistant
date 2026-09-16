"""Read-only whole-system stop proof on the resident arm's existing transport."""
from dataclasses import dataclass
import math
import struct
import threading
from .mobile_wire import crc32c


def stop_query(sequence):
    if type(sequence) is not int or not 0 < sequence <= 0xFFFFFFFF:
        raise ValueError("invalid stop query sequence")
    data = bytearray(36)
    data[:4] = b"AQ\x01\x01"
    struct.pack_into("<I", data, 4, sequence)
    struct.pack_into("<I", data, 32, crc32c(data[:32]))
    return bytes(data)


@dataclass(frozen=True)
class SystemStopStatus:
    sequence: int
    tick_ms: int
    boot_id: int
    started_ms: int
    actions_completed_ms: int
    state: int
    flags: int
    observed_ms: int
    age_ms: int
    max_age_ms: int

    @classmethod
    def decode(cls, data):
        if (len(data) != 64 or data[:4] != b"AT\x01\x01"
                or crc32c(data[:60]) != struct.unpack_from("<I", data, 60)[0]
                or data[24] > 3 or any(data[26:28]) or any(data[40:60])):
            raise ValueError("invalid whole-stop reply")
        reply = cls(*struct.unpack_from("<5I", data, 4), data[24], data[25],
                    *struct.unpack_from("<3I", data, 28))
        if (not reply.sequence or not reply.boot_id or not 0 < reply.max_age_ms < 0x80000000
                or reply.age_ms != (reply.tick_ms - reply.observed_ms) & 0xFFFFFFFF):
            raise ValueError("invalid whole-stop clock evidence")
        return reply

    @property
    def confirmed(self):
        after = (self.observed_ms - self.actions_completed_ms) & 0xFFFFFFFF
        return (self.state == 2 and self.flags == 255 and self.age_ms < self.max_age_ms
                and 0 < after < 0x80000000)


class SystemStopClient:
    def __init__(self, transport, boot_id, clock, *, max_roundtrip_s=0.1):
        if type(boot_id) is not int or not 0 < boot_id <= 0xFFFFFFFF:
            raise ValueError("explicit synchronized boot identity required")
        if not math.isfinite(max_roundtrip_s) or max_roundtrip_s <= 0:
            raise ValueError("invalid roundtrip limit")
        self.transport, self.boot_id, self.clock = transport, boot_id, clock
        self.max_roundtrip_s = max_roundtrip_s
        self.sequence = 0
        self._lock = threading.Lock()

    def request(self):
        # A latched SAFE_STOP response acknowledges admission, never completion.
        return self.transport.safe_stop()

    def status(self):
        with self._lock:
            self.sequence += 1
            started = self.clock()
            reply = SystemStopStatus.decode(self.transport.exchange_mobile(stop_query(self.sequence)))
            received = self.clock()
            if (reply.sequence != self.sequence or reply.boot_id != self.boot_id
                    or not math.isfinite(started) or not math.isfinite(received)
                    or not 0 <= received - started <= self.max_roundtrip_s):
                raise RuntimeError("stale stop reply, clock jump or MCU restart")
            # Subtract the full round trip: never make remote measurements newer.
            observed = started - reply.age_ms / 1000.0
            if reply.confirmed and received - observed >= reply.max_age_ms / 1000.0:
                raise RuntimeError("stop evidence expired in transit")
            return reply, observed
