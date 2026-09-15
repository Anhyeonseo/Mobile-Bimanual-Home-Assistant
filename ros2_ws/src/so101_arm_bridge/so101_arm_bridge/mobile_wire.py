"""Experimental mobile command codec; not sent by the resident arm transport.

Deadline is synchronized STM32 milliseconds, NOT host Unix/monotonic time.
Velocity here is signed servo units, not the sign-magnitude servo wire encoding.
"""

from dataclasses import dataclass
import struct

_FORMAT = struct.Struct("<2sBBIII4i")


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


@dataclass(frozen=True)
class MobileCommand:
    opcode: int
    session: int
    sequence: int
    valid_until_ms: int
    velocity_raw: tuple[int, int, int, int] = (0, 0, 0, 0)

    def validate(self):
        for value in (self.opcode, self.session, self.sequence, self.valid_until_ms):
            if type(value) is not int or not 0 <= value <= 0xFFFFFFFF:
                raise ValueError("mobile header fields must be uint32 integers")
        if (
            self.opcode not in (1, 2, 3)
            or not self.session
            or (self.opcode == 1 and self.sequence != 0)
        ):
            raise ValueError("invalid opcode/session/arm sequence")
        if not isinstance(self.velocity_raw, tuple) or len(self.velocity_raw) != 4:
            raise ValueError("exactly four velocity values required")
        if any(
            type(v) is not int or not -32767 <= v <= 32767 for v in self.velocity_raw
        ):
            raise ValueError("invalid servo velocity")
        if self.opcode != 2 and any(self.velocity_raw):
            raise ValueError("arm/stop cannot carry velocities")

    def encode(self) -> bytes:
        self.validate()
        data = _FORMAT.pack(
            b"AM",
            1,
            self.opcode,
            self.session,
            self.sequence,
            self.valid_until_ms,
            *self.velocity_raw
        )
        return data + struct.pack("<I", crc32c(data))

    @classmethod
    def decode(cls, data: bytes):
        if not isinstance(data, bytes) or len(data) != 36 or data[:3] != b"AM\x01":
            raise ValueError("invalid mobile frame")
        if struct.unpack("<I", data[32:])[0] != crc32c(data[:32]):
            raise ValueError("mobile CRC mismatch")
        _, _, opcode, session, sequence, deadline, *velocity = _FORMAT.unpack(data[:32])
        result = cls(opcode, session, sequence, deadline, tuple(velocity))
        result.validate()
        return result


@dataclass(frozen=True)
class MobileStatus:
    kind: int
    request_sequence: int
    mcu_tick_ms: int
    boot_id: int
    capabilities: int
    session: int
    sequence: int
    state: int
    reason: int
    flags: int
    feedback_tick_ms: int
    velocity_raw: tuple[int, int, int, int]
    lift_position_um: int
    rejected_frames: int

    def feedback_is_fresh(self, max_age_ms=200):
        """Compare MCU sample and response clocks, including uint32 rollover."""
        if type(max_age_ms) is not int or not 0 < max_age_ms < 0x80000000:
            raise ValueError("bounded feedback age required")
        return (
            bool(self.flags & 1)
            and ((self.mcu_tick_ms - self.feedback_tick_ms) & 0xFFFFFFFF) <= max_age_ms
        )

    def ready_for_motion(self, owned_session, max_age_ms=200):
        """Fresh reception alone cannot authorize an old/faulted MCU sample."""
        return (
            type(owned_session) is int
            and 0 < owned_session <= 0xFFFFFFFF
            and self.session == owned_session
            and self.state in (1, 2)
            and self.reason == 0
            and self.flags & 23 == 23
            and self.feedback_is_fresh(max_age_ms)
        )

    @classmethod
    def decode(cls, data: bytes):
        if not isinstance(data, bytes) or len(data) != 64 or data[:3] != b"AS\x01":
            raise ValueError("invalid mobile status")
        if crc32c(data[:60]) != struct.unpack_from("<I", data, 60)[0]:
            raise ValueError("status CRC mismatch")
        values = struct.unpack("<2sBB6IBBHI4iiII", data)
        (
            _,
            _,
            kind,
            req,
            tick,
            boot,
            caps,
            session,
            seq,
            state,
            reason,
            flags,
            stamp,
            *tail,
        ) = values
        if not boot or state > 3 or reason > 4 or flags & ~31:
            raise ValueError("unsupported status fields")
        if kind not in tuple(range(1, 7)) + tuple(v | 0x80 for v in range(1, 7)):
            raise ValueError("unsupported reply kind")
        return cls(
            kind,
            req,
            tick,
            boot,
            caps,
            session,
            seq,
            state,
            reason,
            flags,
            stamp,
            tuple(tail[:4]),
            tail[4],
            tail[5],
        )


def mobile_query(kind: int, nonce: int, sequence: int) -> bytes:
    if type(kind) is not int or kind not in (4, 5, 6):
        raise ValueError("query kind must be HELLO/STATUS/TIME_SYNC")
    for value in (nonce, sequence):
        if type(value) is not int or not 0 <= value <= 0xFFFFFFFF:
            raise ValueError("query fields must be uint32")
    if not nonce:
        raise ValueError("nonzero query nonce required")
    data = _FORMAT.pack(b"AM", 1, kind, nonce, sequence, 0, 0, 0, 0, 0)
    return data + struct.pack("<I", crc32c(data))


class MobileClock:
    """Bounded midpoint clock estimate; no assumptions about equal host/MCU clocks.

    Correlate the TIME_SYNC reply sequence before calling update. A different
    boot ID invalidates existing sessions. RTT is an uncertainty bound, not a
    proof of one-way symmetry. Deadlines subtract that bound conservatively.
    """

    def __init__(self, max_rtt_ms=20, max_age_ms=1000):
        if (
            type(max_rtt_ms) is not int
            or type(max_age_ms) is not int
            or not 0 < max_rtt_ms < 1000
            or not 0 < max_age_ms < 60000
        ):
            raise ValueError("invalid clock budgets")
        self.max_rtt_ms, self.max_age_ms = max_rtt_ms, max_age_ms
        self.sample = None

    def update(
        self,
        status: MobileStatus,
        expected_sequence: int,
        sent_ms: int,
        received_ms: int,
    ):
        for value in (sent_ms, received_ms):
            if type(value) is not int or value < 0:
                raise ValueError("invalid host clock")
        if (
            status.kind != 6
            or status.request_sequence != expected_sequence
            or not 0 <= received_ms - sent_ms <= self.max_rtt_ms
        ):
            raise ValueError("uncorrelated or delayed clock response")
        if self.sample and (sent_ms < self.sample[0] or received_ms <= self.sample[1]):
            raise ValueError("old clock exchange")
        previous_boot = self.sample[2].boot_id if self.sample else None
        self.sample = (sent_ms, received_ms, status)
        return previous_boot is not None and previous_boot != status.boot_id

    def deadline(self, now_ms: int, lifetime_ms: int) -> tuple[int, int]:
        if type(now_ms) is not int or type(lifetime_ms) is not int or not self.sample:
            raise ValueError("clock not synchronized")
        sent, received, status = self.sample
        if not 0 <= now_ms - received < self.max_age_ms or not 1 <= lifetime_ms <= 1000:
            raise ValueError("stale clock or invalid command lifetime")
        uncertainty = received - sent
        if lifetime_ms <= uncertainty:
            raise ValueError("lifetime shorter than clock uncertainty")
        # MCU sampled between host send and receive; earliest consistent MCU
        # time at now is mcu_tick + now - receive. Use that for the deadline.
        return (
            status.mcu_tick_ms + now_ms - received + lifetime_ms - uncertainty
        ) & 0xFFFFFFFF, status.boot_id
