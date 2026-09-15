"""Lift AL/LS v1 codec. Uses the resident arm/mobile transport's existing lock."""

from dataclasses import dataclass
import struct
from .mobile_wire import crc32c


def _u32(v):
    if type(v) is not int or not 0 <= v <= 0xFFFFFFFF:
        raise ValueError("uint32 required")
    return v


@dataclass(frozen=True)
class LiftCommand:
    kind: int
    session: int
    sequence: int
    valid_until_ms: int
    boot_id: int
    target_um: int = 0

    def encode(self):
        if (
            type(self.kind) is not int
            or not 1 <= self.kind <= 6
            or not _u32(self.session)
            or not _u32(self.boot_id)
        ):
            raise ValueError("invalid lift identity")
        _u32(self.sequence)
        _u32(self.valid_until_ms)
        if (
            type(self.target_um) is not int
            or not -(2**31) <= self.target_um < 2**31
            or (self.kind != 2 and self.target_um)
        ):
            raise ValueError("invalid lift height")
        if self.kind == 4 and self.valid_until_ms:
            raise ValueError("status must not renew a lease")
        data = struct.pack(
            "<2sBBIIIiI8x",
            b"AL",
            1,
            self.kind,
            self.session,
            self.sequence,
            self.valid_until_ms,
            self.target_um,
            self.boot_id,
        )
        return data + struct.pack("<I", crc32c(data))

    @classmethod
    def decode(cls, data):
        if (
            not isinstance(data, bytes)
            or len(data) != 36
            or data[:3] != b"AL\x01"
            or any(data[24:32])
            or struct.unpack_from("<I", data, 32)[0] != crc32c(data[:32])
        ):
            raise ValueError("invalid lift packet")
        _, _, kind, session, seq, deadline, target, boot = struct.unpack(
            "<2sBBIIIiI8x", data[:32]
        )
        value = cls(kind, session, seq, deadline, boot, target)
        if value.encode() != data:
            raise ValueError("noncanonical lift packet")
        return value


@dataclass(frozen=True)
class LiftStatus:
    kind: int
    request_sequence: int
    mcu_tick_ms: int
    boot_id: int
    session: int
    sequence: int
    state: int
    fault: int
    flags: int
    observed_ms: int
    height_um: int
    target_um: int
    command_raw: int
    current_ma: int
    position_raw: int
    rejected: int
    valid_until_ms: int

    @classmethod
    def decode(cls, data):
        if (
            not isinstance(data, bytes)
            or len(data) != 64
            or data[:3] != b"LS\x01"
            or data[27]
            or struct.unpack_from("<I", data, 60)[0] != crc32c(data[:60])
        ):
            raise ValueError("invalid lift status")
        kind = data[3]
        if not 1 <= (kind & 127) <= 6 or data[24] > 6 or data[25] > 5 or data[26] & ~31:
            raise ValueError("invalid lift state")
        value = cls(
            kind,
            *struct.unpack_from("<IIIII", data, 4),
            data[24],
            data[25],
            data[26],
            *struct.unpack_from("<IiiiiIII", data, 28)
        )
        if not value.boot_id or value.position_raw >= 4096 or value.current_ma < 0:
            raise ValueError("invalid lift feedback")
        return value

    def holding(self, maximum_age_ms=200):
        return (
            self.state == 5
            and self.fault == 0
            and (self.flags & 29) == 29
            and ((self.mcu_tick_ms - self.observed_ms) & 0xFFFFFFFF) < maximum_age_ms
        )


class LiftClient:
    def __init__(self, mobile_client):
        self.mobile = mobile_client
        self.sequence = 0
        self.session = None
        self.last_session = None

    def _request(self, kind, session, target=0):
        if self.mobile.boot_id is None or self.sequence == 0xFFFFFFFF:
            raise RuntimeError("synchronize mobile clock before lift use")
        self.sequence += 1
        deadline = 0 if kind == 4 else self.mobile._deadline(200)[0]
        packet = LiftCommand(
            kind, session, self.sequence, deadline, self.mobile.boot_id, target
        ).encode()
        try:
            reply = LiftStatus.decode(self.mobile.port.exchange(packet))
            if (
                reply.boot_id != self.mobile.boot_id
                or reply.kind != kind
                or reply.request_sequence != self.sequence
            ):
                raise RuntimeError("lift rejected or MCU restarted")
            if kind != 4 and reply.session != session:
                raise RuntimeError("lift ownership mismatch")
            return reply
        except Exception:
            self.session = None
            raise

    def home(self, session):
        result = self._request(1, session)
        self.session = session
        self.last_session = session
        return result

    def move(self, session, height_um):
        result = self._request(2, session, height_um)
        self.session = session
        self.last_session = session
        return result

    def status(self):
        return self._request(4, self.session or 1)

    def keepalive(self):
        if self.session is None:
            raise RuntimeError("no lift owner")
        return self._request(5, self.session)

    def cancel(self):
        if self.session is None:
            raise RuntimeError("no lift owner")
        result = self._request(3, self.session)
        self.session = None
        return result

    def reset(self, session=None):
        owner = session if session is not None else self.last_session
        if owner is None:
            raise RuntimeError("explicit prior lift session required")
        result = self._request(6, owner)
        self.session = None
        return result
