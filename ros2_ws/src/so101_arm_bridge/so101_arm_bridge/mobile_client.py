"""Synchronous mobile command client for an exclusive, bounded exchange port.

Port contract: exchange(bytes) returns one 64-byte response within the caller's
budget. This module has no serial discovery, auto-arm or automatic reconnect.
"""

from .mobile_wire import MobileCommand, MobileStatus, MobileClock, mobile_query


class MobileClient:
    def __init__(self, port, clock_ms):
        self.port, self.now = port, clock_ms
        self.clock = MobileClock()
        self.sequence = 0
        self.session = None
        self.boot_id = None

    def _sequence(self):
        if self.sequence == 0xFFFFFFFF:
            raise RuntimeError("new client required before sequence wrap")
        self.sequence += 1
        return self.sequence

    def _exchange(self, packet, sequence, kind):
        try:
            result = MobileStatus.decode(self.port.exchange(packet))
            if result.request_sequence != sequence or result.kind != kind:
                raise RuntimeError("request rejected or response mismatch")
            if self.boot_id is not None and result.boot_id != self.boot_id:
                self.session = None
                self.clock.sample = None
                raise RuntimeError("MCU restarted; explicit resynchronization required")
            return result
        except Exception:
            self.session = None
            raise

    def synchronize(self):
        self.session = None
        self.clock.sample = None
        self.boot_id = None
        seq = self._sequence()
        hello = self._exchange(mobile_query(4, seq, seq), seq, 4)
        if hello.capabilities & 7 != 7:
            raise RuntimeError("mobile capabilities missing")
        self.boot_id = hello.boot_id
        seq = self._sequence()
        sent = self.now()
        result = self._exchange(mobile_query(6, seq, seq), seq, 6)
        self.clock.update(result, seq, sent, self.now())
        return result

    def refresh_clock(self):
        if self.boot_id is None:
            raise RuntimeError("explicit synchronization required")
        seq = self._sequence()
        sent = self.now()
        result = self._exchange(mobile_query(6, seq, seq), seq, 6)
        self.clock.update(result, seq, sent, self.now())
        return result

    def _deadline(self, lifetime_ms):
        if self.clock.sample and self.now() - self.clock.sample[1] >= 500:
            self.refresh_clock()
        return self.clock.deadline(self.now(), lifetime_ms)

    def status(self):
        seq = self._sequence()
        return self._exchange(mobile_query(5, seq, seq), seq, 5)

    def arm(self, session):
        if self.session is not None:
            raise RuntimeError("already armed")
        deadline, boot = self._deadline(200)
        result = self._exchange(MobileCommand(1, session, 0, deadline).encode(), 0, 1)
        if result.state != 1 or result.session != session or result.boot_id != boot:
            raise RuntimeError("arm not verified")
        self.session = session
        return result

    def velocity(self, values, lifetime_ms=200):
        if self.session is None:
            raise RuntimeError("explicit arm required")
        deadline, _ = self._deadline(lifetime_ms)
        seq = self._sequence()
        return self._exchange(
            MobileCommand(2, self.session, seq, deadline, tuple(values)).encode(),
            seq,
            2,
        )

    def stop(self):
        if self.session is None:
            raise RuntimeError("no owned mobile session")
        deadline, _ = self._deadline(200)
        seq = self._sequence()
        result = self._exchange(
            MobileCommand(3, self.session, seq, deadline).encode(), seq, 3
        )
        self.session = None
        return result
