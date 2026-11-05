"""Read-only lift development observations; never a motion/runtime profile.

The legacy commissioning firmware has no boot identity. A reader session is
therefore ephemeral: closing the port or losing continuity invalidates its origin.
"""
import math
import uuid


class LiftObservation:
    def __init__(self, *, bottom_confirmed, m_per_turn=0.084):
        if bottom_confirmed is not True:
            raise ValueError('operator must confirm the current mechanical bottom')
        if not math.isfinite(m_per_turn) or not 0 < m_per_turn < 1:
            raise ValueError('invalid lift scale')
        self.m_per_turn = m_per_turn
        self.session_id = str(uuid.uuid4())
        self.origin_id = None
        self.origin = None
        self.last = None
        self.unwrapped = None
        self.quiet = []
        self.initial_raw = None
        self.invalid_reason = None

    def invalidate(self, reason):
        self.invalid_reason = reason

    def add(self, sample, received_s):
        if self.invalid_reason:
            raise ValueError('origin invalid: ' + self.invalid_reason)
        try:
            return self._add(sample, received_s)
        except (KeyError, TypeError, ValueError) as exc:
            self.invalidate(str(exc))
            raise ValueError('lift observation rejected: ' + str(exc)) from exc

    def _add(self, s, received):
        if not math.isfinite(received) or received < 0:
            raise ValueError('invalid host time')
        if s.get('type') != 'snapshot' or s.get('id') != 10 or s.get('telemetry_valid') is not True:
            raise ValueError('fresh ID10 snapshot required')
        for key, high in [('position', 4095), ('speed', 65535), ('sample_tick', 0xffffffff),
                          ('telemetry_age_ms', 100), ('motor_status_raw', 0)]:
            if type(s[key]) is not int or not 0 <= s[key] <= high:
                raise ValueError('invalid or stale ' + key)
        if s.get('goal') != 0 or s.get('torque') not in (0, 1):
            raise ValueError('idle motor required')
        p, tick = s['position'], s['sample_tick']
        if self.last is None:
            self.unwrapped = p
            self.initial_raw = p
        else:
            old_p, old_tick, old_received = self.last
            # Tick rollover is deliberately a new session, like a restart.
            if not 0 < tick - old_tick <= 200 or not 0 < received - old_received <= .2:
                raise ValueError('sample gap, repeated sample or controller restart')
            delta = (p - old_p + 2048) % 4096 - 2048
            if abs(delta) > 256:
                raise ValueError('encoder continuity lost')
            self.unwrapped += delta
        self.last = (p, tick, received)
        if self.origin is None:
            if abs(self.unwrapped - self.initial_raw) > 2:
                raise ValueError('lift moved before bottom origin was established')
            self.quiet = (self.quiet + [(self.unwrapped, s['speed'] & 0x7fff)])[-5:]
            if len(self.quiet) == 5:
                positions = [v[0] for v in self.quiet]
                if max(positions) - min(positions) <= 2 and all(v[1] == 0 for v in self.quiet):
                    self.origin = self.unwrapped
                    self.origin_id = str(uuid.uuid4())
        height = None if self.origin is None else -(self.unwrapped - self.origin) * self.m_per_turn / 4096
        if height is not None and not -.005 <= height <= .645:
            raise ValueError('height outside development travel')
        return dict(session_id=self.session_id, origin_id=self.origin_id,
                    origin_raw_unwrapped=self.origin, lift_raw_unwrapped=self.unwrapped,
                    lift_height_m=height, sample_tick_ms=tick, received_monotonic_s=received,
                    telemetry_age_ms=s['telemetry_age_ms'],
                    state='bottom_candidate_observed' if self.origin is not None else 'waiting_for_quiet_samples',
                    coordinate_use_allowed=False, motion_authorized=False)
