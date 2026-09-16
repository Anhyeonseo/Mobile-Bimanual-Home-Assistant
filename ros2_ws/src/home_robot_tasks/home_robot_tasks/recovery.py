"""Explicit cold recovery. No re-arm, homing, task replay or MCU reset is sent.

A faulted instance stays retired. A new disabled boot, independently verified
secured mechanism and closed old transports are prerequisites for a new runtime.
New jobs still require commissioning/homing/fresh evidence through normal gates.
"""
from dataclasses import dataclass
from .fetch import InvalidTask
from .navigation import number


@dataclass(frozen=True)
class RecoveryEvidence:
    observed_s: float
    previous_boot: int
    new_boot: int
    mechanism_secured: bool
    old_clients_closed: bool
    outputs_disabled: bool
    lift_unhomed: bool
    device_identity_verified: bool
    profile_sha256: str


class RecoveryCoordinator:
    def __init__(self, runtime, boot_id, profile_sha256, clock):
        if type(boot_id) is not int or not 0 < boot_id <= 0xFFFFFFFF:
            raise InvalidTask('recovery boot identity required')
        if not isinstance(profile_sha256,str) or len(profile_sha256)!=64 or any(c not in '0123456789abcdef' for c in profile_sha256):
            raise InvalidTask('recovery profile SHA-256 required')
        self.old, self.boot, self.profile, self.clock = runtime,boot_id,profile_sha256,clock
        self.state, self.reason = 'ACTIVE', None
        self.evidence = None

    def begin(self, reason):
        if self.state != 'ACTIVE': raise InvalidTask('recovery already requested')
        if not isinstance(reason,str) or not reason.strip(): raise InvalidTask('recovery reason required')
        self.reason = reason; self.state = 'AWAITING_SECURED_NEW_BOOT'
        self.old.fault = 'explicit_recovery:' + reason
        # Admission is revoked immediately; existing leases are not released.
        self.old.tick()

    def accept(self, evidence):
        if self.state != 'AWAITING_SECURED_NEW_BOOT' or not isinstance(evidence,RecoveryEvidence):
            raise InvalidTask('new disabled boot evidence required')
        if (not 0 <= self.clock()-number(evidence.observed_s,'recovery time') <= .5
                or evidence.previous_boot != self.boot or type(evidence.new_boot) is not int
                or not self.boot < evidence.new_boot <= 0xFFFFFFFF
                or evidence.profile_sha256 != self.profile
                or any(getattr(evidence,k) is not True for k in ('mechanism_secured','old_clients_closed',
                    'outputs_disabled','lift_unhomed','device_identity_verified'))):
            raise InvalidTask('cold recovery prerequisites not verified')
        self.evidence = evidence; self.state = 'READY_FOR_BOOTSTRAP'

    def bootstrap(self, factory):
        if self.state != 'READY_FOR_BOOTSTRAP': raise InvalidTask('recovery not verified')
        if not 0 <= self.clock()-self.evidence.observed_s <= .5: raise InvalidTask('recovery evidence expired')
        self.state = 'BOOTSTRAPPING'  # A partial factory must not run twice.
        try:
            candidate = factory(self.evidence.new_boot)
            if candidate is self.old or candidate.app.lease.owner is not None or candidate.app.tasks:
                raise InvalidTask('new idle runtime required, no old task replay')
            if candidate.fault is not None or getattr(candidate,'boot_id',None)!=self.evidence.new_boot:
                raise InvalidTask('new runtime boot identity mismatch or fault')
        except Exception:
            self.state = 'BOOTSTRAP_FAILED'
            raise
        self.state = 'RETIRED'
        return candidate
