"""Exact identity binding for the locally built S3 resident firmware candidate.

Reading development limits does not authorize motion. A caller must supply
separately authorized limits with the same digest/ranges before enabling the
adapter, and the application still requires a fresh stationary scene.
"""
from dataclasses import asdict, is_dataclass

from .bimanual_stream_adapter import (
    ResidentBimanualStreamAdapter, OperationalLimits, CANONICAL_JOINT_NAMES,
    BimanualStreamContractError,
)
from .development_ranges import read_development_ranges, calibration_hash
from .servo_position import validate_wire_route


class StationaryPickAdapter(ResidentBimanualStreamAdapter):
    firmware_version = 0x00024911
    gripper_torque_cap_raw = 80
    retains_gripper_goal = True

    def _terminal_bus_available(self, dispatch, tracking, state, executor):
        # SUCCEEDED precedes MCU settling. Read its explicit handoff state;
        # neither a guessed delay nor idle observation-UART availability is
        # evidence that the next finite route can be opened.
        self.last_firmware_runtime = self._transport.stationary_runtime()
        if self.last_firmware_runtime['stop_latched']:
            raise BimanualStreamContractError('stationary runtime stopped: '+str(self.last_firmware_runtime))
        return not dispatch.active and self.last_firmware_runtime['ready_for_stream']

    def __init__(self, transport, range_path, *, authorized_limits=None, motion_authorized=False, **kwargs):
        _, entries, source_hash = read_development_ranges(range_path)
        self._encoder_directions = tuple(e['positive_raw_direction'] for e in entries)
        expected = OperationalLimits(
            CANONICAL_JOINT_NAMES, tuple(e['minimum_urad'] for e in entries),
            tuple(e['maximum_urad'] for e in entries), source_hash, hardware_authorized=False)
        if authorized_limits is not None:
            if (not isinstance(authorized_limits, OperationalLimits)
                    or authorized_limits.joint_names != expected.joint_names
                    or authorized_limits.minimum_urad != expected.minimum_urad
                    or authorized_limits.maximum_urad != expected.maximum_urad
                    or authorized_limits.source_sha256 != source_hash):
                raise BimanualStreamContractError('stationary firmware/host range binding mismatch')
        self.calibration_hash = calibration_hash(source_hash)
        self.last_runtime_diagnostics = None
        self.last_firmware_runtime = None
        super().__init__(transport, authorized_limits or expected, motion_authorized=motion_authorized, **kwargs)

    def _check_servo_route(self, converted, reference):
        try:
            validate_wire_route((row for _, row in converted), self._encoder_directions,
                                reference=reference)
        except ValueError as exc:
            raise BimanualStreamContractError(str(exc)) from exc
        return converted

    def _validate_points(self, points, *, first_reference_urad, first_span_ms):
        converted = super()._validate_points(points, first_reference_urad=first_reference_urad,
                                             first_span_ms=first_span_ms)
        return self._check_servo_route(converted, first_reference_urad)

    def _validate_finite_plan_points(self, points):
        converted = super()._validate_finite_plan_points(points)
        reference = None if self._prepared is None else self._prepared.positions_urad
        return self._check_servo_route(converted, reference)

    def _collect_post_stop_diagnostics(self):
        try:
            self.last_firmware_runtime = self._transport.stationary_runtime()
            runtime = str(self.last_firmware_runtime)
        except Exception as exc:
            runtime = 'unavailable: '+str(exc)
        try:
            failures = str(self._transport.stationary_feedback_failures())
        except Exception as exc:
            failures = 'unavailable: '+str(exc)
        return ('stationary_runtime='+runtime+'; motor_read_failures='+failures+'; '
                +super()._collect_post_stop_diagnostics())

    def poll(self, owner):
        values = super().poll(owner)
        self.last_runtime_diagnostics = {
            name: asdict(value) if is_dataclass(value) else vars(value).copy()
            for name, value in zip(('state', 'executor', 'dispatch', 'tracking'), values)
        }
        return values

    def start(self, owner, points, *, finite=False):
        # Configuration/anchor reads and asynchronous observation cannot share
        # UART ownership. The firmware resumes idle reads at finite completion.
        if not self._motion_authorized:
            raise BimanualStreamContractError('motion_authorized is false')
        if self.state.value != 'ready':
            raise BimanualStreamContractError('adapter is not ready')
        # An unarmed anchor refresh itself owns the motor buses. An already
        # held arm needs no such refresh: keep observations alive throughout
        # finite-plan validation (13 ms measured on Jetson for 367 points).
        if not self._armed:self._transport.stationary_platform(2)
        return super().start(owner, points, finite=finite)

    def _initial_batch_base(self, previous_base):
        # OPEN is a protocol transaction and does not acquire the servo buses.
        # Keep the idle reader alive until it has completed; then suspend it
        # before the firmware's first APPEND starts motion feedback ownership.
        self._transport.stationary_platform(2)
        base = self._healthy_heartbeat().last_heartbeat_ms
        self.last_start_timing = dict(previous_base_tick=previous_base,
            initial_batch_base_tick=base,
            setup_elapsed_ms=(base-previous_base)&0xffffffff,
            policy='reader_until_open_then_fresh_clock_before_initial_append')
        return base

    def activate_stationary_hold(self):
        """Configure at measured position before the camera/stream deadline starts.

        The application revalidates live observations after this bounded setup.
        It never counts configuration time as arm motion or fresh feedback.
        """
        if not self._motion_authorized or self.state.value!='ready' or self._armed or self._owner is not None:
            raise BimanualStreamContractError('authorized unowned READY hold required')
        self._transport.stationary_platform(2)
        try:
            self.refresh_unarmed_anchor()
            self._armed=True
            self._transport.arm(self.calibration_hash)
            self._healthy_heartbeat()
            state=self._transport.enable()
            if state.stop_latched or state.status_code:
                raise BimanualStreamContractError('stationary hold configuration failed')
            self._healthy_heartbeat()
            self._transport.stationary_platform(1)
        except Exception as exc:self._fault(exc)
