"""Synchronous transport for STM32 protocol-v2 commands and feedback."""

from __future__ import annotations

import struct
import math
from collections import deque
from functools import wraps
from threading import RLock
import time
from time import perf_counter as _perf_counter, thread_time as _thread_time
from typing import Any
from serial import SerialTimeoutException

from .stream_protocol_v2 import (
    BimanualDispatchDiagnosticsV2,
    BimanualFeedbackSnapshotV2,
    BimanualTrackingDiagnosticsV2,
    BatchKindV2,
    FrameV2,
    HelloV2,
    StateV2,
    StreamBatchV2,
    StreamExecutorDiagnosticsV2,
    StreamShadowSnapshotV2,
    StreamMessageTypeV2,
    StreamPolicyV2,
    StreamStatusV2,
    decode_frame_v2,
    encode_frame_v2,
    encode_stream_batch_v2,
    encode_stream_open_v2,
    encode_unwrap_shadow_prepare_v2,
    parse_hello_v2,
    parse_dispatch_diagnostics_v2,
    parse_feedback_snapshot_v2,
    parse_tracking_diagnostics_v2,
    parse_executor_diagnostics_v2,
    parse_shadow_snapshot_v2,
    parse_state_v2,
    parse_stream_status_v2,
)


class StreamTransportV2Error(RuntimeError):
    pass


class StreamResponseTimeoutError(StreamTransportV2Error):
    """No matching response arrived before the bounded receive deadline."""


def _serialized(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        started = _perf_counter()
        with self._transaction_lock:
            acquired = _perf_counter()
            previous = self._active_timing
            record = dict(operation=method.__name__, started_s=started,
                          lock_wait_ms=(acquired-started)*1000, phases={})
            if method.__name__ == 'exchange_mobile' and args and isinstance(args[0], bytes):
                record['inner_request_hex'] = args[0][:4].hex()
            self._active_timing = record
            cpu = _thread_time()
            try:
                return method(self, *args, **kwargs)
            except Exception as error:
                record['error'] = type(error).__name__+': '+str(error)
                raise
            finally:
                record['duration_ms'] = (_perf_counter()-started)*1000
                record['thread_cpu_ms'] = (_thread_time()-cpu)*1000
                self._transaction_timings.append(record)
                self._active_timing = previous

    return call


class StreamValidationTransportV2:
    """Exchange v2 packets; motion admission belongs to the resident adapter."""

    def __init__(self, port: Any, response_timeout_s: float = 0.4, *,
                 physical_joint_count: int = 12) -> None:
        if (
            isinstance(response_timeout_s, bool)
            or not isinstance(response_timeout_s, (int, float))
            or not math.isfinite(response_timeout_s)
            or not 0 < response_timeout_s <= 3
        ):
            raise ValueError("bounded response timeout required")
        if type(physical_joint_count) is not int or physical_joint_count not in (6, 12):
            raise ValueError("unsupported physical joint count")
        self._physical_joint_count = physical_joint_count
        self._transaction_lock = RLock()
        self._port = port
        self._timeout_s = response_timeout_s
        self._sequence = 1
        self._rx_residual = bytearray()
        self._write_timeout_count = 0
        self._write_timeout_events = deque(maxlen=20)
        self._uncertain_write = None
        self._active_timing = None
        self._transaction_timings = deque(maxlen=256)

    def timing_diagnostics(self, *, after_sequence=None) -> dict:
        """Copy bounded host timing; no port reads or writes and no lease renewal.

        A long wall time with low thread CPU can be I/O wait OR scheduling/GIL
        delay. These measurements alone do not identify a defective USB link.
        """
        from copy import deepcopy
        with self._transaction_lock:
            return dict(clock='host_perf_counter', transactions=deepcopy([
                row for row in self._transaction_timings
                if after_sequence is None or row.get('sequence',0)>after_sequence]))

    def _timed_call(self, phase, callback, *args):
        if self._active_timing is None:
            return callback(*args)
        wall, cpu = _perf_counter(), _thread_time()
        try:
            return callback(*args)
        finally:
            elapsed, used = _perf_counter()-wall, _thread_time()-cpu
            values = self._active_timing['phases'].setdefault(phase,
                dict(calls=0, wall_ms=0., thread_cpu_ms=0., maximum_call_ms=0.))
            values['calls'] += 1
            values['wall_ms'] += elapsed*1000
            values['thread_cpu_ms'] += used*1000
            values['maximum_call_ms'] = max(values['maximum_call_ms'],elapsed*1000)

    @staticmethod
    def _now_ms() -> int:
        return int(time.monotonic() * 1000.0) & 0xFFFFFFFF

    def _next_sequence(self) -> int:
        if self._sequence > 0xFFFFFFFF:
            raise StreamTransportV2Error("new transport required before sequence wrap")
        sequence = self._sequence
        self._sequence += 1
        return sequence

    def _send(
        self,
        message_type: StreamMessageTypeV2,
        payload: bytes = b"",
    ) -> tuple[int, int]:
        sequence = self._next_sequence()
        if self._active_timing is not None:
            self._active_timing.update(sequence=sequence,message_type=message_type.name)
        sender_time_ms = self._now_ms()
        packet = encode_frame_v2(
            FrameV2(
                message_type=message_type,
                sequence=sequence,
                sender_time_ms=sender_time_ms,
                payload=payload,
            )
        )
        started = time.monotonic()
        try:
            self._write_packet(packet)
        except SerialTimeoutException as exc:
            # POSIX pyserial can raise *after* os.write queued the whole frame.
            # Do not resend any bytes or assume delivery. The caller must still
            # receive the CRC-valid, sequence/type-matched response within its
            # existing receive deadline and validate the command's result.
            self._write_timeout_count += 1
            self._uncertain_write = dict(sequence=sequence,message_type=message_type.name,
                packet_bytes=len(packet),elapsed_s=time.monotonic()-started,
                error=str(exc),resolution='awaiting_response')
            self._write_timeout_events.append(self._uncertain_write)
        return sequence, sender_time_ms

    def write_diagnostics(self) -> dict:
        return dict(timeout_count=self._write_timeout_count,
                    events=[dict(event) for event in self._write_timeout_events])

    def _resolve_uncertain_write(self,sequence,resolution):
        if self._uncertain_write is not None and self._uncertain_write['sequence']==sequence:
            self._uncertain_write['resolution']=resolution
            self._uncertain_write=None

    def _write_packet(self, packet: bytes) -> None:
        # Serial.write uses the configured finite write timeout. Do not flush:
        # POSIX tcdrain can wait indefinitely on a stalled USB/flow-control path.
        if self._timed_call('write',self._port.write,packet) != len(packet):
            raise StreamTransportV2Error(
                "partial serial write; command delivery is uncertain"
            )

    def _read_packet(self) -> bytes | None:
        if b"\x00" in self._rx_residual:
            index = self._rx_residual.index(0)
            packet = bytes(self._rx_residual[: index + 1])
            del self._rx_residual[: index + 1]
            return packet
        # pyserial.read_until loops over single-byte reads in Python. Under
        # concurrent image/JSON processing that can repeatedly yield the GIL
        # while an entire reply is already buffered. Drain available bytes in
        # one bounded read; retain partial and subsequent frames below.
        if hasattr(self._port, 'in_waiting') and callable(getattr(self._port, 'read', None)):
            chunk = self._port.read(max(1, min(self._port.in_waiting, 4096)))
        else:
            chunk = self._port.read_until(b"\x00")
        if not chunk:
            return None
        self._rx_residual.extend(chunk)
        if b"\x00" not in self._rx_residual:
            if len(self._rx_residual) > 4096:
                del self._rx_residual[:-4096]
            return None
        return self._read_packet()

    def _receive(
        self,
        sequence: int,
        message_type: StreamMessageTypeV2,
        *,
        timeout_s: float | None = None,
    ) -> FrameV2:
        deadline = time.monotonic() + (
            self._timeout_s if timeout_s is None else timeout_s
        )
        observed: list[str] = []
        while time.monotonic() < deadline:
            try:
                packet = self._timed_call('read',self._read_packet)
            except Exception:
                self._resolve_uncertain_write(sequence,'receive_error')
                raise
            if packet is None:
                continue
            try:
                frame = self._timed_call('decode',decode_frame_v2,packet)
            except Exception:
                continue
            if frame.sequence == sequence and frame.message_type is message_type:
                self._resolve_uncertain_write(sequence,'matching_response')
                return frame
            if len(observed) < 8:
                observed.append(f"{frame.message_type.name}#{frame.sequence}")
        self._resolve_uncertain_write(sequence,'response_timeout')
        raise StreamResponseTimeoutError(
            f"timeout waiting for {message_type.name} sequence={sequence} "
            f"observed={observed or 'none'}"
        )

    @_serialized
    def enter_binary_mode(self) -> HelloV2:
        self._port.reset_input_buffer()
        self._write_packet(b"P")
        deadline = time.monotonic() + self._timeout_s
        acknowledged = False
        while time.monotonic() < deadline:
            line = self._port.readline().decode("ascii", errors="replace").strip()
            if line == "BINARY_PROTOCOL_READY_RESET_TO_EXIT":
                acknowledged = True
                break
            if not line:
                break
        if not acknowledged:
            self._write_packet(b"\x00")
            self._port.reset_input_buffer()
        sequence, _ = self._send(StreamMessageTypeV2.HELLO_REQUEST)
        hello = parse_hello_v2(
            self._receive(sequence, StreamMessageTypeV2.HELLO_RESPONSE).payload,
            expected_joint_count=self._physical_joint_count,
        )
        return hello

    @_serialized
    def arm(self, calibration_hash: int) -> None:
        sequence, _ = self._send(
            StreamMessageTypeV2.ARM_REQUEST,
            struct.pack("<I", calibration_hash),
        )
        payload = self._receive(
            sequence, StreamMessageTypeV2.ARM_RESPONSE, timeout_s=3.0
        ).payload
        if len(payload) != 8:
            raise StreamTransportV2Error("invalid ARM_RESPONSE length")
        result, state, returned_hash = struct.unpack("<BB2xI", payload)
        if result != 0 or state != 2 or returned_hash != calibration_hash:
            raise StreamTransportV2Error(
                "ARM_REQUEST rejected "
                f"result={result} state={state} "
                f"hash=0x{returned_hash:08X}"
            )

    @_serialized
    def enable(self) -> StateV2:
        sequence, _ = self._send(StreamMessageTypeV2.ENABLE)
        state = parse_state_v2(
            self._receive(sequence, StreamMessageTypeV2.STATE_FEEDBACK).payload,
            expected_joint_count=self._physical_joint_count,
        )
        if state.status_code != 0 or state.stop_latched:
            raise StreamTransportV2Error(
                "ENABLE rejected "
                f"status={state.status_code} latched={int(state.stop_latched)}"
            )
        return state

    @_serialized
    def safe_stop(self) -> StateV2:
        sequence, _ = self._send(StreamMessageTypeV2.SAFE_STOP)
        state = parse_state_v2(
            self._receive(
                sequence, StreamMessageTypeV2.STATE_FEEDBACK, timeout_s=3.0
            ).payload,
            expected_joint_count=self._physical_joint_count,
        )
        if not state.stop_latched:
            raise StreamTransportV2Error("SAFE_STOP was not latched")
        return state

    @_serialized
    def heartbeat(self) -> StateV2:
        sequence, _ = self._send(StreamMessageTypeV2.HEARTBEAT)
        return parse_state_v2(
            self._receive(sequence, StreamMessageTypeV2.STATE_FEEDBACK).payload,
            expected_joint_count=self._physical_joint_count,
        )

    @_serialized
    def read_device_inventory(self, bus: int, motor_id: int) -> dict:
        from .device_inventory import inventory_request, decode_inventory
        packet = inventory_request(bus, motor_id)
        sequence, _ = self._send(StreamMessageTypeV2.DEVICE_INVENTORY_REQUEST, packet)
        response = self._receive(sequence, StreamMessageTypeV2.DEVICE_INVENTORY_RESPONSE)
        return decode_inventory(response.payload, bus, motor_id)

    @_serialized
    def read_servo_tuning(self, bus: int, motor_id: int) -> dict:
        """Opt-in fixed-register read, subject to the existing maintenance gate."""
        from .servo_tuning import tuning_request, decode_tuning
        packet = tuning_request(bus, motor_id)
        sequence, _ = self._send(StreamMessageTypeV2.DEVICE_INVENTORY_REQUEST, packet)
        response = self._receive(sequence, StreamMessageTypeV2.DEVICE_INVENTORY_RESPONSE)
        return decode_tuning(response.payload, bus, motor_id)

    @_serialized
    def stationary_platform(self, operation=0):
        """S3-only cached telemetry / idle reader control, never a motor write."""
        from .stationary_telemetry import request, decode
        sequence, _ = self._send(StreamMessageTypeV2.MOBILE_REQUEST, request(operation))
        response = self._receive(sequence, StreamMessageTypeV2.MOBILE_RESPONSE)
        return decode(response.payload)

    @_serialized
    def stationary_feedback_failures(self):
        """Cached first motor-read failure on each bus; no hardware access."""
        from .stationary_telemetry import failure_request, decode_failures
        sequence, _ = self._send(StreamMessageTypeV2.MOBILE_REQUEST, failure_request())
        response = self._receive(sequence, StreamMessageTypeV2.MOBILE_RESPONSE)
        return decode_failures(response.payload)

    @_serialized
    def stationary_runtime(self):
        """Cached readiness and first-stop record; no motor reads or writes."""
        from .stationary_telemetry import runtime_request, decode_runtime
        sequence, _ = self._send(StreamMessageTypeV2.MOBILE_REQUEST, runtime_request())
        response = self._receive(sequence, StreamMessageTypeV2.MOBILE_RESPONSE)
        return decode_runtime(response.payload)

    @_serialized
    def get_state(self) -> StateV2:
        sequence, _ = self._send(StreamMessageTypeV2.GET_STATE)
        return parse_state_v2(
            self._receive(sequence, StreamMessageTypeV2.STATE_FEEDBACK).payload,
            expected_joint_count=self._physical_joint_count,
        )

    def _checked_exchange(self, request_type, response_type, parser, label,
                          payload=b"", **parse_options):
        """Check both request echoes after matching the outer frame.

        Callers hold the transaction lock. A mismatch is an error, never a
        reason to resend a command whose delivery may already have occurred.
        """
        sequence, sender_time_ms = self._send(request_type, payload)
        response = parser(self._receive(sequence, response_type).payload,
                          **parse_options)
        if response.request_sequence != sequence:
            raise StreamTransportV2Error(f"{label} sequence echo mismatch")
        if response.sender_time_ms_echo != sender_time_ms:
            raise StreamTransportV2Error(f"{label} sender-time echo mismatch")
        return response

    def _stream_exchange(
        self,
        message_type: StreamMessageTypeV2,
        payload: bytes,
    ) -> StreamStatusV2:
        return self._checked_exchange(
            message_type, StreamMessageTypeV2.STREAM_STATUS,
            parse_stream_status_v2, "STREAM_STATUS", payload,
        )

    @_serialized
    def open_stream(self, policy: StreamPolicyV2) -> StreamStatusV2:
        return self._stream_exchange(
            StreamMessageTypeV2.STREAM_OPEN,
            encode_stream_open_v2(policy),
        )

    @_serialized
    def append(self, batch: StreamBatchV2) -> StreamStatusV2:
        return self._stream_exchange(
            StreamMessageTypeV2.SETPOINT_BATCH,
            encode_stream_batch_v2(batch, BatchKindV2.APPEND),
        )

    @_serialized
    def splice(self, batch: StreamBatchV2) -> StreamStatusV2:
        return self._stream_exchange(
            StreamMessageTypeV2.SPLICE,
            encode_stream_batch_v2(batch, BatchKindV2.SPLICE),
        )

    @_serialized
    def get_executor_diagnostics(self) -> StreamExecutorDiagnosticsV2:
        return self._checked_exchange(
            StreamMessageTypeV2.GET_EXECUTOR_DIAGNOSTICS,
            StreamMessageTypeV2.EXECUTOR_DIAGNOSTICS,
            parse_executor_diagnostics_v2, "executor diagnostics",
        )

    @_serialized
    def get_dispatch_diagnostics(self) -> BimanualDispatchDiagnosticsV2:
        return self._checked_exchange(
            StreamMessageTypeV2.GET_DISPATCH_DIAGNOSTICS,
            StreamMessageTypeV2.DISPATCH_DIAGNOSTICS,
            parse_dispatch_diagnostics_v2, "dispatch diagnostics",
        )

    @_serialized
    def get_tracking_diagnostics(self) -> BimanualTrackingDiagnosticsV2:
        return self._checked_exchange(
            StreamMessageTypeV2.GET_TRACKING_DIAGNOSTICS,
            StreamMessageTypeV2.TRACKING_DIAGNOSTICS,
            parse_tracking_diagnostics_v2, "tracking diagnostics",
        )

    @_serialized
    def get_feedback_snapshot(self) -> BimanualFeedbackSnapshotV2:
        return self._checked_exchange(
            StreamMessageTypeV2.GET_FEEDBACK_SNAPSHOT,
            StreamMessageTypeV2.FEEDBACK_SNAPSHOT,
            parse_feedback_snapshot_v2, "feedback snapshot",
            expected_joint_count=self._physical_joint_count,
        )

    @_serialized
    def prepare_shadow(
        self,
        reference_unwrapped_raw: tuple[int, ...] | None = None,
        maximum_reference_delta_raw: int = 0,
    ) -> StreamShadowSnapshotV2:
        if reference_unwrapped_raw is None:
            if maximum_reference_delta_raw != 0:
                raise StreamTransportV2Error(
                    "reference window requires branch references"
                )
            payload = b""
        else:
            payload = encode_unwrap_shadow_prepare_v2(
                reference_unwrapped_raw,
                maximum_reference_delta_raw,
            )
        sequence, _ = self._send(
            StreamMessageTypeV2.PREPARE_SHADOW,
            payload,
        )
        return parse_shadow_snapshot_v2(
            self._receive(
                sequence,
                StreamMessageTypeV2.SHADOW_SNAPSHOT,
                timeout_s=6.0,
            ).payload,
            expected_joint_count=self._physical_joint_count,
        )

    @_serialized
    def exchange_mobile(self, packet: bytes) -> bytes:
        """Same port/sequence/lock as arm traffic; does not open another device.

        Hardware defaults to an unbound endpoint and rejects this operation.
        Caller must still own the whole-robot task; locking is only wire ownership.
        """
        from .mobile_wire import MobileCommand, crc32c

        if (
            not isinstance(packet, bytes)
            or len(packet) != 36
            or packet[:3] not in (b"AM\x01", b"AL\x01", b"AQ\x01", b"AE\x01", b"AC\x01")
        ):
            raise StreamTransportV2Error("invalid mobile payload")
        if struct.unpack_from("<I", packet, 32)[0] != crc32c(packet[:32]):
            raise StreamTransportV2Error("invalid mobile payload CRC")
        if packet[:3] == b"AC\x01":
            from .unified_commissioning import validate_request
            validate_request(packet)
        elif packet[:3] == b"AE\x01":
            from .robot_evidence import validate_request
            validate_request(packet)
        elif packet[:3] == b"AQ\x01":
            if packet[3] != 1 or not struct.unpack_from("<I", packet, 4)[0] or any(packet[8:32]):
                raise StreamTransportV2Error("invalid stop status query")
        elif packet[:3] == b"AL\x01":
            from .lift_wire import LiftCommand
            LiftCommand.decode(packet)
        elif packet[3] in (1, 2, 3):
            MobileCommand.decode(packet)
        elif (
            packet[3] not in (4, 5, 6)
            or not struct.unpack_from("<I", packet, 4)[0]
            or any(packet[16:32])
        ):
            raise StreamTransportV2Error("invalid mobile query")
        seq, _ = self._send(StreamMessageTypeV2.MOBILE_REQUEST, packet)
        payload = self._receive(seq, StreamMessageTypeV2.MOBILE_RESPONSE).payload
        if payload == b"\x01":
            raise StreamTransportV2Error(
                "mobile endpoint is not attached on this firmware"
            )
        if payload == b"\x04":
            raise StreamTransportV2Error("mobile output is unavailable or inhibited")
        if payload == b"\x03":
            raise StreamTransportV2Error("host fault inhibits mobile motion")
        if len(payload) != 65 or payload[0] != 0:
            raise StreamTransportV2Error("mobile envelope rejected or malformed")
        return payload[1:]


class MobileV2Exchange:
    """Inject into MobileClient; retain the resident adapter's existing transport."""

    def __init__(self, transport: StreamValidationTransportV2):
        self.transport = transport

    def exchange(self, packet: bytes) -> bytes:
        return self.transport.exchange_mobile(packet)
