"""Synchronous transport for STM32 protocol-v2 commands and feedback."""

from __future__ import annotations

import struct
import math
from functools import wraps
from threading import RLock
import time
from typing import Any

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
        with self._transaction_lock:
            return method(self, *args, **kwargs)

    return call


class StreamValidationTransportV2:
    """Exchange v2 packets; motion admission belongs to the resident adapter."""

    def __init__(self, port: Any, response_timeout_s: float = 0.4) -> None:
        if (
            isinstance(response_timeout_s, bool)
            or not isinstance(response_timeout_s, (int, float))
            or not math.isfinite(response_timeout_s)
            or not 0 < response_timeout_s <= 3
        ):
            raise ValueError("bounded response timeout required")
        self._transaction_lock = RLock()
        self._port = port
        self._timeout_s = response_timeout_s
        self._sequence = 1
        self._rx_residual = bytearray()

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
        sender_time_ms = self._now_ms()
        self._write_packet(
            encode_frame_v2(
                FrameV2(
                    message_type=message_type,
                    sequence=sequence,
                    sender_time_ms=sender_time_ms,
                    payload=payload,
                )
            )
        )
        return sequence, sender_time_ms

    def _write_packet(self, packet: bytes) -> None:
        # Serial.write uses the configured finite write timeout. Do not flush:
        # POSIX tcdrain can wait indefinitely on a stalled USB/flow-control path.
        if self._port.write(packet) != len(packet):
            raise StreamTransportV2Error(
                "partial serial write; command delivery is uncertain"
            )

    def _read_packet(self) -> bytes | None:
        if b"\x00" in self._rx_residual:
            index = self._rx_residual.index(0)
            packet = bytes(self._rx_residual[: index + 1])
            del self._rx_residual[: index + 1]
            return packet
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
            packet = self._read_packet()
            if packet is None:
                continue
            try:
                frame = decode_frame_v2(packet)
            except Exception:
                continue
            if frame.sequence == sequence and frame.message_type is message_type:
                return frame
            if len(observed) < 8:
                observed.append(f"{frame.message_type.name}#{frame.sequence}")
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
            self._receive(sequence, StreamMessageTypeV2.HELLO_RESPONSE).payload
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
            self._receive(sequence, StreamMessageTypeV2.STATE_FEEDBACK).payload
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
            ).payload
        )
        if not state.stop_latched:
            raise StreamTransportV2Error("SAFE_STOP was not latched")
        return state

    @_serialized
    def heartbeat(self) -> StateV2:
        sequence, _ = self._send(StreamMessageTypeV2.HEARTBEAT)
        return parse_state_v2(
            self._receive(sequence, StreamMessageTypeV2.STATE_FEEDBACK).payload
        )

    @_serialized
    def get_state(self) -> StateV2:
        sequence, _ = self._send(StreamMessageTypeV2.GET_STATE)
        return parse_state_v2(
            self._receive(sequence, StreamMessageTypeV2.STATE_FEEDBACK).payload
        )

    def _stream_exchange(
        self,
        message_type: StreamMessageTypeV2,
        payload: bytes,
    ) -> StreamStatusV2:
        sequence, sender_time_ms = self._send(message_type, payload)
        status = parse_stream_status_v2(
            self._receive(sequence, StreamMessageTypeV2.STREAM_STATUS).payload
        )
        if status.request_sequence != sequence:
            raise StreamTransportV2Error("STREAM_STATUS sequence echo mismatch")
        if status.sender_time_ms_echo != sender_time_ms:
            raise StreamTransportV2Error("STREAM_STATUS sender-time echo mismatch")
        return status

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
        sequence, sender_time_ms = self._send(
            StreamMessageTypeV2.GET_EXECUTOR_DIAGNOSTICS
        )
        diagnostics = parse_executor_diagnostics_v2(
            self._receive(
                sequence,
                StreamMessageTypeV2.EXECUTOR_DIAGNOSTICS,
            ).payload
        )
        if diagnostics.request_sequence != sequence:
            raise StreamTransportV2Error("executor diagnostics sequence echo mismatch")
        if diagnostics.sender_time_ms_echo != sender_time_ms:
            raise StreamTransportV2Error(
                "executor diagnostics sender-time echo mismatch"
            )
        return diagnostics

    @_serialized
    def get_dispatch_diagnostics(self) -> BimanualDispatchDiagnosticsV2:
        sequence, sender_time_ms = self._send(
            StreamMessageTypeV2.GET_DISPATCH_DIAGNOSTICS
        )
        diagnostics = parse_dispatch_diagnostics_v2(
            self._receive(
                sequence,
                StreamMessageTypeV2.DISPATCH_DIAGNOSTICS,
            ).payload
        )
        if diagnostics.request_sequence != sequence:
            raise StreamTransportV2Error("dispatch diagnostics sequence echo mismatch")
        if diagnostics.sender_time_ms_echo != sender_time_ms:
            raise StreamTransportV2Error(
                "dispatch diagnostics sender-time echo mismatch"
            )
        return diagnostics

    @_serialized
    def get_tracking_diagnostics(self) -> BimanualTrackingDiagnosticsV2:
        sequence, sender_time_ms = self._send(
            StreamMessageTypeV2.GET_TRACKING_DIAGNOSTICS
        )
        diagnostics = parse_tracking_diagnostics_v2(
            self._receive(
                sequence,
                StreamMessageTypeV2.TRACKING_DIAGNOSTICS,
            ).payload
        )
        if diagnostics.request_sequence != sequence:
            raise StreamTransportV2Error("tracking diagnostics sequence echo mismatch")
        if diagnostics.sender_time_ms_echo != sender_time_ms:
            raise StreamTransportV2Error(
                "tracking diagnostics sender-time echo mismatch"
            )
        return diagnostics

    @_serialized
    def get_feedback_snapshot(self) -> BimanualFeedbackSnapshotV2:
        sequence, sender_time_ms = self._send(StreamMessageTypeV2.GET_FEEDBACK_SNAPSHOT)
        snapshot = parse_feedback_snapshot_v2(
            self._receive(
                sequence,
                StreamMessageTypeV2.FEEDBACK_SNAPSHOT,
            ).payload
        )
        if snapshot.request_sequence != sequence:
            raise StreamTransportV2Error("feedback snapshot sequence echo mismatch")
        if snapshot.sender_time_ms_echo != sender_time_ms:
            raise StreamTransportV2Error("feedback snapshot sender-time echo mismatch")
        return snapshot

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
            ).payload
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
            or packet[:3] not in (b"AM\x01", b"AL\x01")
        ):
            raise StreamTransportV2Error("invalid mobile payload")
        if struct.unpack_from("<I", packet, 32)[0] != crc32c(packet[:32]):
            raise StreamTransportV2Error("invalid mobile payload CRC")
        if packet[:3] == b"AL\x01":
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
