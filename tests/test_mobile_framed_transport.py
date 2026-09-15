import ctypes
from pathlib import Path
import struct
import subprocess
from threading import Event, Thread

import pytest

from so101_arm_bridge.host_simulator import NativeHostSerial
from so101_arm_bridge.mobile_client import MobileClient
from so101_arm_bridge.mobile_wire import MobileCommand, mobile_query
from so101_arm_bridge.stream_protocol_v2 import (
    FrameV2,
    StreamMessageTypeV2 as Msg,
    decode_frame_v2,
    encode_frame_v2,
)
from so101_arm_bridge.stream_transport_v2 import (
    MobileV2Exchange,
    StreamValidationTransportV2,
    StreamTransportV2Error,
)

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    core = ROOT / "firmware/stm32_actuator"
    out = tmp_path_factory.mktemp("framed-host") / "host.so"
    names = [
        "mobile_endpoint",
        "mobile_framed",
        "mobile_wire",
        "mobile_supervisor",
        "crc32c",
        "cobs",
        "protocol",
        "bus_router",
        "shared_bus",
        "motor_groups",
        "sts3215_packet",
    ]
    subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-shared",
            "-fPIC",
            "-DACTUATOR_PROTOCOL_VERSION=2",
            "-I",
            str(core / "include"),
            str(core / "tests/pc_mobile_simulator.c"),
            *(str(core / "src" / (n + ".c")) for n in names),
            "-o",
            str(out),
        ],
        check=True,
    )
    return out


def make_clients(library, chunk=7):
    clock = [1000]
    serial = NativeHostSerial(library, lambda: clock[0], boot_id=42, chunk_size=chunk)
    transport = StreamValidationTransportV2(serial, response_timeout_s=0.05)
    return (
        MobileClient(MobileV2Exchange(transport), lambda: clock[0]),
        transport,
        serial,
        clock,
    )


@pytest.mark.parametrize("chunk", [1, 7, 1024])
def test_same_host_transport_arm_queries_and_mobile_commands_round_trip_through_c(
    library, chunk
):
    client, arm, serial, clock = make_clients(library, chunk)
    assert arm.get_state().joint_count == 12
    client.synchronize()
    client.arm(55)
    clock[0] += 10
    client.velocity((100, -50, 25, 0))
    assert arm.heartbeat().protocol_version == 2
    clock[0] += 10
    assert client.status().velocity_raw == (100, -50, 25, 0)
    clock[0] += 10
    assert not client.stop().flags & 8
    clock[0] += 10
    assert client.status().flags & 8  # later independent synthetic feedback
    frames = [decode_frame_v2(p) for p in serial.writes]
    assert len({f.sequence for f in frames}) == len(frames)
    assert {f.message_type for f in frames} == {
        Msg.GET_STATE,
        Msg.HEARTBEAT,
        Msg.MOBILE_REQUEST,
    }
    assert all(p[-1] == 0 for p in serial.writes)  # no raw AM packets on the port


def test_unbound_firmware_returns_explicit_rejection_without_arming(library):
    client, arm, serial, _ = make_clients(library)
    serial.library.pc_host_set_attached(0)
    with pytest.raises(StreamTransportV2Error, match="not attached"):
        client.synchronize()
    assert client.session is None
    assert arm.get_state().joint_count == 12


def test_outer_crc_noise_and_inner_malformed_do_not_break_next_arm_query(library):
    _, arm, serial, _ = make_clients(library)
    good = encode_frame_v2(
        FrameV2(
            message_type=Msg.MOBILE_REQUEST,
            sequence=91,
            sender_time_ms=1000,
            payload=mobile_query(5, 1, 1),
        )
    )
    bad = bytearray(good)
    bad[-2] ^= 1
    serial.write(bytes(bad))
    serial.write(b"garbage\x00")
    bad_inner = b"AM\x01" + bytes(33)
    serial.write(
        encode_frame_v2(
            FrameV2(
                message_type=Msg.MOBILE_REQUEST,
                sequence=92,
                sender_time_ms=1000,
                payload=bad_inner,
            )
        )
    )
    assembled = bytearray()
    while serial.pending:
        assembled.extend(serial.read_until(b"\x00"))
    response = decode_frame_v2(bytes(assembled))
    assert response.sequence == 92 and response.payload == b"\x02"
    assert arm.get_state().joint_count == 12


def test_expiry_uses_processing_time_and_reboot_clears_client(library):
    client, _, serial, clock = make_clients(library)
    client.synchronize()
    client.arm(55)
    packet = MobileCommand(2, 55, 88, clock[0] + 50, (100, 0, 0, 0)).encode()
    clock[0] += 51
    result = client.port.exchange(packet)
    assert result[3] == (2 | 0x80)
    serial.library.pc_mobile_reset(43)
    clock[0] += 1
    with pytest.raises(RuntimeError, match="restarted"):
        client.status()
    assert client.session is None and client.clock.sample is None


def test_partial_write_is_error_and_does_not_flush_or_retry(library):
    client, _, serial, _ = make_clients(library)
    calls = []
    serial.write = lambda packet: calls.append(packet) or len(packet) - 1
    serial.flush = lambda: pytest.fail("unbounded flush is forbidden")
    with pytest.raises(StreamTransportV2Error, match="partial"):
        client.synchronize()
    assert len(calls) == 1 and client.session is None


def test_sequence_never_wraps_into_old_replies(library):
    _, arm, _, _ = make_clients(library)
    arm._sequence = 0xFFFFFFFF
    assert arm.get_state().joint_count == 12
    with pytest.raises(StreamTransportV2Error, match="wrap"):
        arm.get_state()


def test_arm_and_mobile_share_transaction_ownership(library):
    _, arm, serial, _ = make_clients(library)
    first_read = Event()
    release = Event()
    second_started = Event()
    results = []
    read = serial.read_until

    def paused_read(delimiter):
        if not first_read.is_set():
            first_read.set()
            assert release.wait(1)
        return read(delimiter)

    serial.read_until = paused_read
    # Give test coordination its own larger receive budget; no hardware involved.
    arm._timeout_s = 1

    def first():
        results.append(arm.get_state().joint_count)

    def second():
        second_started.set()
        results.append(len(arm.exchange_mobile(mobile_query(5, 1, 1))))

    a, b = Thread(target=first), Thread(target=second)
    a.start()
    assert first_read.wait(1)
    b.start()
    assert second_started.wait(1)
    assert len(serial.writes) == 1
    release.set()
    a.join(2)
    b.join(2)
    assert not a.is_alive() and not b.is_alive()
    assert sorted(results) == [12, 64]


@pytest.mark.parametrize(
    "status,message",
    [(1, "not attached"), (3, "host fault"), (4, "output is unavailable")],
)
def test_board_admission_failure_is_not_a_mobile_acknowledgement(status, message):
    class RejectedOutput:
        reply = b""

        def write(self, packet):
            request = decode_frame_v2(packet)
            self.reply = encode_frame_v2(
                FrameV2(
                    message_type=Msg.MOBILE_RESPONSE,
                    sequence=request.sequence,
                    payload=bytes([status]),
                )
            )
            return len(packet)

        def read_until(self, delimiter):
            value, self.reply = self.reply, b""
            return value

    transport = StreamValidationTransportV2(RejectedOutput())
    with pytest.raises(StreamTransportV2Error, match=message):
        transport.exchange_mobile(MobileCommand(1, 1, 0, 100).encode())
