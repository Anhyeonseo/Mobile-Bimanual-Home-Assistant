import ctypes
from dataclasses import replace
from pathlib import Path
import subprocess
import pytest
from so101_arm_bridge.lift_wire import LiftCommand, LiftStatus
from so101_arm_bridge.mobile_wire import crc32c

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture()
def lift(tmp_path):
    core = ROOT / "firmware/stm32_actuator"
    path = tmp_path / "lift.so"
    subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-shared",
            "-fPIC",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(core / "include"),
            str(ROOT / "tests/fixtures/lift_endpoint_harness.c"),
            *(
                str(core / "src" / f)
                for f in ("lift_endpoint.c", "lift_controller.c", "crc32c.c")
            ),
            "-o",
            str(path),
        ],
        check=True,
    )
    lib = ctypes.CDLL(str(path))
    lib.lift_test_exchange.argtypes = [
        ctypes.c_char_p,
        ctypes.c_uint,
        ctypes.c_uint,
        ctypes.c_void_p,
    ]
    lib.lift_test_init()
    lib.lift_test_sample(0, 1000, 0, 0, 0, 1)

    def request(command, now):
        data = command.encode() if isinstance(command, LiftCommand) else command
        out = ctypes.create_string_buffer(64)
        if not lib.lift_test_exchange(data, len(data), now, out):
            raise ValueError("C rejected packet")
        return LiftStatus.decode(out.raw)

    return lib, request


@pytest.mark.parametrize("kind", range(1, 7))
def test_codec_round_trip(kind):
    q = LiftCommand(kind, 1, 2, 0 if kind == 4 else 100, 42, 1234 if kind == 2 else 0)
    assert LiftCommand.decode(q.encode()) == q


@pytest.mark.parametrize(
    "changes",
    [
        {"session": 0},
        {"boot_id": 0},
        {"sequence": True},
        {"target_um": 1},
        {"valid_until_ms": -1},
        {"kind": 7},
    ],
)
def test_invalid_command(changes):
    with pytest.raises(ValueError):
        replace(LiftCommand(1, 1, 1, 100, 42), **changes).encode()


def test_actual_c_home_hold_move_and_timeout(lift):
    lib, request = lift
    reply = request(LiftCommand(1, 10, 1, 100, 42), 0)
    assert reply.state == 1 and reply.command_raw == -20 and reply.session == 10
    lib.lift_test_sample(10, 999, 0, 600, 0, 1)
    lib.lift_test_sample(30, 999, 0, 600, 0, 1)
    lib.lift_test_sample(31, 999, 0, 0, 1, 1)
    assert not request(LiftCommand(4, 10, 2, 0, 42), 31).holding()
    lib.lift_test_zero(32)
    lib.lift_test_sample(33, 999, 0, 0, 1, 1)
    reply = request(LiftCommand(4, 10, 3, 0, 42), 33)
    assert reply.holding() and reply.height_um == 0 and not reply.flags & 2
    reply = request(LiftCommand(2, 10, 4, 100, 42, 10000), 34)
    assert reply.state == 3 and reply.target_um == 10000 and reply.command_raw == 100
    reply = request(LiftCommand(4, 10, 5, 0, 42), 100)
    assert (
        reply.state == 6
        and reply.fault == 1
        and not reply.holding()
        and reply.command_raw == 0
    )


def test_crc_truncation_boot_replay_and_reserved_bits(lift):
    _, request = lift
    q = LiftCommand(1, 1, 1, 100, 42)
    for i in range(36):
        b = bytearray(q.encode())
        b[i] ^= 1
        with pytest.raises(ValueError):
            request(bytes(b), 0)
        with pytest.raises(ValueError):
            LiftCommand.decode(bytes(b))
    for n in range(36):
        with pytest.raises(ValueError):
            request(q.encode()[:n], 0)
    assert request(replace(q, boot_id=43), 0).kind == 129
    b = bytearray(q.encode())
    b[24] = 1
    b[32:] = crc32c(b[:32]).to_bytes(4, "little")
    assert request(bytes(b), 0).kind == 129
    assert request(q, 0).kind == 1
    assert request(q, 0).kind == 129
    assert request(LiftCommand(5, 1, 1, 100, 42), 1).kind == 133
    assert request(LiftCommand(3, 1, 2, 100, 42), 1).kind == 3
