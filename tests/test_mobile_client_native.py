"""Host client -> actual C core -> queued STS packet -> ideal test feedback."""

import ctypes
from pathlib import Path
import subprocess
import pytest
from so101_arm_bridge.mobile_client import MobileClient
from so101_arm_bridge.mobile_simulator import NativeMobileSimulator

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    core = ROOT / "firmware/stm32_actuator"
    output = tmp_path_factory.mktemp("mobile-native") / "mobile.so"
    sources = [
        "mobile_endpoint",
        "mobile_wire",
        "mobile_supervisor",
        "crc32c",
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
            "-I",
            str(core / "include"),
            str(core / "tests/pc_mobile_simulator.c"),
            *(str(core / "src" / f"{name}.c") for name in sources),
            "-o",
            str(output),
        ],
        check=True,
    )
    return output


def client(library, initial=1000):
    clock = [initial]
    port = NativeMobileSimulator(library, lambda: clock[0], boot_id=42)
    return MobileClient(port, lambda: clock[0]), port, clock


def test_round_trip_and_fresh_stop_confirmation(library):
    c, p, t = client(library)
    c.synchronize()
    c.arm(123)
    t[0] += 10
    c.velocity((100, -100, 50, 0))
    t[0] += 10
    assert c.status().velocity_raw == (100, -100, 50, 0)
    t[0] += 10
    stop = c.stop()
    assert (
        stop.state == 3 and not stop.flags & 8
    )  # sending zero is not stopped evidence
    t[0] += 10
    assert c.status().flags & 8
    assert p.library.pc_mobile_transmissions() > 0
    with pytest.raises(RuntimeError):
        c.velocity((1, 0, 0, 0))


@pytest.mark.parametrize("initial", [1000, 0xFFFFFFF0])
def test_soak_clock_refresh_wrap_and_rearm(library, initial):
    c, p, t = client(library, initial)
    c.synchronize()
    c.arm(123)
    for i in range(5000):
        t[0] += 10
        c.velocity(((i % 200) - 100, 50, -50, 0))
    assert c.session == 123 and p.library.pc_mobile_transmissions() >= 10000
    t[0] += 10
    c.stop()
    t[0] += 10
    c.status()
    t[0] += 10
    c.arm(124)
    assert c.session == 124


def test_deadline_expiry_latches_and_requires_new_session(library):
    c, p, t = client(library)
    c.synchronize()
    c.arm(123)
    c.velocity((100, 0, 0, 0), lifetime_ms=100)
    t[0] += 101
    assert c.status().state == 3
    with pytest.raises(RuntimeError):
        c.velocity((100, 0, 0, 0))
    assert c.session is None


def test_reboot_and_transport_loss_do_not_rearm(library):
    c, p, t = client(library)
    c.synchronize()
    c.arm(123)
    p.library.pc_mobile_reset(43)
    t[0] += 10
    with pytest.raises(RuntimeError, match="restarted"):
        c.status()
    assert c.session is None and c.clock.sample is None
    c.synchronize()
    c.arm(124)

    def broken(_):
        raise TimeoutError("injected transport loss")

    p.exchange = broken
    with pytest.raises(TimeoutError):
        c.velocity((1, 2, 3, 0))
    assert c.session is None
