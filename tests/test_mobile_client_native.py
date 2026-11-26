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
            "-DPC_MOBILE_MAX_WHEEL_RAW=100",
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


def test_installed_100_limit_reproduces_rejection_and_bounded_nav2_is_accepted(library):
    from so101_arm_bridge.mobile_client import MobileReplyError
    from home_robot_tasks.base_motion import TeleopCommandModel
    import json
    model=TeleopCommandModel(json.loads((ROOT/'config/home_assistant/s4_base.command-candidate.json').read_text()))
    c,p,t=client(library);c.synchronize();c.arm(1)
    for _ in range(6):
        t[0]+=30;c.velocity((0,0,0,0),lifetime_ms=150)
    raw=tuple(round(v) for v in model.raw_command((.016,0,0)))+(0,)
    t[0]+=30
    with pytest.raises(MobileReplyError,match='controller rejected') as error:c.velocity(raw,lifetime_ms=150)
    assert error.value.diagnostics['response']['kind']==130
    assert error.value.diagnostics['response']['state']==2
    assert error.value.diagnostics['command']['velocity_raw']==raw
    assert c.session is None
    # New isolated simulated boot, never an automatic physical retry.
    c,p,t=client(library);c.synchronize();c.arm(1);model.limit=100
    for twist in ((.016,0,0),(.016,.013,.09),(-.016,-.013,-.09)):
        t[0]+=30;raw=tuple(round(v) for v in model.raw_command(twist))+(0,)
        assert c.velocity(raw,lifetime_ms=150).ready_for_motion(1)


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


def test_late_clock_refresh_reuses_only_original_live_bound(library):
    c,p,t=client(library);c.synchronize();previous=c.clock.sample;exchange=p.exchange
    def delayed(q):
        result=exchange(q);t[0]+=25;return result
    p.exchange=delayed;t[0]=1500
    assert c.refresh_clock() is previous[2]
    assert c.clock.sample is previous and c.clock_refresh_deferred==1
    assert c.clock.deadline(t[0],150)==(1675,42)
    t[0]=1970;c.refresh_clock();assert c.clock.sample is previous
    t[0]=1980
    with pytest.raises(ValueError,match='delayed'):c.refresh_clock()
    with pytest.raises(ValueError,match='stale'):c.clock.deadline(t[0],150)


def test_late_clock_refresh_never_accepts_reboot(library):
    c,p,t=client(library);c.synchronize();exchange=p.exchange
    def delayed(q):
        result=exchange(q);t[0]+=25;return result
    p.exchange=delayed;p.library.pc_mobile_reset(43);t[0]=1500
    with pytest.raises(RuntimeError,match='restarted'):c.refresh_clock()
    assert c.clock.sample is None and c.session is None


def test_initial_clock_still_requires_a_precise_exchange(library):
    c,p,t=client(library);exchange=p.exchange
    def delayed(q):
        result=exchange(q);t[0]+=25;return result
    p.exchange=delayed
    with pytest.raises(ValueError,match='delayed'):c.synchronize()
    assert c.clock.sample is None and c.session is None


@pytest.mark.parametrize('proven', [False, True])
def test_owned_park_needs_board_proof_and_retires_the_session(library, proven):
    c, p, t = client(library)
    p.library.pc_mobile_set_park_proof(int(proven))
    assert c.synchronize().capabilities & 8
    c.arm(123)
    c.velocity((0, 0, 0, 0))
    t[0] += 10
    if not proven:
        with pytest.raises(RuntimeError):
            c.park()
        assert c.status().state == 2
        return
    result = c.park()
    assert result.state == 0 and result.session == 0 and c.session is None
    with pytest.raises(RuntimeError):
        c.arm(123)
    t[0] += 10
    c.arm(124)
