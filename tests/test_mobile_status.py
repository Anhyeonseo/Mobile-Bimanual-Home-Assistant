import ctypes
from pathlib import Path
import subprocess
import pytest
from so101_arm_bridge.mobile_wire import MobileStatus, MobileClock, mobile_query, crc32c

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def endpoint(tmp_path_factory):
    temp = tmp_path_factory.mktemp("endpoint")
    source = temp / "fixture.c"
    source.write_text(
        """#include "actuator_core/mobile_endpoint.h"
int query(const unsigned char *input,int length,unsigned char *out){
 actuator_mobile_config_t c={{100,100,100,50},2,100,200,0,100000};
 actuator_mobile_supervisor_t s;actuator_mobile_endpoint_t e;
 actuator_mobile_feedback_t f={1200,{1,-2,3,0},12345,true,true,true};int ready=0;
 actuator_mobile_init(&s,&c);actuator_mobile_feedback(&s,&f,1200);actuator_mobile_endpoint_init(&e,&s,42);
 for(int i=0;i<length;i++)ready+=actuator_mobile_endpoint_feed(&e,input[i],1234,out);return ready;
}"""
    )
    core = ROOT / "firmware/stm32_actuator"
    library = temp / "endpoint.so"
    subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-shared",
            "-fPIC",
            "-I",
            str(core / "include"),
            str(source),
            *(
                str(core / "src" / f)
                for f in (
                    "mobile_endpoint.c",
                    "mobile_wire.c",
                    "mobile_supervisor.c",
                    "crc32c.c",
                )
            ),
            "-o",
            str(library),
        ],
        check=True,
    )
    lib = ctypes.CDLL(str(library))
    lib.query.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_uint8)]
    lib.query.restype = ctypes.c_int
    return lib


def query(endpoint, kind=6):
    data = mobile_query(kind, 5, 7)
    output = (ctypes.c_uint8 * 64)()
    assert endpoint.query(data, len(data), output) == 1
    return MobileStatus.decode(bytes(output))


@pytest.mark.parametrize("kind", [4, 5, 6])
def test_status_generated_by_c_decodes_on_host(endpoint, kind):
    result = query(endpoint, kind)
    assert result.kind == kind and result.boot_id == 42 and result.capabilities == 7
    assert result.velocity_raw == (1, -2, 3, 0) and result.lift_position_um == 12345
    assert result.feedback_tick_ms == 1200 and result.mcu_tick_ms == 1234


def test_clock_uncertainty_staleness_and_reboot(endpoint):
    from dataclasses import replace

    status = query(endpoint)
    clock = MobileClock()
    assert not clock.update(status, 7, 10000, 10010)
    deadline, boot = clock.deadline(10015, 50)
    assert (deadline, boot) == (1279, 42)
    with pytest.raises(ValueError):
        clock.deadline(12000, 50)
    with pytest.raises(ValueError):
        clock.update(status, 8, 10020, 10030)
    with pytest.raises(ValueError):
        clock.update(status, 7, 10020, 10050)
    assert clock.update(replace(status, boot_id=43), 7, 10020, 10030)


def test_query_crc_and_stream_resync(endpoint):
    data = mobile_query(5, 5, 7)
    bad = bytearray(data)
    bad[20] ^= 1
    output = (ctypes.c_uint8 * 64)()
    blob = b"junk" + bytes(bad) + data
    assert endpoint.query(blob, len(blob), output) == 1
    status = MobileStatus.decode(bytes(output))
    assert status.rejected_frames > 0
    damaged = bytes(output[:-1]) + bytes([output[-1] ^ 1])
    with pytest.raises(ValueError):
        MobileStatus.decode(damaged)


@pytest.mark.parametrize(
    "change",
    [
        {"state": 0},
        {"state": 3},
        {"reason": 2},
        {"session": 8},
        {"flags": 7},
        {"flags": 21},
        {"flags": 19},
        {"feedback_tick_ms": 900},
        {"feedback_tick_ms": 2000},
    ],
)
def test_fresh_response_does_not_hide_bad_mobile_motion_evidence(endpoint, change):
    from dataclasses import replace

    status = replace(query(endpoint), state=2, session=7)
    assert status.ready_for_motion(7)
    assert not replace(status, **change).ready_for_motion(7)


def test_mobile_feedback_age_rollover_and_owned_session(endpoint):
    from dataclasses import replace

    status = replace(
        query(endpoint), state=2, session=7, mcu_tick_ms=20, feedback_tick_ms=0xFFFFFFF0
    )
    assert status.ready_for_motion(7)
    assert not status.ready_for_motion(True)
    assert not status.ready_for_motion(None)
    assert not status.feedback_is_fresh(30)
    with pytest.raises(ValueError):
        status.feedback_is_fresh(True)
