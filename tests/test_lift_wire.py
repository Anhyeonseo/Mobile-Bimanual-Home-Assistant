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


@pytest.mark.parametrize("kind", range(1, 8))
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
        {"kind": 8},
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


def test_explicit_bottom_reference_does_not_home_or_claim_hold(lift):
    lib, request = lift
    q = LiftCommand(7, 10, 1, 100, 42)
    assert request(replace(q, boot_id=43), 0).kind == 135
    reply = request(q, 0)
    assert reply.kind == 7 and reply.state == 4 and reply.command_raw == 0
    assert reply.height_um == 0 and not reply.holding()
    assert request(q, 0).kind == 135
    lib.lift_test_zero(1)
    lib.lift_test_sample(2, 1000, 0, 0, 1, 1)
    assert request(LiftCommand(4, 10, 2, 0, 42), 2).holding()
    lib.lift_test_init()
    lib.lift_test_sample(0, 1000, 0, 0, 0, 1)
    assert not request(LiftCommand(4, 10, 3, 0, 42), 0).flags & 1

@pytest.mark.parametrize('base',[100,2**32-80])
def test_lift_hold_uses_new_position_samples_and_drive_readback(lift,base):
    lib,_=lift
    def sample(offset,pos=4095,drive=1,speed=0,healthy=1,stamp=None):
        return lib.lift_test_hold((base+offset)%2**32,(base+(offset if stamp is None else stamp))%2**32,
                                  pos,speed,drive,healthy)
    assert not sample(0)
    assert not sample(100,stamp=0)  # repeated sample cannot satisfy dwell
    assert not sample(50,pos=0)  # modulo wrap is one count, not a jump
    assert sample(100,pos=1)
    assert not sample(110,drive=0)  # missing torque/goal proof invalidates immediately
    assert not sample(120)
    assert not sample(170)
    assert sample(220)
    assert not sample(230,pos=5)  # total drift from anchor, not adjacent deltas
    assert not sample(240,speed=1)
    assert not sample(250,healthy=0)
    assert not sample(600,stamp=250)


@pytest.mark.parametrize('speed,arms_safe,age', [(3, 1, 0), (0, 0, 0), (0, 1, 101)])
def test_bottom_reference_rejects_moving_unsecured_or_old_feedback(lift, speed, arms_safe, age):
    lib, request = lift
    lib.lift_test_sample(1, 1000, speed, 0, 0, arms_safe)
    assert request(LiftCommand(7, 10, 1, 200, 42), 1 + age).kind == 135


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


@pytest.mark.parametrize('base', [0, 2**32-150])
def test_lift_position_evidence_retains_hold_only_with_advancing_samples(lift,base):
    lib,_=lift
    def sample(t,speed=0,moving=0,pos=3832,maintaining=1,stamp=None):
        return lib.lift_test_report((base+t)%2**32,(base+(t if stamp is None else stamp))%2**32,
                                   pos,speed,1,1,moving,maintaining)
    assert sample(0)==0
    assert sample(50)==0
    assert sample(100)==1
    assert sample(120,50,1)==3
    assert sample(150,-50,1)==3
    assert sample(219,50,1)==3
    assert sample(220,50,1)==3
    assert sample(319,50,1,stamp=220)==3
    assert sample(320,50,1,stamp=220)==0  # stale position cannot renew proof
    assert sample(330,50,1)==0  # cannot recreate initial proof from noise


@pytest.mark.parametrize('changes', [
    {'pos':3833}, {'speed':51}, {'speed':-51}, {'drive':0},
    {'healthy':0}, {'moving':2}, {'maintaining':0}, {'stamp':99}, {'now':400},
])
def test_lift_report_rejects_position_power_time_or_scope_changes(lift,changes):
    lib,_=lift
    for t in (0,50,100):lib.lift_test_report(t,t,3832,0,1,1,0,1)
    row=dict(now=120,stamp=120,pos=3832,speed=50,drive=1,healthy=1,moving=1,maintaining=1)
    row.update(changes)
    assert lib.lift_test_report(*row.values())==0


def test_lift_report_strict_recovery_and_repeated_stamp_do_not_renew(lift):
    lib,_=lift
    for t in (0,50,100):lib.lift_test_report(t,t,3832,0,1,1,0,1)
    assert lib.lift_test_report(120,120,3832,50,1,1,1,1)==3
    assert lib.lift_test_report(140,140,3832,0,1,1,0,1)==1
    assert lib.lift_test_report(200,200,3832,50,1,1,1,1)==3
    assert lib.lift_test_report(299,200,3832,50,1,1,1,1)==3
    assert lib.lift_test_report(300,200,3832,50,1,1,1,1)==0


@pytest.mark.parametrize("speed", [0, 50, -50])
def test_lift_controller_expires_report_even_without_new_sample(lift,speed):
    lib,request=lift
    lib.lift_test_sample(1,1000,0,0,0,1)
    assert request(LiftCommand(7,10,1,200,42),1).kind==7
    lib.lift_test_zero(1)
    lib.lift_test_sample(2,1000,0,0,1,1)
    assert lib.lift_test_poll_state(2)==5
    lib.lift_test_transient_sample(20,1000,speed,80)
    assert not lib.lift_test_move(21,20000)  # no new lift motion during recheck
    assert lib.lift_test_poll_state(99)==5
    assert lib.lift_test_poll_state(100)==6


def test_lift_controller_cannot_establish_initial_hold_from_transient(lift):
    lib,request=lift
    lib.lift_test_sample(1,1000,0,0,0,1)
    request(LiftCommand(7,10,1,200,42),1);lib.lift_test_zero(1)
    lib.lift_test_transient_sample(2,1000,50,80)
    assert lib.lift_test_poll_state(2)==4


def test_completed_reference_releases_motion_interlock_before_idle_check(lift):
    lib,request=lift
    result=request(LiftCommand(7,1,1,100,42),0)
    assert result.kind==7 and result.flags&2
    lib.lift_test_zero(1)
    # Independent zero/hold proof completes; the base-motion interlock changes
    # before the endpoint owner next polls. Holding itself is still healthy.
    lib.lift_test_sample(20,1000,0,0,1,0)
    result=request(LiftCommand(4,1,2,0,42),20)
    assert result.state==5 and result.fault==0 and not result.flags&2
    lib.lift_test_sample(30,1000,0,0,0,0)
    assert request(LiftCommand(4,1,3,0,42),30).fault==1


def test_pending_reference_keeps_motion_interlock(lift):
    lib,request=lift
    assert request(LiftCommand(7,1,1,100,42),0).kind==7
    lib.lift_test_sample(20,1000,0,0,0,0)
    result=request(LiftCommand(4,1,2,0,42),20)
    assert result.state==6 and result.fault==1


@pytest.mark.parametrize('base',[0,2**32-200])
def test_lift_report_renews_only_with_new_80ms_position_samples(lift,base):
    lib,_=lift
    def sample(t,speed=0,moving=0):
        return lib.lift_test_report((base+t)%2**32,(base+t)%2**32,3832,speed,1,1,moving,1)
    for t in (0,80,160):sample(t)
    assert sample(240,-50,1)==3
    # One report at the actual four-axis cadence must reach the next observation.
    assert sample(320,-50,1)==3
    assert sample(339,-50,1)==3
    assert sample(340,-50,1)==3
    for t in range(420,2500,80):assert sample(t,50,1)==3


def test_single_noisy_lift_sample_survives_to_next_real_read(lift):
    lib,request=lift
    for t in (0,80,160):lib.lift_test_report(t,t,1000,0,1,1,0,1)
    lib.lift_test_sample(160,1000,0,0,0,1)
    assert request(LiftCommand(7,10,1,200,42),160).kind==7
    lib.lift_test_zero(160)
    assert lib.lift_test_reported_sample(162,1000,0,0)
    assert lib.lift_test_poll_state(162)==5
    assert lib.lift_test_reported_sample(242,1000,-50,1)
    assert lib.lift_test_poll_state(263)==5  # old code faulted before next 80 ms read
    assert lib.lift_test_reported_sample(322,1000,0,0)
    assert lib.lift_test_poll_state(342)==5


def test_bottom_encoder_quantization_does_not_become_negative_base_height(lift):
    lib,request=lift
    lib.lift_test_mini1_scale()
    lib.lift_test_sample(1,1000,0,0,0,1)
    assert request(LiftCommand(7,10,1,200,42),1).kind==7
    lib.lift_test_zero(1);lib.lift_test_sample(2,1000,0,0,1,1)
    assert lib.lift_test_poll_state(2)==5
    # Use the actual Mini 1 scale, 84000um/revolution (20.5um/count).
    lib.lift_test_sample(3,999,0,0,1,1)
    assert lib.lift_test_base_height(3)==0
    # Raw position/height remains unchanged and visible in status.
    assert request(LiftCommand(5,10,2,0,42),3).height_um==-20
    lib.lift_test_sample(4,998,0,0,1,1)
    assert lib.lift_test_base_height(4)==0
    lib.lift_test_sample(5,997,0,0,1,1)
    assert lib.lift_test_base_height(5)==-61


def test_bottom_height_normalization_never_hides_motion_or_unproven_hold(lift):
    lib,request=lift
    lib.lift_test_sample(1,1000,0,0,0,1)
    request(LiftCommand(7,10,1,200,42),1)
    # Pending hold cannot normalize anything.
    lib.lift_test_sample(2,999,0,0,0,1)
    assert lib.lift_test_base_height(2)==-100


@pytest.mark.parametrize('failure',['stale','position','drive','speed','health'])
def test_sustained_lift_report_requires_independent_position_drive_and_freshness(lift,failure):
    lib,request=lift
    for t in (0,80,160):lib.lift_test_report(t,t,1000,0,1,1,0,1)
    lib.lift_test_sample(160,1000,0,0,0,1)
    request(LiftCommand(7,10,1,200,42),160);lib.lift_test_zero(160)
    assert lib.lift_test_reported_sample(162,1000,0,0)
    for t in range(242,1443,80):
        assert lib.lift_test_reported_sample(t,1000,50,1)
        assert lib.lift_test_poll_state(t+60)==5
    if failure=='stale':
        assert lib.lift_test_poll_state(1542)==6
    elif failure=='position':
        lib.lift_test_reported_sample(1522,1001,50,1)
        assert lib.lift_test_poll_state(1522)==6
    elif failure=='speed':
        lib.lift_test_reported_sample(1522,1000,51,1)
        assert lib.lift_test_poll_state(1522)==6
    else:
        assert lib.lift_test_report(1522,1522,1000,50,0 if failure=='drive' else 1,0 if failure=='health' else 1,1,1)==0


def test_bottom_normalization_also_preserves_fresh_position_backed_hold(lift):
    lib,request=lift;lib.lift_test_mini1_scale()
    for t in (0,80,160):lib.lift_test_report(t,t,1000,0,1,1,0,1)
    lib.lift_test_sample(160,1000,0,0,0,1)
    request(LiftCommand(7,10,1,200,42),160);lib.lift_test_zero(160)
    lib.lift_test_reported_sample(162,1000,0,0)
    lib.lift_test_reported_sample(242,999,0,0)
    lib.lift_test_reported_sample(322,999,50,1)
    assert lib.lift_test_poll_state(382)==5
    assert lib.lift_test_base_height(382)==0
    assert request(LiftCommand(5,10,2,0,42),382).height_um==-20


@pytest.mark.parametrize('start',[1000,4095])
def test_bottom_transport_uses_absolute_working_envelope_not_speed_bits(lift,start):
    lib,request=lift;lib.lift_test_init();lib.lift_test_transport_scale()
    lib.lift_test_sample(1,start,0,0,0,1)
    request(LiftCommand(7,10,1,200,42),1);lib.lift_test_zero(1)
    lib.lift_test_sample(2,start,0,0,1,1)
    # Includes count wrap, alternating speed signs and sustained +/-1.9mm
    # within the existing 2mm position tolerance; no repeated emergency latch.
    for i in range(1,101):
        t=2+i*80;offset=(i%3-1)*90
        assert lib.lift_test_transport_sample(t,t,t,(start+offset)%4096,(-1)**i*100,0,1,1)
        assert lib.lift_test_transport_ready(t+60)
        assert lib.lift_test_poll_state(t+60)==5
    assert lib.lift_test_poll_state(t+250)==6  # frozen data still fails


@pytest.mark.parametrize('failure',['position','torque','current','health','position_age','drive_age','future','repeated','higher_target'])
def test_bottom_transport_preserves_fault_boundaries(lift,failure):
    lib,request=lift;lib.lift_test_transport_scale()
    lib.lift_test_sample(1,1000,0,0,0,1)
    request(LiftCommand(7,10,1,200,42),1);lib.lift_test_zero(1);lib.lift_test_sample(2,1000,0,0,1,1)
    assert lib.lift_test_transport_sample(82,82,82,1001,50,0,1,1)
    assert lib.lift_test_transport_ready(82)
    args=[162,162,162,1001,50,0,1,1]
    if failure=='position':args[3]=1100  # +2.05mm beyond original target, not per-frame drift
    elif failure=='torque':args[6]=0
    elif failure=='current':args[5]=1001
    elif failure=='health':args[7]=0
    elif failure=='position_age':args[0]=412
    elif failure=='drive_age':args[2]=2;args[0]=252
    elif failure=='future':args[1]=163
    elif failure=='repeated':args[1]=82
    elif failure=='higher_target':lib.lift_test_transport_target(10000)
    lib.lift_test_transport_sample(*args)
    if failure in ('future','repeated'):
        # Rejected samples may leave previous evidence alive, but cannot renew it.
        assert not lib.lift_test_transport_ready(332)
        assert lib.lift_test_poll_state(332)==6
    else:assert not lib.lift_test_transport_ready(args[0])


def test_bottom_transport_never_establishes_initial_hold(lift):
    lib,request=lift;lib.lift_test_transport_scale()
    lib.lift_test_sample(1,1000,0,0,0,1)
    request(LiftCommand(7,10,1,200,42),1);lib.lift_test_zero(1)
    lib.lift_test_transport_sample(2,2,2,1000,50,0,1,1)
    assert not lib.lift_test_transport_ready(2)
    assert lib.lift_test_poll_state(2)!=5


@pytest.mark.parametrize('corruption',[None,'boot','sequence','session','fault','active','stale'])
def test_keepalive_finishing_hold_race_uses_actual_c_terminal_reply(lift,corruption):
    from types import SimpleNamespace as NS
    from so101_arm_bridge.lift_wire import LiftClient
    import struct
    lib,_=lift;calls=[]
    def exchange(packet):
        command=LiftCommand.decode(packet);calls.append(command.kind)
        now=0
        if command.kind==5:
            # Complete after the host read pending, before renewal arrives.
            lib.lift_test_zero(1);lib.lift_test_sample(2,1000,0,0,1,1);now=2
        out=ctypes.create_string_buffer(64)
        assert lib.lift_test_exchange(packet,len(packet),now,out)
        raw=bytearray(out.raw)
        if command.kind==5:
            assert LiftStatus.decode(bytes(raw)).kind==133
            if corruption=='boot':struct.pack_into('<I',raw,12,43)
            if corruption=='sequence':struct.pack_into('<I',raw,4,999)
            if corruption=='session':struct.pack_into('<I',raw,16,99)
            if corruption=='fault':raw[25]=1
            if corruption=='active':raw[26]|=2
            if corruption=='stale':struct.pack_into('<I',raw,8,202)
            struct.pack_into('<I',raw,60,crc32c(bytes(raw[:60])))
        return bytes(raw)
    client=LiftClient(NS(boot_id=42,port=NS(exchange=exchange),_deadline=lambda _: (100,42)))
    assert client.reference_bottom(10,operator_confirmed=True).state==4
    if corruption:
        with pytest.raises(RuntimeError):client.keepalive()
    else:
        result=client.keepalive()
        assert result.holding(100) and result.kind==133 and client.session is None
    assert calls==[7,5]  # no renewal retry or new movement operation
