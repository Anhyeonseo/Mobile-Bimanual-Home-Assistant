from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import struct
import pytest

from so101_arm_bridge.unified_arm_adapter import UnifiedArmAdapter
from so101_arm_bridge.unified_commissioning import FIRMWARE
from so101_arm_bridge.mobile_wire import crc32c
from so101_arm_bridge.bimanual_stream_adapter import BimanualStreamError
from tools.diagnostics.carry_stream_sim import SimClock,SimTransport

ROOT=Path(__file__).resolve().parents[1]

class Transport(SimTransport):
    def __init__(self,clock):
        super().__init__(clock,(0.,)*12)
        self.boot=2;self.profile=99;self.flags=113;self.calibration=None
    def enter_binary_mode(self):
        h=super().enter_binary_mode();h.firmware_version=FIRMWARE
        h.left_calibration_hash=h.right_calibration_hash=self.calibration
        return h
    def exchange_mobile(self,request):
        assert request[:4]==b'AC\x01\x01'
        self._io('commission_query')
        b=bytearray(64);b[:4]=b'CS\x01\x01'
        struct.pack_into('<IIIII',b,4,struct.unpack_from('<I',request,4)[0],self.tick,self.boot,0,0)
        b[24]=self.flags;struct.pack_into('<I',b,28,self.profile)
        struct.pack_into('<I',b,60,crc32c(b[:60]));return bytes(b)
    def stationary_platform(self,*args):
        raise AssertionError('legacy AP must not be sent')

def make_adapter(monkeypatch,authorized=True):
    import so101_arm_bridge.bimanual_stream_adapter as bridge
    clock=SimClock();monkeypatch.setattr(bridge,'time',NS(monotonic=clock,sleep=clock.advance))
    t=Transport(clock);path=ROOT/'config/home_assistant/arm_ranges.development.json'
    preview=UnifiedArmAdapter(t,path)
    a=UnifiedArmAdapter(t,path,authorized_limits=replace(preview.motion_limits,hardware_authorized=authorized),
                        motion_authorized=authorized)
    t.calibration=a.calibration_hash
    return a,t,clock

def test_integrated_finite_current_pose_and_next_leg_use_no_legacy_ap(monkeypatch):
    a,t,clock=make_adapter(monkeypatch)
    a.prepare();before=list(t.calls);points=a.current_pose_hold_points()
    assert t.calls==before and all(p.positions_rad==(0.,)*12 for p in points)
    a.start('supervised-hold',points,finite=True)
    for _ in range(30):
        clock.advance(.02);a.poll('supervised-hold')
        if a.state.value=='ready':break
    assert a.state.value=='ready' and t.enabled and a._armed
    t.flags=115
    a.start('supervised-hold',points,finite=True)
    assert a.state.value=='active' and t.calls.count('arm')==1
    assert a.last_start_timing['policy']=='unified_mcu_handoff_at_append'

@pytest.mark.parametrize('change', ['unprepared','stopped','already_running','old_firmware'])
def test_invalid_initial_board_never_enables_torque(monkeypatch,change):
    a,t,_=make_adapter(monkeypatch)
    if change=='unprepared':t.flags=0;t.boot=0
    elif change=='stopped':t.flags=25
    elif change=='already_running':t.flags=115
    else:
        original=t.enter_binary_mode
        def old():
            h=original();h.firmware_version=0x24911;return h
        t.enter_binary_mode=old
    with pytest.raises(BimanualStreamError):a.prepare()
    assert not t.enabled and 'arm' not in t.calls

@pytest.mark.parametrize('change', ['boot','profile','stop'])
def test_changed_commission_identity_blocks_start(monkeypatch,change):
    a,t,_=make_adapter(monkeypatch);a.prepare();points=a.current_pose_hold_points()
    if change=='boot':t.boot+=1
    elif change=='profile':t.profile+=1
    else:t.flags|=8
    with pytest.raises(BimanualStreamError):a.start('task',points,finite=True)
    assert not t.enabled and 'arm' not in t.calls

def test_unauthorized_and_legacy_hold_do_not_send_activation(monkeypatch):
    a,t,_=make_adapter(monkeypatch,False);a.prepare();points=a.current_pose_hold_points()
    before=list(t.calls)
    with pytest.raises(BimanualStreamError):a.start('task',points,finite=True)
    with pytest.raises(BimanualStreamError):a.activate_stationary_hold()
    assert t.calls==before and not t.enabled

def test_current_pose_hold_uses_last_unpowered_anchor_not_earlier_preview(monkeypatch):
    a,t,_=make_adapter(monkeypatch);a.prepare()
    latest=(10000,)*12
    original=t.prepare_shadow
    def settled():
        snapshot=original();snapshot.anchor_positions_urad=latest;return snapshot
    monkeypatch.setattr(t,'prepare_shadow',settled)
    a.start_current_pose_hold('supported-hold')
    assert t.goal_log and all(q==latest for q in t.goal_log)
    assert not a._current_pose_only


def test_current_pick_firmware_enforces_bound_profile_before_shadow(monkeypatch):
    _, t, _ = make_adapter(monkeypatch)
    board = (0x2492E, 2696088964)
    original = t.enter_binary_mode
    def hello():
        packet = original()
        packet.firmware_version = board[0]
        return packet
    t.enter_binary_mode = hello
    adapter = UnifiedArmAdapter(t, ROOT/'config/home_assistant/arm_ranges.development.json', pick_board=board)
    with pytest.raises(BimanualStreamError, match='profile changed'):
        adapter.prepare()
    t.profile = board[1]
    assert adapter.prepare().positions_urad == (0,) * 12
    assert 'arm' not in t.calls
