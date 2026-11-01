"""Lift-only commissioning: no arm/wheel writes, deadman and recording failures."""
import json
from pathlib import Path
import subprocess
import sys
import pytest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/run"))
import teleop_lift as m


@pytest.fixture(scope="module")
def binaries(tmp_path_factory):
    out = tmp_path_factory.mktemp("lift-teleop")
    board = ROOT / "firmware/stm32_g474_single_arm/Core"
    core = ROOT / "firmware/stm32_actuator"
    decl = out / "hal.h"
    decl.write_text('#include "stm32g4xx_hal.h"\nvoid HAL_Delay(uint32_t);\nHAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);\n#define __HAL_UART_CLEAR_OREFLAG(u) ((void)(u))\n#define __HAL_UART_SEND_REQ(u,r) ((void)(u))\n')
    flags = ["gcc", "-O2", "-std=c11", "-Wall", "-Wextra", "-Werror", "-DHOST_LIFT_TELEOP_ONLY=1", "-DHOST_LEFT_ONLY_BUILD=1", "-DACTUATOR_MOBILE_FIRST_ID=7", "-include", str(decl), "-I", str(ROOT / "tests/fixtures/servo_hal"), "-I", str(board / "Inc"), "-I", str(core / "include")]
    subprocess.run([*flags, str(ROOT / "tests/fixtures/lift_teleop_harness.c"), str(board / "Src/lift_teleop_app.c"), str(board / "Src/host_uart_rx.c"), "-o", str(out / "app")], check=True)
    subprocess.run([*flags, str(ROOT / "tests/fixtures/lift_teleop_gate.c"), str(board / "Src/servo_transport.c"), str(core / "src/bus_router.c"), str(core / "src/shared_bus.c"), "-o", str(out / "gate")], check=True)
    return out


@pytest.mark.parametrize("scenario", range(37))
def test_firmware(binaries, scenario):
    result = subprocess.run([str(binaries / "app"), str(scenario)], check=True, capture_output=True, text=True)
    for line in result.stdout.splitlines():
        if line.startswith('{'):
            json.loads(line)  # Diagnostic fields must not truncate JSON replies.
            assert len(line) < 1280


def test_transport_all_ids_registers_values_and_transmit_apis(binaries):
    subprocess.run([str(binaries / "gate")], check=True)


def row(i, counts=0, **extra):
    return dict(relative_counts=counts, sample_tick=i*50, sample_count=i+1,
                speed_raw=0, goal_raw=0, torque_raw=1, fault=False, **extra)


def test_input_timeout_stop_dominance_and_bottom():
    assert m.command_for("s", 1, "Z", 0) == ("D", 1, None)
    assert m.command_for("", 1.26, "D", 1)[0] == "Z"
    assert m.command_for(" s", 2, "D", 1)[0] == "Z"
    assert m.command_for("bs", 2, "D", 1) == ("Z", 2, "bottom")
    assert m.command_for("xs", 2, "D", 1)[2] == "abort"
    assert m.command_for("w", 2, "Z", 1)[0] == "Z"
    with pytest.raises(RuntimeError): m.command_for("s"*65, 2, "Z", 1)


def test_stable_marks_and_scale_never_assume_persistent_origin():
    c = m.Capture()
    for i in range(9): c.add(row(i))
    assert c.stationary() == 0
    c.top = 0
    result = c.result(40960, 90, 26)
    assert result["mm_per_turn"] == 64
    assert result["mm_per_count"] == .015625
    assert not result["absolute_origin_persists"]
    assert not result["hardware_motion_validated"]


@pytest.mark.parametrize("change", [dict(sample_tick=251),dict(sample_count=1),dict(torque_raw=0),dict(fault=True)])
def test_lost_continuity_or_torque_invalidates(change):
    c = m.Capture(); c.add(row(0))
    with pytest.raises(RuntimeError): c.add(row(1) | change)


def test_stationary_rejects_drift_or_motion():
    c = m.Capture()
    for i in range(9): c.add(row(i, counts=i*4))
    assert c.stationary() is None
    c = m.Capture()
    for i in range(9): c.add(row(i) | dict(goal_raw=100))
    assert c.stationary() is None


class Port:
    def __init__(self, replies): self.data=bytearray(b"".join(json.dumps(r).encode()+b"\n" for r in replies)); self.writes=[]
    def reset_input_buffer(self): pass
    def read(self, n): result=bytes(self.data[:n]);del self.data[:n];return result
    def write(self, data): self.writes.append(data);return len(data)


def test_default_is_info_only_and_requires_exact_image():
    p = Port([m.INFO]);assert m.inspect(p)==m.INFO;assert p.writes==[b"INFO\n"]
    p = Port([m.INFO | dict(firmware="mobile-base-teleop-v4")])
    with pytest.raises(RuntimeError): m.inspect(p)
    assert p.writes == [b"INFO\n"]


def test_backup_is_durable_and_never_overwritten(tmp_path):
    p = tmp_path/"backup.json";m.save_new(p, dict(a=1))
    with pytest.raises(FileExistsError): m.save_new(p, dict(a=2))
    assert json.loads(p.read_text())==dict(a=1)


def test_prepare_backup_failure_sends_no_writes(monkeypatch, tmp_path):
    before=dict(torque=0)
    monkeypatch.setattr(m,"inspect",lambda p: m.INFO)
    monkeypatch.setattr(m,"snapshot",lambda p: before)
    p=Port([]);file=tmp_path/"backup.json";file.write_text("existing")
    with pytest.raises(FileExistsError): m.prepare(p,file)
    assert p.writes==[]


def test_not_interactive_never_arms(monkeypatch,tmp_path):
    monkeypatch.setattr(m.os,"isatty",lambda fd: False);p=Port([])
    with pytest.raises(RuntimeError): m.run(p,0,tmp_path/"x.json")
    assert p.writes==[]


@pytest.mark.parametrize("failure", [None, "ack", "hangup", "stop", "startup", "fault", "stop_latched"])
def test_interactive_recording_cleanup_and_invalidating_failures(monkeypatch, tmp_path, failure):
    class SessionPort(Port):
        counts = sequence = 0
        def __init__(self): super().__init__([])
        def write(self, data):
            self.writes.append(data)
            if data == b"STATUS\n":
                reply = dict(type="status", armed=False, fault=False, motor_write_count=0)
            elif data.startswith(b"ARM "):
                reply = dict(type="armed", session=123, sequence=0, armed=True, fault=False)
                if failure == "startup":
                    reply = startup_reject() | dict(motor_write_count=0)
            elif data.startswith(b"D "):
                self.sequence += 1
                motion=data.decode().split()[3]
                if motion=="D":
                    assert self.sequence>=10
                    if failure=="hangup": raise KeyboardInterrupt()
                    self.counts+=100
                reply = row(self.sequence,self.counts,type="ack",session=123,sequence=self.sequence,armed=True)
                reply["goal_raw"]=100 if motion=="D" else 0
                if failure=="ack" and self.sequence==11: reply["sequence"]=99
                if failure=="fault" and self.sequence==11:
                    reply.update(type="fault", fault=True, armed=False, reason="MODE_OR_TORQUE_CHANGED")
            else:
                assert data==b"STOP\n"
                reply=dict(type="stopped",armed=False,stop_confirmed=failure!="stop",fault=failure in ("fault", "stop_latched"))
            self.data.extend(json.dumps(reply).encode()+b"\n");return len(data)
    restored=[]
    monkeypatch.setattr(m,"inspect",lambda p: m.INFO)
    eeprom=[0]*40;eeprom[33]=1
    monkeypatch.setattr(m,"snapshot",lambda p: dict(eeprom=eeprom,goal=0))
    monkeypatch.setattr(m.os,"isatty",lambda fd: True)
    monkeypatch.setattr(m.termios,"tcgetattr",lambda fd: ["old"])
    monkeypatch.setattr(m.termios,"tcsetattr",lambda *a: restored.append(a[-1]))
    monkeypatch.setattr(m.tty,"setcbreak",lambda fd: None)
    monkeypatch.setattr(m.select,"select",lambda *a: ([1],[],[]))
    keys=iter([b""]*9+[b"s"]*10+[b"b"]+[b""]*20)
    # A real tty with no keypress is select-inactive, not EOF.
    current=[b""]
    def select_key(*args):
        current[0]=next(keys);return ([1] if current[0] else [],[],[])
    monkeypatch.setattr(m.select,"select",select_key)
    monkeypatch.setattr(m.os,"read",lambda *a: current[0])
    monkeypatch.setattr(m.secrets,"randbelow",lambda n: 122)
    monkeypatch.setattr(m.time,"sleep",lambda s: None)
    p=SessionPort();file=tmp_path/"measurement.json"
    if failure in ("ack","stop","startup","fault","stop_latched"):
        with pytest.raises(RuntimeError): m.run(p,1,file)
    else: m.run(p,1,file)
    assert restored==[["old"]]
    if failure == "startup":
        assert all(w == b"STATUS\n" or w.startswith(b"ARM ") for w in p.writes)
    else:
        assert p.writes[-1]==b"STOP\n"
    report=json.loads(file.read_text())
    if failure == "startup":
        assert report["startup_rejected_without_motor_writes"] is True
        assert report["startup_reply"]["motor_write_count"] == 0 and report["stop"] is None
    if failure in ("fault", "stop_latched"):
        assert report["controller_fault_latched"] is True
        assert report["stop"]["stop_confirmed"] is True
        assert "STOP 확인:" not in report["error"]
        if failure == "fault": assert "MODE_OR_TORQUE_CHANGED" in report["error"]
    assert report["calibration_valid"] is (failure is None)
    if failure is None:
        assert report["travel_counts"]==1000 and report["mm_per_turn"]==2621.44
    else: assert report["status"]=="incomplete_do_not_use"


def test_partial_reply_survives_timeout_for_stop_cleanup(monkeypatch):
    p=Port([dict(type="fault",reason="INVALID_FRAME"),dict(type="stopped",armed=False,stop_confirmed=True)])
    with monkeypatch.context() as patch:
        ticks=iter([0,0,.005,.02])
        patch.setattr(m.time,"monotonic",lambda: next(ticks))
        with pytest.raises(RuntimeError,match="partial_bytes=2"): m.receive(p,.01)
    assert bytes(p._lift_reply_buffer)==b'{"'
    assert m.stop(p)["stop_confirmed"] is True
    assert p.writes==[b"STOP\n"]


def test_motor_free_link_check(monkeypatch,tmp_path):
    class LinkPort(Port):
        def __init__(self):super().__init__([])
        def write(self,data):
            self.writes.append(data)
            for line in data.decode().splitlines():
                if line=="STATUS": reply=dict(type="status",armed=False,fault=False,motor_write_count=0)
                else:
                    assert line.startswith("ECHO ")
                    reply=dict(type="echo",nonce=int(line.split()[1],16),motor_write_count=0)
                self.data.extend(json.dumps(reply).encode()+b"\n")
            return len(data)
    monkeypatch.setattr(m,"inspect",lambda p: m.INFO)
    p=LinkPort(); result=m.link_check(p,tmp_path/"link.json")
    assert result["responses"]==200
    assert all(w.startswith((b"STATUS",b"ECHO")) for w in p.writes)


@pytest.mark.parametrize("fail_seq", [None, 40])
def test_powered_idle_check_never_requests_descent_and_always_stops(monkeypatch,tmp_path,fail_seq):
    class IdlePort(Port):
        def __init__(self):super().__init__([])
        def write(self,data):
            self.writes.append(data);parts=data.decode().split()
            if parts[0]=="TEST":
                reply=dict(type="armed",armed=True,fault=False,session=123,sequence=0,nonzero_speed_writes=0)
            elif parts[0]=="D":
                assert parts[3]=="Z"
                seq=int(parts[2],16)
                reply=row(seq,type="ack",armed=True,session=123,sequence=seq,nonzero_speed_writes=0)
                if seq==fail_seq: reply["sequence"]=999
            else:
                assert parts[0]=="STOP"
                reply=dict(type="stopped",armed=False,fault=False,stop_confirmed=True)
            self.data.extend(json.dumps(reply).encode()+b"\n");return len(data)
    monkeypatch.setattr(m,"inspect",lambda p: m.INFO)
    eeprom=[0]*40;eeprom[33]=1
    monkeypatch.setattr(m,"snapshot",lambda p: dict(eeprom=eeprom,goal=0))
    monkeypatch.setattr(m.secrets,"randbelow",lambda n:122)
    monkeypatch.setattr(m.time,"sleep",lambda t:None)
    p=IdlePort();path=tmp_path/"idle.json"
    if fail_seq:
        with pytest.raises(RuntimeError):m.idle_check(p,path)
    else: assert m.idle_check(p,path)["status"]=="idle_check_passed"
    assert p.writes[-1]==b"STOP\n"
    assert json.loads(path.read_text())["nonzero_commands_attempted"]==0


@pytest.mark.parametrize("mode", ["held", "released", "space", "abort", "bad_ack"])
def test_visible_direction_check_deadline_release_and_cleanup(monkeypatch,tmp_path,mode):
    now=[0.]; input_round=[0]; key=[b""]; calls=[]
    class VisiblePort(Port):
        counts=0
        def __init__(self):super().__init__([])
        def write(self,data):
            self.writes.append(data);parts=data.decode().split()
            if parts[0]=="STATUS":
                reply=dict(type="status",armed=False,fault=False,motor_write_count=0)
            elif parts[0]=="ARM":
                reply=dict(type="armed",armed=True,fault=False,session=123,sequence=0)
            elif parts[0]=="D":
                seq=int(parts[2],16);motion=parts[3];calls.append((now[0],motion))
                if motion=="D":self.counts+=5
                reply=row(seq,self.counts,type="ack",armed=True,session=123,sequence=seq)
                reply["goal_raw"]=100 if motion=="D" else 0
                if mode=="bad_ack" and motion=="D":reply["sequence"]=999
            else:
                assert parts[0]=="STOP"
                calls.append((now[0],"STOP"))
                reply=dict(type="stopped",armed=False,fault=False,stop_confirmed=True,relative_counts=self.counts)
            self.data.extend(json.dumps(reply).encode()+b"\n");return len(data)
    def select_key(*args):
        input_round[0]+=1;i=input_round[0]
        key[0]=b"s" if i>=10 else b""
        if mode=="released" and i>=12:key[0]=b""
        if mode=="abort" and i==13:key[0]=b"x"
        if mode=="space" and i==13:key[0]=b" s"
        return ([1] if key[0] else [],[],[])
    monkeypatch.setattr(m,"inspect",lambda p:m.INFO)
    eeprom=[0]*40;eeprom[33]=1
    monkeypatch.setattr(m,"snapshot",lambda p:dict(eeprom=eeprom,goal=0))
    monkeypatch.setattr(m.os,"isatty",lambda fd:True)
    monkeypatch.setattr(m.os,"read",lambda *a:key[0])
    monkeypatch.setattr(m.select,"select",select_key)
    monkeypatch.setattr(m.termios,"tcgetattr",lambda fd:[])
    monkeypatch.setattr(m.termios,"tcsetattr",lambda *a:None)
    monkeypatch.setattr(m.tty,"setcbreak",lambda fd:None)
    monkeypatch.setattr(m.secrets,"randbelow",lambda n:122)
    monkeypatch.setattr(m.time,"monotonic",lambda:now[0])
    monkeypatch.setattr(m.time,"sleep",lambda dt:now.__setitem__(0,now[0]+dt))
    p=VisiblePort();file=tmp_path/"visible.json"
    if mode=="bad_ack":
        with pytest.raises(RuntimeError):m.run(p,1,file,pulse_check=True,check_duration=2)
    else:m.run(p,1,file,pulse_check=True,check_duration=2)
    assert p.writes[-1]==b"STOP\n" and not any(w.startswith(b"TEST ") for w in p.writes)
    first=next(t for t,motion in calls if motion=="D")
    assert calls[-1][0]-first<=2.051
    if mode=="released":
        # One initial S is sufficient for this bounded test even with no OS
        # key repeat. The ordinary run lease is covered separately above.
        assert sum(motion=="D" for _,motion in calls)>=39
    if mode=="space":assert calls[-1][0]-first<.3
    report=json.loads(file.read_text())
    assert report["calibration_valid"] is False
    if mode in ("held","released"):
        assert report["status"]=="direction_check_finished"
        assert report["observed_delta_counts"]==p.counts
        assert report["physical_direction_confirmed"] is False
    if mode=="space":assert report["status"]=="direction_check_stopped_by_operator"


def startup_reject(**changes):
    return dict(type='error', reason='ARM_NOT_STATIONARY', armed=False,
                fault=False, torque_raw=0, goal_raw=0, motor_write_count=12,
                speed_raw=32818, **changes)


def test_startup_waits_only_for_explicit_nonmoving_enable_rejection(monkeypatch):
    monkeypatch.setattr(m.time, 'sleep', lambda dt: None)
    replies = [dict(type='status', armed=False, fault=False, motor_write_count=12),
               startup_reject(), startup_reject(),
               dict(type='armed', armed=True, fault=False, session=123, sequence=0)]
    p = Port(replies); trace = []
    m.arm_when_stationary(p, 123, False, trace.append)
    assert len(trace) == 4
    assert p.writes[0] == b'STATUS\n'
    assert len(p.writes) == 4 and all(w.startswith(b'ARM ') for w in p.writes[1:])


@pytest.mark.parametrize('changes', [dict(fault=True), dict(armed=True),
    dict(torque_raw=1), dict(goal_raw=100), dict(motor_write_count=13),
    dict(reason='ARM_TORQUE_READ'), dict(type='fault')])
def test_startup_never_retries_other_or_uncertain_rejections(monkeypatch, changes):
    monkeypatch.setattr(m.time, 'sleep', lambda dt: pytest.fail('unexpected retry'))
    p=Port([dict(type='status',armed=False,fault=False,motor_write_count=12),
            startup_reject() | changes])
    with pytest.raises(RuntimeError): m.arm_when_stationary(p,123,False,lambda r:None)
    assert len(p.writes)==2


def test_startup_continuous_motion_times_out_without_drive(monkeypatch):
    now=[0.]
    monkeypatch.setattr(m.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(m.time,'sleep',lambda dt:now.__setitem__(0,now[0]+dt))
    p=Port([dict(type='status',armed=False,fault=False,motor_write_count=12)] + [startup_reject()]*20)
    with pytest.raises(m.StartupNotStationary) as exc:
        m.arm_when_stationary(p,123,False,lambda r:None)
    assert exc.value.reply['motor_write_count']==12
    assert len(p.writes)==21 and not any(w.startswith(b'D ') for w in p.writes)


def test_startup_reply_timeout_is_not_retried(monkeypatch):
    p=Port([]); replies=iter([dict(type='status',armed=False,fault=False,motor_write_count=0)])
    def receive(port,seconds):
        try:return next(replies)
        except StopIteration:raise RuntimeError('firmware reply timeout')
    monkeypatch.setattr(m,'receive',receive)
    with pytest.raises(RuntimeError,match='reply timeout'):
        m.arm_when_stationary(p,123,False,lambda r:None)
    assert len(p.writes)==2
