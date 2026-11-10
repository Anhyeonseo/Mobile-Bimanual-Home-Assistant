"""Wheel-only writes with unchanged actuator-side arm/gripper/lift goals."""
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest

ROOT=Path(__file__).resolve().parents[1]
BOARD=ROOT/'firmware/stm32_g474_single_arm/Core'
CORE=ROOT/'firmware/stm32_actuator'


@pytest.fixture(scope='module')
def binaries(tmp_path_factory):
    out=tmp_path_factory.mktemp('wheel-hold')
    decl=out/'hal.h'
    decl.write_text('#include "stm32g4xx_hal.h"\nvoid HAL_Delay(uint32_t);\nHAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);\n')
    common=['gcc','-O2','-std=c11','-Wall','-Wextra','-Werror',
        '-DACTUATOR_MOBILE_FIRST_ID=7','-DHOST_MOBILE_BASE_TELEOP_ONLY=1','-DHOST_MOBILE_BASE_PRESERVE_HOLD=1',
        '-include',str(decl),'-I',str(ROOT/'tests/fixtures/servo_hal'),'-I',str(BOARD/'Inc'),'-I',str(CORE/'include')]
    subprocess.run([*common,str(ROOT/'tests/fixtures/wheel_hold_harness.c'),
        str(BOARD/'Src/mobile_base_teleop_app.c'),str(BOARD/'Src/wheel_hold_monitor.c'),str(BOARD/'Src/wheel_hold_setup.c'),'-o',str(out/'app')],check=True)
    subprocess.run([*common,str(ROOT/'tests/fixtures/wheel_hold_gate.c'),str(BOARD/'Src/servo_transport.c'),
        str(CORE/'src/bus_router.c'),str(CORE/'src/shared_bus.c'),'-o',str(out/'gate')],check=True)
    subprocess.run([*common,str(ROOT/'tests/fixtures/wheel_hold_setup_harness.c'),
        str(BOARD/'Src/mobile_base_teleop_app.c'),str(BOARD/'Src/wheel_hold_monitor.c'),str(BOARD/'Src/wheel_hold_setup.c'),
        str(BOARD/'Src/servo_transport.c'),str(CORE/'src/bus_router.c'),str(CORE/'src/shared_bus.c'),
        '-o',str(out/'setup')],check=True)
    return out


@pytest.mark.parametrize('scenario',list(range(21))+[24,25,26,27])
def test_firmware_hold_survives_wheel_stop_and_faults(binaries,scenario):
    result=subprocess.run([str(binaries/'app'),str(scenario)],capture_output=True,text=True)
    assert result.returncode==0,result.stderr+result.stdout
    messages=[json.loads(s) for s in result.stdout.splitlines() if s]
    assert messages[0]['arm_lift_writes']=='explicit_setup_only'
    if scenario in (0,1,2,15,17):
        assert any(m.get('stop_confirmed') and m.get('hold_confirmed') for m in messages)


def test_all_transmit_paths_block_arm_lift_and_right_bus_writes(binaries):
    subprocess.run([str(binaries/'gate')],check=True,timeout=30)


@pytest.mark.parametrize('scenario',range(39))
def test_stationary_setup_current_pose_and_transport_permissions(binaries,scenario):
    result=subprocess.run([str(binaries/'setup'),str(scenario)],capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stderr+result.stdout


@pytest.mark.parametrize('scenario',[21,22,23])
def test_ten_minute_virtual_drive_and_link_loss(binaries,scenario):
    result=subprocess.run([str(binaries/'app'),str(scenario)],capture_output=True,text=True,timeout=20)
    assert result.returncode==0,result.stderr+result.stdout
    messages=[json.loads(line) for line in result.stdout.splitlines() if line]
    summary=messages[0]
    assert summary['arm_lift_preserved']
    if scenario==22: assert summary['expected_link_loss_stop']
    else:
        assert summary['simulated_ms']>=600000
        assert summary['commands']>10000


@pytest.fixture
def host():
    spec=importlib.util.spec_from_file_location('hold_host',ROOT/'tools/run/teleop_mobile_base.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.PRESERVE_HOLD=True
    return module


class Port:
    def __init__(self,replies):self.data=bytearray(b''.join(json.dumps(x).encode()+b'\n' for x in replies));self.writes=[]
    def reset_input_buffer(self):pass
    def read(self,n):result=bytes(self.data[:n]);del self.data[:n];return result
    def write(self,data):self.writes.append(data);return len(data)


def test_hold_mode_rejects_unmonitored_legacy_firmware_before_arm(host):
    old=dict(firmware='mobile-base-teleop-v4',host_baud=921600,servo_baud=1000000,
        ids=[7,8,9,10],max_raw=200,watchdog_ms=400)
    port=Port([old])
    with pytest.raises(RuntimeError):host.inspect(port)
    assert port.writes==[b'INFO\n']
    good=old|dict(firmware='mobile-base-hold-teleop-v7',hold_monitor=True,held_axes=13,arm_lift_writes='explicit_setup_only',hold_setup=True)
    assert host.inspect(Port([good]))==good


def test_wheel_stop_alone_is_not_reported_as_hold_confirmation(host):
    with pytest.raises(RuntimeError,match='hold unconfirmed'):
        host.stop(Port([dict(type='stopped',armed=False,stop_confirmed=True,hold_confirmed=False)]))
    with pytest.raises(RuntimeError,match='hold not confirmed'):
        host.validate(dict(type='ack',session=1,sequence=1,armed=True,fault=False), 'ack',1,1)


def test_hold_preflight_has_no_wheel_arm_or_stop(host):
    info=dict(firmware='mobile-base-hold-teleop-v7',host_baud=921600,servo_baud=1000000,
        ids=[7,8,9,10],max_raw=200,watchdog_ms=400,hold_monitor=True,held_axes=13,arm_lift_writes='explicit_setup_only',hold_setup=True)
    port=Port([info,dict(type='hold',armed=False,fault=False,hold_confirmed=True)])
    assert host.check_hold(port)['motor_write_commands']==0
    assert port.writes==[b'INFO\n',b'HOLD\n']


def test_partial_reply_survives_timeout_during_cleanup(host):
    port=Port([]);port.data.extend(b'{"type":"stopped",')
    with pytest.raises(RuntimeError,match='timeout'):host.receive(port,.001)
    port.data.extend(b'"armed":false,"stop_confirmed":true,"hold_confirmed":true}\n')
    assert host.stop(port)['hold_confirmed'] is True


@pytest.mark.parametrize('key',['u','j','U','J'])
def test_no_lift_keys(host,key):
    assert host.command_for(key,1,'Z',0)[0]=='Z'


def test_actual_keyboard_loop_ten_virtual_minutes(host,monkeypatch):
    clock=[0.0]
    monkeypatch.setattr(host.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(host.time,'sleep',lambda seconds:clock.__setitem__(0,clock[0]+seconds))
    monkeypatch.setattr(host.os,'isatty',lambda fd:True)
    monkeypatch.setattr(host.termios,'tcgetattr',lambda fd:[])
    monkeypatch.setattr(host.termios,'tcsetattr',lambda *args:None)
    monkeypatch.setattr(host.tty,'setcbreak',lambda fd:None)
    monkeypatch.setattr(host.select,'select',lambda *args:([1],[],[]))
    monkeypatch.setattr(host.os,'read',lambda *args:b'x' if clock[0]>=600 else b'wasdqe '[int(clock[0]/2)%7:int(clock[0]/2)%7+1])
    class TimedPort(Port):
        commands=0
        def write(self,data):
            self.writes.append(data)
            parts=data.decode().split()
            if parts[0]=='INFO':
                reply=dict(firmware='mobile-base-hold-teleop-v7',host_baud=921600,servo_baud=1000000,
                    ids=[7,8,9,10],max_raw=200,watchdog_ms=400,hold_monitor=True,held_axes=13,arm_lift_writes='explicit_setup_only',hold_setup=True)
            elif parts[0]=='ARM':
                self.nonce=int(parts[1],16)
                reply=dict(type='armed',session=self.nonce,sequence=0,armed=True,fault=False,hold_confirmed=True)
            elif parts[0]=='D':
                self.commands+=1
                assert int(parts[2],16)==self.commands
                # Normal replies plus occasional 150 ms latency, below watchdog.
                clock[0]+=.15 if self.commands%100==0 else .01
                reply=dict(type='ack',session=self.nonce,sequence=self.commands,armed=True,fault=False,hold_confirmed=True)
            else:
                assert data==b'STOP\n'
                reply=dict(type='stopped',armed=False,stop_confirmed=True,hold_confirmed=True)
            self.data.extend(json.dumps(reply).encode()+b'\n')
            return len(data)
    port=TimedPort([])
    assert host.run(port,1)['hold_confirmed']
    assert clock[0]>=600 and port.commands>11000
    assert port.writes[-1]==b'STOP\n'


@pytest.mark.parametrize('execute',[False,True])
def test_host_setup_is_explicit_and_never_arms_wheels(host,monkeypatch,execute):
    monkeypatch.setattr(host.secrets,'randbelow',lambda n:122)
    info=dict(firmware='mobile-base-hold-teleop-v7',host_baud=921600,servo_baud=1000000,
        ids=[7,8,9,10],max_raw=200,watchdog_ms=400,hold_monitor=True,held_axes=13,
        arm_lift_writes='explicit_setup_only',hold_setup=True)
    plan=dict(type='setup',ok=True,pending=True,token=123)
    reply=dict(type='setup',ok=True,pending=False,reason='SETUP_HOLD_READY')
    settled=dict(type='stopped',armed=False,fault=False,stop_confirmed=True,hold_confirmed=False)
    port=Port([info,*([settled] if execute else []),plan,reply,info,dict(type='hold',armed=False,fault=False,hold_confirmed=True)])
    result=host.prepare_hold(port,execute)
    assert result['executed']==execute
    assert [w.split()[0] for w in port.writes]==([b'INFO',b'STOP',b'PREP',b'HSET',b'INFO',b'HOLD'] if execute else [b'INFO',b'PREP'])


def test_lost_setup_reply_never_retries_activation_or_releases_torque(host,monkeypatch):
    replies=iter([dict(type='stopped',armed=False,fault=False,stop_confirmed=True),dict(type='setup',ok=True,pending=True,token=123)])
    monkeypatch.setattr(host.secrets,'randbelow',lambda n:122)
    monkeypatch.setattr(host,'inspect',lambda p:None)
    def receive(*args):
        try:return next(replies)
        except StopIteration:raise RuntimeError('firmware reply timeout')
    monkeypatch.setattr(host,'receive',receive)
    port=Port([])
    with pytest.raises(RuntimeError,match='timeout'):host.prepare_hold(port,True)
    assert [w.split()[0] for w in port.writes]==[b'STOP',b'PREP',b'HSET']
