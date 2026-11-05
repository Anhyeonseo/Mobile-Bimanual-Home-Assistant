import json
from pathlib import Path
import subprocess
import pytest
ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def binaries(tmp_path_factory):
    out=tmp_path_factory.mktemp('lift-v5')
    board=ROOT/'firmware/stm32_g474_single_arm/Core'
    core=ROOT/'firmware/stm32_actuator'
    decl=out/'hal.h'
    decl.write_text('#include "stm32g4xx_hal.h"\nvoid HAL_Delay(uint32_t);\nHAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);\n#define __HAL_UART_CLEAR_OREFLAG(u) ((void)(u))\n#define __HAL_UART_SEND_REQ(u,r) ((void)(u))\n')
    flags=['gcc','-O2','-std=c11','-Wall','-Wextra','-Werror','-DHOST_LIFT_BIDIRECTIONAL=1','-DHOST_LIFT_TELEOP_ONLY=1','-DHOST_LEFT_ONLY_BUILD=1','-DACTUATOR_MOBILE_FIRST_ID=7','-include',str(decl),'-I',str(ROOT/'tests/fixtures/servo_hal'),'-I',str(board/'Inc'),'-I',str(core/'include')]
    subprocess.run([*flags,str(ROOT/'tests/fixtures/lift_bidirectional_harness.c'),str(board/'Src/lift_teleop_app.c'),str(board/'Src/host_uart_rx.c'),'-o',str(out/'app')],check=True)
    subprocess.run([*flags,str(ROOT/'tests/fixtures/lift_teleop_gate.c'),str(board/'Src/servo_transport.c'),str(core/'src/bus_router.c'),str(core/'src/shared_bus.c'),'-o',str(out/'gate')],check=True)
    return out


@pytest.mark.parametrize('scenario', range(15))
def test_bidirectional_firmware(binaries,scenario):
    r=subprocess.run([str(binaries/'app'),str(scenario)],capture_output=True,text=True)
    assert r.returncode==0,r.stderr+r.stdout
    for line in r.stdout.splitlines():
        if line.startswith('{'):json.loads(line)


def test_v5_transport_still_blocks_other_motors_and_other_speeds(binaries):
    subprocess.run([str(binaries/'gate')],check=True)

import sys
sys.path.insert(0,str(ROOT/'tools/run'))
import teleop_lift_bidirectional as host
import teleop_lift as base


def test_keys_release_stop_and_opposing_keys():
    assert host.command_for('w',1,'Z',0)==('U',1,None)
    assert host.command_for('s',1,'Z',0)==('D',1,None)
    assert host.command_for('',1.26,'U',1)[0]=='Z'
    assert host.command_for('sw',2,'U',1)[0]=='Z'
    assert host.command_for(' w',2,'U',1)[0]=='Z'
    assert host.command_for('bw',2,'U',1)[2]=='mark'
    assert host.command_for('xw',2,'U',1)[2]=='exit'


@pytest.mark.parametrize('failure',[None,'timeout','reference','fault','stop','startup'])
def test_host_stop_and_invalid_measurement_on_fault(monkeypatch,tmp_path,failure):
    class Port:
        def __init__(self):self.data=bytearray();self.writes=[];self.count=0;self.seq=0
        def write(self,data):
            self.writes.append(data)
            if data==b'STATUS\n':
                row=dict(type='status',armed=False,fault=False,reference_id=77,motor_write_count=0)
            elif data.startswith(b'ARM '):
                row=dict(type='armed',armed=True,fault=False,session=123,sequence=0,reference_id=77)
                if failure=='startup':
                    row.update(type='error',reason='ARM_NOT_STATIONARY',armed=False,torque_raw=0,goal_raw=0,motor_write_count=0)
            elif data.startswith(b'D '):
                self.seq+=1;motion=data.decode().split()[3]
                if motion=='U':
                    if failure=='timeout':raise RuntimeError('write timeout')
                    self.count-=100
                assert motion in ('U','Z')
                row=dict(type='ack',session=123,sequence=self.seq,armed=True,fault=False,
                         reference_id=77,relative_counts=self.count,sample_tick=self.seq*50,
                         sample_count=self.seq,speed_raw=0,goal_raw=32868 if motion=='U' else 0,torque_raw=1)
                if self.seq==11:
                    if failure=='reference':row['reference_id']=99
                    if failure=='fault':row.update(type='fault',fault=True)
            else:
                assert data==b'STOP\n'
                row=dict(type='stopped',armed=False,fault=False,stop_confirmed=failure!='stop')
            self.data.extend(json.dumps(row).encode()+b'\n');return len(data)
        def read(self,n):r=bytes(self.data[:n]);del self.data[:n];return r
    monkeypatch.setattr(base,'inspect',lambda *args: host.INFO)
    monkeypatch.setattr(host.os,'isatty',lambda fd:True)
    monkeypatch.setattr(host.termios,'tcgetattr',lambda fd:['old'])
    restored=[]
    monkeypatch.setattr(host.termios,'tcsetattr',lambda *a:restored.append(a[-1]))
    monkeypatch.setattr(host.tty,'setcbreak',lambda fd:None)
    monkeypatch.setattr(host.secrets,'randbelow',lambda n:122)
    monkeypatch.setattr(host.time,'sleep',lambda t:None)
    keys=iter([b'']*9+[b'w']*10+[b'b']+[b'']*20)
    active=[b'']
    def key_poll(*args):
        active[0]=next(keys);return ([1] if active[0] else [],[],[])
    monkeypatch.setattr(host.select,'select',key_poll)
    monkeypatch.setattr(host.os,'read',lambda *a:active[0])
    p=Port();path=tmp_path/'report.json'
    if failure:
        with pytest.raises(RuntimeError):host.run(p,1,path)
    else:host.run(p,1,path)
    assert restored==[['old']]
    report=json.loads(path.read_text())
    assert report['calibration_valid'] is False
    if failure=='startup':assert b'STOP\n' not in p.writes
    else:assert p.writes[-1]==b'STOP\n'
    if failure:assert report['status']=='failed'
    else:
        assert report['status']=='intermediate_point_recorded'
        assert report['relative_counts']==-1000
        assert report['height_above_bottom_m']==pytest.approx(1000*.084/4096)


def test_reference_cannot_be_redefined_by_restarting_cli(tmp_path):
    class Port:
        def __init__(self):self.writes=[];self.data=bytearray(json.dumps(dict(type='status',armed=False,fault=False,reference_id=77)).encode()+b'\n')
        def write(self,b):self.writes.append(b);return len(b)
        def read(self,n):r=bytes(self.data[:n]);del self.data[:n];return r
    # INFO is a strict version boundary; this test isolates the reference preflight.
    p=Port()
    from unittest.mock import patch
    with patch.object(base,'inspect',return_value=host.INFO):
        with pytest.raises(RuntimeError):host.set_bottom(p,tmp_path/'ref.json')
    assert p.writes==[b'STATUS\n']
