from pathlib import Path
import importlib.util
import json
import subprocess
import sys

import pytest
from test_left_calibration import FakeLink

ROOT=Path(__file__).resolve().parents[1]
BOARD=ROOT/'firmware/stm32_g474_single_arm/Core'
CORE=ROOT/'firmware/stm32_actuator'
FAKE=ROOT/'tests/fixtures/servo_hal'
sys.path.insert(0,str(ROOT/'tools/run'))
spec=importlib.util.spec_from_file_location('calibrate_arm_home',ROOT/'tools/run/calibrate_arm_home.py')
cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)


@pytest.fixture(scope='module')
def binary(tmp_path_factory):
    out=tmp_path_factory.mktemp('arm-calibration')/'app'
    subprocess.run(['gcc','-O2','-std=c11','-Wall','-Wextra','-Werror',
        '-Wno-misleading-indentation','-DHOST_BIMANUAL_CALIBRATION_ONLY=1',
        '-I',str(FAKE),'-I',str(BOARD/'Inc'),'-I',str(CORE/'include'),
        str(FAKE/'arm_calibration_harness.c'),'-o',str(out)],check=True)
    return out


@pytest.mark.parametrize('scenario',range(9))
def test_actual_command_parser_isolates_buses_and_prevents_motion(binary,scenario):
    subprocess.run([str(binary),str(scenario)],check=True)


def test_both_transport_buses_block_noncalibration_writes(tmp_path):
    original=(FAKE/'left_calibration_gate.c').read_text()
    original=original.replace('assert(!ServoTransport_Register(&other));','assert(ServoTransport_Register(&other));')
    original=original.replace(' for(unsigned id=', ' for(unsigned bus=0;bus<2;bus++)\n for(unsigned id=')
    original=original.replace('ServoTransport_BeginService(&huart1,','ServoTransport_BeginService(bus?&other:&huart1,')
    original=original.replace('ServoTransport_Transmit(&huart1,','ServoTransport_Transmit(bus?&other:&huart1,')
    original=original.replace('ServoTransport_End(&huart1,','ServoTransport_End(bus?&other:&huart1,')
    original=original.replace('assert(other.sent==0);','assert(other.sent==12 && huart1.sent==12);')
    source=tmp_path/'gate.c';source.write_text(original);exe=tmp_path/'gate'
    subprocess.run(['gcc','-O2','-std=c11','-Wall','-Wextra','-Werror','-DHOST_BIMANUAL_CALIBRATION_ONLY=1',
        '-I',str(FAKE),'-I',str(BOARD/'Inc'),'-I',str(CORE/'include'),str(source),
        str(BOARD/'Src/servo_transport.c'),str(CORE/'src/bus_router.c'),str(CORE/'src/shared_bus.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=30)


class DualLink:
    def __init__(self,path,arm,**kwargs):self.fake=FakeLink(path,**kwargs);self.arm=arm;self.calls=[];self.wrong_arm=False
    def exchange(self,command,*,scan=False):
        self.calls.append(command)
        prefix,inner=command.split(' ',1)
        assert prefix==self.arm.upper()
        result=self.fake.exchange(inner,scan=scan)
        if inner=='INFO':result.update(firmware='arm-calibration-v1',arm='wrong' if self.wrong_arm else self.arm)
        if inner.startswith('CENTER'):
            saved=json.loads(self.fake.path.read_text())
            assert saved['arm']==self.arm and saved['reference_note']=='supported home; gripper middle'
        return result


@pytest.mark.parametrize('arm',['left','right'])
def test_default_is_read_only_and_center_has_selected_arm_backup(tmp_path,arm):
    path=tmp_path/(arm+'.json');link=DualLink(path,arm)
    result=cli.capture(link,path,arm)
    assert result['status']=='backup_saved' and all(c.split()[1] in ('INFO','SCAN') for c in link.calls)
    path=tmp_path/(arm+'-center.json');link=DualLink(path,arm)
    result=cli.capture(link,path,arm,center=True,reference_note='supported home; gripper middle')
    assert result['status']=='centered_verified' and result['arm']==arm
    assert len([c for c in link.calls if ' CENTER ' in c])==6
    assert not any(' OFF' in c for c in link.calls)


@pytest.mark.parametrize('fault',['wrong_arm','drift','uncertain','no_reference'])
def test_no_unconfirmed_or_repeated_centering(tmp_path,fault):
    path=tmp_path/'record.json';link=DualLink(path,'right',fail=fault=='uncertain',drift=fault=='drift')
    link.wrong_arm=fault=='wrong_arm'
    with pytest.raises((RuntimeError,ValueError,TimeoutError)):
        cli.capture(link,path,'right',center=True,reference_note='' if fault=='no_reference' else 'supported home; gripper middle')
    assert sum(' CENTER ' in c for c in link.calls)==int(fault=='uncertain')


def test_power_cycle_check_only_reads_and_matches_saved_eeprom(tmp_path):
    source=tmp_path/'center.json';link=DualLink(source,'left')
    cli.capture(link,source,'left',center=True,reference_note='supported home; gripper middle')
    link.calls.clear();link.fake.path=tmp_path/'verified.json'
    r=cli.capture(link,link.fake.path,'left',verify_record=source)
    assert r['status']=='persistence_verified' and all(c.split()[1] in ('INFO','SCAN') for c in link.calls)
