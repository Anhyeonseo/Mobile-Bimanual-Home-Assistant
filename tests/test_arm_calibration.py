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


@pytest.fixture
def range_path(tmp_path):
    # Synthetic measured range for FakeLink's old raw Home ~1000. After
    # recentering the entire range fits, with EEPROM offsets matching the fake.
    d=json.loads((ROOT/'config/home_assistant/historical/pre-home-20261110/arm_ranges.development.json').read_text())
    for side in ('left','right'):
        d['calibration'][side]['offsets_raw']=[0]*6
        for name,e in d['arms'][side].items():
            e.update(minimum_unwrapped_raw=0,maximum_unwrapped_raw=3000)
            e['minimum_urad'],e['maximum_urad']=sorted(round(e['positive_raw_direction']*(raw-2048)*6283185/4096) for raw in (0,3000))
            if name=='gripper':e['maximum_open_raw']=3000
    path=tmp_path/'ranges.json';path.write_text(json.dumps(d));return path


@pytest.mark.parametrize('arm',['left','right'])
def test_default_is_read_only_and_center_has_selected_arm_backup(tmp_path,arm,range_path):
    path=tmp_path/(arm+'.json');link=DualLink(path,arm)
    result=cli.capture(link,path,arm)
    assert result['status']=='backup_saved' and all(c.split()[1] in ('INFO','SCAN') for c in link.calls)
    path=tmp_path/(arm+'-center.json');link=DualLink(path,arm)
    result=cli.capture(link,path,arm,center=True,reference_note='supported home; gripper middle',range_path=range_path)
    assert result['status']=='centered_verified' and result['arm']==arm
    assert len([c for c in link.calls if ' CENTER ' in c])==6
    assert [result['center_reference_rows'][str(i)]['position'] for i in range(1,7)]==list(range(1001,1007))
    assert not any(' OFF' in c for c in link.calls)


@pytest.mark.parametrize('fault',['wrong_arm','drift','uncertain','no_reference'])
def test_no_unconfirmed_or_repeated_centering(tmp_path,fault,range_path):
    path=tmp_path/'record.json';link=DualLink(path,'right',fail=fault=='uncertain',drift=fault=='drift')
    link.wrong_arm=fault=='wrong_arm'
    with pytest.raises((RuntimeError,ValueError,TimeoutError)):
        cli.capture(link,path,'right',center=True,reference_note='' if fault=='no_reference' else 'supported home; gripper middle',range_path=range_path)
    assert sum(' CENTER ' in c for c in link.calls)==int(fault=='uncertain')


def test_power_cycle_check_only_reads_and_matches_saved_eeprom(tmp_path,range_path):
    source=tmp_path/'center.json';link=DualLink(source,'left')
    cli.capture(link,source,'left',center=True,reference_note='supported home; gripper middle',range_path=range_path)
    link.calls.clear();link.fake.path=tmp_path/'verified.json'
    r=cli.capture(link,link.fake.path,'left',verify_record=source)
    assert r['status']=='persistence_verified' and all(c.split()[1] in ('INFO','SCAN') for c in link.calls)


def test_corrupt_persistence_reference_is_rejected_before_port_exchange(tmp_path,range_path):
    path=tmp_path/'center.json';link=DualLink(path,'left')
    cli.capture(link,path,'left',center=True,reference_note='supported home; gripper middle',range_path=range_path)
    bad=json.loads(path.read_text());bad['after'].reverse();path.write_text(json.dumps(bad))
    link.calls.clear()
    with pytest.raises(ValueError):
        cli.capture(link,tmp_path/'verified.json','left',verify_record=path)
    assert link.calls==[]


@pytest.mark.parametrize('defect',['seam','offset'])
def test_all_six_axes_preflight_before_any_center(tmp_path,range_path,defect):
    path=tmp_path/'rejected.json';link=DualLink(path,'left')
    if defect=='seam':link.fake.rows[0]['position']=1
    else:
        raw=bytearray.fromhex(link.fake.rows[0]['eeprom_hex']);raw[31]=1
        link.fake.rows[0]['eeprom_hex']=raw.hex()
    with pytest.raises(ValueError):
        cli.capture(link,path,'left',center=True,reference_note='supported home; gripper middle',range_path=range_path)
    assert not any(' CENTER ' in c for c in link.calls)
    record=json.loads(path.read_text())
    assert record['status']=='preflight_rejected_no_calibration_writes'
    if defect=='seam':
        joint=record['centering_preflight']['joints'][0]
        assert joint['feasible_home_raw_interval']==[1,1096]


def test_feasibility_preview_has_no_calibration_or_motion_writes(tmp_path,range_path):
    path=tmp_path/'preview.json';link=DualLink(path,'right')
    result=cli.capture(link,path,'right',check_home=True,range_path=range_path)
    assert result['centering_preflight']['safe_to_center']
    assert all(c.split()[1] in ('INFO','SCAN') for c in link.calls)


def test_small_home_drift_is_rechecked_before_first_calibration_write(tmp_path,range_path):
    path=tmp_path/'drift.json';link=DualLink(path,'left')
    # Exactly fits initially (max 4095); a one-count drift makes it cross.
    link.fake.rows[0]['position']=953
    exchange=link.exchange
    def drift(command,*,scan=False):
        if command.endswith('SCAN') and link.fake.scans==2:
            link.fake.rows[0]['position']-=1
        return exchange(command,scan=scan)
    link.exchange=drift
    with pytest.raises(ValueError,match='drift'):
        cli.capture(link,path,'left',center=True,reference_note='supported home; gripper middle',range_path=range_path)
    assert not any(' CENTER ' in c for c in link.calls)
