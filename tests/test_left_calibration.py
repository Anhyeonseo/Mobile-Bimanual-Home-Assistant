from pathlib import Path
import subprocess
import pytest
ROOT=Path(__file__).resolve().parents[1]
BOARD=ROOT/'firmware/stm32_g474_single_arm/Core'
CORE=ROOT/'firmware/stm32_actuator'
FAKE=ROOT/'tests/fixtures/servo_hal'
@pytest.fixture(scope='module')
def binaries(tmp_path_factory):
    out=tmp_path_factory.mktemp('left-calibration')
    common=['gcc','-O2','-std=c11','-Wall','-Wextra','-Werror','-DHOST_LEFT_ONLY_CALIBRATION_ONLY=1','-DHOST_LEFT_ONLY_BUILD=1','-I',str(FAKE),'-I',str(BOARD/'Inc'),'-I',str(CORE/'include')]
    subprocess.run([*common,str(FAKE/'left_calibration_harness.c'),'-o',str(out/'app')],check=True)
    subprocess.run([*common,str(FAKE/'left_calibration_gate.c'),str(BOARD/'Src/servo_transport.c'),str(CORE/'src/bus_router.c'),str(CORE/'src/shared_bus.c'),'-o',str(out/'gate')],check=True)
    return out
@pytest.mark.parametrize('scenario',range(9))
def test_center_without_motion_and_reject_unsafe_or_uncertain_retry(binaries,scenario):
    subprocess.run([str(binaries/'app'),str(scenario)],check=True)
def test_transport_all_ids_addresses_and_values(binaries):
    subprocess.run([str(binaries/'gate')],check=True,timeout=30)

import copy,importlib.util
spec=importlib.util.spec_from_file_location('calibrate_left_home',ROOT/'tools/run/calibrate_left_home.py')
cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
class FakeLink:
    def __init__(self,path,fail=False,drift=False):
        self.path=path;self.fail=fail;self.drift=drift;self.calls=[];self.rows=[];self.scans=0
        for id in range(1,7):
            raw=bytearray(40);raw[5]=id
            self.rows.append(dict(id=id,ok=True,position=1000+id,torque=0,lock=1,eeprom_hex=raw.hex()))
    def exchange(self,command,scan=False):
        self.calls.append(command)
        if command=='INFO':return dict(firmware='left-calibration-v1',motion_enabled=False,ids=list(range(1,7)),consumed_mask=0)
        if command=='OFF':return {'off_ok':True}
        if command=='SCAN':
            self.scans+=1
            if self.drift and self.scans==3:self.rows[0]['position']+=100
            return copy.deepcopy(self.rows)
        assert self.path.exists() # Durable backup precedes first write.
        id=int(command.split()[1]);row=self.rows[id-1]
        if self.fail:raise TimeoutError('uncertain')
        row['position']=2048;raw=bytearray.fromhex(row['eeprom_hex']);raw[31]=77;row['eeprom_hex']=raw.hex()
        return dict(id=id,center_ok=True,torque_off_verified=True)
def test_host_backups_before_write_and_verifies_all(tmp_path):
    path=tmp_path/'receipt.json';link=FakeLink(path)
    r=cli.run(link,path,center=True)
    assert r['status']=='centered_verified' and len(r['results'])==6
    assert len([c for c in link.calls if c.startswith('CENTER')])==6
@pytest.mark.parametrize('defect',['uncertain','drift','backup_failure'])
def test_host_aborts_without_retry(tmp_path,monkeypatch,defect):
    path=tmp_path/'receipt.json';link=FakeLink(path,fail=defect=='uncertain',drift=defect=='drift')
    if defect=='backup_failure':monkeypatch.setattr(cli,'save',lambda *_: (_ for _ in ()).throw(OSError('disk full')))
    with pytest.raises((RuntimeError,TimeoutError,OSError)):cli.run(link,path,center=True)
    assert len([c for c in link.calls if c.startswith('CENTER')])==int(defect=='uncertain')
