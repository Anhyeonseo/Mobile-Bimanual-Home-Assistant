import copy
import importlib.util
import itertools
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools/run'))
spec = importlib.util.spec_from_file_location('measure_arm_range',ROOT/'tools/run/measure_arm_range.py')
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)


def rows(position=2048):
    result=[]
    for i in range(1,7):
        raw=bytearray(40);raw[5]=i
        result.append(dict(id=i,ok=True,torque=0,position=position if i==1 else 2048,eeprom_hex=raw.hex()))
    return result


def test_continuous_sweep_tracks_wrap_and_returns_to_home():
    sweep=m.Sweep(rows(),0)
    positions=list(range(2048,4199,100))+[4200]+list(range(4100,2048,-100))+[2048]
    for i,p in enumerate(positions):sweep.add(rows(p%4096),i*.1)
    result=sweep.finish()
    assert result['minimum_unwrapped_raw']==2048
    assert result['maximum_unwrapped_raw']==4200
    assert result['final_unwrapped_raw']==2048


@pytest.mark.parametrize('kind',['torque','offset','gap','jump','halfturn'])
def test_rejects_lost_continuity_and_changed_motor_state(kind):
    sweep=m.Sweep(rows(),0);sweep.add(rows(),0)
    next_rows=rows();stamp=.1
    if kind=='torque':next_rows[3]['torque']=1
    if kind=='offset':next_rows[3]['eeprom_hex']='00'*40
    if kind=='gap':stamp=1.1
    if kind=='jump':next_rows[0]['position']=2600
    if kind=='halfturn':next_rows[0]['position']=0
    with pytest.raises(ValueError):sweep.add(next_rows,stamp)


def test_requires_actual_movement_but_not_home_start_or_return():
    sweep=m.Sweep(rows(),0)
    for i in range(5):sweep.add(rows(100),i*.1)
    with pytest.raises(ValueError):sweep.finish()
    sweep.add(rows(250),.5)
    assert sweep.finish()['final_unwrapped_raw']==250


def test_other_joint_movement_does_not_change_selected_range():
    sweep=m.Sweep(rows(),0)
    for i,p in enumerate([2142,2300,2400,2250,2200]):
        sample=rows(p)
        sample[2]['position']=(i*1000)%4096
        sample[3]['position']=(4000-i*850)%4096
        sweep.add(sample,i*.1)
    assert sweep.finish()['minimum_unwrapped_raw']==2142
    assert sweep.finish()['maximum_unwrapped_raw']==2400


def test_arbitrary_start_tracks_negative_wrap_without_rebinding_home():
    sweep=m.Sweep(rows(),0)
    for i,p in enumerate([10,4090,4000,3900,3800]):sweep.add(rows(p),i*.1)
    assert sweep.finish()['minimum_unwrapped_raw']==-296
    assert sweep.finish()['maximum_unwrapped_raw']==10


class Link:
    def __init__(self):self.commands=[];self.positions=iter([2048,2200,2300,2150,2048])
    def exchange(self,command,scan=False):
        self.commands.append(command)
        if command=='INFO':return dict(firmware='arm-calibration-v1',arm='left',motion_enabled=False,ids=list(range(1,7)))
        assert command=='SCAN' and scan
        return rows(next(self.positions))


def test_capture_only_reads_and_preserves_calibration_and_trace(tmp_path):
    link=Link();actions=iter([None]*4+['finish']);times=itertools.count(0,.1)
    out=tmp_path/'range.json'
    result=m.capture(link,dict(arm='left',after=rows()),out,0,lambda:next(actions),
        calibration_sha256='abc',clock=lambda:next(times),sleep=lambda _:None)
    assert set(link.commands)=={'INFO','SCAN'}
    assert result['status']=='captured_pending_operator_review'
    assert result['maximum_unwrapped_raw']==2300
    assert result['calibration_sha256']=='abc'
    assert not result['firmware_limit_authorized']
    assert not result['home_branch_verified']
    assert result['other_joint_repositioning_allowed']
    assert not result['collision_free_joint_combinations_verified']
    assert len(out.with_suffix('.jsonl').read_text().splitlines())==5


def test_abort_keeps_incomplete_evidence_without_writes(tmp_path):
    out=tmp_path/'aborted.json';link=Link();times=itertools.count(0,.1)
    with pytest.raises(RuntimeError):
        m.capture(link,dict(arm='left',after=rows()),out,0,lambda:'abort',
            calibration_sha256='abc',clock=lambda:next(times),sleep=lambda _:None)
    record=json.loads(out.read_text())
    assert record['status']=='incomplete_do_not_use' and record['calibration_sha256']=='abc'
    assert set(link.commands)=={'INFO','SCAN'}
