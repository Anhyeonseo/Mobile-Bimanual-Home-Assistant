"""Persisted Home evidence may translate data, never change measured apertures."""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from tools.diagnostics.rebase_home_profiles import rebase_jaw_record as translate
from home_robot_tasks.installed_jaw import InstalledLeftJaw

ROOT=Path(__file__).resolve().parents[1]


def inputs():
    c=ROOT/'config/home_assistant';old=c/'historical/pre-home-20261110'
    read=lambda p:json.loads(p.read_bytes())
    a=read(c/'arm_model_alignment.development.json')
    records={side:tuple((ROOT/a['arms'][side][key]).read_bytes() for key in ('reference_report','persistence_report'))
             for side in ('left','right')}
    return [read(old/'left_pad_gap.observed.json'),read(old/'s2_coordinates.development.json'),
            read(c/'historical/pre-height-20261111/s2_coordinates.development.json'),(old/'arm_ranges.development.json').read_bytes(),
            read(c/'arm_ranges.development.json'),records]


def test_record_rebase_preserves_physical_measurements_and_check():
    args=inputs();original=deepcopy(args[0]);out=translate(*args)
    assert args[0]==original
    assert out['coordinate_rebase']['raw_shift']==-25
    assert [p['gap_mm'] for p in out['samples']]==[5.,30.,60.]
    old=InstalledLeftJaw(original,args[1]);new=InstalledLeftJaw(out,args[2])
    for gap in (5.,17.,30.,44.6,59.9,60.):
        assert new.raw_at_least(gap)==old.raw_at_least(gap)-25
    assert new.check_residual_mm==pytest.approx(.4)
    assert out['physical_gap_remeasurement'] is False and out['motion_authorized'] is False


def test_camera_only_profile_rebind_preserves_all_jaw_measurements():
    c=ROOT/'config/home_assistant';old=c/'historical/pre-height-20261111'
    read=lambda p:json.loads(p.read_bytes())
    before,after=(read(folder/'left_pad_gap.observed.json') for folder in (old,c))
    old_profile,new_profile=(read(folder/'s2_coordinates.development.json') for folder in (old,c))
    assert old_profile['arms']==new_profile['arms']
    assert before['samples']==after['samples'] and before['check']==after['check']
    a,b=InstalledLeftJaw(before,old_profile),InstalledLeftJaw(after,new_profile)
    for raw in (2000,2100,2200,2300):assert a.gap_mm(raw)==b.gap_mm(raw)
    assert not after['camera_only_profile_rebind']['new_measurements']


@pytest.mark.parametrize('change',['wrong_range','wrong_persistence','wrong_camera','wrong_source_id'])
def test_rebase_rejects_unproven_coordinate_changes(change):
    a=inputs()
    if change=='wrong_range':a[4]['calibration']['left']['offsets_raw'][5]+=1
    elif change=='wrong_persistence':
        p=json.loads(a[5]['left'][1]);p['calibration_registers_match']=False
        a[5]['left']=(a[5]['left'][0],json.dumps(p).encode())
    elif change=='wrong_camera':a[2]['base_from_optical'][0][3]+=.01
    else:a[0]['calibration_id']='not-this-robot'
    with pytest.raises(ValueError):translate(*a)
