"""The shared admission path rejects incomplete routes and faulty rehearsals."""
from dataclasses import dataclass
import json

import pytest

from tools.lib.pick_plan_validation import validate_pick_plan


@dataclass
class Program:
    inspection_only = False
    capture_id: str = 'capture-one'


def fixture(tmp_path, monkeypatch, *, native_error=False, rehearsal_error=False):
    from tools.diagnostics import prepare_carry_execution, carry_stream_sim
    from home_robot_tasks import carry_program
    calls=[]
    path=tmp_path/'plan.json';path.write_text(json.dumps({'status':'OFFLINE_ROUTE_FOUND'}))
    def inputs(*args):
        calls.append('inputs');return Program(), object()
    def native(*args, **kwargs):
        calls.append('native')
        if native_error:raise ValueError('rounded wire path collides')
        return {'checked':True}
    def simulate(program, fault):
        calls.append(fault)
        state='HOLDING' if fault is None else 'CANCELLED' if fault=='cancel' else 'STOPPING' if fault=='stop_unconfirmed' else 'FAILED'
        if rehearsal_error and fault=='base_moved':state='HOLDING'
        return {'state':state}
    monkeypatch.setattr(prepare_carry_execution,'load_inputs',inputs)
    monkeypatch.setattr(prepare_carry_execution,'validate_native',native)
    monkeypatch.setattr(carry_stream_sim,'simulate',simulate)
    monkeypatch.setattr(carry_program,'manifest',lambda p:{'capture_id':p.capture_id})
    return path,calls


def test_shared_validator_records_existing_native_and_all_fault_checks(tmp_path,monkeypatch):
    path,calls=fixture(tmp_path,monkeypatch)
    result=validate_pick_plan(path,tmp_path/'prepared',tmp_path/'meshes',tmp_path/'proof')
    assert calls==['inputs','native',None,'no_follow','slip','empty_close','timeout','dispatch','stale','cancel','base_moved','stop_unconfirmed']
    assert result['simulation_passed']
    assert json.loads((tmp_path/'proof/program.json').read_text())['capture_id']=='capture-one'
    assert json.loads((tmp_path/'proof/report.json').read_text())==result


@pytest.mark.parametrize('defect',['no_route','native','rehearsal'])
def test_no_admitted_program_is_published_when_a_check_fails(tmp_path,monkeypatch,defect):
    path,calls=fixture(tmp_path,monkeypatch,native_error=defect=='native',rehearsal_error=defect=='rehearsal')
    if defect=='no_route':path.write_text(json.dumps({'status':'NO_COMPLETE_ROUTE'}))
    with pytest.raises(ValueError):
        validate_pick_plan(path,tmp_path/'prepared',tmp_path/'meshes',tmp_path/'proof')
    assert not (tmp_path/'proof/program.json').exists()
    assert not (tmp_path/'proof/report.json').exists()
    if defect=='no_route':assert not calls
    if defect=='native':assert calls==['inputs','native']
