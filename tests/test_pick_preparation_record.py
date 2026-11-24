import json
from types import SimpleNamespace as NS
import pytest
from home_robot_tasks.pick_hardware import inspect_controller_preparation
from tools.run import prepare_unified_controller as prep

@pytest.mark.parametrize('failed',[False,True])
def test_preparation_preserves_observation_on_rejection(tmp_path,monkeypatch,failed):
    calls=[];artifacts=[]
    def inspect(transport,report,**kw):
        calls.append(kw)
        report['inventory_before']={'devices':[{'bus':'left','requested_id':10,'torque':1}]}
        if failed:raise RuntimeError('commissioning rejected: reason=7')
        report['commission_final']={'flags':17}
    monkeypatch.setattr(prep,'inspect',inspect)
    ctx=NS(folder=tmp_path,artifact=lambda p,label:artifacts.append(p))
    if failed:
        with pytest.raises(RuntimeError,match='reason=7'):
            inspect_controller_preparation(object(),ctx,prepare=True)
    else:
        assert inspect_controller_preparation(object(),ctx,prepare=True)['commission_final']['flags']==17
    saved=json.loads((tmp_path/'controller-preparation.json').read_text())
    assert saved['inventory_before']['devices'][0]['torque']==1
    assert ('error' in saved)==failed
    assert len(calls)==len(artifacts)==1
