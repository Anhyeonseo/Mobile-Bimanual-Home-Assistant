from copy import deepcopy
from dataclasses import replace
import pytest
from test_pick_stationary_hold import fixture
from home_robot_tasks.pick_calibration import RISE_BOARD,top_model_height
from home_robot_tasks.platform_preparation import stationary_platform_is_holding


def test_fixed_station_does_not_use_wheel_speed_or_wheel_sample_age():
    profile,ref,m,l,g=fixture()
    ref['base_reference_basis']='operator_fixed_station';g.top_reference=deepcopy(ref)
    m.flags=23;m.velocity_raw=(50,0,0,0);m.feedback_tick_ms=1
    assert not stationary_platform_is_holding(m,l,RISE_BOARD)
    g.update(m,l,20.,20.008)
    assert g.last_observation['accepted']
    assert g.last_observation['mobile']['flags']==23
    assert g.last_observation['lift']['flags']==25
    assert g.last_observation['hold_proof']=='operator_fixed_base_and_measured_lift_hold'
    assert top_model_height(l,profile,RISE_BOARD,ref,stationary_mobile=m)==g.current_height


@pytest.mark.parametrize('which,change',[
 ('mobile',dict(state=1)),('mobile',dict(session=1)),('mobile',dict(reason=1)),
 ('mobile',dict(boot_id=999)),('lift',dict(fault=1)),('lift',dict(command_raw=1)),
 ('lift',dict(height_um=650000)),('lift',dict(observed_ms=1)),('lift',dict(flags=27))])
def test_fixed_station_still_rejects_actual_motion_fault_height_loss(which,change):
    _,ref,m,l,g=fixture();g.top_reference=dict(ref,base_reference_basis='operator_fixed_station')
    m.flags=23
    if which=='mobile':m.__dict__.update(change)
    else:l=replace(l,**change)
    with pytest.raises(ValueError):g.update(m,l,20.,20.008)


def test_top_pick_prepares_then_captures_once_without_preview_or_recapture(monkeypatch,tmp_path):
    from types import SimpleNamespace as NS
    from home_robot_tasks.pick_web_backend import WebPickBackend
    from home_robot_tasks.pick_backend import PipelineBackend
    b=WebPickBackend(dict(mode='station',candidate_source='graspnet',top_baseline=str(tmp_path/'baseline')))
    (tmp_path/'baseline').write_text('{}');b.worker=object();calls=[]
    def prepare(ctx,options):
        assert options['fixed_base_station'] is True
        assert options['camera_mode']=='near' and not options['rise_after_observation']
        calls.append('prepare');b.hardware=NS(options=options,guard=NS(supported_top=True))
    def capture(me,ctx,value):calls.append('capture');return dict(value,snapshot='one-new-capture')
    b._prepare_hardware=prepare
    monkeypatch.setattr(PipelineBackend,'capture',capture)
    b._camera_capture=lambda *a:pytest.fail('unexpected preview capture')
    ctx=NS(progress=lambda *a,**kw:None,note=lambda *a,**kw:None)
    value=b.capture(ctx,dict(operation='top-pick',supported_empty_top=True,supported_startup_release=True))
    assert b.prepare_pick(ctx,value)==value
    assert calls==['prepare','capture'] and not value.get('preview_only')


def test_initial_top_reference_also_accepts_fixed_station_hold_without_wheel_bit():
    from home_robot_tasks.pick_calibration import make_top_reference
    profile,ref,m,l,g=fixture();m.flags=23
    reference=make_top_reference(l,profile,RISE_BOARD,operator_confirmed=True,
        fixed_base_station=True,stationary_mobile=m)
    assert reference['base_reference_basis']=='operator_fixed_station'
    assert top_model_height(l,profile,RISE_BOARD,reference,stationary_mobile=m)==.630
