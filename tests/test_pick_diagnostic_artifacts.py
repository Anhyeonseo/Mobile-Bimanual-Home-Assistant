import json
from types import SimpleNamespace as NS
import pytest
from home_robot_tasks.pick_service import PickStation
from home_robot_tasks.pick_demo import DemoBackend
from home_robot_tasks.pick_hardware import UnifiedPickHardware
from home_robot_tasks.pick_calibration import RISE_BOARD

def test_jsonl_is_a_downloadable_non_image_and_still_confined(tmp_path):
    station=PickStation(DemoBackend(),tmp_path/'out')
    station.data['current']={'artifacts':[]}
    path=station.output/'lift-progress.jsonl';path.write_text('{"height_um":529582}\n')
    station.artifact(path,'progress')
    row=station.data['current']['artifacts'][0]
    assert row['image'] is False and station.assets[row['id']]==path
    outside=tmp_path/'outside.jsonl';outside.write_text('{}\n')
    (station.output/'link.jsonl').symlink_to(outside)
    with pytest.raises(ValueError,match='inside'):station.artifact(station.output/'link.jsonl','bad')
    station.language.shutdown()

@pytest.mark.parametrize('fault',[False,True])
@pytest.mark.parametrize('broken_display',[False,True])
def test_lift_outcome_is_not_replaced_by_progress_artifact(monkeypatch,tmp_path,fault,broken_display):
    from home_robot_tasks import pick_lift_transition,pick_capture,pick_calibration,unified_carry_inputs
    station=PickStation(DemoBackend(),tmp_path)
    station.data['current']={'artifacts':[]}
    h=UnifiedPickHardware(dict(rise_after_observation=True))
    h.guard=NS(supported_top=False,board=RISE_BOARD,boot_id=4,current_height=0.,require_fresh=lambda _:None)
    h.history.append(NS());h.profile={'calibration_id':'test'};h.lift=NS(status=lambda:NS(height_um=0))
    monkeypatch.setattr(pick_capture,'model_positions',lambda *_:{'lift':0.})
    def rise(*a,**kw):
        (tmp_path/'lift-progress.jsonl').write_text('{"height_um":529582}\n')
        if fault:
            error=ValueError('lift fault=1');error.lift_status={'fault':1,'height_um':529582};raise error
        return {'final':NS(boot_id=4,session=2,position_raw=1,height_um=654588,target_um=654588)}
    monkeypatch.setattr(pick_lift_transition,'rise_to_top',rise)
    monkeypatch.setattr(pick_calibration,'top_model_height',lambda *a:.63)
    monkeypatch.setattr(unified_carry_inputs,'UnifiedPlatformGuard',lambda *a,**k:NS())
    def abort():
        assert fault
        raise RuntimeError('secondary stop failure')
    h.abort=abort
    h.transport=NS(stationary_runtime=lambda:{'first_stop':'transport'},stationary_feedback_failures=lambda:{'left':1})
    def artifact(path,label):
        if broken_display:raise ValueError('renderer failed')
        station.artifact(path,label)
    notes=[]
    ctx=NS(folder=tmp_path,service=lambda:None,check=lambda:None,note=lambda msg,**kw:notes.append(msg),
           artifact=artifact,pause=lambda _:h.history.append(NS()))
    try:
        if fault:
            with pytest.raises(ValueError,match='lift fault=1'):
                h.rise_to_pick_height(ctx,dict(start_model_joint_positions_si={'lift':0.}))
            report=json.loads((tmp_path/'lift-failure.json').read_text())
            assert report['last_lift']['fault']==1 and report['stop_error']=='secondary stop failure'
            assert report['controller_runtime']=={'first_stop':'transport'}
        else:
            h.rise_to_pick_height(ctx,dict(start_model_joint_positions_si={'lift':0.}))
            assert len(h.history)==3
        if broken_display:assert any('표시 실패' in note for note in notes)
        else:assert any(row['label']=='리프트 상승 관측 기록' and not row['image'] for row in station.data['current']['artifacts'])
    finally:h.jobs.shutdown();station.language.shutdown()


def test_stop_diagnostics_survive_original_failure_and_artifact_error(tmp_path):
    import json
    from types import SimpleNamespace as NS
    from home_robot_tasks.pick_hardware import UnifiedPickHardware
    h=UnifiedPickHardware.__new__(UnifiedPickHardware)
    events=[]
    h.guard=NS(last_observation={'mobile':{'state':3},'lift':{'fault':1}})
    h.transport=NS(stationary_runtime=lambda:{'mobile_board':{'source':2}},
                   stationary_feedback_failures=lambda:{'left':{'count':0}})
    def artifact(*args):raise RuntimeError('UI unavailable')
    h.diagnostic_context=NS(folder=tmp_path,artifact=artifact,note=lambda *a:events.append(a))
    def abort():raise RuntimeError('original stop failure')
    h._abort=abort
    with pytest.raises(RuntimeError,match='original stop failure'):h.abort()
    data=json.loads((tmp_path/'controller-stop-diagnostics.json').read_text())
    assert data['controller_runtime']['mobile_board']['source']==2
    assert data['last_platform_observation']['lift']['fault']==1
    assert events
