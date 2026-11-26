from collections import deque
from types import SimpleNamespace as NS

import pytest
from home_robot_tasks.carry_live_inputs import JointExposure, ObservationNotFresh
from home_robot_tasks.pick_hardware import UnifiedPickHardware
import home_robot_tasks.pick_hardware as hardware


def fixture(monkeypatch, *, sent=10., rtt=.025814481, mask=4095, positions=(0,)*12, times=None):
    clock=iter(times or [sent,sent+rtt,sent+rtt+.001,sent+rtt+.005])
    monkeypatch.setattr(hardware.time,'monotonic',lambda:next(clock))
    events=[]
    packet=NS(status_code=0,joint_count=12,present_mask=mask,
              sample_age_ms=(81,61,41,21,1,101)*2,positions_urad=positions,completed_pairs=2)
    h=UnifiedPickHardware.__new__(UnifiedPickHardware)
    h.prepared=True;h.stopped=False;h.rising=False;h.session=None;h.inputs=None
    h.camera=NS(check=lambda:None)
    h.anchor=NS(positions_urad=(0,)*12)
    h.adapter=NS(keepalive=lambda:events.append('keepalive'),feedback_snapshot=lambda:packet,
                 motion_limits=NS(minimum_urad=(-100000,)*12,maximum_urad=(100000,)*12))
    h.history=deque([JointExposure(1,9.9,9.999,(0.,)*12)])
    h.evidence=NS(publish_mcu_snapshot=lambda *a,**kw:events.append('publish_measured'))
    h.mobile=NS(status=lambda:NS());h.lift=NS(status=lambda:NS())
    h.guard=NS(update=lambda *a:events.append('platform_checked'))
    return h,events


def test_isolated_25ms_reply_is_discarded_without_refreshing_pose(monkeypatch):
    h,events=fixture(monkeypatch);old=list(h.history)
    h.tick()
    assert list(h.history)==old
    assert events==['keepalive','publish_measured','platform_checked']


def test_continued_delays_expire_original_200ms_evidence(monkeypatch):
    h,events=fixture(monkeypatch,times=[10.1,10.13,10.14,10.17,10.18,10.21])
    with pytest.raises(ObservationNotFresh):h.tick()
    assert h.history[-1].lower_s==9.9
    assert events==['keepalive']*3


def test_reacquires_after_recorded_93ms_reply_before_departure(monkeypatch):
    h,events=fixture(monkeypatch,times=[10.1,10.19325,10.194,10.198,10.199,10.202])
    h.tick()
    assert events==['keepalive','keepalive','publish_measured','platform_checked']
    assert h.history[-1].lower_s==pytest.approx(10.194-.101)


@pytest.mark.parametrize('kw',[dict(mask=2047),dict(positions=(50000,)*12)])
def test_missing_joint_or_pose_drift_still_fails(monkeypatch,kw):
    h,events=fixture(monkeypatch,**kw)
    with pytest.raises(ValueError):h.tick()
    assert events==['keepalive']


def test_next_timely_reply_updates_history(monkeypatch):
    h,events=fixture(monkeypatch,rtt=.004)
    h.tick()
    assert h.history[-1].sequence==2 and len(h.history)==2
    assert h.history[-1].lower_s==pytest.approx(9.899)


@pytest.mark.parametrize('fail_rise',[False,True])
def test_station_reports_lift_stage_and_never_plans_after_failed_rise(tmp_path, fail_rise):
    from test_pick_service import RecordingBackend, start, wait_for
    class StationBackend(RecordingBackend):
        def __init__(self):
            super().__init__(); self.mode='station'; self.options={}; self.kit_id='test-only'
        def check_operation(self,operation,*confirmations):return operation
        def prepare_pick(self,ctx,value):
            self.calls.append('prepare_pick');return value
        def rise(self,ctx,value):
            self.calls.append('rise')
            current=ctx.station.snapshot()
            assert current['phase']=='LIFTING'
            assert current['current']['stages'][4]['status']=='running'
            if fail_rise:raise ValueError('test lift fault')
            return value
        def verify(self,ctx,value):
            self.calls.append('verify')
            return dict(status='PLANNED_ONLY',physical_success=False,reason='test only')
    backend=StationBackend();station=start(tmp_path,backend)
    try:
        station.submit('지우개',operation='observe-rise-pick',supported_empty_bottom=True)
        wait_for(lambda:station.snapshot()['history'])
        rows=station.snapshot()['current']['stages']
        assert len(rows)==8 and rows[4]['id']=='LIFTING'
        if fail_rise:
            assert rows[4]['status']=='failed'
            assert all(r['status']=='skipped' for r in rows[5:])
            assert 'plan' not in backend.calls and 'execute' not in backend.calls
        else:
            assert all(r['status']=='done' for r in rows)
            assert backend.calls==['prepare','capture','detect','candidates','prepare_pick','rise','plan','execute','verify']
    finally:station.close()


def test_preview_timers_advance_and_hardware_failure_belongs_to_preparation(tmp_path):
    from home_robot_tasks.pick_web_backend import WebPickBackend
    from home_robot_tasks.pick_service import PickStation
    from test_pick_pipeline import FakeCompute
    from test_pick_service import wait_for
    baseline=tmp_path/'baseline.json';baseline.write_text('{}')
    calls=[]
    class VisibleStages(WebPickBackend):
        def _camera_capture(self,ctx,mode):
            assert ctx.station.snapshot()['phase']=='CAPTURING'
            ctx.pause(.015);calls.append('capture')
            return dict(snapshot='preview',camera_mode=mode)
        def _detect_camera(self,ctx,value):
            snap=ctx.station.snapshot()
            assert snap['phase']=='DETECTING' and snap['current']['stages'][0]['status']=='done'
            capture_seconds=snap['current']['stages'][0]['seconds']
            ctx.pause(.025)
            assert ctx.station.snapshot()['current']['stages'][0]['seconds']==capture_seconds
            calls.append('detect');return value
        def _camera_result(self,ctx,value):
            assert ctx.station.snapshot()['phase']=='GENERATING_GRASPS'
            ctx.pause(.015);calls.append('candidates')
            return dict(status='CAMERA_CANDIDATES_ONLY',reason=None,count=32,surface_error=None)
        def _prepare_hardware(self,ctx,options):
            assert ctx.station.snapshot()['phase']=='PREPARING_PICK'
            calls.append('prepare');raise ValueError('expected locked, stopped, torque 0')
    backend=VisibleStages(dict(mode='station',camera_mode='wide',baseline=str(baseline)),worker_factory=lambda *_:FakeCompute())
    station=PickStation(backend,tmp_path/'station');station.start()
    try:
        wait_for(lambda:station.snapshot()['accepting'])
        station.submit('지우개',operation='observe-rise-pick',supported_empty_bottom=True)
        wait_for(lambda:station.snapshot()['history'])
        rows=station.snapshot()['current']['stages']
        assert [r['status'] for r in rows]==['done','done','done','failed','skipped','skipped','skipped','skipped']
        assert all(r['seconds']>0 for r in rows[:4])
        assert calls==['capture','detect','candidates','prepare']
    finally:station.close()
