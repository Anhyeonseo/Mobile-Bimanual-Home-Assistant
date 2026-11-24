from dataclasses import replace
from types import SimpleNamespace as NS
import json
import pytest
from home_robot_tasks.pick_lift_transition import rise_to_top
from home_robot_tasks.pick_calibration import RISE_BOARD,rise_target_um
from home_robot_tasks.pick_hardware import UnifiedPickHardware
from so101_arm_bridge.lift_wire import LiftStatus
from test_pick_lift_transition import arrival_fixture


@pytest.mark.parametrize('failure',[None,'cancel','fault','stop_ack'])
def test_slow_healthy_ascent_crosses_180s_and_still_honors_faults(failure):
    profile,data=arrival_fixture();target=rise_target_um(RISE_BOARD)
    status=LiftStatus(**data['start_revalidation']['arrival_evidence']['platform']['lift'])
    now=[0.];events=[]
    class Lift:
        def __init__(self):self.mobile=NS(boot_id=status.boot_id)
        active=False
        def status(self):
            if not self.active:return replace(status,height_um=0,target_um=0)
            if failure=='fault' and now[0]>=200:return replace(status,fault=2)
            if now[0]<320:return replace(status,state=3,flags=31,command_raw=100,height_um=int(target*now[0]/320))
            return replace(status,height_um=target,target_um=target)
        def move(self,*_):self.active=True;events.append('move')
        def keepalive(self):events.append('keepalive')
    lift=Lift()
    def stop_request():
        events.append('stop')
        if failure=='stop_ack':raise RuntimeError('stop ACK lost')
    stop=NS(request=stop_request,status=lambda:(NS(confirmed=True,flags=255),now[0]))
    def check():
        if failure in ('cancel','stop_ack') and now[0]>=200:raise RuntimeError('operator cancelled')
    proof=dict(status='LIFT_PATH_CHECKED',calibration_id=profile['calibration_id'],target_um=target)
    kw=dict(clock=lambda:now[0],sleep=lambda _:now.__setitem__(0,now[0]+20),check=check)
    if failure:
        with pytest.raises((ValueError,RuntimeError),match='fault during rise' if failure=='fault' else 'operator cancelled'):
            rise_to_top(lift,lift.status,stop,proof,profile,RISE_BOARD,**kw)
        assert 'stop' in events and now[0]>=200
    else:
        result=rise_to_top(lift,lift.status,stop,proof,profile,RISE_BOARD,**kw)
        assert result['final'].height_um==target and now[0]>=320 and 'stop' not in events
    assert events.count('move')==1


def stop_hardware(monkeypatch):
    from home_robot_tasks import pick_hardware as module
    now=[10.]
    monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(module.time,'sleep',lambda duration:now.__setitem__(0,now[0]+duration))
    h=UnifiedPickHardware({});h.prepared=True
    packet=NS(status_code=0,joint_count=12,present_mask=4095,positions_urad=(0,)*12,sample_age_ms=(40,)*12)
    h.transport=NS(get_feedback_snapshot=lambda:packet)
    h.anchor=NS(positions_urad=(0,)*12)
    h.adapter=NS(motion_limits=NS(minimum_urad=(-100000,)*12,maximum_urad=(100000,)*12))
    published=[]
    h.evidence=NS(publish_mcu_snapshot=lambda p,s,r,**kw:published.append((p,s,r,kw)))
    h.stop=NS(request=lambda:None,status=lambda:(NS(confirmed=len(published)>=2),now[0]))
    return h,packet,published


def test_empty_lift_stop_continues_fresh_measured_evidence_without_motion(monkeypatch):
    h,packet,published=stop_hardware(monkeypatch)
    try:
        h.abort()
        assert h.stop_confirmed and len(published)==2
        assert all(p[0] is packet and p[3]==dict(arms_safe=True,lift_hold=False,payload_safe=True) for p in published)
        assert packet.sample_age_ms==(40,)*12
    finally:h.jobs.shutdown()


@pytest.mark.parametrize('fault',['missing','drift','stale'])
def test_stop_never_renews_invalid_arm_evidence(monkeypatch,fault):
    h,packet,published=stop_hardware(monkeypatch)
    if fault=='missing':packet.present_mask=2047
    if fault=='drift':packet.positions_urad=(60000,)*12
    if fault=='stale':packet.sample_age_ms=(300,)*12
    try:
        with pytest.raises(RuntimeError,match='integrated stop not confirmed.*evidence_error='):h.abort()
        assert not published and not h.stop_confirmed
    finally:h.jobs.shutdown()


@pytest.mark.parametrize('stage',['arm_execution','scene_admission','unprepared'])
def test_empty_stop_path_does_not_infer_payload_or_other_motion_state(monkeypatch,stage):
    h,_,published=stop_hardware(monkeypatch)
    if stage=='arm_execution':h.session=object()
    if stage=='scene_admission':h.inputs=object()
    if stage=='unprepared':h.prepared=False
    try:h._refresh_empty_stop_evidence();assert not published
    finally:h.jobs.shutdown()


def test_hardware_preserves_original_rise_error_when_stop_confirmation_fails(monkeypatch,tmp_path):
    from home_robot_tasks import pick_lift_transition,pick_capture
    h=UnifiedPickHardware(dict(rise_after_observation=True))
    h.guard=NS(supported_top=False,board=RISE_BOARD,boot_id=4,current_height=0.,require_fresh=lambda _:None)
    h.history.append(NS());h.profile={};h.lift=NS(status=lambda:NS(height_um=0))
    monkeypatch.setattr(pick_capture,'model_positions',lambda *_:{'lift':0.})
    def rise(*a,**kw):raise ValueError('original lift fault')
    def abort():raise RuntimeError('secondary stop failure')
    monkeypatch.setattr(pick_lift_transition,'rise_to_top',rise)
    h.abort=abort
    ctx=NS(folder=tmp_path,service=lambda:None,check=lambda:None,note=lambda *_:None,artifact=lambda *_:None)
    try:
        with pytest.raises(ValueError,match='original lift fault'):
            h.rise_to_pick_height(ctx,dict(start_model_joint_positions_si={'lift':0.}))
        report=json.loads((tmp_path/'lift-failure.json').read_text())
        assert report['reason']=='original lift fault' and report['stop_error']=='secondary stop failure'
        assert not h.rising
    finally:h.jobs.shutdown()
