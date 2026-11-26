from types import SimpleNamespace as NS
import pytest
from home_robot_tasks.pick_hardware import UnifiedPickHardware
from home_robot_tasks.pick_web_backend import WebPickBackend
from home_robot_tasks.pick_service import PickStation, Rejected
from test_pick_rise_recovery import stop_hardware
from test_pick_service import wait_for


@pytest.mark.parametrize('supported,contact,relation,expected',[
    (False,0.,'unknown',True), (False,.05,'unknown',False),
    (False,0.,'empty',False), (True,.05,'empty',True)])
def test_stop_refreshes_post_motion_samples_without_restarting_motion(monkeypatch,supported,contact,relation,expected):
    h,packet,published=stop_hardware(monkeypatch)
    h.anchor.positions_urad=(-90000,)*12  # Old capture anchor must not be used after motion.
    packet.positions_urad=(0,)*5+(round(contact*1e6),)+(0,)*6
    h.inputs=object();h.reset_supported_empty=supported
    h.session=NS(program=NS(stages=[NS(attached=True)],contact_control_interval=(-.01,.01)),
        index=1,observation_policy='snapshot_then_verify',last_feedback=NS(payload_relation=relation))
    try:
        h._refresh_stop_evidence()
        assert len(published)==1 and published[0][3]['payload_safe'] is expected
        assert published[0][0] is packet and packet.sample_age_ms==(40,)*12
    finally:h.jobs.shutdown()


def test_post_motion_stop_rejects_stale_snapshot(monkeypatch):
    h,packet,published=stop_hardware(monkeypatch);h.inputs=object();h.reset_supported_empty=True
    packet.sample_age_ms=(300,)*12
    try:
        with pytest.raises(ValueError):h._refresh_stop_evidence()
        assert not published
    finally:h.jobs.shutdown()


@pytest.mark.parametrize('active,owner',[(False,None),(True,'trial/hold/0'),(True,None)])
def test_next_test_retires_only_existing_hold_clears_trial_and_preserves_owner(monkeypatch,active,owner):
    from home_robot_tasks import carry_live_inputs
    h,packet,_=stop_hardware(monkeypatch);events=[];old_id=h.session_id
    h.options={'single_capture_test':True}
    motion=NS(owner=owner,maintain=lambda:events.append('maintain'),
        poll=lambda child:(events.append(child) or NS(state='SUCCEEDED')),
        pending_stop_reason=lambda:None,retire_completed=lambda child:events.append('retire'),
        ready=lambda:True)
    h.session=NS(state='HOLDING',active=active,motion=motion,goal='trial',hold_cycle=0,
        program=NS(stages=[NS(timing={'duration_s':.3})]))
    h.reader=NS(refresh=lambda:events.append('read'));h.inputs=object()
    h.latest=('old image',);h.contact_tcp='old contact';h.verification_anchor='old pose'
    h.guard=NS(require_fresh=lambda now:None,height=.630)
    h.adapter.keepalive=lambda:None;h.adapter.feedback_snapshot=lambda:packet
    monkeypatch.setattr(carry_live_inputs,'joint_exposure',lambda *a:NS(positions_rad=(0.,)*12))
    adapter=h.adapter;guard=h.guard
    try:
        h.prepare_next_test(NS(check=lambda:None,note=lambda msg:None))
        assert h.adapter is adapter and h.guard is guard and h.guard.height==.630
        assert h.session is h.inputs is h.reader is h.latest is None
        assert h.contact_tcp is h.verification_anchor is None
        assert h.session_id!=old_id and h.anchor is packet and len(h.history)==1
        assert not h.stopped and not h.stop_confirmed
        assert ('trial/hold/0' in events) is active
    finally:h.jobs.shutdown()


def test_two_completed_trials_reset_to_new_requests_without_recommissioning(tmp_path):
    events=[]
    class Backend(WebPickBackend):
        def prepare(self,ctx):
            self.worker=NS(closed=False,close=lambda:events.append('worker-close'))
            self.prepared=True;self.blocked=''
        def check_operation(self,op,*flags):return op
        def capture(self,ctx,value):
            events.append(('capture',value['id']))
            if self.hardware is None:
                events.append('commission')
                self.hardware=NS(prepared=True,stopped=False,options={'single_capture_test':True},
                    session=None,tick=lambda:None,close=lambda:None,
                    prepare_next_test=lambda ctx:setattr(self.hardware,'session',None))
            return value
        def detect(self,ctx,v):return v
        candidates=plan=detect
        def execute(self,ctx,v):
            self.after_motion=True;self.hardware.session=NS(state='HOLDING')
            return dict(v,bundle='new-'+v['id'])
    b=Backend({'mode':'station'});s=PickStation(b,tmp_path);s.start()
    try:
        wait_for(lambda:s.snapshot()['accepting'])
        for i in range(2):
            identifier=s.submit('지우개',operation='top-pick',supported_empty_top=True)
            wait_for(lambda:len(s.snapshot()['history'])==i+1 and s.snapshot()['reset_available'])
            assert s.snapshot()['next_test_ready'] and not s.snapshot()['accepting']
            with pytest.raises(Rejected):s.request_reset()
            s.request_reset(supported_empty=True)
            wait_for(lambda:s.snapshot()['accepting'])
            snap=s.snapshot()
            assert snap['current'] is snap['last_result'] is None
            assert not s.assets and not b.after_motion and b.hardware.session is None
            assert snap['history'][0]['id']==identifier
        assert events.count('commission')==1 and 'worker-close' not in events
        assert len({event[1] for event in events if isinstance(event,tuple) and event[0]=='capture'})==2
    finally:s.close()


def test_plan_rejection_reset_keeps_healthy_empty_hold_and_models():
    b=WebPickBackend({'mode':'station'});events=[]
    b.prepared=True;b.blocked=''
    b.worker=object()
    h=NS(prepared=True,stopped=False,options={'single_capture_test':True},session=None,inputs=None,
         prepare_next_test=lambda ctx:events.append('reset-trial'),
         abort=lambda:pytest.fail('healthy plan rejection must not latch STOP'))
    b.hardware=h;worker=b.worker
    assert b.ready()[0] and b.can_prepare_next_test()
    b.reset(NS(supported_empty=True))
    assert events==['reset-trial'] and b.hardware is h and b.worker is worker and b.ready()[0]
    h.inputs=object()  # Incomplete execution admission is not an empty hold.
    assert not b.can_prepare_next_test()


def test_empty_hold_reset_clears_capture_without_torque_or_stop(monkeypatch):
    from home_robot_tasks import carry_live_inputs
    h,packet,_=stop_hardware(monkeypatch);h.options={'single_capture_test':True}
    h.latest=('old pixels',);h.guard=NS(require_fresh=lambda now:None)
    h.tick=lambda:None;h.adapter.keepalive=lambda:None;h.adapter.feedback_snapshot=lambda:packet
    monkeypatch.setattr(carry_live_inputs,'joint_exposure',lambda *a:NS(positions_rad=(0.,)*12))
    try:
        h.prepare_next_test(NS(check=lambda:None,note=lambda msg:None))
        assert h.latest is None and h.session is None and h.anchor is packet and not h.stopped
    finally:h.jobs.shutdown()
