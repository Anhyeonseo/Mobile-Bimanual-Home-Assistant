"""Execution setup must service measured state and reject changes during setup."""
from collections import deque
import threading
import time
from types import SimpleNamespace as NS

import pytest
from home_robot_tasks import carry_bundle, carry_live_inputs, pick_calibration, pick_hardware
from home_robot_tasks.pick_hardware import UnifiedPickHardware
from home_robot_tasks.pick_service import Context


@pytest.mark.parametrize('change', ['height', 'anchor', 'invalid_calibration'])
def test_slow_admission_keeps_owner_servicing_and_rechecks_live_anchor(monkeypatch,tmp_path,change):
    h=UnifiedPickHardware({})
    h.profile={}
    h.guard=NS(supported_top=True,height=.63,current_height=.63)
    h.history.append(NS(positions_rad=(0.,)*12))
    p=NS(planned_lift_m=.63,stages=[NS(points=[NS(positions_rad=(0.,)*12)])])
    model=dict(lift_reference_basis=pick_calibration.TOP_BASIS,
               calibration_revision=pick_calibration.REVISION,approach_mode='vertical',
               fk={},profile={'base_from_optical':None})
    checked=(p,model,{},None,None,None,{'aligned_depth_intrinsics':None})
    monkeypatch.setattr(carry_bundle,'sha',lambda _: 'hash')
    monkeypatch.setattr(carry_bundle,'validate_installed_binding',lambda *a: None)
    worker_threads=[]
    def audit(*a,**k):
        worker_threads.append(threading.get_ident())
        until=time.perf_counter()+.35
        while time.perf_counter()<until:pass
        if change=='invalid_calibration':raise ValueError('bad calibration')
        return {'within_recorded_ranges':True}
    monkeypatch.setattr(pick_calibration,'audit_top_program',audit)
    monkeypatch.setattr(carry_live_inputs,'MeasuredFK',lambda _: None)
    monkeypatch.setattr(carry_live_inputs,'LiveCarryInputs',lambda *a,**kw: NS())
    ticks=[];owner=threading.get_ident()
    def tick():
        assert threading.get_ident()==owner
        ticks.append(time.perf_counter())
        if len(ticks)>5:
            if change=='height':h.guard.current_height=.64
            if change=='anchor':h.history.append(NS(positions_rad=(.03,)*12))
    station=NS(backend=NS(tick=tick),shutdown=threading.Event(),cancelled=threading.Event(),note=lambda *a,**kw:None)
    ctx=Context(station,tmp_path)
    expected={'height':'lift height changed','anchor':'plan anchor differs','invalid_calibration':'bad calibration'}[change]
    try:
        with pytest.raises(ValueError,match=expected):h.execute(ctx,checked)
        assert len(ticks)>=10 and max(b-a for a,b in zip(ticks,ticks[1:]))<.1
        assert worker_threads and owner not in worker_threads
        assert h.session is None and h.inputs is None
    finally:h.jobs.shutdown()


def test_service_trace_distinguishes_owner_gap_from_failed_transaction(monkeypatch):
    h=UnifiedPickHardware.__new__(UnifiedPickHardware)
    h.prepared=True;h.stopped=False;h.rising=False
    h.execution_phase='load_checked_bundle'
    stamps=iter([1.,1.01,1.31,1.33])
    monkeypatch.setattr(pick_hardware.time,'perf_counter',lambda:next(stamps))
    h._tick=lambda:None
    h.tick()
    def fail():raise RuntimeError('latched stop')
    h._tick=fail
    with pytest.raises(RuntimeError,match='latched stop'):h.tick()
    a,b=h.service_trace
    assert a['gap_ms'] is None
    assert b['gap_ms']==pytest.approx(300.)
    assert b['duration_ms']==pytest.approx(20.)
    assert b['phase']=='load_checked_bundle' and b['error']=='latched stop'
