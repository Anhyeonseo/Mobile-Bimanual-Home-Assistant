from dataclasses import dataclass
import json
from types import SimpleNamespace

import pytest

from tools.run.arm_motion_trial import (execute, leg_points, main, summarize, targets,
    measurement_reference, trial_envelope, require_envelope, trial_hardware_factory)


@pytest.mark.parametrize('degrees,seconds', [(1,2),(5,4),(-5,4)])
def test_time_doubling_preserves_path_endpoints_and_other_joints(degrees, seconds):
    anchor = tuple(.01*i for i in range(12))
    start, end = targets(anchor, 1, degrees, (-2,)*12, (2,)*12)
    a, b = leg_points(start, end, seconds), leg_points(start, end, seconds*2)
    assert all(a[i][1] == pytest.approx(b[2*i][1]) for i in range(seconds*20+1))
    assert a[-1][0] == 3100+seconds*1000 and b[-1][0] == 3100+seconds*2000
    for points in (a, b, leg_points(end, start, 2)):
        assert all(t2-t1 == 50 for (t1, _), (t2, _) in zip(points, points[1:]))
        assert all(q[j] == anchor[j] for _, q in points for j in range(12) if j != 1)
    assert a[-61][1] == a[-1][1] == end


@pytest.mark.parametrize('index,degrees', [(5,1),(11,1),(1,0),(1,5.01),(1,float('nan'))])
def test_reject_unbounded_or_gripper_motion(index, degrees):
    with pytest.raises(ValueError):
        targets((0,)*12, index, degrees, (-2,)*12, (2,)*12)


def test_reject_limit_instead_of_clipping():
    with pytest.raises(ValueError):
        targets((0,)*12, 1, 1, (-2,)*12, (.001,)*12)


def test_offline_preview_does_not_load_hardware(capsys):
    assert main([]) == 0
    preview = json.loads(capsys.readouterr().out)
    assert preview['hardware_commands'] == 0 and not preview['actual_anchor_checked']
    with pytest.raises(SystemExit):
        main(['--execute'])
    with pytest.raises(SystemExit):
        main(['--degrees','5'])  # Larger excursion must explicitly slow down.
    assert main(['--degrees','5','--seconds','4']) == 0
    preview = json.loads(capsys.readouterr().out)
    assert preview['durations_s'] == [4,8]


def test_summary_uses_joint_source_time_and_deduplicates_across_tick_wrap():
    origin = 0xffffff00
    wire = [dict(apply_tick=(origin+t)&0xffffffff, positions_urad=[t*10 if t < 2000 else 20000]*12)
            for t in range(0,5001,50)]
    reads = []
    for t in range(0,5001,50):
        packet = dict(firmware_tick_ms=(origin+t+120)&0xffffffff,
                      sample_age_ms=[120]*12, status_code=0, present_mask=4095,
                      positions_urad=[t*10 if t < 2000 else 20000]*12)
        reads.extend([dict(packet=packet),dict(packet=packet)])
    result = summarize(dict(accepted_samples=wire, feedback=reads, seconds=2), 1)
    assert result['unique_samples'] == 101
    assert result['tracking_rms_rad'] == pytest.approx(0)
    assert result['final_hold_peak_to_peak_rad'] == 0
    assert result['maximum_sample_gap_s'] == .05
    reads[-1]['packet'] = dict(reads[-1]['packet'], positions_urad=[1234]*12)
    with pytest.raises(ValueError, match='conflicting'):
        summarize(dict(accepted_samples=wire, feedback=reads, seconds=2), 1)


def test_reference_uses_each_measurement_age_across_wrap_and_holds_before_first_point():
    origin=0xfffffff0
    wire=[dict(apply_tick=origin,positions_urad=(0,)*12),
          dict(apply_tick=(origin+100)&0xffffffff,positions_urad=(80000,)*12)]
    packet=SimpleNamespace(firmware_tick_ms=(origin+100)&0xffffffff,
                           sample_age_ms=(0,50,100,150)*3)
    assert measurement_reference(packet,wire,(-123,)*12) == (80000,40000,0,-123)*3


@pytest.mark.parametrize('fault',[None,'tracking','other_arm','stale','range','platform'])
def test_motion_service_keeps_freshness_tracking_ranges_and_platform_checks(tmp_path, fault):
    from tools.run.arm_motion_trial import TrialContext
    from so101_arm_bridge.stream_protocol_v2 import BimanualFeedbackSnapshotV2
    # 80 mrad is outside the former fixed-pose guard, but on this declared
    # moving trajectory. The 49,088 urad error allowance must remain unchanged.
    positions=[0]*12;positions[1]=80000
    ages=[0]*12
    if fault=='tracking': positions[1]=129089  # 49,089 beyond expected.
    if fault=='other_arm': positions[7]=49089
    if fault=='stale': ages[1]=250
    packet=BimanualFeedbackSnapshotV2(0,12,4095,1,0,200,10,tuple(positions),tuple(ages))
    upper=[2000000]*12
    if fault=='range': upper[1]=70000
    limits=SimpleNamespace(minimum_urad=(-2000000,)*12,maximum_urad=tuple(upper))
    evidence=[];platform=[]
    def update(*_):
        platform.append(True)
        if fault=='platform': raise ValueError('lift hold lost')
    target=[0]*12;target[1]=.08
    h=SimpleNamespace(prepared=True,camera=SimpleNamespace(check=lambda:None),
        adapter=SimpleNamespace(state=SimpleNamespace(value='ready'),keepalive=lambda:None,
            feedback_snapshot=lambda:packet,motion_limits=limits),
        motion_envelope=trial_envelope((0,)*12,target),
        evidence=SimpleNamespace(publish_mcu_snapshot=lambda *a,**kw:evidence.append(packet)),
        mobile=SimpleNamespace(status=lambda:None),lift=SimpleNamespace(status=lambda:None),
        guard=SimpleNamespace(update=update,require_fresh=lambda _:None))
    ctx=TrialContext(h,tmp_path,{},clock=lambda:10.)
    ctx.leg=dict(initial_urad=(0,)*12,feedback=[],accepted_samples=[
        dict(apply_tick=100,positions_urad=(0,)*12),
        dict(apply_tick=200,positions_urad=(0,80000)+(0,)*10)])
    if fault is None:
        ctx.service()
        assert evidence and platform
    else:
        with pytest.raises(ValueError):ctx.service()
        if fault!='platform': assert not evidence and not platform
    assert ctx.leg['feedback'][0]['packet']['positions_urad']==packet.positions_urad


@pytest.mark.parametrize('outside',[False,True])
def test_stop_observation_is_bounded_to_sweep_without_claiming_stop_completion(outside):
    from so101_arm_bridge.stream_protocol_v2 import BimanualFeedbackSnapshotV2
    positions=[0]*12;positions[1]=80000 if not outside else 129089
    packet=BimanualFeedbackSnapshotV2(0,12,4095,1,0,200,10,tuple(positions),(0,)*12)
    h=trial_hardware_factory({})
    try:
        h.prepared=True
        h.adapter=SimpleNamespace(motion_limits=SimpleNamespace(
            minimum_urad=(-2000000,)*12,maximum_urad=(2000000,)*12))
        target=[0]*12;target[1]=.08
        h.motion_envelope=trial_envelope((0,)*12,target)
        h.transport=SimpleNamespace(get_feedback_snapshot=lambda:packet)
        sent=[]
        h.evidence=SimpleNamespace(publish_mcu_snapshot=lambda *a,**kw:sent.append(packet))
        if outside:
            with pytest.raises(ValueError,match='trial envelope'):h._refresh_empty_stop_evidence()
            assert not sent
        else:
            h._refresh_empty_stop_evidence()
            assert sent and not h.stop_confirmed
    finally:
        h.close()


@pytest.mark.parametrize('degrees,seconds', [(1,2),(5,4),(-5,4)])
def test_trial_feeds_real_adapter_and_records_all_accepted_samples(monkeypatch, tmp_path, degrees, seconds):
    from tools.run.arm_motion_trial import TrialContext, run_leg
    from test_unified_arm_adapter import make_adapter
    adapter, transport, clock = make_adapter(monkeypatch)
    adapter.prepare()
    # Use the same one-owner preparation hold followed by repeated finite
    # routes as the actual trial. All I/O goes to the in-memory MCU transport.
    adapter.start_current_pose_hold('pick-station')
    while adapter.state.value == 'active':
        clock.advance(.02)
        adapter.poll('pick-station')
    from so101_arm_bridge.stream_protocol_v2 import BimanualFeedbackSnapshotV2
    original_read = transport.get_feedback_snapshot
    def read():
        return BimanualFeedbackSnapshotV2(**vars(original_read()),request_sequence=1,sender_time_ms_echo=0)
    monkeypatch.setattr(transport,'get_feedback_snapshot',read)
    anchor, end = targets((0.,)*12, 1, degrees, (-2,)*12, (2,)*12)
    evidence = []
    hardware = SimpleNamespace(adapter=adapter, prepared=True, tick=lambda: None,
        camera=SimpleNamespace(check=lambda:None),
        motion_envelope=trial_envelope(anchor,end),
        evidence=SimpleNamespace(publish_mcu_snapshot=lambda *a,**kw:evidence.append((a,kw))),
        mobile=SimpleNamespace(status=lambda:None),lift=SimpleNamespace(status=lambda:None),
        guard=SimpleNamespace(update=lambda *a:None,require_fresh=lambda *a:None))
    ctx = TrialContext(hardware, tmp_path, dict(events=[],artifacts=[]), clock=clock, sleep=clock.advance)
    for seconds in (seconds,seconds*2):
        for start, finish in ((anchor,end),(end,anchor)):
            leg = dict(seconds=seconds, accepted_samples=[], feedback=[])
            points = leg_points(start,finish,seconds)
            run_leg(ctx, points, leg)
            assert len(leg['accepted_samples']) == len(points)
            assert tuple(transport.command) == tuple(round(v*1e6) for v in finish)
            assert summarize(leg,1)['maximum_error_rad'] < 2e-6
    assert transport.calls.count('arm') == 1
    assert transport.calls.count('enable') == 1
    assert not transport.stop_latched
    assert len(evidence)>100


@dataclass
class WireSample:
    apply_tick: int
    positions_urad: tuple


@pytest.mark.parametrize('failure', ['prepare', 'start', 'poll', 'cancel', 'close', None])
def test_lifecycle_keeps_cleanup_on_failure_without_retry(monkeypatch, tmp_path, capsys, failure):
    from tools.run import arm_motion_trial as trial
    monkeypatch.setattr(trial.sys.stdin, 'isatty', lambda: True)
    baseline = tmp_path/'baseline.json'
    baseline.write_text('{}')
    calls = []
    class Adapter:
        state = SimpleNamespace(value='ready')
        _route_samples = ()
        motion_limits = SimpleNamespace(minimum_urad=(-2000000,)*12, maximum_urad=(2000000,)*12)
        def start(self, owner, points, finite):
            calls.append('start')
            assert owner == 'pick-station' and finite
            if failure == 'start': raise RuntimeError('start failed')
            self._route_samples = tuple(WireSample(p.offset_ms, tuple(round(q*1e6) for q in p.positions_rad)) for p in points)
            self.state = SimpleNamespace(value='active')
        def poll(self, owner):
            calls.append('poll')
            if failure == 'poll': raise RuntimeError('poll failed')
            self.state = SimpleNamespace(value='ready')
    class Hardware:
        def __init__(self, options):
            assert options['lift_target_um'] == 0 and options['supported_empty_top']
            self.adapter = Adapter()
            self.anchor = SimpleNamespace(positions_urad=(0,)*12)
            self.prepared = False
            self.stop_confirmed = False
            self.stop = None
        def prepare(self, ctx):
            calls.append('prepare')
            if failure == 'prepare': raise RuntimeError('prepare failed')
            self.stop = object()
            if failure == 'cancel': ctx.cancelled = True
        def tick(self): pass
        def close(self):
            calls.append('close')
            if failure == 'close': raise RuntimeError('STOP unconfirmed')
            self.stop_confirmed = True
    # Isolate state-machine/cleanup verification from packet validation, which
    # remains in the real hardware service and its existing dedicated tests.
    def service(ctx):
        ctx.check()
        if ctx.leg is not None and ctx.hardware.adapter.state.value == 'active':
            ctx.remember_commands()
            ctx.hardware.adapter.poll('pick-station')
    monkeypatch.setattr(trial.TrialContext, 'service', service)
    root = __import__('pathlib').Path(__file__).resolve().parents[1]
    args = SimpleNamespace(supported_empty_top=True, clearance_confirmed=True,
        runtime_root=root, baseline=baseline, output=tmp_path/'report', joint='left_shoulder',
        degrees=1., seconds=2, repeats=1, port='fake')
    result = execute(args, hardware_factory=Hardware)
    report = json.loads((args.output/'trial.json').read_text())
    assert calls[-1] == 'close' and calls.count('prepare') == 1
    assert result == (0 if failure is None else 2)
    assert calls.count('start') == (0 if failure in ('prepare','cancel') else 4 if failure in (None,'close') else 1)
    if failure == 'close': assert report['status'] == 'STOP_UNCONFIRMED'
    if failure == 'cancel': assert report['status'] == 'CANCELLED'
    output = capsys.readouterr().out
    if failure == 'prepare':
        assert 'prepare failed' in output
        assert report['stop_session_created'] is False
        assert '시험용 STOP 세션 생성 전에' in output
    if failure == 'close':
        assert 'STOP unconfirmed' in output
        assert report['stop_session_created'] is True
