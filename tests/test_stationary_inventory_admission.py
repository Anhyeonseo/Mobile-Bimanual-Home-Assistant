"""Reject an out-of-range resting pose before continuous reads or motor writes."""
import json
from pathlib import Path
import pytest
from so101_arm_bridge.stationary_runtime import verify_inventory
from so101_arm_bridge.limit_manifest import JOINTS

CONFIG=Path(__file__).resolve().parents[1]/'config/home_assistant'


class Inventory:
    def __init__(self,ranges,bad=None):self.ranges=ranges;self.bad=bad;self.calls=[]
    def read_device_inventory(self,bus,motor):
        self.calls.append((bus,motor))
        if motor>6:return {'complete':True,'command':{'goal_speed_raw':0}}
        side=('left','right')[bus]
        raw=2048
        if self.bad==(bus,motor):raw=self.ranges['arms'][side][JOINTS[motor-1]]['maximum_unwrapped_raw']+1
        return {'complete':True,'offset_and_mode':{'offset_raw':self.ranges['calibration'][side]['offsets_raw'][motor-1],'operating_mode':0},'command':{'torque_enabled_raw':0},'feedback':{'position_raw':raw}}


@pytest.mark.parametrize('bad',[(0,3),(1,3)])
def test_elbow_outside_range_rejected_by_individual_reads(bad):
    ranges=json.loads((CONFIG/'arm_ranges.development.json').read_text())
    transport=Inventory(ranges,bad)
    with pytest.raises(ValueError,match='idle reader not started'):verify_inventory(transport,ranges)
    assert transport.calls[-1]==bad


def test_in_range_inventory_preserves_all_platform_checks():
    ranges=json.loads((CONFIG/'arm_ranges.development.json').read_text())
    transport=Inventory(ranges)
    assert len(verify_inventory(transport,ranges))==16
    assert transport.calls[-4:]==[(0,i) for i in range(7,11)]


@pytest.mark.parametrize('defect',[None,'drift','speed','torque','missing','incomplete'])
def test_same_pose_hold_requires_observed_unpowered_stability(defect):
    from copy import deepcopy
    from tools.run.verify_ready_hold import check_resting_samples
    rows=[dict(bus=side,requested_id=i,complete=True,command={'torque_enabled_raw':0},
               feedback={'position_raw':2048,'speed_raw':0})
          for side in ('left','right') for i in range(1,7)]
    samples=[deepcopy(rows) for _ in range(3)]
    if defect=='drift':samples[-1][2]['feedback']['position_raw']+=3
    elif defect=='speed':samples[-1][2]['feedback']['speed_raw']=1
    elif defect=='torque':samples[-1][2]['command']['torque_enabled_raw']=1
    elif defect=='missing':samples[-1].pop()
    elif defect=='incomplete':samples[-1][2]['complete']=False
    if defect:
        with pytest.raises(ValueError):check_resting_samples(samples)
    else:check_resting_samples(samples)


def round_trip_fixture():
    import math
    from dataclasses import asdict
    from home_robot_tasks.carry_trajectory import MotionBounds,retime
    q=[0.]*12;target=q.copy();target[2]=math.radians(2)
    points,_=retime([q,target,q],MotionBounds((-4.,)*12,(4.,)*12,(.04,)*12,(.08,)*12,(.4,)*12),dwell_s=1)
    return dict(kind='empty_left_arm_round_trip',schema_version=1,status='NATIVE_GEOMETRY_PASSED',
                hypothetical_lift=False,range_sha256='range',moving_joint_index=2,fixed_lift_m=.5,platform_reference_raw=[0]*4,
                points=[asdict(p) for p in points],start_rad=q,checked_states=len(points))


@pytest.mark.parametrize('defect',[None,'hash','range','right','jaw','too_far','no_return','time','unchecked','hypothetical'])
def test_short_trial_rejects_unreviewed_or_expanded_motion(tmp_path,defect):
    import hashlib
    from tools.run.verify_ready_hold import load_round_trip
    r=round_trip_fixture();middle=len(r['points'])//3
    # JSON representation is what the runtime consumes.
    r=json.loads(json.dumps(r))
    if defect=='range':r['range_sha256']='old'
    elif defect=='right':r['points'][middle]['positions_rad'][8]=.01
    elif defect=='jaw':r['points'][middle]['positions_rad'][5]=.01
    elif defect=='too_far':r['points'][middle]['positions_rad'][2]=.04
    elif defect=='no_return':r['points'][-1]['positions_rad'][2]=.01
    elif defect=='time':r['points'][middle]['offset_ms']=0
    elif defect=='unchecked':r['checked_states']-=1
    elif defect=='hypothetical':r['hypothetical_lift']=True
    path=tmp_path/'round-trip.json';path.write_text(json.dumps(r))
    checksum=hashlib.sha256(path.read_bytes()).hexdigest()
    if defect=='hash':checksum='0'*64
    if defect:
        with pytest.raises(ValueError):load_round_trip(path,checksum,'range')
    else:assert load_round_trip(path,checksum,'range')['moving_joint_index']==2


@pytest.mark.parametrize('fault',[None,'no_follow','platform_moved'])
def test_existing_resident_runs_bounded_return_with_independent_measurements(monkeypatch,fault):
    from types import SimpleNamespace as NS
    from dataclasses import is_dataclass,asdict
    from tools.run import verify_ready_hold as h
    from tools.diagnostics.carry_stream_sim import SimClock,SimTransport
    import so101_arm_bridge.bimanual_stream_adapter as bridge
    from so101_arm_bridge.stationary_pick_adapter import StationaryPickAdapter
    from home_robot_tasks.carry_live_inputs import FixedPlatformGuard
    clock=SimClock();mock_time=NS(monotonic=clock,sleep=clock.advance)
    monkeypatch.setattr(h,'time',mock_time);monkeypatch.setattr(bridge,'time',mock_time)
    monkeypatch.setattr(h,'asdict',lambda x:asdict(x) if is_dataclass(x) else vars(x).copy())
    class Transport(SimTransport):
        def _update(self):
            if fault!='no_follow':super()._update()
        def stationary_platform(self,operation=0):
            raw=[0,0,0,0]
            if fault=='platform_moved' and self.horizon:raw[0]=10
            return dict(active=True,present_mask=15,firmware_tick_ms=self.tick,sequence=self.tick,
                        positions_raw=raw,sample_age_ms=[0]*4)
    class Adapter(StationaryPickAdapter):
        firmware_version=bridge.F8_FIRMWARE_VERSION;calibration_hash=bridge.CALIBRATION_HASH
        def __init__(self,t,limits):
            bridge.ResidentBimanualStreamAdapter.__init__(self,t,limits,motion_authorized=True)
            self._encoder_directions=(1,)*12
    t=Transport(clock,(0.,)*12)
    limits=bridge.OperationalLimits(bridge.CANONICAL_JOINT_NAMES,(-4000000,)*12,(4000000,)*12)
    a=Adapter(t,limits);a.prepare();t.arm(a.calibration_hash);t.enable();a._armed=True
    report=dict(before=[dict(requested_id=i,feedback={'position_raw':0}) for i in range(7,11)])
    s=NS(FixedPlatformGuard=FixedPlatformGuard,camera_profile=lambda:dict(lift={'meters_per_count':-.00002}))
    if fault:
        with pytest.raises(ValueError,match='below 1 degree' if fault=='no_follow' else 'moved'):
            h.run_round_trip(s,t,a,round_trip_fixture(),report,lambda:None)
    else:
        h.run_round_trip(s,t,a,round_trip_fixture(),report,lambda:None)
        assert report['round_trip_result']['return_error_rad']==0
        assert report['round_trip_result']['measured_peak_rad']>.034
        assert a.state.value=='ready' and t.enabled
