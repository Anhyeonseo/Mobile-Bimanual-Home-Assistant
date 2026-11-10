from copy import deepcopy
import json
from pathlib import Path

import pytest

from home_robot_tasks.installed_jaw import InstalledLeftJaw, SECTION, INTERIOR_SECTION
from tools.diagnostics.calibrate_left_jaw_gap import measured_point, profile

ROOT=Path(__file__).resolve().parents[1]


def stock_conversion():
    from home_robot_tasks.installed_jaw import StockLeftJaw
    from tools.diagnostics.run_s3_stationary_lab import stock_jaw
    coordinates=json.loads((ROOT/'config/home_assistant/s2_coordinates.development.json').read_text())
    limits=json.loads((ROOT/'config/home_assistant/arm_ranges.development.json').read_text())['arms']['left']['gripper']
    return StockLeftJaw(stock_jaw(),coordinates,limits),coordinates


def test_stock_surface_clearance_catches_tip_below_plane_with_tcp_above():
    import numpy as np
    from scipy.spatial.transform import Rotation
    jaw,_=stock_conversion();stock=jaw.stock
    pose=np.eye(4);pose[:3,:3]=np.diag([1.,-1.,-1.]);plane=np.eye(4)
    raw=stock.plane_clearance(pose,.3,plane)
    low=min(v['clearance_m'] for v in raw.values())
    assert low<0
    pose[2,3]=-low/2
    crossed=stock.plane_clearance(pose,.3,plane)
    assert pose[2,3]>0 and min(v['clearance_m'] for v in crossed.values())<0
    pose[2,3]=-low+.004
    clear=stock.plane_clearance(pose,.3,plane)
    assert min(v['clearance_m'] for v in clear.values())==pytest.approx(.004)
    # Changing the shared reference must not change signed surface distance.
    frame=np.eye(4);frame[:3,:3]=Rotation.from_euler('xyz',[.1,.2,.3]).as_matrix();frame[:3,3]=[.2,-.1,.4]
    changed=stock.plane_clearance(frame@pose,.3,frame@plane)
    for finger in clear:assert changed[finger]['clearance_m']==pytest.approx(clear[finger]['clearance_m'])


def test_stock_conversion_uses_collision_model_section_and_rounds_outward():
    import numpy as np
    from home_robot_tasks.s2_geometry import model_angle_from_raw
    jaw,coordinates=stock_conversion()
    joint=coordinates['arms']['left']['alignment']['gripper']
    for raw in range(jaw.raw_min,jaw.raw_max+1,7):
        angle=model_angle_from_raw(raw,joint,'gripper')
        expected=float(np.interp(angle,jaw.stock.angles,jaw.stock.gaps)*1000)
        assert jaw.gap_mm(raw)==pytest.approx(expected,abs=1e-10)
        assert jaw.raw_at_least(expected)==raw
    for gap in np.linspace(jaw.gap_mm(jaw.raw_min),jaw.gap_mm(jaw.raw_max),101):
        raw=jaw.raw_at_least(float(gap))
        assert jaw.gap_mm(raw)>=gap
        if raw>jaw.raw_min:assert jaw.gap_mm(raw-1)<gap


def test_stock_contact_interval_does_not_borrow_measured_front_allowance():
    jaw,_=stock_conversion()
    plan=dict(status='BOUNDED_CLOSE',entry_gap_m=.049,target_gap_m=.035,
              upper_contact_gap_m=.039,visible_span_m=.037)
    preview=jaw.preview(plan);envelope=jaw.contact_envelope(plan)
    assert preview['status']=='NOMINAL_STL_GAP_PREVIEW'
    assert preview['physically_measured'] is False and not preview['hardware_authorized']
    assert envelope['gap_check_allowance_mm']==0
    assert jaw.gap_mm(envelope['upper_contact_raw'])<=39
    assert jaw.gap_mm(envelope['upper_contact_raw']+1)>39
    # This is a comparison of declared models, not a physical acceptance test.
    assert jaw.gap_mm(2222)==pytest.approx(35.89956,abs=.01)


@pytest.mark.parametrize('gap',[float('nan'),float('inf'),-1,1000])
def test_stock_conversion_never_extrapolates(gap):
    jaw,_=stock_conversion()
    with pytest.raises(ValueError):jaw.raw_at_least(gap)


@pytest.mark.parametrize('upper',[float('nan'),.034,.050])
def test_stock_contact_interval_cannot_extend_beyond_bounded_close(upper):
    jaw,_=stock_conversion()
    plan=dict(status='BOUNDED_CLOSE',entry_gap_m=.049,target_gap_m=.035,
              upper_contact_gap_m=upper)
    with pytest.raises(ValueError,match='invalid nominal contact envelope'):
        jaw.contact_envelope(plan)


def point(raw, gap, key):
    return dict(section=SECTION,raw=raw,raw_min=raw-1,raw_max=raw+1,gap_mm=gap,
                release='bench-test',captures=[{'sha256':key}])


def synthetic_jaw(coordinates):
    """Explicit fictitious pad data for algorithms, never physical evidence."""
    samples = [point(1950, 1, 'a'), point(2250, 24, 'b'), point(2650, 60, 'c')]
    return InstalledLeftJaw(profile(samples, point(2450, 42, 'd'), coordinates), coordinates)


def example():
    coordinates=json.loads((ROOT/'config/home_assistant/s2_coordinates.development.json').read_text())
    samples=[point(1950,1,'a'),point(2250,24,'b'),point(2650,60,'c')]
    return samples,coordinates


def test_inverse_uses_measured_control_sign_and_does_not_extrapolate():
    samples,coordinates=example()
    check=point(2450,43,'d')
    record=profile(samples,check,coordinates)
    assert record['check']['absolute_error_mm']==pytest.approx(1.)
    jaw=InstalledLeftJaw(record,coordinates)
    assert jaw.gap_mm(2450)==pytest.approx(42.)
    assert jaw.raw_at_least(42.1)==2452
    assert jaw.preview({'status':'BOUNDED_CLOSE','entry_gap_m':.058,'min_search_gap_m':.033})['hardware_authorized'] is False
    assert jaw.preview({'status':'BOUNDED_CLOSE','entry_gap_m':.061,'min_search_gap_m':.033})['status']=='OUTSIDE_MEASURED_PAD_RANGE'


def test_fit_rejects_reused_capture_and_unusable_registration():
    samples,coordinates=example()
    with pytest.raises(ValueError,match='independent'):
        profile(samples,point(2450,42,'a'),coordinates)
    record=profile(samples,point(2450,50,'d'),coordinates)
    assert record['status']=='CHECK_FAILED'
    with pytest.raises(ValueError):
        InstalledLeftJaw(record,coordinates)
    record=profile(samples,point(2450,42,'d'),coordinates)
    with pytest.raises(ValueError):
        InstalledLeftJaw(record,dict(coordinates,calibration_id='other'))


@pytest.mark.parametrize('problem',['moving','writes','replay','bad_status','torque','span'])
def test_point_requires_multiple_stationary_readings_in_one_pose(tmp_path,problem):
    row=dict(firmware='left-grip-bench-v3',active=0,locked=0,off_verified=1,
             torque=0,sample_valid=1,status_raw=0,sequence=0,speed_raw=0,
             reason='START_POSE_OR_TORQUE',writes=60,position=1953)
    report=dict(mode='inspect',release='bench-test',observed_at='2026-11-07T01:00:00Z',inspections=[
        dict(identity=dict(row),before=dict(row,ticket=i,tick_ms=i+30)) for i in range(1,5)])
    path=tmp_path/'reads.json'
    path.write_text(json.dumps(report))
    assert measured_point([path],0)['raw']==1953
    bad=deepcopy(report)
    bad['inspections'][-1]['before'].update({'moving':{'speed_raw':8},'writes':{'writes':62},
        'replay':{'ticket':3},'bad_status':{'status_raw':8},'torque':{'torque':1},
        'span':{'position':1961}}[problem])
    path.write_text(json.dumps(bad))
    with pytest.raises(ValueError):
        measured_point([path],0)


def test_inboard_gap_cannot_be_used_to_calibrate_the_front_faces():
    samples,coordinates=example()
    check=point(2450,42,'d')
    with pytest.raises(ValueError,match='same front'):
        profile(samples,dict(check,section=INTERIOR_SECTION),coordinates)
    samples[0]['section']=INTERIOR_SECTION
    with pytest.raises(ValueError,match='mix jaw sections'):
        profile(samples,check,coordinates)


def test_step_feedback_uses_integer_bounds_without_closing_past_the_limit():
    samples,coordinates=example()
    jaw=InstalledLeftJaw(profile(samples,point(2450,42,'d'),coordinates),coordinates)
    plan=dict(status='BOUNDED_CLOSE',entry_gap_m=.05707,min_search_gap_m=.03328,upper_contact_gap_m=.03728)
    bounds=jaw.preview(plan)
    entry,floor=bounds['entry_raw'],bounds['lower_search_raw']
    action,goal=jaw.step_preview(plan,entry,'free')
    assert action=='CLOSE_STEP' and floor<=goal<entry
    assert jaw.step_preview(plan,entry,'rising')==('WAIT_AT_CURRENT_GAP',None)
    assert jaw.step_preview(plan,entry,'persistent')==('UNEXPECTED_RESISTANCE',None)
    assert jaw.step_preview(plan,floor,'free')==('STOP_NO_CONTACT',None)
    assert jaw.step_preview(plan,floor-1,'free')==('STOP_OUTSIDE_PLAN',None)


def test_independent_gap_error_changes_contact_classification_only():
    config=ROOT/'config/home_assistant/historical/pre-home-20261110'
    record=json.loads((config/'left_pad_gap.observed.json').read_text())
    coordinates=json.loads((config/'s2_coordinates.development.json').read_text())
    # A forged cached residual must not widen the allowance.
    record['check']['absolute_error_mm']=20
    jaw=InstalledLeftJaw(record,coordinates)
    plan=dict(status='BOUNDED_CLOSE',entry_gap_m=.05707,target_gap_m=.0332786947,
              upper_contact_gap_m=.0372786947)
    before=jaw.preview(plan)
    envelope=jaw.contact_envelope(plan)
    assert envelope['gap_check_allowance_mm']==pytest.approx(.4)
    assert envelope['upper_contact_raw']==2225
    assert before==jaw.preview(plan) and before['lower_search_raw']==2194
    assert jaw.step_preview(plan,2224,'persistent')==('CONTACT_CANDIDATE',None)
    assert jaw.step_preview(plan,2226,'persistent')==('UNEXPECTED_RESISTANCE',None)
    assert jaw.gap_mm(envelope['upper_contact_raw'])<=envelope['upper_contact_gap_mm']
    clipped=jaw.contact_envelope(dict(plan,entry_gap_m=.0374))
    assert clipped['upper_contact_gap_mm']==pytest.approx(37.4)
