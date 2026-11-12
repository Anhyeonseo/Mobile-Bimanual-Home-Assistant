import copy
import json
import math
from pathlib import Path
import numpy as np
import pytest
from home_robot_tasks.s4_recorded_mapping import align, encoder_path, interpolate, pose, scan_points, transform
from test_s4_base_preparation import teleop_capture

ROOT=Path(__file__).resolve().parents[1]


def test_registration_direction_with_outliers():
    rng=np.random.default_rng(14)
    target=rng.uniform(-1,1,(300,2))
    expected=transform((.015,-.012,.008))
    source=(target-expected[:2,2])@expected[:2,:2]
    source=np.vstack((source,[[5.,4.],[7.,5.]]))
    matrix,quality=align(source,target)
    assert np.allclose(matrix,expected,atol=1e-5)
    assert quality['rmse_m']<1e-5
    with pytest.raises(ValueError,match='overlap'):align(source+100,target)


def test_pose_interpolation_refuses_unobserved_times_and_nonfinite_rows():
    rows=np.array([[1,0,0,3.1],[2,.2,-.1,3.2]])
    assert np.allclose(interpolate(rows,[1.5]),[[.1,-.05,3.15]])
    for t in ([.99],[2.01],[float('nan')]):
        with pytest.raises(ValueError):interpolate(rows,t)
    with pytest.raises(ValueError):interpolate(rows[::-1],[1.5])


def test_negative_ray_timing_deskews_each_ray_into_reference_base_frame():
    scan=dict(ranges=[1.,1.],angle_min=0.,angle_increment=math.pi/2,
              observed_s=2.,time_increment=-.1)
    odom=np.array([[1.,0,0,0],[2.,1.,0,0]])
    points=scan_points(scan,transform((.15,0,0)),odom)
    assert np.allclose(points,[[1.15,0],[.05,1]])
    assert np.allclose(scan_points(scan,transform((.15,0,0))),[[1.15,0],[.15,1]])


def test_encoder_uses_positions_even_when_reported_speed_has_stationary_bias():
    geometry=json.loads((ROOT/'config/home_assistant/s4_base.nominal.json').read_text())
    records=teleop_capture();biased=copy.deepcopy(records)
    for row in biased:
        if row['event']=='reply':
            for wheel in row['reply']['wheel_feedback']:wheel['speed_raw']=50
    a,_=encoder_path(records,geometry);b,_=encoder_path(biased,geometry)
    assert np.allclose(a,b)
    assert abs(a[-1,3])>0
    assert np.max(np.abs(np.diff(a[:,3])))<.02
    bad=copy.deepcopy(records);bad[-1]['error']='lost hold'
    with pytest.raises(ValueError):encoder_path(bad,geometry)


def test_localization_comparison_handles_angle_wrap_and_clock_offsets():
    from home_robot_tasks.s4_recorded_mapping import localization_comparison
    reference=[dict(stamp=100+t,xy_yaw=[.1*t,0.,3.1+.02*t]) for t in (1,2,3,4)]
    actual=[dict(stamp=200+t,xy_yaw=[.1*t+.01,0.,3.1+.02*t-2*math.pi]) for t in (1,2,3,4)]
    result=localization_comparison(actual,200,reference,100)
    assert result['tracking_sanity_passed']
    assert result['final_slam_difference_m']==pytest.approx(.01)
    assert result['final_slam_difference_deg']==pytest.approx(0,abs=1e-10)
    assert not result['physical_localization_qualified']
    assert not result['encoder_agreement_required']
    for row in actual:row['xy_yaw'][0]+=.5
    assert not localization_comparison(actual,200,reference,100)['tracking_sanity_passed']
    with pytest.raises(ValueError,match='shared'):localization_comparison(actual,300,reference,100)


def test_new_capture_localization_scores_fixed_map_without_slam_time_overlap():
    from home_robot_tasks.s4_recorded_mapping import localization_map_review
    grid=np.zeros((101,101),dtype=int);angles=np.arange(360)*math.tau/360
    endpoints=np.column_stack((np.cos(angles),np.sin(angles)))
    cells=np.floor((endpoints+2.525)/.05).astype(int);grid[cells[:,1],cells[:,0]]=100
    saved=dict(width=101,height=101,resolution=.05,origin=[-2.525,-2.525],data=grid.ravel().tolist())
    odom=np.array([[t,0.,0.,0.] for t in np.linspace(0,30,301)])
    poses=[dict(stamp=100+t,xy_yaw=[0.,0.,0.]) for t in range(1,30)]
    scans=[dict(observed_s=float(t),time_increment=-.0005,ranges=[1.]*360,
                angle_min=0.,angle_increment=math.tau/360) for t in np.linspace(1,29,60)]
    result=localization_map_review(poses,100,scans,odom,np.eye(3),saved)
    assert result['tracking_sanity_passed']
    assert result['after_10s_scan_to_map']['p90_m']<.04
    assert not result['physical_localization_qualified']
    assert not result['measured_return_error_available']
    assert not result['encoder_agreement_required']
    for row in poses:row['xy_yaw'][0]=1.
    assert not localization_map_review(poses,100,scans,odom,np.eye(3),saved)['tracking_sanity_passed']
    with pytest.raises(ValueError,match='coverage'):
        localization_map_review(poses,100,scans[:3],odom,np.eye(3),saved)


def test_initial_scan_map_search_recovers_distinct_pose_without_future_scans():
    from home_robot_tasks.s4_recorded_mapping import map_initial_pose_candidates
    rng=np.random.default_rng(31);angles=np.arange(360)*math.tau/360
    ranges=rng.uniform(.5,1.5,360)
    scan=dict(observed_s=1.,time_increment=-.0005,ranges=ranges.tolist(),angle_min=0.,angle_increment=math.tau/360)
    odom=np.array([[0.,0,0,0],[2.,0,0,0]])
    expected=np.array([.3,-.2,.5]);T=transform(expected)
    points=np.column_stack((ranges*np.cos(angles),ranges*np.sin(angles)))@T[:2,:2].T+T[:2,2]
    grid=np.zeros((100,100),dtype=int);cells=np.floor((points+2.5)/.05).astype(int);grid[cells[:,1],cells[:,0]]=100
    saved=dict(width=100,height=100,resolution=.05,origin=[-2.5,-2.5],data=grid.ravel().tolist())
    result=map_initial_pose_candidates(scan,odom,np.eye(3),saved)
    assert np.allclose(result['candidates'][0]['xy_yaw'],expected,atol=.04)
    assert not result['later_scans_used']
    assert not result['automatic_motion_authorized']
    saved['data']=[0]*10000
    with pytest.raises(ValueError,match='occupied'):map_initial_pose_candidates(scan,odom,np.eye(3),saved)


@pytest.mark.parametrize('failure',['implicit','during_drive','after_drive','camera','timing','tf','cleanup','mismatch','geometry','none'])
def test_offline_startup_gap_exception_never_accepts_other_failures(failure):
    from home_robot_tasks.s4_recorded_mapping import qualify_replay_capture
    records=teleop_capture();odom=np.array([[10.,0,0,0],[13.,0,0,0]])
    times=[1.,1.6]+list(np.arange(9.,13.3,.2))
    scans=[dict(observed_s=t,time_increment=-.00055,frame='lidar_a',count=360) for t in times]
    # The warm-up-to-drive interval is also an explicitly retained pre-drive gap.
    report=dict(mode='real_sensor',passed=False,teleop_finished=records[-1],gaps_over_035=2,
                receipt_timing=dict(passed=True),health=dict(scan_ready=True),driver_exit_code=0,
                tf_lookup_checks=[dict(maximum_matrix_error=0.)])
    if failure=='during_drive':scans=[s for s in scans if not 11.<s['observed_s']<11.5]
    if failure=='after_drive':scans.append(dict(scans[-1],observed_s=15.))
    if failure=='camera':report['rgbd']=dict(frames=10,minimum_depth_gap_s=.03,maximum_depth_gap_s=.4)
    if failure=='timing':report['receipt_timing']['passed']=False
    if failure=='tf':report['tf_lookup_checks'][0]['maximum_matrix_error']=.01
    if failure=='cleanup':report['camera_cleanup_error']='camera error'
    if failure=='mismatch':report['teleop_finished']={}
    if failure=='geometry':scans[4]['frame']='wrong'
    def run():return qualify_replay_capture(report,records,scans,odom,allow_startup_scan_gaps=failure!='implicit')
    if failure!='none':
        with pytest.raises(ValueError):run()
    else:
        selected,result=run()
        assert len(selected)>10 and result['maximum_selected_scan_gap_s']<.21
        assert not result['source_capture_passed'] and not result['whole_capture_qualified']
        assert len(result['excluded_pre_drive_gaps'])==2 and report['passed'] is False
