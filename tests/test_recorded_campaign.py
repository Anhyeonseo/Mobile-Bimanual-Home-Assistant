import copy
import json
from pathlib import Path

import numpy as np
import pytest

from home_robot_tasks.recorded_campaign import acknowledged_epochs, join_boundary, sample_odometry
from home_robot_tasks.s4_recorded_mapping import encoder_path, transform
from test_s4_base_preparation import teleop_capture

ROOT = Path(__file__).resolve().parents[1]


def test_failed_tail_preserves_only_real_acknowledgements_without_rewriting_source():
    records = teleop_capture()
    records[-1]['error'] = 'controller fault'
    records[-2]['reply']['fault'] = True
    original = copy.deepcopy(records)
    epochs, rejected = acknowledged_epochs(records)
    assert len(epochs) == 1 and rejected
    assert len(epochs[0]) == sum(r['event'] == 'reply' for r in records)-1
    assert records == original


def test_campaign_odometry_matches_existing_verified_path():
    records = teleop_capture()
    geometry = json.loads((ROOT/'config/home_assistant/s4_base.nominal.json').read_text())
    epochs, rejected = acknowledged_epochs(records)
    assert not rejected and len(epochs) == 1
    actual = sample_odometry(epochs[0], geometry)
    expected, _ = encoder_path(records, geometry)
    assert np.allclose(actual, expected)


def test_missing_ack_splits_epochs_instead_of_interpolating_gap():
    records = teleop_capture()
    replies = [i for i, row in enumerate(records) if row['event'] == 'reply']
    del records[replies[len(replies)//2]]
    epochs, rejected = acknowledged_epochs(records)
    assert len(epochs) == 2
    assert any('missing' in r['reason'] for r in rejected)
    assert epochs[0][-1]['sequence']+2 == epochs[1][0]['sequence']


def test_boundary_registration_direction_and_large_unknown_displacement():
    target = np.random.default_rng(11).uniform(-1.8, 1.8, (400, 2))
    expected = transform((.025, -.015, .015))
    source = (target-expected[:2, 2])@expected[:2, :2]
    actual, report = join_boundary(source, target)
    assert np.allclose(actual, expected, atol=1e-4)
    assert report['within_10cm_fraction'] > .99
    with pytest.raises(ValueError):
        join_boundary(source+10, target)


def test_wrong_firmware_and_regressing_clock_refused():
    records = teleop_capture()
    records[0]['firmware']['wheel_feedback'] = False
    with pytest.raises(ValueError):
        acknowledged_epochs(records)


def test_all_scans_remain_usable_without_any_wheel_feedback_including_recording_gap():
    import math
    from home_robot_tasks.recorded_campaign import sensor_odometry
    ranges = np.random.default_rng(15).uniform(.5, 2.5, 360).tolist()
    scans = [dict(observed_s=t, frame='lidar_a', count=360, ranges=ranges,
                  angle_min=0., angle_increment=math.tau/360, time_increment=-.00055)
             for t in (1., 1.2, 1.4, 2.6, 2.8)]
    rows, quality = sensor_odometry(scans, [], np.eye(3))
    assert len(rows) == len(scans)+1
    assert np.allclose(rows[:, 1:], 0., atol=1e-8)
    assert not any(q['encoder_covered'] for q in quality)
    assert sum('gap_s' in q for q in quality) == 1


def test_final_graph_export_maps_accelerated_ros_time_back_to_source_time():
    from tools.run.check_slam_replay import normalized_graph_poses
    report = dict(capture_window={'epochs':[{'replay_interval_s':[1.,20.]}]},
                  replay_clock_offset_s=99., replay_rate=4.)
    rows = [dict(replay_stamp=t,xy_yaw=[0.,0.,0.]) for t in (100.,101.,102.)]
    result = normalized_graph_poses('Karto loading log\n'+json.dumps(rows), report)
    assert [r['stamp']-report['replay_clock_offset_s'] for r in result] == [1.,5.,9.]
    assert [r['replay_stamp'] for r in result] == [100.,101.,102.]
    rows[1]['replay_stamp'] = rows[0]['replay_stamp']
    with pytest.raises(ValueError):normalized_graph_poses(json.dumps(rows), report)
    records = teleop_capture()
    records[-1]['host_monotonic_ns'] = -1
    with pytest.raises(ValueError):
        acknowledged_epochs(records)


def test_body_shadow_does_not_rotate_compress_or_mutate_scan():
    import math
    from home_robot_tasks.recorded_campaign import mask_body_shadow
    visibility = json.loads((ROOT/'config/home_assistant/lidar_visibility.review.json').read_text())
    scan = dict(frame='lidar_a', count=360, ranges=[1.]*360, angle_min=0.,
                angle_increment=math.pi/180, time_increment=-.00055)
    original = copy.deepcopy(scan)
    masked, count = mask_body_shadow(scan, visibility)
    assert scan == original and count == 91
    assert len(masked['ranges']) == 360 and masked['time_increment'] == scan['time_increment']
    assert masked['ranges'][0] is None and masked['ranges'][180] == 1.
    assert sum(v is not None for v in masked['ranges']) == 269
    scan['angle_min'] = .1
    with pytest.raises(ValueError):mask_body_shadow(scan, visibility)


def test_lidar_only_preparation_uses_all_chunks_without_camera_or_encoder_files(tmp_path):
    import math
    from home_robot_tasks.recorded_campaign import prepare_lidar_campaign, load_prepared, digest
    segments = []
    ranges = np.random.default_rng(25).uniform(.5,2.5,360).tolist()
    hashes = []
    for name in ('finished', 'faulted-recovery'):
        segment = tmp_path/name
        (segment/'capture').mkdir(parents=True)
        scans = [dict(frame='lidar_a', count=360, ranges=ranges, observed_s=t,
                      angle_min=0., angle_increment=math.pi/180, time_increment=-.00055)
                 for t in (5.,5.2,5.4)]
        path = segment/'capture/scans.json'
        path.write_text(json.dumps(scans))
        hashes.append(digest(path));segments.append(segment)
    output = tmp_path/'result'
    report = prepare_lidar_campaign(ROOT,segments,output)
    rows, scans, loaded = load_prepared(output)
    assert report['scans'] == len(scans) == 6 and report['rgbd_frames'] == 0
    assert not report['wheel_feedback_used'] and not (output/'rgbd').exists()
    assert np.allclose(rows[:,1:],0.,atol=1e-8)
    assert len(report['boundaries']) == 1
    assert hashes == [digest(s/'capture/scans.json') for s in segments]


def test_partial_view_lidar_recovers_known_translation_and_rotation_without_encoder():
    import math
    from home_robot_tasks.recorded_campaign import lidar_submap_odometry, mask_body_shadow
    visibility = json.loads((ROOT/'config/home_assistant/lidar_visibility.review.json').read_text())
    mount = transform((.151,-.0017,math.pi))
    expected = np.array([[.4*t,.2*t,.4*t] for t in np.linspace(0.,1.,41)])
    scans = []
    for i, p in enumerate(expected):
        sensor = transform(p)@mount
        ranges = []
        for a in np.arange(360)*math.pi/180:
            direction = sensor[:2,:2]@np.array([math.cos(a),math.sin(a)])
            origin = sensor[:2,2]
            candidates = [(wall-origin[axis])/direction[axis]
                for axis in (0,1) for wall in (-2.,2.)
                if abs(direction[axis])>1e-9 and (wall-origin[axis])/direction[axis]>0]
            ranges.append(min(candidates))
        scan = dict(frame='lidar_a',count=360,ranges=ranges,angle_min=0.,
            angle_increment=math.pi/180,time_increment=-.00055,observed_s=1+i*.2,source_segment='synthetic')
        scans.append(mask_body_shadow(scan,visibility)[0])
    actual, _ = lidar_submap_odometry(scans,mount)
    assert np.max(np.linalg.norm(actual[1:,1:3]-expected[:,:2],axis=1)) < .01
    assert np.max(abs(actual[1:,3]-expected[:,2])) < math.radians(.5)


def test_rgbd_retiming_uses_each_boot_and_retains_uncovered_frames():
    from home_robot_tasks.recorded_campaign import retime_rgbd_frame
    row=dict(file='000001.npz',color_time_s=10.02,depth_time_s=10.,capture_id='rgbd-1')
    first=retime_rgbd_frame(row,'boot-a',dict(id='a:0',time_offset_s=100.),[9.,11.])
    second=retime_rgbd_frame(row,'boot-b',dict(id='b:0',time_offset_s=500.),[11.,20.])
    assert first['color_time_s']==pytest.approx(110.02)
    assert second['color_time_s']==pytest.approx(510.02)
    assert first['pose_covered'] and not second['pose_covered']
    assert first['file']!=second['file'] and first['source_color_time_s']==row['color_time_s']
    assert row['file']=='000001.npz'


def test_rgbd_attachment_preserves_lidar_and_refuses_another_map(tmp_path):
    import math
    from home_robot_tasks.recorded_campaign import prepare_lidar_campaign,attach_rgbd_campaign,verify_rgbd_attachment,digest
    segment=tmp_path/'raw';capture=segment/'capture';(capture/'rgbd').mkdir(parents=True)
    scans=[dict(frame='lidar_a',count=360,ranges=np.random.default_rng(3).uniform(.5,2.5,360).tolist(),
                angle_min=0.,angle_increment=math.pi/180,time_increment=-.00055,observed_s=t)for t in [10.,10.2,10.4]]
    (capture/'scans.json').write_text(json.dumps(scans))
    (capture/'report.json').write_text(json.dumps(dict(passed=False,sensor_frames={'transforms':[]},scan_height_above_floor_m=.16)))
    (capture/'rgbd/metadata.json').write_text(json.dumps(dict(native_depth_intrinsics={},color_intrinsics={},depth_scale_m=.001,depth_to_color={})))
    original=capture/'rgbd/000000.npz';original.write_bytes(b'immutable-test-frame')
    frame=dict(file=original.name,sha256=digest(original),color_time_s=10.1,depth_time_s=10.1,capture_id='a')
    (capture/'rgbd/frames.jsonl').write_text(json.dumps(frame)+'\n')
    parent=tmp_path/'lidar';prepare_lidar_campaign(ROOT,[segment],parent)
    attached=tmp_path/'attached';r=attach_rgbd_campaign(parent,[segment],attached)
    assert r['rgbd_frames']==1 and r['sources'][0]['source_capture_passed'] is False
    assert digest(attached/'scans.json')==digest(parent/'scans.json')
    sr=dict(source_sha256={str(parent/n):digest(parent/n)for n in ('scans.json','report.json','trajectories.npz')})
    assert verify_rgbd_attachment(attached,sr,parent)['lidar_only']
    sr['source_sha256'][str(parent/'report.json')]='wrong-map'
    with pytest.raises(ValueError,match='different lidar preparation'):verify_rgbd_attachment(attached,sr,parent)
