"""Nominal integration and adversarial stream tests; no physical acceptance evidence."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import numpy as np
import pytest
from home_robot_tasks import s2_development as dev, s2_commissioning as s2

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('independent_s2_fk',ROOT/'tests/test_s2_commissioning.py')
independent=importlib.util.module_from_spec(spec);spec.loader.exec_module(independent)

@pytest.fixture
def setup():
    g=s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
    p=dev.build(ROOT)
    raw={side:deepcopy(p['arms'][side]['reference_raw']) for side in s2.ARMS}
    evidence={'calibration_id':p['calibration_id'],'session_id':'test-session','controller_boot_id':'test-boot',
        'origin_id':'test-bottom','arm_anchor_id':'test-arm-anchor','clock_id':'test-clock',
        'lift_bottom_confirmed':True,'arm_turns_anchored':True,'lift_origin_raw_unwrapped':2000,
        'arm_raw_unwrapped':raw,'established_s':100.}
    snap={**{k:evidence[k] for k in dev.ObservationSession.KEYS},
        **{k:p[k] for k in ('calibration_id','assembly_revision','camera_device','source_frame')},
        'sequence':1,'capture_id':'capture-1','raw_encoding':'continuous_unwrapped_counts',
        'camera_time_s':100.1,'lift_time_s':100.1,'received_s':100.12,
        'joint_times_s':{side:dict.fromkeys(s2.JOINTS,100.1) for side in s2.ARMS},
        'lift_raw_unwrapped':2000,'raw':raw,'camera_xyz_m':[.1,.05,.6]}
    return g,p,evidence,snap


def test_nominal_adapter_overlays_common_mesh_and_first_joint(setup):
    g,p,_,_=setup
    import xml.etree.ElementTree as ET
    urdf=ET.parse(ROOT/'config/home_assistant/official_sources/so101_new_calib.urdf')
    v=next(v for v in urdf.findall('./link[@name="base_link"]/visual') if v.find('geometry/mesh').get('filename').endswith('/base_so101_v2.stl'))
    model_from_mesh=s2.origin({'origin':dict(v.find('origin').attrib)})
    js={j['name']:j for j in g['mobile_reference_joints']}
    mounts=json.loads((ROOT/'config/home_assistant/arm_mounts.nominal.json').read_text())
    for side in s2.ARMS:
        adapter=np.array(mounts['arms'][side]['cad_arm_from_model_root'])
        np.testing.assert_allclose(adapter@model_from_mesh,np.eye(4),atol=1e-12)
        assert mounts['arms'][side]['shared_vertex_count']>=1900
        assert mounts['arms'][side]['first_joint_center_distance_m']<.001
        expected=np.array(p['lift']['platform_at_lift_zero'])@s2.origin(js[side+'_base'])
        np.testing.assert_allclose(np.array(p['arms'][side]['base_from_arm_at_lift_zero'])@model_from_mesh,expected,atol=1e-12)


def test_generated_urdf_matches_independent_tree_and_floor(setup):
    g,p,_,snap=setup
    xml=dev.export_urdf(p,g)
    rng=np.random.default_rng(39)
    for h in [0,.25,.61,p['lift']['travel_m']]:
        raw={side:{j:snap['raw'][side][j]+float(rng.uniform(-200,200)) for j in s2.JOINTS} for side in s2.ARMS}
        result=dev.evaluate(p,g,raw,h,camera_xyz_m=snap['camera_xyz_m'])
        tf=independent.independent_fk(xml,result['model_joint_positions_si'])
        for side in s2.ARMS:
            np.testing.assert_allclose(tf[side+'_tcp'],result['base_from_tcp'][side],atol=1e-12)
        np.testing.assert_allclose(tf[p['source_frame']],p['base_from_optical'],atol=1e-12)
        assert tf['base_footprint'][2,3]==pytest.approx(-.0702)
        platform=np.array(p['lift']['platform_at_lift_zero']);platform[:3,3]+=np.array(p['lift']['axis_base'])*h
        height=(platform@[*p['lift']['plate_local_point_m'],1])[2]-p['ground_z_m']
        assert height==pytest.approx(.2555906229597231+h*p['lift']['axis_base'][2])


def test_reference_and_unwrapped_count_direction_no_shortest_path(setup):
    g,p,_,snap=setup
    for side in s2.ARMS:
        a=p['arms'][side]
        q=s2.model_angles(a['reference_raw'],a['alignment'])
        assert q==pytest.approx(a['reference_model_rad'])
        raw=deepcopy(a['reference_raw']);raw['base']=4095
        before=s2.model_angles(raw,a['alignment'])['base']
        raw['base']=4096
        assert s2.model_angles(raw,a['alignment'])['base']-before==pytest.approx(2*np.pi/4096)
        raw['base']=0
        assert abs(s2.model_angles(raw,a['alignment'])['base']-before)>6
    p['arms']['left']['base_from_arm_at_lift_zero'][0][3]+=.01
    with pytest.raises(ValueError): dev.export_urdf(p,g)


def test_development_never_passes_strict_acceptance(setup):
    g,p,_,_=setup
    for allow in [False,True]:
        with pytest.raises(ValueError): s2.export_urdf(p,g,allow_simulation=allow)
    p['coordinate_use_allowed']=True
    p['calibration_id']='s2-dev-'+s2.digest({k:v for k,v in p.items() if k!='calibration_id'})[:24]
    with pytest.raises(ValueError):dev.validate(p,g)


def test_operator_camera_height_shift_is_base_z_only_and_applied_once(setup):
    g,p,_,_=setup
    installation=json.loads((ROOT/'config/home_assistant/installation.development.json').read_text())
    choice=installation['d415']['development_transform_choice']
    nominal=s2.rigid(choice['base_from_optical']);adjusted=dev.camera_transform(choice)
    assert choice['operator_height_adjustment']['offset_m']==-.0091
    np.testing.assert_allclose(adjusted[:3,:3],nominal[:3,:3],atol=0)
    for point in ([0,0,.4],[.1,-.1,.6],[-.2,.2,1.]):
        delta=adjusted@np.r_[point,1]-nominal@np.r_[point,1]
        np.testing.assert_allclose(delta,[0,0,-.0091,0],atol=1e-12)
    np.testing.assert_allclose(p['base_from_optical'],adjusted,atol=1e-12)
    np.testing.assert_allclose(dev.build(ROOT)['base_from_optical'],adjusted,atol=1e-12)
    assert not p['strict_commissioning_passed'] and not p['camera_height_adjustment']['measured_extrinsic']
    bad=deepcopy(p);bad['base_from_optical'][2][3]-=.0091
    bad['calibration_id']='s2-dev-'+s2.digest({k:v for k,v in bad.items() if k!='calibration_id'})[:24]
    with pytest.raises(ValueError,match='height adjustment differs'):dev.validate(bad,g)


@pytest.mark.parametrize('fault',['unconfirmed','wrong_axis','oversize'])
def test_camera_height_adjustment_requires_bounded_explicit_evidence(fault):
    choice=deepcopy(json.loads((ROOT/'config/home_assistant/installation.development.json').read_text())['d415']['development_transform_choice'])
    adjustment=choice['operator_height_adjustment']
    if fault=='unconfirmed':adjustment['operator_confirmed']=False
    if fault=='wrong_axis':adjustment['axis']='x'
    if fault=='oversize':adjustment['offset_m']=-.1
    with pytest.raises(ValueError):dev.camera_transform(choice)


def next_snapshot(snap):
    s=deepcopy(snap);s['sequence']+=1;s['capture_id']='capture-'+str(s['sequence'])
    for k in ['camera_time_s','lift_time_s','received_s']:s[k]+=.05
    for side in s2.ARMS:
        for j in s2.JOINTS:s['joint_times_s'][side][j]+=.05
    s['lift_raw_unwrapped']-=10
    return s


def test_session_valid_stream_preserves_acquisition_time(setup):
    g,p,e,s=setup
    session=dev.ObservationSession(p,g,e)
    out=session.ingest(s,now_s=s['received_s'],clock_id=e['clock_id'])
    n=next_snapshot(s)
    out2=session.ingest(n,now_s=n['received_s'],clock_id=e['clock_id'])
    assert out['observed_s']==s['camera_time_s']
    assert out2['model_joint_positions_si']['s2_lift_joint']==pytest.approx(10*.084/4096)
    assert out2['motion_authorized'] is False
    session.invalidate()
    with pytest.raises(ValueError,match='session invalid'):session.ingest(next_snapshot(n),now_s=100.22,clock_id=e['clock_id'])

@pytest.mark.parametrize('failure',[
    'reset','origin','clock','anchor','profile','device','modulo','future','stale','skew','replay',
    'sequence','gap','repeated_motor','jump','nan','height','before_reference','first_gap','first_jump','caller_clock'])
def test_bad_snapshot_latches_invalid_session(setup,failure):
    g,p,e,s=setup
    session=dev.ObservationSession(p,g,e)
    if failure not in ['first_gap','first_jump']:
        session.ingest(s,now_s=s['received_s'],clock_id=e['clock_id'])
        n=next_snapshot(s)
    else:n=deepcopy(s)
    clock=e['clock_id']
    if failure=='reset':n['controller_boot_id']='new'
    if failure=='origin':n['origin_id']='new'
    if failure=='clock':n['clock_id']='new'
    if failure=='anchor':n['arm_anchor_id']='new'
    if failure=='profile':n['calibration_id']='new'
    if failure=='device':n['camera_device']={}
    if failure=='modulo':n['raw_encoding']='modulo_4096'
    if failure=='future':n['camera_time_s']=1000
    if failure=='stale':n['camera_time_s']=90
    if failure=='skew':n['camera_time_s']-=.06
    if failure=='replay':n['capture_id']=s['capture_id']
    if failure=='sequence':n['sequence']=s['sequence']
    if failure in ['gap','first_gap']:
        n['camera_time_s']+=1;n['lift_time_s']+=1;n['received_s']+=1
        for side in s2.ARMS:n['joint_times_s'][side]=dict.fromkeys(s2.JOINTS,n['lift_time_s'])
    if failure=='repeated_motor':n['joint_times_s']['left']['base']=s['joint_times_s']['left']['base']
    if failure in ['jump','first_jump']:n['raw']['left']['base']+=4096
    if failure=='nan':n['raw']['right']['shoulder']=float('nan')
    if failure=='height':n['lift_raw_unwrapped']+=10000
    if failure=='before_reference':n['camera_time_s']=99.99
    if failure=='caller_clock':clock='wall-clock'
    with pytest.raises((ValueError,KeyError)):session.ingest(n,now_s=n['received_s'],clock_id=clock)
    assert session.valid is False
    with pytest.raises(ValueError,match='session invalid'):session.ingest(next_snapshot(s),now_s=100.17,clock_id=e['clock_id'])


def test_delayed_report_bound_to_capture_not_latest_pose(setup):
    g,p,e,s=setup;session=dev.ObservationSession(p,g,e)
    captured=session.ingest(s,now_s=s['received_s'],clock_id=e['clock_id'])
    session.ingest(next_snapshot(s),now_s=100.17,clock_id=e['clock_id'])
    report={'schema_version':2,'capture_id':s['capture_id'],'camera_device':p['camera_device'],
        'coordinate_frame':p['source_frame'],'detections':[{'id':0,'label':'black cup','score':.9,
        'surface_point':{'valid':True,'xyz_m':[.1,.05,.6]}}]}
    out=dev.transform_report(p,g,report,capture_snapshot=captured,now_s=101.,clock_id=e['clock_id'])
    assert out['observed_s']==100.1 and out['executable_entities'] is False
    assert out['detections'][0]['base_surface_xyz_m']==pytest.approx(captured['object_surface_xyz_m'])
    assert out['detections'][0]['grasp_pose'] is None
    for bad in ['capture','frame','device','depth','duplicate','stale']:
        r=deepcopy(report)
        if bad=='capture':r['capture_id']='other'
        if bad=='frame':r['coordinate_frame']='depth_camera_optical_frame'
        if bad=='device':r['camera_device']={}
        if bad=='depth':r['detections'][0]['surface_point']['xyz_m'][2]=-1
        if bad=='duplicate':r['detections']*=2
        with pytest.raises(ValueError):dev.transform_report(p,g,r,capture_snapshot=captured,now_s=106. if bad=='stale' else 101.,clock_id=e['clock_id'])


def test_viewer_uses_same_mounts_reference_and_prismatic_axis(setup):
    g,p,_,s=setup
    spec=importlib.util.spec_from_file_location('dev_viewer_tests',ROOT/'tests/test_s2_visualization.py')
    v=importlib.util.module_from_spec(spec);spec.loader.exec_module(v)
    data=v.viewer.bundle(g,p)
    assert data['mode']=='development'
    for h in [0,.32,p['lift']['travel_m']]:
        js=v.evaluate_js(data,{**data['initial'],'lift':h})
        expected=dev.evaluate(p,g,s['raw'],h)
        for side in s2.ARMS:
            np.testing.assert_allclose(js['frames'][side+'_tcp'],expected['base_from_tcp'][side],atol=1e-12)
        np.testing.assert_allclose(js['frames']['camera'],p['base_from_optical'])
    text=v.viewer.render(data)
    assert '/*__S2_' not in text and len(text.encode())<1_000_000


@pytest.mark.parametrize('side', ['left','right'])
def test_control_model_conversion_preserves_home_and_continuous_encoder_branch(setup, side):
    from home_robot_tasks import s2_geometry as geo
    geometry, profile, _, _ = setup
    arm = profile['arms'][side]
    ranges = json.loads((ROOT/'config/home_assistant/arm_ranges.development.json').read_text())['arms'][side]
    for j in geo.JOINTS:
        m = arm['alignment'][j]
        assert m['raw_to_control_sign'] == ranges[j]['positive_raw_direction']
        assert m['control_to_model_sign'] == (-1 if j in ('elbow','wrist_flex','gripper') else 1)
        assert geo.model_angle_from_raw(arm['reference_raw'][j],m,j) == pytest.approx(arm['reference_model_rad'][j])
        for raw in (-1,0,200,500,2048,4095,4096,4097):
            angle = geo.model_angle_from_raw(raw,m,j)
            control = geo.control_from_model(angle,m)
            assert control == pytest.approx(ranges[j]['positive_raw_direction']*(raw-2048)*np.pi/2048)
            assert geo.model_from_control(control,m) == pytest.approx(angle)
            assert 2048+round(ranges[j]['positive_raw_direction']*control*2048/np.pi) == raw
        assert geo.model_angle_from_raw(500,m,j)-geo.model_angle_from_raw(200,m,j) == pytest.approx(300*np.pi/2048)
    raw = {s:dict(profile['arms'][s]['reference_raw']) for s in geo.ARMS}
    raw[side]['elbow'] += 30
    raw[side]['gripper'] += 30
    result = dev.evaluate(profile,geometry,raw,.51)
    for j in ('elbow','gripper'):
        assert result['model_joint_positions_si'][f'{side}_model_{j}'] == pytest.approx(arm['reference_model_rad'][j]+30*np.pi/2048)
    assert result['motion_authorized'] is False
