"""Synthetic evidence tests; none of these constitute a physical robot survey."""
from copy import deepcopy
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from home_robot_tasks import s2_commissioning as s2

ROOT = Path(__file__).resolve().parents[1]
GEOMETRY = ROOT / 'config/home_assistant/official_geometry.json'


def quaternion_matrix(q):
    w,x,y,z = np.asarray(q) / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def euler(angles):
    cr,cp,cy = np.cos(np.asarray(angles)/2)
    sr,sp,sy = np.sin(np.asarray(angles)/2)
    return quaternion_matrix([cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
                              cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy])


def axis_angle(axis, q):
    return quaternion_matrix([np.cos(q/2), *(np.asarray(axis)*np.sin(q/2))])


def independent_fk(xml, positions):
    """Generic URDF tree traversal, independent of implementation's chain and math."""
    root = ET.fromstring(xml)
    pending = list(root.findall('joint'))
    children = {j.find('child').get('link') for j in pending}
    links = {e.get('name') for e in root.findall('link')}
    roots = links - children
    assert len(roots) == 1
    result = {roots.pop(): np.eye(4)}
    while pending:
        old = len(pending)
        for j in pending[:]:
            parent, child = j.find('parent').get('link'), j.find('child').get('link')
            if parent not in result:
                continue
            o = j.find('origin')
            m = np.eye(4)
            if o is not None:
                m[:3, :3] = euler([float(v) for v in o.get('rpy', '0 0 0').split()])
                m[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
            movement = np.eye(4)
            if j.get('type') != 'fixed':
                axis = np.array([float(v) for v in j.find('axis').get('xyz').split()])
                q = positions[j.get('name')]
                if j.get('type') == 'prismatic':
                    movement[:3, 3] = axis * q
                else:
                    movement[:3, :3] = axis_angle(axis, q)
            result[child] = result[parent] @ m @ movement
            pending.remove(j)
        assert len(pending) < old, 'broken/cyclic URDF tree'
    return result


@pytest.fixture
def geometry():
    return s2.load_geometry(GEOMETRY)


def evidence(geometry, provenance='simulation'):
    d = s2.template(geometry)
    d.update(operator='synthetic test only', assembly_revision='fixture-1', base_reference='fixture origin +X forward +Y left +Z up', stock_parts_confirmed=True, provenance=provenance)
    camera = np.eye(4)
    camera[:3, :3] = euler([.1, -.2, .3])
    camera[:3, 3] = [.02, -.03, -.8]
    d['camera'].update(mount_revision=d['assembly_revision'], base_reference=d['base_reference'], provenance=provenance)
    points = [[-.2,-.1,.5], [.2,-.1,.5], [-.2,.1,.5], [.2,.1,.5], [0,-.15,.7], [.1,.15,.9], [-.15,.12,.8], [.13,-.06,.6], [0,.17,.75]]
    for row, p in zip(d['camera']['fit'] + d['camera']['check'], points):
        row.update(camera_xyz_m=p, base_xyz_m=(camera @ [*p,1])[:3].tolist())
    d['camera_device'].update(device_id_hash='a'*64, factory_calibration_sha256='b'*64, stream_profile_id='color-z16-aligned-1280x720-30', fixed_to_base_confirmed=True,
                              depth_checks=[dict(reference_z_m=z, observed_z_m=z) for z in [.5,.8,1.2]])
    lift = d['lift']
    lift.update(bottom_platform_height_m=.26, origin_raw_unwrapped=12000, origin_id='bottom-1', session_id='session-2')
    lift['checks'] = [dict(raw_unwrapped=12000-h/.084*4096, platform_height_m=.26+h) for h in [0,.3,.63,0]]
    lift['rehome_check'].update(previous_session_id='session-1', new_session_id='session-2', reference_height_m=.26, observed_height_m=.26)
    rng = np.random.default_rng(42)
    upstream_xml = (GEOMETRY.parent / 'official_sources/so101_new_calib.urdf').read_text()
    for side_i, side in enumerate(s2.ARMS):
        arm = d['arms'][side]
        offset = np.array([.05,-.2,.1,-.15,.2,-.1])
        sign = np.array([1,1,-1,-1,1,-1]) * (-1 if side_i else 1)
        for idx, joint in enumerate(s2.JOINTS):
            arm['alignment_checks'][joint] = [dict(unwrapped_raw=2048+(q-offset[idx])/sign[idx]*4096/math.tau, model_rad=q) for q in [0,.4]]
        mount = np.eye(4)
        mount[:3, :3] = euler([.03,-.05, .2 if side_i else -.2])
        mount[:3, 3] = [.4, -.16 if side_i else .16, .32]
        for i, row in enumerate(arm['fit'] + arm['check']):
            q = rng.uniform(-.65,.65,6)
            raw = 2048 + (q-offset)/sign*4096/math.tau
            h = [0,.15,.3][i % 3]
            model_tcp = independent_fk(upstream_xml, dict(zip(s2.UPSTREAM, q)))['gripper_frame_link']
            base_tcp = mount @ model_tcp
            base_tcp[2,3] += h
            row.update(raw=dict(zip(s2.JOINTS, raw)), lift_raw_unwrapped=12000-h/.084*4096,
                       base_tcp_xyz_m=base_tcp[:3,3].tolist(), clock_id='fixture-clock', camera_time_s=10+i, joints_time_s=10+i, lift_time_s=10+i)
            if 'camera_tcp_xyz_m' in row:
                row['camera_tcp_xyz_m'] = (np.linalg.inv(camera) @ np.r_[base_tcp[:3,3],1])[:3].tolist()
                row['base_tcp_rotation'] = base_tcp[:3,:3].tolist()
    for i, row in enumerate(d['live_checks']):
        row.update(capture_id=f'live-{i}', assembly_revision=d['assembly_revision'],
                   camera_device={k:d['camera_device'][k] for k in ['device_id_hash','stream_profile_id','factory_calibration_sha256']},
                   session_id=lift['session_id'],origin_id=lift['origin_id'],clock_id='fixture-clock',
                   received_s=30+i+.02,camera_time_s=30+i,joints_time_s=30+i,lift_time_s=30+i,
                   camera_xyz_m=d['arms']['left']['check'][0]['camera_tcp_xyz_m'],
                   lift_raw_unwrapped=12000,raw={side:d['arms'][side]['check'][0]['raw'] for side in s2.ARMS})
    return json.loads(json.dumps(d))


def snapshot(profile, data):
    row = data['arms']['left']['check'][0]
    return {**{k: deepcopy(profile[k]) for k in ['calibration_id','assembly_revision','camera_device','source_frame']},
            'session_id': profile['lift']['session_id'], 'origin_id': profile['lift']['origin_id'],
            'clock_id': 'fixture-clock', 'camera_time_s': 20., 'joints_time_s': 20., 'lift_time_s': 20.,
            'camera_xyz_m': row['camera_tcp_xyz_m'], 'lift_raw_unwrapped': row['lift_raw_unwrapped'],
            'raw': {side: data['arms'][side]['check'][0]['raw'] for side in s2.ARMS}}


def test_official_identity_and_checked_in_template(geometry):
    assert geometry['robot_model'] == 'alohamini1'
    assert geometry['lift_nominal_m_per_turn'] == .084
    assert len(geometry['arm_joints']) == 7
    checked_in = json.loads((GEOMETRY.parent / 's2_measurements.template.json').read_text())
    assert checked_in == s2.template(geometry)
    r = s2.commission(checked_in, geometry)
    assert r['status'] == 'needs_physical_checks' and not r['s2_complete']
    assert 'assembly_revision' in r['missing']


def test_changed_official_source_or_derived_geometry_rejected(geometry, tmp_path):
    import shutil
    shutil.copytree(GEOMETRY.parent / 'official_sources', tmp_path / 'official_sources')
    path = tmp_path / 'official_geometry.json'
    edited = deepcopy(geometry)
    edited['arm_joints'][0]['origin']['xyz'] = '0 0 0'
    path.write_text(json.dumps(edited))
    with pytest.raises(ValueError, match='derived geometry'): s2.load_geometry(path)
    path.write_text(json.dumps(geometry))
    (tmp_path / 'official_sources/so101_new_calib.urdf').write_text('<robot/>')
    with pytest.raises(ValueError, match='source hash'): s2.load_geometry(path)
    geometry['sources'] = []
    path.write_text(json.dumps(geometry))
    with pytest.raises(ValueError, match='three pinned'): s2.load_geometry(path)


def test_nominal_fk_matches_independent_urdf_and_no_encoder_modulo(geometry):
    arm = evidence(geometry)['arms']['left']
    mapping = s2.alignment(arm['alignment_checks'])
    raw = dict.fromkeys(s2.JOINTS, 2048)
    raw.update(shoulder=4229, wrist_flex=-105, wrist_roll=-723)
    q = s2.model_angles(raw, mapping)
    expected = independent_fk((GEOMETRY.parent/'official_sources/so101_new_calib.urdf').read_text(), dict(zip(s2.UPSTREAM, q.values())))
    actual = s2.arm_transforms(geometry, q)
    for link in actual: np.testing.assert_allclose(actual[link], expected[link], atol=1e-12)
    changed = s2.model_angles({**raw,'shoulder':raw['shoulder']-4096},mapping)
    assert q['shoulder'] - changed['shoulder'] == pytest.approx(math.tau)
    a = s2.model_angles({**raw,'base':200},mapping)['base']
    b = s2.model_angles({**raw,'base':500},mapping)['base']
    assert b-a == pytest.approx(300*math.tau/4096)


def test_full_synthetic_acceptance_and_explicit_simulation_boundary(geometry):
    data = evidence(geometry)
    report = s2.commission(data, geometry)
    assert report['status'] == 'simulation_pass'
    assert not report['s2_complete'] and not report['motion_authorized']
    assert report['arms']['left']['check_max_m'] < 1e-12
    profile = report['profile']
    snap = snapshot(profile,data)
    with pytest.raises(ValueError, match='not accepted'): s2.observe(profile,geometry,snap,now_s=20)
    out = s2.observe(profile,geometry,snap,now_s=20,allow_simulation=True)
    assert out['grasp_pose'] is None and not out['motion_authorized']
    np.testing.assert_allclose(out['object_surface_xyz_m'],data['arms']['left']['check'][0]['base_tcp_xyz_m'],atol=1e-12)
    # Contract test only: these generated numbers are NOT real measurement evidence.
    measured = s2.commission(evidence(geometry, 'measured'), geometry)
    assert measured['s2_complete'] and measured['coordinate_use_allowed']
    assert not measured['motion_authorized']


@pytest.mark.parametrize('fault',['camera','depth','lift_scale','rehome','left_holdout','right_holdout','orientation','joint_direction','camera_tcp'])
def test_bad_physical_evidence_cannot_be_hidden_by_mount_fit(geometry,fault):
    data = evidence(geometry)
    if fault == 'camera': data['camera']['check'][0]['base_xyz_m'][0] += .05
    elif fault == 'depth': data['camera_device']['depth_checks'][0]['observed_z_m'] += .03
    elif fault == 'lift_scale': data['lift']['checks'][2]['platform_height_m'] += .02
    elif fault == 'rehome': data['lift']['rehome_check'].update(reference_height_m=.5,observed_height_m=.5)
    elif fault in ('left_holdout','right_holdout'): data['arms'][fault.split('_')[0]]['check'][0]['base_tcp_xyz_m'][0] += .04
    elif fault == 'orientation':
        row=data['arms']['left']['check'][0]
        row['base_tcp_rotation']=(euler([.2,0,0]) @ np.array(row['base_tcp_rotation'])).tolist()
    elif fault == 'joint_direction':
        rows = data['arms']['right']['alignment_checks']['elbow']
        rows[1]['unwrapped_raw'] = 2*rows[0]['unwrapped_raw']-rows[1]['unwrapped_raw']
    else: data['arms']['left']['check'][0]['camera_tcp_xyz_m'][0] += .04
    result = s2.commission(data,geometry)
    assert result['status']=='rejected' and result['failures']
    assert not result['profile']['coordinate_use_allowed']
    with pytest.raises(ValueError): s2.export_urdf(result['profile'],geometry,allow_simulation=True)


@pytest.mark.parametrize('fault',['model','units','geometry','old_profile','missing_arm','nan','bool','reused_pose','time_skew','bad_depth','bad_rotation','missing_alignment','short_alignment','bad_scale','depth_coverage','origin_restart','lift_coverage','arm_coverage'])
def test_invalid_evidence_is_rejected(geometry,fault):
    d=evidence(geometry)
    if fault=='model': d['robot_model']='alohamini2'
    elif fault=='units': d['units']='mm'
    elif fault=='geometry': d['geometry_sha256']='wrong'
    elif fault=='old_profile': d['camera']['mount_revision']='old'
    elif fault=='missing_arm': del d['arms']['right']
    elif fault=='nan': d['arms']['left']['check'][0]['raw']['elbow']=float('nan')
    elif fault=='bool': d['arms']['left']['check'][0]['raw']['elbow']=True
    elif fault=='reused_pose':
        d['arms']['left']['check'][0]=deepcopy(d['arms']['left']['fit'][0])
        d['arms']['left']['check'][0]['id']='new-name'
    elif fault=='time_skew': d['arms']['right']['check'][0]['joints_time_s'] += .1
    elif fault=='bad_depth': d['arms']['left']['check'][0]['camera_tcp_xyz_m'][2]=-1
    elif fault=='bad_rotation': d['arms']['left']['check'][0]['base_tcp_rotation']=[[-1,0,0],[0,1,0],[0,0,1]]
    elif fault=='missing_alignment': del d['arms']['right']['alignment_checks']['base']
    elif fault=='short_alignment': d['arms']['left']['alignment_checks']['base'][1]=deepcopy(d['arms']['left']['alignment_checks']['base'][0])
    elif fault=='bad_scale': d['arms']['left']['alignment_checks']['base'][1]['unwrapped_raw'] += 100
    elif fault=='depth_coverage':
        for row in d['camera_device']['depth_checks']: row.update(reference_z_m=.5,observed_z_m=.5)
    elif fault=='origin_restart': d['lift']['rehome_check']['previous_session_id']='session-2'
    elif fault=='lift_coverage': d['lift']['checks']=[deepcopy(d['lift']['checks'][0])]*4
    else:
        for row in d['arms']['left']['fit']+d['arms']['left']['check']: row['lift_raw_unwrapped']=12000
    with pytest.raises((ValueError,KeyError)): s2.commission(d,geometry)


@pytest.mark.parametrize('fault',['stale','future_joint','skew','device','intrinsics','frame','assembly','origin','session','calibration','negative_depth','invalid_profile','nan_now'])
def test_runtime_binding_freshness_and_units(geometry,fault):
    d=evidence(geometry)
    profile=s2.commission(d,geometry)['profile']
    snap=snapshot(profile,d)
    now=20.
    if fault=='stale': now=21.
    elif fault=='future_joint': snap['joints_time_s']=20.01
    elif fault=='skew': snap['lift_time_s']=19.9
    elif fault=='device': snap['camera_device']['device_id_hash']='c'*64
    elif fault=='intrinsics': snap['camera_device']['factory_calibration_sha256']='c'*64
    elif fault=='frame': snap['source_frame']='camera_depth_optical_frame'
    elif fault=='assembly': snap['assembly_revision']='changed'
    elif fault=='origin': snap['origin_id']='changed'
    elif fault=='session': snap['session_id']='changed'
    elif fault=='calibration': snap['calibration_id']='changed'
    elif fault=='negative_depth': snap['camera_xyz_m']=[0,0,-1]
    elif fault=='invalid_profile': profile['base_from_optical'][0][3]+=.01
    else: now=float('nan')
    with pytest.raises(ValueError): s2.observe(profile,geometry,snap,now_s=now,allow_simulation=True)


def test_export_urdf_and_lift_only_move_arms_not_camera(geometry):
    d=evidence(geometry)
    profile=s2.commission(d,geometry)['profile']
    xml=s2.export_urdf(profile,geometry,allow_simulation=True)
    assert 'collision' not in {e.tag for e in ET.fromstring(xml).iter()}
    snap=snapshot(profile,d)
    original=s2.observe(profile,geometry,snap,now_s=20,allow_simulation=True)
    tree=independent_fk(xml,original['model_joint_positions_si'])
    for side in s2.ARMS: np.testing.assert_allclose(tree[side+'_tcp'],original['base_from_tcp'][side],atol=1e-12)
    np.testing.assert_allclose(tree[profile['source_frame']],profile['base_from_optical'],atol=1e-12)
    snap['lift_raw_unwrapped'] -= .2/.084*4096
    raised=s2.observe(profile,geometry,snap,now_s=20,allow_simulation=True)
    moved=independent_fk(xml,raised['model_joint_positions_si'])
    for side in s2.ARMS:
        np.testing.assert_allclose(moved[side+'_tcp'][:3,3]-tree[side+'_tcp'][:3,3],[0,0,.2],atol=1e-12)
    np.testing.assert_allclose(moved[profile['source_frame']],tree[profile['source_frame']],atol=1e-12)
    assert raised['object_surface_xyz_m']==original['object_surface_xyz_m']


def test_rebinding_origin_invalidates_old_snapshot_without_refitting_geometry(geometry):
    d=evidence(geometry)
    p=s2.commission(d,geometry)['profile']
    e=dict(previous_calibration_id=p['calibration_id'],assembly_revision=p['assembly_revision'],provenance='simulation',operator='fixture',
           platform_height_m=.26,origin_id='bottom-2',session_id='session-3',origin_raw_unwrapped=22000)
    new=s2.rebind_origin(p,e)
    assert new['arms']==p['arms'] and new['base_from_optical']==p['base_from_optical']
    assert new['calibration_id']!=p['calibration_id'] and not new['coordinate_use_allowed']
    snap=snapshot(p,d)
    with pytest.raises(ValueError): s2.observe(new,geometry,snap,now_s=20,allow_simulation=True)
    snap.update(calibration_id=new['calibration_id'],origin_id='bottom-2',session_id='session-3')
    snap['lift_raw_unwrapped']+=10000
    before=s2.observe(p,geometry,snapshot(p,d),now_s=20,allow_simulation=True)
    after=s2.observe(new,geometry,snap,now_s=20,allow_simulation=True)
    assert before['base_from_tcp']==after['base_from_tcp']
    for key,value in [('platform_height_m',.3),('provenance','measured'),('previous_calibration_id','old'),('origin_id','bottom-1')]:
        with pytest.raises(ValueError): s2.rebind_origin(p,{**e,key:value})


def test_cli_full_workflow_and_output_preservation(geometry,tmp_path):
    cli=[sys.executable,str(ROOT/'tools/diagnostics/s2_coordinates.py')]
    def run(*args): return subprocess.run([*cli,*map(str,args)],capture_output=True,text=True)
    blank=tmp_path/'blank.json'
    assert run('prepare','--output',blank).returncode==0
    incomplete=tmp_path/'incomplete.json'
    assert run('commission','--record',blank,'--output',incomplete).returncode==2
    assert not json.loads(incomplete.read_text())['s2_complete']
    d=evidence(geometry)
    record=tmp_path/'record.json'; record.write_text(json.dumps(d))
    report=tmp_path/'report.json'
    assert run('commission','--record',record,'--output',report).returncode==0
    original=report.read_bytes()
    assert run('commission','--record',record,'--output',report).returncode==2
    assert report.read_bytes()==original
    p=json.loads(report.read_text())['profile']
    snap=tmp_path/'snapshot.json'; snap.write_text(json.dumps(snapshot(p,d)))
    assert run('observe','--profile',report,'--snapshot',snap,'--now-s',20,'--allow-simulation','--output',tmp_path/'coordinates.json').returncode==0
    assert run('export-urdf','--profile',report,'--allow-simulation','--output',tmp_path/'tf.urdf').returncode==0
    assert run('export-urdf','--profile',report,'--output',tmp_path/'no.urdf').returncode==2
    assert not (tmp_path/'no.urdf').exists()


def test_grounding_sam_reports_keep_capture_context_and_invalid_depth(geometry):
    from home_robot_tasks.assistant.contracts import Context, Stamp
    d=evidence(geometry)
    profile=s2.commission(d,geometry)['profile']
    snap=snapshot(profile,d)
    snap['capture_id']='capture-1'
    snap['received_s']=20.
    context=Context('survey','1',profile['calibration_id'],snap['clock_id'],'base_link')
    stamp=Stamp(20.,context,'capture-1')
    report={'schema_version':2,'coordinate_frame':profile['source_frame'],'detections':[
        {'id':0,'label':'black marker','score':.9,'surface_point':{'valid':True,'xyz_m':snap['camera_xyz_m']}},
        {'id':1,'label':'cup','score':.8,'surface_point':{'valid':False,'reason':'no_valid_depth'}}]}
    result=s2.observe_report(profile,geometry,snap,report,stamp=stamp,context=context,now_s=20.,allow_simulation=True)
    assert result['entities'][0]['entity']['geometry_kind']=='surface_point'
    np.testing.assert_allclose(result['entities'][0]['entity']['position_m'],d['arms']['left']['check'][0]['base_tcp_xyz_m'],atol=1e-12)
    assert result['entities'][1]['entity'] is None
    assert result['entities'][1]['reason']=='no_valid_depth'
    assert not result['motion_authorized']
    delayed=s2.observe_report(profile,geometry,snap,report,stamp=stamp,context=context,now_s=22.,allow_simulation=True)
    assert delayed['observed_s']==20. and delayed['inference_age_s']==2.
    assert delayed['coordinate_mode']=='capture_time'
    assert delayed['entities']==result['entities']
    from home_robot_tasks.fetch import InvalidTask
    with pytest.raises(InvalidTask,match='stale'):
        s2.observe_report(profile,geometry,snap,report,stamp=stamp,context=context,now_s=26.,allow_simulation=True)
    with pytest.raises(ValueError,match='stale'):
        s2.observe_report(profile,geometry,{**snap,'received_s':21.},report,stamp=stamp,context=context,now_s=22.,allow_simulation=True)
    with pytest.raises(ValueError,match='capture/context'):
        s2.observe_report(profile,geometry,snap,report,stamp=Stamp(20.,context,'other'),context=context,now_s=20.,allow_simulation=True)


@pytest.mark.parametrize('fault',['missing','stale','camera','session','clock','duplicate','time_reversal'])
def test_completion_requires_real_time_input_contract_evidence(geometry,fault):
    d=evidence(geometry)
    rows=d['live_checks']
    if fault=='missing': d['live_checks']=[]
    elif fault=='stale': rows[0]['received_s']+=1
    elif fault=='camera': rows[0]['camera_device']['device_id_hash']='c'*64
    elif fault=='session': rows[0]['session_id']='old'
    elif fault=='clock': rows[1]['clock_id']='another'
    elif fault=='duplicate': rows[1]['capture_id']=rows[0]['capture_id']
    else:
        rows[1].update(camera_time_s=29,joints_time_s=29,lift_time_s=29,received_s=29.02)
    with pytest.raises(ValueError): s2.commission(d,geometry)
