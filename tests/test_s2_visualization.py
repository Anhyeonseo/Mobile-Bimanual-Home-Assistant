"""Viewer transforms must agree with S2 math, not just look plausible."""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess

import numpy as np
import pytest
from home_robot_tasks import s2_commissioning as s2

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('s2_visualize',ROOT/'tools/diagnostics/s2_visualize.py')
viewer=importlib.util.module_from_spec(spec);spec.loader.exec_module(viewer)
fixtures=importlib.util.spec_from_file_location('s2_visualization_fixtures',ROOT/'tests/test_s2_commissioning.py')
fixture=importlib.util.module_from_spec(fixtures);fixtures.loader.exec_module(fixture)


def geometry(): return s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json')


def evaluate_js(data,state):
    node=shutil.which('node')
    if not node: pytest.skip('Node required for browser/Python kinematics comparison')
    script="const fs=require('fs'),v=require(process.argv[1]);const i=JSON.parse(fs.readFileSync(0,'utf8'));console.log(JSON.stringify(v.evaluate(i.data,i.state)));"
    run=subprocess.run([node,'-e',script,str(ROOT/'tools/diagnostics/s2_viewer/viewer.js')],input=json.dumps(dict(data=data,state=state)),text=True,capture_output=True,check=True)
    return json.loads(run.stdout)


@pytest.mark.parametrize('measured',[False,True])
def test_js_model_agrees_with_python_for_both_arms_and_lift(measured):
    g=geometry()
    profile=s2.commission(fixture.evidence(g,'measured' if measured else 'simulation'),g)['profile']
    data=viewer.bundle(g,profile)
    assert data['mode']==('measured' if measured else 'simulation')
    rng=np.random.default_rng(33)
    for height in [0,.32,.64]:
        state={'angles':{side:dict(zip(s2.JOINTS,rng.uniform(-1,1,6).tolist())) for side in s2.ARMS},'lift':height,'point':[.05,-.1,.6]}
        out=evaluate_js(data,state)
        for side in s2.ARMS:
            mount=np.array(profile['arms'][side]['base_from_arm_at_lift_zero']);mount[2,3]+=height
            transforms=s2.arm_transforms(g,state['angles'][side])
            for link,m in transforms.items(): np.testing.assert_allclose(out['arms'][side][link],mount@m,atol=1e-12)
            tcp=mount@transforms['gripper_frame_link']@profile['arms'][side]['reference_from_tcp']
            np.testing.assert_allclose(out['frames'][side+'_tcp'],tcp,atol=1e-12)
        np.testing.assert_allclose(out['frames']['camera'],profile['base_from_optical'],atol=1e-12)
        np.testing.assert_allclose(np.array(out['frames']['object'])[:3,3],(np.array(profile['base_from_optical'])@[*state['point'],1])[:3],atol=1e-12)


def test_illustration_is_never_a_calibrated_mount_or_home():
    data=viewer.bundle(geometry())
    assert data['mode']=='illustration' and data['calibration_id'] is None
    assert data['motion_authorized'] is False
    assert all(q==0 for a in data['initial']['angles'].values() for q in a.values())
    text=viewer.render(data)
    assert '실측 전 · 장착 배치 예시' in text
    assert '실물 Home과 다를 수 있음' in text
    assert '/*__S2_' not in text and '<html' not in text
    assert len(text.encode())<1_000_000


def test_profile_changes_are_not_visualized_as_accepted():
    g=geometry();profile=s2.commission(fixture.evidence(g),g)['profile']
    profile['base_from_optical'][0][3]+=.1
    with pytest.raises(ValueError):viewer.bundle(g,profile)


def test_display_wheels_contact_ground_and_planned_camera_stays_fixed():
    import base64
    data=viewer.bundle(geometry(),installation={})
    mesh=data['surfaces']['meshes']['wheel1.STL']
    q=np.frombuffer(base64.b64decode(mesh['vertices']),dtype='<u2').reshape(-1,3)
    vertices=np.asarray(mesh['min'])+q/65535*np.asarray(mesh['span'])
    for pose in data['display']['cad_from_wheels']:
        assembled=np.asarray(data['surfaces']['display_from_cad'])@pose
        transformed=vertices@assembled[:3,:3].T+assembled[:3,3]
        assert transformed[:,2].min()==pytest.approx(data['display']['ground_z_m'])
    assert data['display']['ground_z_m']==pytest.approx(-.04949552,abs=1e-7)
    expected=[s2.origin(j) for j in geometry()['mobile_reference_joints'] if j['name'] in ('wheel1','wheel2','wheel3')]
    np.testing.assert_allclose(data['display']['cad_from_wheels'],expected)
    assert mesh['min']==mesh['source_min_m'] and mesh['bounds_preserved']
    assert mesh['span'][2]==pytest.approx(.09899105,abs=1e-7)
    before=evaluate_js(data,data['initial'])
    after=evaluate_js(data,{**data['initial'],'lift':.64})
    np.testing.assert_allclose(before['frames']['camera'],after['frames']['camera'])
    camera=np.asarray(before['frames']['camera'])
    rear_top=(camera@np.r_[data['camera_model']['rear_top_center_in_color_optical_m'],1])[:3]
    np.testing.assert_allclose(rear_top,data['display']['camera_mount_upper_edge_m'])
    assert camera[0,2]>0 and camera[2,2]<0
    assert data['display']['assembly_calibrated'] is False


def test_nominal_base_origin_and_camera_body_relationship_are_preserved():
    g=geometry();data=viewer.bundle(g);base=np.array(data['surfaces']['display_from_cad'])
    wheel_origins=np.array([s2.origin(j)[:3,3] for j in g['mobile_reference_joints'] if j['name'] in ('wheel1','wheel2','wheel3')])
    np.testing.assert_allclose(base@np.r_[wheel_origins.mean(0),1],[0,0,0,1],atol=1e-12)
    np.testing.assert_allclose(base[:3,:3]@base[:3,:3].T,np.eye(3),atol=1e-12)
    assert np.linalg.det(base[:3,:3])==pytest.approx(1)
    # The mount surface and camera must not move relative to the original CAD.
    cad_edge=np.linalg.inv(base)@[*data['display']['camera_mount_upper_edge_m'],1]
    np.testing.assert_allclose(cad_edge[:3],[-.00186,-.10113,1.07373],atol=1e-12)
    import sys
    sys.path.insert(0,str(ROOT/'tools/diagnostics'))
    from s2_lift_visualize import endpoint_bundle
    endpoints=endpoint_bundle();contacts=json.loads((ROOT/'tools/diagnostics/s2_viewer/lift-contacts.json').read_text())
    raw_top=contacts['contacts']['low']['platform_top_cad_z_m']-contacts['contacts']['low']['q_m']*contacts['axis_cad'][2]-.064
    for key,pose in endpoints['endpoint_poses'].items():
        top=(np.asarray(pose['platform'])@[0,0,raw_top,1])[2]
        assert top-endpoints['display']['ground_z_m']==pytest.approx(pose['floor_to_platform_m'])


def test_historical_snapshot_has_exact_coordinates_and_capture_label():
    g=geometry();d=fixture.evidence(g);p=s2.commission(d,g)['profile']
    snap=fixture.snapshot(p,d);snap['received_s']=20.02
    data=viewer.bundle(g,p,snap)
    assert data['captured_s']==20
    out=evaluate_js(data,data['initial'])
    expected=s2.observe(p,g,snap,now_s=20.02,allow_simulation=True)
    for side in s2.ARMS:np.testing.assert_allclose(out['frames'][side+'_tcp'],expected['base_from_tcp'][side],atol=1e-12)
    with pytest.raises(ValueError):viewer.bundle(g,None,snap)


def test_gripper_opening_does_not_move_fixed_reference_tcp():
    data=viewer.bundle(geometry());state=data['initial']
    before=evaluate_js(data,state)
    state['angles']['left']['gripper']=.9
    after=evaluate_js(data,state)
    np.testing.assert_allclose(before['frames']['left_tcp'],after['frames']['left_tcp'])
    assert not np.allclose(before['arms']['left']['moving_jaw_so101_v1_link'],after['arms']['left']['moving_jaw_so101_v1_link'])


def test_meshes_are_finite_indexed_surfaces_with_official_visual_origins():
    import base64
    import hashlib
    import xml.etree.ElementTree as ET
    data=viewer.bundle(geometry());surfaces=data['surfaces']
    for name,mesh in surfaces['meshes'].items():
        vertices=np.frombuffer(base64.b64decode(mesh['vertices']),dtype='<u2').reshape(-1,3)
        faces=np.frombuffer(base64.b64decode(mesh['faces']),dtype='<u2').reshape(-1,3)
        assert len(vertices)==mesh['vertex_count'] and len(faces)==mesh['triangle_count']
        assert faces.min()>=0 and faces.max()<len(vertices)
        assert np.isfinite(mesh['min']).all() and np.all(np.asarray(mesh['span'])>0)
        assert len(faces)<mesh['original_triangles']
        local=ROOT/'ros2_ws/src/so101_description/meshes'/name
        if local.exists(): assert hashlib.sha256(local.read_bytes()).hexdigest()==mesh['source_sha256']
    urdf=ET.parse(ROOT/'config/home_assistant/official_sources/so101_new_calib.urdf').getroot()
    expected=[]
    for link in urdf.findall('link'):
        for visual in link.findall('visual'):
            expected.append((link.get('name'),Path(visual.find('geometry/mesh').get('filename')).name,
                             s2.origin({'origin':visual.find('origin').attrib})))
    assert len(expected)==len(surfaces['arm_visuals'])
    for (link,name,matrix),actual in zip(expected,surfaces['arm_visuals']):
        assert actual['link']==link and actual['mesh']==name
        np.testing.assert_allclose(actual['origin'],matrix,atol=1e-12)
    assert 'webgl' in viewer.render(data)
    assert len(viewer.render(data).encode())<1_000_000


def test_illustration_carriage_uses_official_prismatic_joint_not_arm_midpoint():
    g=geometry();data=viewer.bundle(g)
    contacts=json.loads((ROOT/'tools/diagnostics/s2_viewer/lift-contacts.json').read_text())
    joint=next(j for j in g['mobile_reference_joints'] if j['name']=='vertical_move')
    axis=np.array([float(v) for v in joint['axis']['xyz'].split()]);axis/=np.linalg.norm(axis)
    for height in [0,.2,contacts['travel_m']]:
        pose=evaluate_js(data,{**data['initial'],'lift':height})
        move=np.eye(4);move[:3,3]=axis*(contacts['contacts']['low']['q_m']+height)
        expected=s2.nominal_base_from_cad(g)@s2.origin(joint)@move
        np.testing.assert_allclose(pose['platform'],expected,atol=1e-12)
    # Deliberately disturb illustrative arms: this must not move the carriage.
    before=evaluate_js(data,data['initial'])['platform']
    data['mounts']['left'][0][3]+=1
    np.testing.assert_allclose(evaluate_js(data,data['initial'])['platform'],before)


def test_confirmed_floor_clearance_changes_only_floor_not_wheel_or_camera_poses():
    nominal=viewer.bundle(geometry(),installation={});installed=viewer.bundle(geometry())
    assert installed['display']['floor_basis']=='operator_reported_underside_clearance'
    assert installed['display']['underside_z_m']-installed['display']['ground_z_m']==pytest.approx(.028)
    assert nominal['display']['ground_z_m']-installed['display']['ground_z_m']==pytest.approx(.020704478067)
    for key in ['cad_from_wheels','platform_at_lift_zero']:
        np.testing.assert_allclose(nominal['display'][key],installed['display'][key])
    np.testing.assert_allclose(nominal['base_from_optical'],installed['base_from_optical'])
    assert installed['display']['wheel_ground_geometry_verified'] is False
    import sys
    sys.path.insert(0,str(ROOT/'tools/diagnostics'))
    from s2_lift_visualize import endpoint_bundle
    endpoints=endpoint_bundle()
    saved=json.loads((ROOT/'tools/diagnostics/s2_viewer/lift-floor-reference.json').read_text())
    assert endpoints['floor_reference']==saved
    for key,pose in endpoints['endpoint_poses'].items():
        top=(np.asarray(pose['platform'])@[*pose['plate_local_point_m'],1])[2]
        assert top-endpoints['display']['ground_z_m']==pytest.approx(pose['floor_to_platform_m'])
    assert endpoints['endpoint_poses']['low']['floor_to_platform_m']==pytest.approx(.255590623)
    assert endpoints['endpoint_poses']['high']['right_floor_height_m']==pytest.approx(.900137473)


def test_unconfirmed_floor_measurement_is_not_applied():
    installation={'base_reference':{'floor_measurement':{'operator_confirmed':False,'clearance_m':.028}}}
    data=viewer.bundle(geometry(),installation=installation)
    assert data['display']['floor_basis']=='nominal_wheel_surface'
    assert data['display']['ground_z_m']==pytest.approx(-.04949552193)
