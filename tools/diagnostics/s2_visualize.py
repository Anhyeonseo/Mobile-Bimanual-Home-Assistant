#!/usr/bin/env python3
"""Build a self-contained, read-only S2 coordinate viewer from pinned geometry."""
import argparse
import json
from pathlib import Path
import sys
import math

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/home_robot_tasks'))
from home_robot_tasks import s2_commissioning as s2
from home_robot_tasks.d415_geometry import nominal_model


def bundle(geometry, profile=None, snapshot=None, *, installation=None):
    if profile is not None and profile.get('record_kind') == 's2_nominal_development_coordinates':
        from home_robot_tasks import s2_development as dev
        dev.validate(profile, geometry)
        if snapshot is not None:
            raise ValueError('development viewer takes the saved reference pose; use replay for capture records')
        data = bundle(geometry, installation=installation)
        data.update(mode='development', calibration_id=profile['calibration_id'],
                    mounts={side:profile['arms'][side]['base_from_arm_at_lift_zero'] for side in s2.ARMS},
                    reference_from_tcp={side:profile['arms'][side]['reference_from_tcp'] for side in s2.ARMS},
                    base_from_optical=profile['base_from_optical'])
        data['surfaces']['display_from_cad'] = profile['base_from_cad']
        data['display'].update(software_rendering=True, ground_z_m=profile['ground_z_m'],
            platform_at_lift_zero=profile['lift']['platform_at_lift_zero'],
            lift_axis_base=profile['lift']['axis_base'], lift_travel_m=profile['lift']['travel_m'],
            plate_local_point_m=profile['lift']['plate_local_point_m'],
            base_reference=profile['base_definition'])
        data['initial'].update(lift=0., angles={side:profile['arms'][side]['reference_model_rad'] for side in s2.ARMS})
        return data
    camera_model=nominal_model(ROOT/'config/home_assistant/d415_official_geometry.json')
    if snapshot is not None and profile is None:
        raise ValueError('snapshot requires a validated profile')
    if profile is None:
        # Explicit illustration: these are NOT the Mini1 CAD mounting datums.
        mounts = {}
        for side, y in [('left', .16), ('right', -.16)]:
            m = s2.np.eye(4); m[:3, 3] = [0, y, .26]
            mounts[side] = m.tolist()
        # Front-facing sloped surface of the front antenna, not its top cap.
        # From the pinned CAD face normal: pitch 45deg, small CAD yaw -0.01193.
        # The official housing rear top center sits on the face's upper edge.
        a, yaw = math.radians(45), -.01193
        rotation = s2.np.array([[0, -math.sin(a), math.cos(a)],
                                [-1, 0, 0], [0, -math.cos(a), -math.sin(a)]])
        rotation = s2.np.array([[math.cos(yaw),-math.sin(yaw),0],
                                [math.sin(yaw),math.cos(yaw),0],[0,0,1]]) @ rotation
        camera_matrix = s2.np.eye(4)
        camera_matrix[:3,:3] = rotation
        camera_matrix[:3,3] = s2.np.array([.10113, -.00186, 1.07373]) - rotation @ camera_model['rear_top_center_in_color_optical_m']
        camera = camera_matrix.tolist()
        tcp = {side:s2.np.eye(4).tolist() for side in s2.ARMS}
        angles = {side:dict.fromkeys(s2.JOINTS, 0.) for side in s2.ARMS}
        mode, calibration = 'illustration', None
        lift = .2
    else:
        s2.export_urdf(profile, geometry, allow_simulation=True)  # hash, identity, acceptance
        mounts = {side:profile['arms'][side]['base_from_arm_at_lift_zero'] for side in s2.ARMS}
        tcp = {side:profile['arms'][side]['reference_from_tcp'] for side in s2.ARMS}
        camera = profile['base_from_optical']
        angles = {side:{j:profile['arms'][side]['alignment'][j]['home_model_rad'] for j in s2.JOINTS} for side in s2.ARMS}
        lift = 0.
        mode = 'measured' if profile['coordinate_use_allowed'] else 'simulation'
        calibration = profile['calibration_id']
    point = [0., .05, .6]
    captured = None
    if snapshot is not None:
        # A historical view of the capture, never an assertion of current state.
        out = s2.observe(profile, geometry, snapshot, now_s=snapshot['received_s'], allow_simulation=True)
        lift = out['model_joint_positions_si']['s2_lift_joint']
        angles = {side:s2.model_angles(snapshot['raw'][side],profile['arms'][side]['alignment']) for side in s2.ARMS}
        point = snapshot['camera_xyz_m']
        captured = snapshot['camera_time_s']
    joints = []
    for j in geometry['arm_joints']:
        limits = j.get('limit', {})
        joints.append(dict(name=j['name'], type=j['type'], parent=j['parent']['link'], child=j['child']['link'],
                           origin=s2.origin(j).tolist(), axis=[float(v) for v in j.get('axis',{}).get('xyz','0 0 1').split()],
                           limits=[float(limits.get('lower',-math.pi)),float(limits.get('upper',math.pi))]))
    surfaces = json.loads((Path(__file__).with_name('s2_viewer')/'meshes.json').read_text())
    if surfaces['geometry_sha256'] != s2.digest(geometry):
        raise ValueError('display meshes belong to a different official geometry revision')
    camera_mesh=json.loads((Path(__file__).with_name('s2_viewer')/'d415-mesh.json').read_text())
    if camera_mesh['source_sha256']!=camera_model['mesh_sha256']:
        raise ValueError('D415 display mesh source mismatch')
    surfaces['meshes']['d415.stl']=camera_mesh
    # Preserve official motor/wheel joint origins. Ground comes from the full
    # source wheel surface at those poses, never the decimated display radius.
    wheel_poses = json.loads(json.dumps(surfaces['cad_from_wheels']))
    display = dict(ground_z_m=min(surfaces['wheel_contact_z_m']),
                   cad_from_wheels=wheel_poses, assembly_calibrated=False,
                   highlight_platform=True, inspection_lighting=True,
                   camera_mount_upper_edge_m=[.10113, -.00186, 1.07373] if profile is None else None)
    if profile is None:
        # Migrate the complete illustration by one rigid transform. Do not
        # reinterpret an accepted profile's independently defined base frame.
        old=s2.np.asarray(surfaces['display_from_cad'])
        base=s2.nominal_base_from_cad(geometry)
        shift=base@s2.np.linalg.inv(old)
        surfaces['display_from_cad']=base.tolist()
        camera=(shift@s2.np.asarray(camera)).tolist()
        mounts={side:(shift@s2.np.asarray(m)).tolist() for side,m in mounts.items()}
        display['ground_z_m']=float((shift@[0,0,display['ground_z_m'],1])[2])
        display['camera_mount_upper_edge_m']=(shift@[*display['camera_mount_upper_edge_m'],1])[:3].tolist()
        display['base_reference']='nominal_mini1_wheel_axle_centroid'
        display['nominal_ground_z_m']=display['ground_z_m']
        display['floor_basis']='nominal_wheel_surface'
        if installation is None:
            installation=json.loads((ROOT/'config/home_assistant/installation.development.json').read_text())
        floor=installation.get('base_reference',{}).get('floor_measurement',{})
        if floor.get('operator_confirmed') is True:
            if floor.get('datum')!='floor_to_chassis_bottom_plate_underside':
                raise ValueError('unsupported floor measurement datum')
            clearance=s2.number(floor['clearance_m'],'floor clearance',maximum=.2)
            if clearance<=0:raise ValueError('floor clearance must be positive')
            underside=s2.number(floor['underside_cad_z_m'],'underside CAD Z')
            display['ground_z_m']=float((base@[0,0,underside-clearance,1])[2])
            display['underside_z_m']=float((base@[0,0,underside,1])[2])
            display['floor_basis']='operator_reported_underside_clearance'
            display['floor_measurement']=floor
            display['wheel_ground_geometry_verified']=False

        contacts=json.loads((Path(__file__).with_name('s2_viewer')/'lift-contacts.json').read_text())
        if contacts['geometry_sha256']!=s2.digest(geometry):
            raise ValueError('lift contact geometry mismatch')
        joint=next(j for j in geometry['mobile_reference_joints'] if j['name']=='vertical_move')
        axis=s2.np.array([float(v) for v in joint['axis']['xyz'].split()]);axis/=s2.np.linalg.norm(axis)
        origin=s2.origin(joint)
        motion=s2.np.eye(4);motion[:3,3]=axis*contacts['contacts']['low']['q_m']
        platform=base@origin@motion
        display['platform_at_lift_zero']=platform.tolist()
        display['lift_axis_base']=(base[:3,:3]@origin[:3,:3]@axis).tolist()
        display['lift_travel_m']=contacts['travel_m']
        display['plate_local_point_m']=contacts['height_datum']['local_point_m']
        # Keep standalone SO101 orientation explicitly illustrative, but use the
        # Mini1 mounting locations rather than centering the carriage on arms.
        for side in s2.ARMS:
            anchor=next(j for j in geometry['mobile_reference_joints'] if j['name']==side+'_base')
            mounts[side]=s2.np.asarray(mounts[side])
            mounts[side][:3,3]=(platform@s2.origin(anchor))[:3,3]
            mounts[side]=mounts[side].tolist()

    else:
        display['base_reference']='profile_defined; CAD body placement remains illustrative'
    return dict(schema_version=1, robot_model='alohamini1', arm_model='SO-ARM101',
                mode=mode, calibration_id=calibration, geometry_sha256=s2.digest(geometry),
                captured_s=captured, motion_authorized=False,
                joints=joints, names=dict(zip(s2.UPSTREAM,s2.JOINTS)),
                mounts=mounts, reference_from_tcp=tcp, base_from_optical=camera,
                surfaces=surfaces, display=display,camera_model=camera_model,
                initial=dict(angles=angles,lift=lift,point=point),
                sources=[{'repository':s['repository'],'commit':s['commit']} for s in geometry['sources']])


def render(data):
    directory=Path(__file__).with_name('s2_viewer')
    encoded=json.dumps(data,ensure_ascii=False,allow_nan=False,separators=(',',':')).replace('<','\\u003c')
    html=(directory/'viewer.html').read_text()
    return html.replace('/*__S2_DATA__*/',encoded).replace('/*__S2_SCRIPT__*/',
            (directory/'mesh-renderer.js').read_text()+'\n'+(directory/'viewer.js').read_text())


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile',type=Path,help='accepted commissioning report or profile; otherwise clearly marked illustration')
    p.add_argument('--snapshot',type=Path,help='optional synchronized capture including received_s; historical display only')
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(argv)
    try:
        geometry=s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
        profile=json.loads(args.profile.read_text()) if args.profile else None
        if profile is not None: profile=profile.get('profile',profile)
        snapshot=json.loads(args.snapshot.read_text()) if args.snapshot else None
        result=render(bundle(geometry,profile,snapshot))
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open('x') as f: f.write(result)
        print(args.output)
        return 0
    except (OSError,ValueError,KeyError,TypeError) as e:
        print(f'S2 시각화 생성 실패: {e}',file=sys.stderr)
        return 2


if __name__=='__main__': raise SystemExit(main())
