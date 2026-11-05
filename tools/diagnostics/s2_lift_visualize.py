#!/usr/bin/env python3
"""Show selected-surface CAD lift contacts; never authorizes hardware motion."""
import argparse
import json
from pathlib import Path
from s2_visualize import ROOT, bundle, s2
from s2_lift_contacts import floor_height_comparison


def endpoint_bundle(*, installation=None):
    geometry=s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
    data=bundle(geometry, installation=installation)
    contacts=json.loads((Path(__file__).with_name('s2_viewer')/'lift-contacts.json').read_text())
    if contacts['geometry_sha256']!=s2.digest(geometry):raise ValueError('lift contact geometry mismatch')
    for name,digest in contacts['source_mesh_sha256'].items():
        if data['surfaces']['meshes'][name]['source_sha256']!=digest:raise ValueError('lift contact mesh mismatch')
    data['display'].update(lift_only=True,highlight_platform=True,inspection_lighting=True)
    data['surfaces']['meshes']={k:v for k,v in data['surfaces']['meshes'].items() if k in ('base_link.STL','vertical_link.STL','wheel1.STL','d415.stl')}
    inspection=json.loads((Path(__file__).with_name('s2_viewer')/'lift-inspection-meshes.json').read_text())
    for name,mesh in inspection['meshes'].items():
        if mesh['source_sha256']!=contacts['source_mesh_sha256'][name]:raise ValueError('inspection mesh mismatch')
        data['surfaces']['meshes'][name]=mesh
    data['surfaces']['arm_visuals']=[]
    joint=next(j for j in geometry['mobile_reference_joints'] if j['name']=='vertical_move')
    origin=s2.origin(joint)
    axis=s2.np.array([float(v) for v in joint['axis']['xyz'].split()]);axis/=s2.np.linalg.norm(axis)
    cad=s2.np.asarray(data['surfaces']['display_from_cad'])
    data['endpoint_poses']={}
    floor_report=None
    if data['display']['floor_basis']=='operator_reported_underside_clearance':
        floor=data['display']['floor_measurement']
        floor_report=floor_height_comparison(contacts,floor['clearance_m'],underside_cad_z_m=floor['underside_cad_z_m'])
        floor_report['measurement_evidence']=floor
        data['floor_reference']=floor_report

    for key,record in contacts['contacts'].items():
        q=record['q_m'];height=(floor_report['endpoints'][key]['left']['floor_height_m'] if floor_report else record['floor_to_platform_top_m'])
        motion=s2.np.eye(4);motion[:3,3]=axis*q
        transform=cad@origin@motion
        contact=(cad@s2.np.r_[record['contact_cad_m'],1])[:3]
        data['endpoint_poses'][key]={'platform':transform.tolist(),'floor_to_platform_m':height,'joint_offset_m':float(q),
                                     'contact_m':contact.tolist(),'contact_key':key,
                                     'plate_local_point_m':contacts['height_datum']['local_point_m'],
                                     'right_floor_height_m':(floor_report['endpoints'][key]['right']['floor_height_m'] if floor_report else record['arm_mounting_plates']['right']['floor_height_m'])}
    data['display']['contact_triangles']=contacts['contacts']
    data['endpoint_basis']=contacts['method']
    data['selected_surface_contacts_computed']=True
    data['mechanical_limits_verified']=False
    return data


def render_endpoints():
    directory=Path(__file__).with_name('s2_viewer')
    data=json.dumps(endpoint_bundle(),ensure_ascii=False,allow_nan=False,separators=(',',':')).replace('<','\\u003c')
    return (directory/'lift-endpoints.html').read_text().replace('/*__S2_DATA__*/',data).replace('/*__S2_SCRIPT__*/',(directory/'mesh-renderer.js').read_text()+'\n'+(directory/'lift-endpoints.js').read_text())


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:f.write(render_endpoints())
    print(a.output)
