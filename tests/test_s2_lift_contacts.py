"""Selected stop faces must overlap; a global bounding box is not a stop."""
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/diagnostics'))
spec=importlib.util.spec_from_file_location('s2_lift_contacts',ROOT/'tools/diagnostics/s2_lift_contacts.py')
solver=importlib.util.module_from_spec(spec);spec.loader.exec_module(solver)


def plane(z,up=True,x=0):
    v=np.array([[x-1,-1,z],[x+1,-1,z],[x+1,1,z],[x-1,1,z]],float)
    triangles=v[[[0,1,2],[0,2,3]]]
    return triangles if up else triangles[:,[0,2,1]]


@pytest.mark.parametrize('axis',[[0,0,1],[0,.001,1]])
def test_first_contact_ignores_disjoint_taller_surface(axis):
    axis=np.array(axis,float);axis/=np.linalg.norm(axis)
    lower=np.concatenate([plane(.05),plane(.4,x=5)])
    moving=np.concatenate([plane(.5,False),plane(.7)])
    low=solver.contact_limit(lower,moving,axis,np.ones(len(lower),bool),'low')
    assert low['q_m']==pytest.approx(-.45/axis[2])
    assert low['fixed_triangle']<2
    upper=np.concatenate([plane(1,False),plane(.8,False,x=5)])
    high=solver.contact_limit(upper,moving,axis,np.ones(len(upper),bool),'high')
    assert high['q_m']==pytest.approx(.3/axis[2])
    assert high['fixed_triangle']<2


def test_bounding_boxes_overlap_but_triangles_do_not():
    fixed=np.array([[[0,0,0],[2,0,0],[0,2,0]]],float)
    moving=np.array([[[2,2,1],[2,.5,1],[.5,2,1]]],float)  # downward facing
    with pytest.raises(ValueError,match='does not bound'):
        solver.contact_limit(fixed,moving,[0,0,1],np.ones(1,bool),'low')


def test_pinned_contact_witnesses_and_scope():
    d=ROOT/'tools/diagnostics/s2_viewer'
    report=json.loads((d/'lift-contacts.json').read_text());meshes=json.loads((d/'meshes.json').read_text())
    for name,digest in report['source_mesh_sha256'].items():assert meshes['meshes'][name]['source_sha256']==digest
    assert report['height_datum']['seed_triangle']==19718
    assert np.mean(report['height_datum']['local_vertices_m'],axis=0)[2]==pytest.approx(.68817,abs=3e-5)
    assert report['historical_heights_used'] is False
    assert report['physical_verified'] is False and report['motion_authorized'] is False
    assert report['travel_m']==pytest.approx(.6438468174)
    assert report['upper_surface_selection']['triangle_count']==7194
    assert report['upper_surface_selection']['bounds_min_m'][2]<.84
    assert report['lower_surface_selection']['triangle_count']==9866
    assert report['lower_surface_selection']['bounds_min_m'][2]==pytest.approx(0)
    assert report['contacts']['high']['contact_cad_m'][2]==pytest.approx(.8650930524)
    for key,c in report['contacts'].items():
        fixed=np.array(c['fixed_vertices']);moving=np.array(c['moving_vertices_at_contact']);p=np.array(c['contact_cad_m'])
        for tri in [fixed,moving]:
            normal=np.cross(tri[1]-tri[0],tri[2]-tri[0]);normal/=np.linalg.norm(normal)
            assert abs(np.dot(p-tri[0],normal))<1e-8
            bary=np.linalg.lstsq(np.column_stack([tri[1]-tri[0],tri[2]-tri[0]]),p-tri[0],rcond=None)[0]
            assert np.min(bary)>-1e-7 and sum(bary)<1+1e-7
        expected=.234886145 if key=='low' else .878732962
        assert c['floor_to_platform_top_m']==pytest.approx(expected,abs=1e-7)


def test_thick_stop_underside_is_first_contact_not_thin_top():
    # Connected box: selecting a thin top Z slab would miss 0.2 m of thickness.
    vertices=np.array([[-1,-1,.8],[1,-1,.8],[1,1,.8],[-1,1,.8],
                       [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]])
    faces=np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],
                    [0,1,5],[0,5,4],[1,2,6],[1,6,5],
                    [2,3,7],[2,7,6],[3,0,4],[3,4,7]])
    solid=vertices[faces];fixed=np.concatenate([solid,plane(.75,False,x=5)])
    labels=solver.connected_triangle_labels(fixed);mask=labels==labels[2]
    assert mask.sum()==12
    result=solver.contact_limit(fixed,plane(.5),[0,0,1],mask,'high')
    assert result['q_m']==pytest.approx(.3)
    # Before contact the lowest underside is clear; after it, this face is crossed.
    assert .8-(.5+result['q_m']-.001)>0
    assert .8-(.5+result['q_m']+.001)<0


def test_arm_plate_datum_matches_official_mounts_not_carriage_maximum():
    report=json.loads((ROOT/'tools/diagnostics/s2_viewer/lift-contacts.json').read_text())
    geometry=solver.s2.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
    for side,datum in report['arm_height_datums'].items():
        joint=next(j for j in geometry['mobile_reference_joints'] if j['name']==side+'_base')
        official=solver.s2.origin(joint)[:3,3]
        np.testing.assert_allclose(datum['local_point_m'][:2],official[:2],atol=1e-12)
        assert abs(datum['local_point_m'][2]-official[2])<3e-6
        point=np.array(datum['local_point_m']);center=np.array(datum['plane_center_local_m'])
        assert abs(np.dot(point-center,datum['plane_normal_local']))<1e-12
        for contact in report['contacts'].values():
            h=contact['arm_mounting_plates'][side]
            assert h['floor_height_m']==pytest.approx(h['cad_point_m'][2]-report['ground_cad_z_m'])
            assert h['cad_point_m'][2]<contact['moving_envelope_top_cad_z_m']
    low=report['contacts']['low']['arm_mounting_plates']
    assert low['right']['floor_height_m']-low['left']['floor_height_m']==pytest.approx(.000700033,abs=1e-9)


def test_floor_clearance_changes_absolute_height_not_travel_or_cad_points():
    report=json.loads((ROOT/'tools/diagnostics/s2_viewer/lift-contacts.json').read_text())
    before=json.dumps(report,sort_keys=True)
    result=solver.floor_height_comparison(report,.028)
    assert json.dumps(report,sort_keys=True)==before
    assert result['height_correction_m']==pytest.approx(.020704478067)
    assert result['travel_m']==report['travel_m']
    assert result['wheel_and_motor_mounts_inferred'] is False
    assert result['full_kinematic_calibration_verified'] is False
    for side in ('left','right'):
        low=result['endpoints']['low'][side];high=result['endpoints']['high'][side]
        assert low['floor_height_m']==pytest.approx(low['cad_point_m'][2]+.028)
        assert high['floor_height_m']-low['floor_height_m']==pytest.approx(result['travel_m'],abs=1e-9)
    for bad in [float('nan'),float('inf'),-.028,0,.2]:
        with pytest.raises(ValueError):solver.floor_height_comparison(report,bad)
