"""Actual RGB-D boundary regression plus adversarial corner/gap examples."""
from pathlib import Path
import json
import numpy as np
import pytest
from home_perception.geometry.target_ownership import associate
from home_perception.geometry.plane import backproject


def test_recorded_diagonal_returns_stay_measured_material_not_separate_obstacles():
    fixture=Path(__file__).parent/'fixtures/depth_boundary_20261119.npz'
    with np.load(fixture) as f:
        depth=f['depth'];mask=f['mask'];intr=json.loads(str(f['intr_json']))
        points=backproject(depth,intr,depth>0);heights=(points-f['point'])@f['normal']
        before=depth.copy();result=associate(depth,intr,mask,points,heights,f['clean_target'],float(f['rms']))
    assert result.target[55:57,25:27].all()
    assert not result.target[depth==0].any()
    np.testing.assert_array_equal(depth,before)
    assert result.evidence['adjacency']==8 and result.evidence['diagonal_measured_gap_guard']


@pytest.mark.parametrize('barrier',['missing','table','depth_step','protected_corner','protected_missing_corner','protected_point'])
def test_diagonal_surface_join_does_not_cut_measured_gaps_or_merge_protected_objects(barrier):
    depth=np.full((80,80),.42);mask=np.zeros_like(depth,bool);mask[40:60,40:60]=True;depth[mask]=.4
    intr=dict(fx=300.,fy=300.,ppx=40.,ppy=40.)
    depth[39,39]=.401
    depth[39,40]=depth[40,39]=0.
    protected=np.zeros_like(mask)
    if barrier=='table':depth[39,40]=.42
    if barrier=='depth_step':depth[39,39]=.37
    if barrier=='protected_corner':depth[39,40]=.401;protected[39,40]=True
    if barrier=='protected_missing_corner':protected[39,40]=True
    if barrier=='protected_point':protected[39,39]=True
    valid=depth>0;points=backproject(depth,intr,valid);clean=backproject(depth,intr,mask)
    result=associate(depth,intr,mask,points,.42-points[:,2],clean,.001,protected=protected)
    assert bool(result.target[39,39])==(barrier=='missing')
    assert not np.any(result.target&protected)
    assert not np.any(result.target&(depth==.42))
    assert not np.any(result.target&(depth==0))
