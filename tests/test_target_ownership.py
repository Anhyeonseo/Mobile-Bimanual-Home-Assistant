"""Measured object sides versus separate neighbours and missing evidence."""
import numpy as np
import pytest

from home_perception.geometry.plane import backproject
from home_perception.geometry.target_ownership import associate


def fixture(focal=620.,z=.4):
    depth=np.full((90,140),z+.02)
    mask=np.zeros(depth.shape,bool);mask[25:65,50:100]=True
    depth[mask]=z
    intr=dict(fx=focal,fy=focal,ppx=70.,ppy=45.)
    return depth,mask,intr,z+.02,backproject(depth,intr,mask)


def ownership(depth,mask,intr,table,clean):
    valid=depth>0;points=backproject(depth,intr,valid)
    return associate(depth,intr,mask,points,table-points[:,2],clean,.0011)


def test_connected_sloping_side_is_not_compared_only_to_clean_top():
    depth,mask,intr,table,clean=fixture()
    for x,drop in zip(range(49,44,-1),(.001,.003,.005,.007,.009)):
        depth[26:64,x]=.4+drop
    result=ownership(depth,mask,intr,table,clean)
    assert result.target[40,45]  # 9 mm below top; still an observed continuous side.
    assert result.target[26:64,45:50].all()
    assert not result.target[40,44]  # table remains table
    assert result.evidence['associated_rim_points']==38*5
    assert not result.evidence['missing_depth_filled']


@pytest.mark.parametrize('separator',['table','missing','depth_step'])
def test_nearby_separate_surface_is_not_swallowed(separator):
    depth,mask,intr,table,clean=fixture()
    depth[26:64,49]=.401
    depth[26:64,47]=.401
    depth[26:64,48]={'table':table,'missing':0.,'depth_step':.37}[separator]
    result=ownership(depth,mask,intr,table,clean)
    assert result.target[40,49]
    assert not result.target[40,47] and not result.target[40,48]
    if separator!='depth_step':assert result.uncertain_boundary[40,47]


def test_unlabelled_small_obstacle_keeps_its_returns():
    depth,mask,intr,table,clean=fixture()
    depth[40:43,48]=.37
    result=ownership(depth,mask,intr,table,clean)
    assert not result.target[40:43,48].any()
    assert np.count_nonzero((depth<table-.003)&~result.target)==3


@pytest.mark.parametrize('focal,z',[(300.,.4),(620.,.4),(900.,.4),(620.,.8)])
def test_sampling_scale_changes_budget_but_keeps_table_boundary(focal,z):
    depth,mask,intr,table,clean=fixture(focal,z)
    depth[26:64,49]=z+.001
    result=ownership(depth,mask,intr,table,clean)
    assert result.target[26:64,49].all()
    assert not result.target[40,48]
    assert result.evidence['radius_m_cap']==.006


def test_growth_cannot_chain_into_far_unmasked_material():
    depth,mask,intr,table,clean=fixture()
    depth[26:64,20:50]=.401
    result=ownership(depth,mask,intr,table,clean)
    assert result.target[40,49]
    assert not result.target[40,42] and not result.target[40,20]


def test_segmentation_leak_does_not_seed_a_different_depth_component():
    depth,mask,intr,table,clean=fixture()
    mask[30:40,110:120]=True;depth[30:40,110:120]=.35
    result=ownership(depth,mask,intr,table,clean)
    assert not result.target[30:40,110:120].any()


def test_touching_known_instance_cannot_be_grown_into_target():
    depth,mask,intr,table,clean=fixture()
    protected=np.zeros_like(mask);protected[26:64,47:50]=True
    depth[protected]=.401
    points=backproject(depth,intr,depth>0)
    result=associate(depth,intr,mask,points,table-points[:,2],clean,.0011,protected=protected)
    assert not np.any(result.target&protected)
    assert result.evidence['protected_instance_pixels']==int(protected.sum())
