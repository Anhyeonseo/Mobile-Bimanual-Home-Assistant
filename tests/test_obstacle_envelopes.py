"""Padding may change; measured geometry, independent objects and holes may not."""
from itertools import product

import numpy as np
import pytest
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from home_perception.geometry.obstacle_envelopes import separate_target_padding
from home_perception.geometry.pick_scene import measured_obstacle_boxes


def covered(pieces, points):
    result=np.zeros(len(points),dtype=bool)
    for piece in pieces:
        e=ConvexHull(piece['vertices']).equations
        result|=np.all(points@e[:,:3].T+e[:,3]<=1e-10,axis=1)
    return result


@pytest.mark.parametrize('angle',[0.,17.,48.,92.,155.])
def test_real_boundary_returns_remain_world_material_without_padding_inside_target(angle):
    rotation=Rotation.from_euler('z',angle,degrees=True).as_matrix()
    target=np.array(list(product([-.04,.04],[-.02,.02],[0.,.015])))@rotation.T
    points=np.array([[.04005,.003,.006],[.04020,.0032,.0058]])@rotation.T
    bounds=np.array([points.min(axis=0)-.001,points.max(axis=0)+.001]);bounds[0,2]=0.
    pieces=separate_target_padding(bounds,points,target)
    assert pieces and covered(pieces,points).all()
    # Interior target samples must not be claimed as obstacles by padding.
    interior=np.array(list(product(np.linspace(.0392,.0399,5),[.003],[.003,.006])))@rotation.T
    assert not covered(pieces,interior).any()
    # Every output vertex stays in the original nominal envelope.
    for piece in pieces:
        vertices=np.array(piece['vertices'])
        assert np.all(vertices>=bounds[0]-1e-10) and np.all(vertices<=bounds[1]+1e-10)


@pytest.mark.parametrize('point',[[.0399,0,.006],[.04,0,.006],[.0400005,0,.006]])
def test_actual_overlap_or_unresolved_contact_is_not_clipped_away(point):
    target=np.array(list(product([-.04,.04],[-.02,.02],[0.,.015])))
    point=np.array(point);bounds=np.array([point-.001,point+.001]);bounds[0,2]=0.
    assert separate_target_padding(bounds,np.array([point]),target) is None


def test_far_object_and_lower_floor_keep_original_envelopes():
    target=np.array(list(product([-.04,.04],[-.02,.02],[0.,.015])))
    points=np.array([[.07,.003,.03],[.04,.003,-.4]])
    original=measured_obstacle_boxes(points)
    boxes,refinements=measured_obstacle_boxes(points,target_vertices=target)
    assert boxes==original and refinements==[]


def test_an_independent_return_inside_the_target_volume_prevents_refinement():
    target=np.array(list(product([-.04,.04],[-.02,.02],[0.,.015])))
    points=np.array([[.04005,0,.006],[.0399,.0001,.007]])
    bounds=np.array([points.min(axis=0)-.001,points.max(axis=0)+.001]);bounds[0,2]=0.
    assert separate_target_padding(bounds,points,target) is None
