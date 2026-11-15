import numpy as np
from home_robot_tasks.recorded_depth import RecordedRayEvidence


def state(volume,key):
    a=volume.arrays();i=np.flatnonzero(np.all(a['voxels']==key,axis=1))[0]
    return {k:bool(a[k][i]) for k in ['occupied','cleared','uncertain']}


def test_later_measured_ray_can_retire_old_hit_but_missing_ray_cannot():
    v=RecordedRayEvidence(resolution=.1,minimum_height=.1,maximum_height=1.)
    for _ in range(2):v.integrate_points([[.55,0,.5]],[0,0,.5],[1,0,0])
    assert state(v,[5,0,5])['occupied']
    v.integrate_points(np.empty((0,3)),[0,0,.5],[1,0,0])
    assert state(v,[5,0,5])['occupied']
    for _ in range(6):v.integrate_points([[1.,0,.5]],[0,0,.5],[1,0,0])
    assert state(v,[5,0,5])['cleared']


def test_single_hit_is_uncertain_and_same_frame_hit_wins_over_clear():
    v=RecordedRayEvidence(resolution=.1,minimum_height=.1,maximum_height=1.)
    points=np.array([[.55,0,.5],[1.,0,.5]])
    v.integrate_points(points,[0,0,.5],[1,0,0])
    assert state(v,[5,0,5])['uncertain']
    v.integrate_points(points,[0,0,.5],[1,0,0])
    assert state(v,[5,0,5])['occupied']
    assert v.cells[(5,0,5)][1]==0


def test_floor_endpoint_is_not_obstacle_but_ray_clears_air_above():
    v=RecordedRayEvidence(resolution=.1,minimum_height=.1,maximum_height=1.1)
    forward=np.array([1,0,-1])/np.sqrt(2)
    for _ in range(2):v.integrate_points([[1.,0,0]],[0,0,1.],forward)
    a=v.arrays()
    assert not a['occupied'].any() and a['cleared'].any()
    assert not (a['voxels'][:,2]==0).any()
