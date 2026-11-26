import math
import numpy as np
from tools.run.cabinet_path_review import PathEvidence, command_in_path, laser_sees_through, overlaps


def test_laser_requires_bracketing_rays_even_across_zero():
    scan = dict(angle_min=0.,angle_increment=math.pi/180,lidar_xy_yaw=[0.,0.,0.],ranges=[1.]*360)
    corners = np.array([[.49,-.013],[.51,-.013],[.51,.013],[.49,.013]])
    assert laser_sees_through(corners,scan)
    for missing in (None,.50):
        scan['ranges'][359] = missing
        assert not laser_sees_through(corners,scan)


def test_line_segment_intersects_between_vertices():
    assert overlaps(np.array([[.5,.01]]),np.array([[0.,0.],[1.,0.]]),.02)[0]


def test_only_measured_wheel_sweep_is_admissible():
    review = dict(pose=[0.,0.,0.],goal=[.25,0.,0.])
    assert command_in_path(review,[.10,0.,0.],[(0.,0.,0.),(.05,.001,0.)])
    assert not command_in_path(review,[.10,0.,0.],[(.04,.03,0.)])
    assert not command_in_path(review,[.25,0.,0.],[(.04,0.,0.)])
    assert not command_in_path(review,[.10,0.,0.],[(.01,0.,.1)])


def test_foreground_or_missing_depth_blocks_middle_of_path():
    setup = PathEvidence(np.zeros((4,4)),[0.,0.],.05,dict(padded_outline_xy_m=[[.2,0]],clearance_m=.03,
                         ground_z_m=.5),{},[],[0.,0.,0.],{})
    depth = np.ones((51,51))
    intr = dict(fx=100.,fy=100.,ppx=25.,ppy=25.)
    args = ({(0,0,0)},set(),dict(lidar_z_base_m=.2),depth,intr,np.eye(4))
    assert setup.visible(*args,{}) == (None,None)
    depth[26,26]=0.
    assert setup.visible(*args,{})[0]
    depth[:]=.3
    assert setup.visible(*args,{})[0]=='nearer_surface'


def test_disputed_midroute_cell_requires_two_separate_sensor_pairs(tmp_path,monkeypatch):
    import json
    import threading
    import time
    from types import SimpleNamespace
    from tools.run import cabinet_navigation
    from tools.run.cabinet_path_review import CabinetReview

    poly = [[-.03,-.03],[.03,-.03],[.03,.03],[-.03,.03]]
    (tmp_path/'upper-sections.json').write_text(json.dumps({10:[poly]}))
    (tmp_path/'saved-depth.scene').write_text('cabinet-depth-v1 .05 .03 1\n10 1 1\n4 -.03 -.03 .03 -.03 .03 .03 -.03 .03\n40 40\n')
    body = dict(footprint_xy_m=poly,base_footprint_xy_m=poly,padded_outline_xy_m=poly,
                component_footprints=[poly],component_z_base_m=[[.5,.55]],self_mask_xy_m=poly,
                clearance_m=.01,ground_z_m=0.,saved_depth_scene='saved-depth.scene')
    grid = np.zeros((40,40),int)
    grid[20,23] = 100
    area = dict(origin_xy_m=[-1.,-1.],resolution_m=.05,feasible_by_yaw=np.zeros((72,40,40),bool))
    saved = dict(width=40,height=40,resolution=.05,origin=[-1.,-1.],data=grid.ravel().tolist())
    goal = [.25,0.,0.]
    monkeypatch.setattr(cabinet_navigation,'front_view_goals',lambda *a,**kw: iter([goal]))
    updates = []
    state = SimpleNamespace(trial=SimpleNamespace(area=area,navigation_body=body),lock=threading.RLock(),
        nav2_control={'review_cabinet':True},cabinet_review=None,pose=[0.,0.,0.],
        sensors_ready=lambda now: True,scan_points=[[2.,0.]],depth_points=[[2.,0.,.52]],
        record=SimpleNamespace(write=lambda *a,**kw:None),apply_navigation_update=updates.append)
    camera = np.eye(4)
    camera[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
    camera[:3,3]=[-.5,0.,.52]
    args=SimpleNamespace(area=tmp_path/'upper-area.npz',cabinet_route={})
    review = CabinetReview()
    for expected in [False,True]:
        stamp=time.monotonic()
        state.scan_stamp=state.pose_stamp=stamp
        state.stationary_localization=dict(ready=True,observed_s=stamp)
        state.scan_history=[dict(ranges=[2.]*360,angle_min=0.,angle_increment=math.pi/180,
                            lidar_xy_yaw=[-.1,0.,0.],lidar_z_base_m=.52,observed_s=stamp)]
        state.cabinet_face=dict(stamp=stamp,face=dict(yaw=0.,point=[.34,0.]))
        review.observe(state,args,saved,np.full((240,320),2.),stamp,
                       dict(fx=250.,fy=250.,ppx=160.,ppy=120.),camera)
        assert bool(updates) == expected
    assert state.cabinet_review['checked_boxes']>0 and state.cabinet_review['goal']==goal
    assert grid[20,23]==100 and updates[0]['fused'][20,23]==0
    assert updates[0]['area']['feasible_by_yaw'][0,20,25]
