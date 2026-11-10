"""Observed surfaces, occlusion and payload checks; synthetic pixels in an explicitly known camera."""
import numpy as np
import pytest

from home_perception.geometry.pick_scene import build_pick_scene, InsufficientScene
from home_perception.geometry.plane import Plane, backproject, target_support_plane
from home_robot_tasks.stationary_pick_sim import FixturePayload, Rejected
from home_robot_tasks.so101_jaw import touches_slices
from scipy.spatial.transform import Rotation
from tools.diagnostics import run_s3_stationary_lab as lab


@pytest.fixture
def tabletop():
    intr = dict(fx=200., fy=200., ppx=50., ppy=50.)
    depth = np.full((101, 101), .65)
    depth[:, :10] = .90  # a lower floor beyond the edge
    mask = np.zeros_like(depth, dtype=bool)
    mask[44:57, 44:57] = True
    depth[mask] = .625
    depth[45:53, 72:80] = .57  # an unlabelled, taller neighbour
    depth[15:22, 26:34] = 0.  # no return
    plane = Plane(np.array([0., 0., -1.]), np.array([0., 0., .65]), .001, .85)
    target = backproject(depth, intr, mask)
    mask[45:53, 72:80] = True  # segmentation leaked onto the neighbour
    return depth, intr, plane, mask, target


def covered(boxes, point):
    boxes = np.asarray(boxes)
    return np.any(np.all((point >= boxes[:, 0]) & (point <= boxes[:, 1]), axis=1))


def test_support_clutter_and_payload_are_separate(tabletop):
    scene = build_pick_scene(*tabletop)
    assert covered(scene.support_boxes, np.array([0., 0., -.05]))
    assert not covered(scene.obstacle_boxes, np.array([0., 0., .015]))
    assert covered(scene.obstacle_boxes, np.array([.071, .0, .065]))
    assert not covered(scene.obstacle_boxes, np.array([-.19, 0., -.08]))
    np.testing.assert_allclose(scene.target_bounds[:, 2], [0., .025])
    assert scene.target_pixels.sum() == 13*13
    assert not scene.target_pixels[48, 75]


def test_support_must_be_observed_near_the_target(tabletop):
    depth, intr, plane, mask, cloud = tabletop
    depth[12:90, 18:90] = np.where(mask[12:90, 18:90], depth[12:90, 18:90], 0.)
    with pytest.raises(InsufficientScene, match='NO_LOCAL_SUPPORT'):
        build_pick_scene(depth, intr, plane, mask, cloud)


def test_local_table_selected_when_floor_dominates_the_image():
    depth = np.full((180, 240), 1.10)
    depth[55:125, 168:238] = .65
    mask = np.zeros_like(depth, dtype=bool)
    mask[82:99, 193:210] = True
    depth[mask] = .635
    intr = dict(fx=200., fy=200., ppx=120., ppy=90.)
    up = np.array([0.,0.,-1.])
    floor = Plane(up, np.array([0.,0.,1.10]), .001, .8)
    table = target_support_plane(depth, intr, mask, floor, up)
    assert table.point[2] == pytest.approx(.65)
    assert target_support_plane(depth, intr, mask, table, up) is table
    depth[~mask] = 1.10
    assert target_support_plane(depth, intr, mask, floor, up) is None
    depth[~mask] = 0.
    assert target_support_plane(depth, intr, mask, floor, up) is None


@pytest.mark.parametrize('uv,optical_z,reason', [
    ((30, 18), .59, 'DEPTH_UNKNOWN'), ((75, 49), .60, 'OCCLUDED'),
    ((104, 40), .6, 'OUT_OF_VIEW'), ((50, 50), .635, None),
])
def test_depth_visibility_for_approach_and_lift(tabletop, uv, optical_z, reason):
    scene = build_pick_scene(*tabletop)
    point = np.array([(uv[0]-50)/200*optical_z, (uv[1]-50)/200*optical_z, optical_z])
    middle = (scene.support_from_optical @ [*point, 1.])[:3]
    if reason:
        with pytest.raises(InsufficientScene, match=reason):
            scene.check_visible(middle-.002, middle+.002)
    else:
        scene.check_visible(middle-.002, middle+.002)


def test_one_missing_pixel_needs_enclosing_background_beyond_the_tool(tabletop):
    scene = build_pick_scene(*tabletop)
    scene.depth[50,40] = 0.
    scene.intr = dict(scene.intr, fx=400., fy=400.)
    middle = (scene.support_from_optical @ [-.015,0.,.60,1.])[:3]
    scene.check_visible(middle-.003, middle+.003)
    assert scene.depth[50,40] == 0.
    scene.depth[50,41] = 0.
    with pytest.raises(InsufficientScene, match='DEPTH_UNKNOWN'):
        scene.check_visible(middle-.003, middle+.003)
    scene.depth[50,41] = .68  # surrounding depths disagree at an edge
    with pytest.raises(InsufficientScene, match='DEPTH_UNKNOWN'):
        scene.check_visible(middle-.003, middle+.003)
    scene.depth[50,41] = .59
    with pytest.raises(InsufficientScene, match='OCCLUDED'):
        scene.check_visible(middle-.003, middle+.003, depth_gaps=[])


def test_unresolved_depth_is_reported_without_approving_the_path(tabletop):
    scene = build_pick_scene(*tabletop)
    middle = (scene.support_from_optical @ [-.059,-.0944,.59,1.])[:3]
    gaps = []
    scene.check_visible(middle-.002, middle+.002, depth_gaps=gaps)
    assert gaps[0]['checked_pixels'] >= gaps[0]['missing_pixels'] > 0


def test_attached_object_can_hit_an_obstacle_clear_of_the_empty_jaw():
    # Isolate the planner's loaded segment using the SO-ARM101 FK and a corner outside the jaw.
    arm = lab.model(.61)
    q = arm.anchor
    pose = arm.fk(q)[0]
    center = pose[:3, 3] + [.036, .0, .0]
    body = np.eye(4)
    body[:3, 3] = center
    payload = FixturePayload(body, np.array([.008, .006, .005]))
    scene = [(center-.002, center+.002)]
    arm.segment(q, q, scene, 0., .018, .016)
    with pytest.raises(Rejected, match='PAYLOAD_SCENE_INTERSECTION'):
        arm.segment(q, q, scene, .009, .018, .016, payload=payload)


def test_stock_fingers_and_the_gap_swept_on_closing():
    jaw = lab.stock_jaw()
    pose = np.eye(4)
    for x, open_touch in [(0., True), (-.025, False), (-.057, True)]:
        center = np.array([x, 0., 0.])
        obstacle = np.array([[center-.0003, center+.0003]])
        assert jaw.may_touch(.057, pose, obstacle, margin=0.) == open_touch
        assert jaw.may_touch(.057, pose, obstacle, closing=True, margin=0.)


def test_closing_sweep_refines_sampling_padding_but_preserves_contact_and_margin():
    from dataclasses import replace
    jaw = replace(lab.stock_jaw(), angles=np.radians([0., 8.]), gaps=np.array([.001, .02]),
                  fixed_slices=np.array([[[-.052,-.001,-.001],[-.049,.001,.001]]]),
                  moving_slices=np.array([[[.0499,-.0001,-.0001],[.0501,.0001,.0001]]]),
                  hinge_axis=np.array([0.,0.,1.]), hinge_point=np.zeros(3),
                  hinge_radius_m=.051, hand_radius_m=.053)
    direction = np.array([np.cos(np.radians(3.)), np.sin(np.radians(3.)), 0.])
    def obstacle(radius):
        center = radius * direction
        return np.array([[center-.00005, center+.00005]])
    assert not jaw.may_touch(.02, np.eye(4), obstacle(.051), closing=True, margin=0.)
    assert jaw.may_touch(.02, np.eye(4), obstacle(.050), closing=True, margin=0.)
    assert jaw.may_touch(.02, np.eye(4), obstacle(.051), closing=True, margin=.001)


def test_long_table_row_below_a_tilted_tool_is_clear():
    pose = np.eye(4)
    pose[:3,:3] = Rotation.from_euler('y', 30, degrees=True).as_matrix()
    pose[2,3] = .025
    slices = np.array([[[-.005,-.005,-.01], [.005,.005,.01]]])
    half = np.array([[.5,.01,.1]])
    assert not touches_slices(slices, pose, np.array([[0.,0.,-.1]]), half, .003)
    assert touches_slices(slices, pose, np.array([[0.,0.,-.08]]), half, .003)


@pytest.mark.parametrize('mounted,table_z,expected', [
    (True, .84, 'NO_VIRTUAL_PATH'),
    (False, .84, 'VIRTUAL_PATH_FOUND'),
    (False, .841, 'PATH_REQUIRES_GRIPPER_CLEARANCE_CONFIRMATION'),
    (False, .835, 'VIRTUAL_PATH_FOUND'),
])
def test_complete_so101_path_with_a_generated_depth_scene(mounted, table_z, expected):
    from home_robot_tasks.stationary_pick_sim import plan_pick

    arm = lab.model(.61)
    # The overhead view is a positive software control; the mounted case uses the actual project profile.
    camera = np.array(arm.profile['base_from_optical'])
    if not mounted:
        camera = np.eye(4)
        camera[:3, :3] = [[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]]
        camera[:3, 3] = [.19, .10, 1.40]
        arm.profile['base_from_optical'] = camera.tolist()
    intr = lab.read(lab.CONFIG/'camera_device.observed.json')['sdk_calibration']['color_intrinsics']
    v, u = np.indices((intr['height'], intr['width']))
    rays = np.stack(((u-intr['ppx'])/intr['fx'], (v-intr['ppy'])/intr['fy'], np.ones_like(u)), axis=-1)
    rays = rays @ camera[:3, :3].T

    def surface(height):
        z = (height-camera[2, 3])/rays[:, :, 2]
        return z, camera[:3, 3] + z[:, :, None]*rays

    table_depth, table = surface(table_z)
    area = (table[:, :, 0] >= .149) & (table[:, :, 0] <= .245) & (table[:, :, 1] >= .055) & (table[:, :, 1] <= .123)
    depth = np.where(area, table_depth, surface(.30)[0])
    z, points = surface(.852)
    x, y, yaw = lab.CASES[2]
    angle = np.radians(yaw)
    for center_y in [y-.035, y]:
        dx, dy = points[:, :, 0]-x, points[:, :, 1]-center_y
        mask = ((dx*np.cos(angle)+dy*np.sin(angle))/.014)**2 + ((-dx*np.sin(angle)+dy*np.cos(angle))/.004)**2 <= 1
        depth[mask] = z[mask]
    inverse = np.linalg.inv(camera)
    plane = Plane(inverse[:3, :3] @ [0., 0., 1.], (inverse @ [x, y, table_z, 1.])[:3], 0., .5)
    scene = build_pick_scene(depth, intr, plane, mask, backproject(depth, intr, mask))
    review, group, _ = lab.fixture(arm, lab.CASES[2], 1, candidate_source='yeoeun-surface', marker_length_m=.028)
    path = plan_pick(arm, review, group, None, observed_scene=scene)
    assert path['status'] == expected
    if mounted:
        assert any(a.get('reason') == 'GRIPPER_OR_PAYLOAD_OUT_OF_VIEW' for a in path['attempts'])
    elif expected == 'PATH_REQUIRES_GRIPPER_CLEARANCE_CONFIRMATION':
        assert any(a.get('jaw_margin_samples') and a.get('geometric_path_requires_confirmation') for a in path['attempts'])
    else:
        assert [s['stage'] for s in path['attempts'][-1]['stages']] == ['pick_pregrasp', 'pick_approach', 'pick_retreat']
        # Same geometry and reachable poses, but with a persistent unobserved part of the corridor.
        original_check = scene.check_visible
        def with_gap(low, high, *, depth_gaps):
            original_check(low, high, depth_gaps=depth_gaps)
            depth_gaps.append(dict(missing_pixels=9, checked_pixels=55))
        scene.check_visible = with_gap
        uncertain = plan_pick(arm, review, group, None, observed_scene=scene)
        assert uncertain['status'] == 'PATH_REQUIRES_GRIPPER_CLEARANCE_CONFIRMATION'
        assert any(a.get('geometric_path_requires_confirmation') for a in uncertain['attempts'])
        assert not uncertain['executable'] and not uncertain['motion_authorized']
    assert not path['motion_authorized'] and not path['physical_task_completed']
def test_nominal_obstacle_envelope_preserves_returns_without_voxel_roundup():
    from home_perception.geometry.pick_scene import measured_obstacle_boxes
    points=np.array([[.006,.007,.0052],[.007,.008,.0055],[.026,.007,.030]])
    boxes=np.array(measured_obstacle_boxes(points))
    assert len(boxes)==2
    assert np.isclose(boxes[0,1,2],.0065)  # 5.5 mm return + explicit 1 mm, not 10 mm ceil.
    assert np.all(boxes[:,0,2]==0.)
    assert all(any(np.all(p>=lo)&np.all(p<=hi) for lo,hi in boxes) for p in points)
    # A distinct tall neighbour remains present; thin nominal envelopes do not
    # delete any measured obstacle point or bridge separate columns.
    assert np.isclose(boxes[1,1,2],.031)


def test_depth_coherent_boundary_is_target_material_but_neighbour_is_not():
    intr=dict(fx=400.,fy=400.,ppx=50.,ppy=50.)
    depth=np.full((101,101),.65);mask=np.zeros_like(depth,dtype=bool);mask[40:61,40:61]=True
    depth[mask]=.625
    target=backproject(depth,intr,mask)
    depth[40:61,61]=.625  # Real target edge lies one pixel outside SAM.
    depth[49:52,62]=.57  # Close in the image, but a separate taller neighbour.
    plane=Plane(np.array([0.,0.,-1.]),np.array([0.,0.,.65]),.001,.9)
    scene=build_pick_scene(depth,intr,plane,mask,target)
    assert scene.target_pixels[45,61] and not scene.target_pixels[50,62]
    assert scene.target_boundary_points==21
    assert scene.describe()['target_ownership']['method']=='observed_surface_connectivity'
    assert scene.describe()['target_ownership']['seed_distance_m']==.005


def test_thin_elongated_body_side_stays_with_target_but_tall_outlier_does_not():
    intr=dict(fx=400.,fy=400.,ppx=50.,ppy=50.)
    depth=np.full((101,101),.65);mask=np.zeros_like(depth,dtype=bool);mask[45:56,20:81]=True
    depth[mask]=.625;top_cluster=backproject(depth,intr,mask)
    depth[50,50]=.642  # side return below the denoised top cluster, inside body footprint
    depth[51,50]=.57   # actual taller neighbour/outlier must remain an obstacle
    plane=Plane(np.array([0.,0.,-1.]),np.array([0.,0.,.65]),.001,.9)
    scene=build_pick_scene(depth,intr,plane,mask,top_cluster)
    assert scene.target_pixels[50,50] and not scene.target_pixels[51,50]
    assert scene.target_body_points==1
    from collections import Counter
    faces=scene.payload_mesh['faces'];edges=Counter(tuple(sorted((f[i],f[(i+1)%3]))) for f in faces for i in range(3))
    assert set(edges.values())=={2}  # closed carried-body mesh


def test_unmasked_observed_body_is_not_duplicated_as_world_obstacle():
    intr=dict(fx=400.,fy=400.,ppx=50.,ppy=50.)
    depth=np.full((101,101),.65);mask=np.zeros_like(depth,dtype=bool);mask[45:56,20:81]=True
    depth[mask]=.625;top_cluster=backproject(depth,intr,mask)
    mask[50,50]=False;depth[50,50]=.642
    plane=Plane(np.array([0.,0.,-1.]),np.array([0.,0.,.65]),.001,.9)
    scene=build_pick_scene(depth,intr,plane,mask,top_cluster)
    assert scene.target_pixels[50,50]
    assert scene.describe()['target_ownership']['solid_footprint_supported']
    if scene.nominal_obstacle_boxes:
        assert not covered(scene.nominal_obstacle_boxes,np.array([0.,0.,.008]))
    assert scene.ownership_assessment['status']=='CONSISTENT'


def test_visible_hole_prevents_absorbing_an_unmasked_object_inside_convex_extent():
    intr=dict(fx=400.,fy=400.,ppx=50.,ppy=50.)
    depth=np.full((101,101),.65);mask=np.zeros_like(depth,dtype=bool);mask[40:61,10:91]=True
    mask[46:55,45:56]=False;depth[mask]=.625
    target=backproject(depth,intr,mask)
    depth[50,50]=.64  # separate little return in the observed tabletop hole
    plane=Plane(np.array([0.,0.,-1.]),np.array([0.,0.,.65]),.001,.9)
    scene=build_pick_scene(depth,intr,plane,mask,target)
    assert not scene.target_pixels[50,50]
    assert not scene.describe()['target_ownership']['solid_footprint_supported']
    assert covered(scene.nominal_obstacle_boxes,np.array([0.,0.,.01]))
    assert scene.ownership_assessment['status']=='TARGET_WORLD_OWNERSHIP_UNRESOLVED'
    x0,y0,x1,y1=scene.ownership_assessment['roi_xyxy']
    assert x0<=50<x1 and y0<=50<y1
    assert scene.ownership_assessment['next_action']=='recapture_target_boundary'
