"""Stationary, request-local review of disputed cabinet approach cells."""
import math
from pathlib import Path

import numpy as np

from home_robot_tasks.furniture_navigation import visible_free_boxes
from home_robot_tasks.navigation_trial import NearGoal, swept_path_obstacle


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def laser_sees_through(corners, scan):
    """The complete angular interval must see beyond the farthest corner."""
    x, y, yaw = scan['lidar_xy_yaw']
    local = (corners - [x, y]) @ rotation(yaw)
    bearing = np.arctan2(local[:, 1], local[:, 0])
    center = math.atan2(local[:, 1].mean(), local[:, 0].mean())
    angles = center + (bearing-center+math.pi) % math.tau - math.pi
    step = scan['angle_increment']
    ranges = np.asarray(scan['ranges'], float)
    if len(ranges) != 360 or not math.isclose(step, math.tau/360, abs_tol=1e-6):
        return False
    indices = (angles - scan['angle_min']) / step
    rays = ranges[np.arange(math.floor(indices.min()), math.ceil(indices.max())+1) % len(ranges)]
    far = np.linalg.norm(local, axis=1).max()
    return bool(np.all(np.isfinite(rays) & (rays >= .25) & (rays <= 3.5) & (rays > far + .01)))


def scene_bands(path):
    with Path(path).open() as stream:
        version, size, margin, count = stream.readline().split()
        if version != 'cabinet-depth-v1' or float(size) != .05:
            raise ValueError('지원하지 않는 상단 깊이 지도')
        for _ in range(int(count)):
            level, parts, cells = map(int, stream.readline().split())
            polygons = [np.fromstring(stream.readline(), sep=' ')[1:].reshape(-1, 2)
                        for _ in range(parts)]
            occupied = np.array([list(map(int, stream.readline().split())) for _ in range(cells)])
            yield level, polygons, occupied


def swept_polygon(poly, start, goal):
    """Bound linear translation and interpolated yaw, including between sample rotations."""
    import cv2
    poly = np.asarray(poly, dtype=float)
    turn = math.remainder(goal[2]-start[2], math.tau)
    samples = max(1, math.ceil(abs(turn)/math.radians(3)))
    cloud = [poly@rotation(start[2]+t*turn).T + (1-t)*np.array(start[:2])+t*np.array(goal[:2])
             for t in np.linspace(0., 1., samples+1)]
    return cv2.convexHull(np.vstack(cloud).astype('float32')).reshape(-1, 2)


def overlaps(centers, hull, radius):
    """Conservative cell-square/convex sweep test via separating edge normals."""
    from scipy.spatial import ConvexHull
    if len(hull) < 3:
        edge = hull[-1]-hull[0]
        t = np.clip((centers-hull[0])@edge / max(float(edge@edge), 1e-20), 0., 1.)
        return np.linalg.norm(centers-hull[0]-t[:,None]*edge,axis=1) <= radius
    edges = ConvexHull(hull).equations
    return (centers @ edges[:, :2].T + edges[:, 2] <= radius).all(axis=1)


TRACK_XY = .012
TRACK_YAW = math.radians(2)


def in_reviewed_path(start, goal, pose):
    delta = np.subtract(goal[:2], start[:2])
    length2 = delta @ delta
    t = np.clip((np.subtract(pose[:2], start[:2]) @ delta)/length2, 0., 1.) if length2 else 0.
    xy = np.array(start[:2])+t*delta
    turn = math.remainder(goal[2]-start[2], math.tau)
    return (math.dist(xy, pose[:2]) <= TRACK_XY
            and abs(math.remainder(pose[2]-start[2]-t*turn, math.tau)) <= TRACK_YAW)


def command_in_path(review, pose, sweep):
    rot = rotation(pose[2])
    return all(in_reviewed_path(review['pose'], review['goal'],
               [*(np.array(pose[:2])+rot@[x,y]), pose[2]+a]) for x,y,a in sweep)


class PathEvidence:
    """Evaluate disputed parts of original map cells in the stationary sensor frame."""
    def __init__(self, grid, origin, resolution, body, sections, saved_scene, pose, face):
        self.grid, self.origin, self.res = grid, np.asarray(origin), resolution
        self.body, self.sections, self.scene, self.pose = body, sections, saved_scene, pose
        self.reach = np.linalg.norm(body['padded_outline_xy_m'], axis=1).max()
        self.margin = body['clearance_m']+TRACK_XY+self.reach*(TRACK_YAW+.001)
        yy,xx = np.nonzero((grid>=65)|(grid<0))
        centers = self.origin + resolution*(np.column_stack((xx,yy))+.5)
        near = np.linalg.norm(centers-pose[:2],axis=1) < .65+self.reach+resolution
        self.cells = np.column_stack((xx[near], yy[near]))
        self.centers = centers[near]
        self.face = face

    def requirements(self, goal):
        body = self.body
        hull = swept_polygon(body['footprint_xy_m'],self.pose,goal)
        # Include off-centre overlap from the archived 5 cm / 5 degree admission lattice.
        coarse_margin = self.margin + self.res*math.sqrt(2) + self.reach*math.radians(2.5)
        near = overlaps(self.centers,hull,coarse_margin)
        cells = self.cells[near]
        # Only the measured portion intersecting the swept robot is disputed.
        # Unknown cells remain in Nav2; positive rays may correct occupied cells only.
        saved_boxes = set()
        for level, polygons, voxels in self.scene:
            centers = (voxels+.5)*.05
            nearby = voxels[np.linalg.norm(centers-self.pose[:2],axis=1)<.65+self.reach+.1]
            for poly in polygons:
                mask = overlaps((nearby+.5)*.05,swept_polygon(poly,self.pose,goal),self.margin+.05/math.sqrt(2))
                # The native critic independently retains the original 3-D scene.
                saved_boxes.update((int(x),int(y),level) for x,y in nearby[mask])
        sub = self.res/4
        offsets = np.array([(x,y) for x in range(4) for y in range(4)])
        indices = (cells[:,None,:]*4+offsets).reshape(-1,2)
        centers = self.origin+(indices+.5)*sub
        needed = set()
        for level, polys in self.sections.items():
            hit = np.zeros(len(centers),bool)
            for poly in polys:
                hit |= overlaps(centers,swept_polygon(poly,self.pose,goal),self.margin+sub/math.sqrt(2))
            needed.update((int(x),int(y),level) for x,y in indices[hit])
        unknown = {(int(x),int(y)) for x,y in cells if self.grid[y,x]<0}
        overlap_unknown = any((x//4,y//4) in unknown for x,y,_ in needed)
        cells = np.asarray([(x,y) for x,y in cells if self.grid[y,x]>=65],dtype=int).reshape(-1,2)
        return (cells, needed, saved_boxes, overlap_unknown)

    @staticmethod
    def depth_reason(point, depth, intr, cam):
        x,y,z = (np.asarray(point)-cam[:3,3])@cam[:3,:3]
        if z <= .1:
            return 'outside_depth_view'
        u,v = round(x*intr['fx']/z + intr['ppx']), round(y*intr['fy']/z + intr['ppy'])
        if not (0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]):
            return 'outside_depth_view'
        measured = depth[v,u]
        if not math.isfinite(measured) or measured <= 0:
            return 'missing_depth'
        return 'nearer_surface' if measured < z-.025 else 'depth_box_unconfirmed'

    def visible(self, needed, saved_boxes, scan, depth, intr, cam, cache):
        """Require rays through disputed swept boxes. Missing pixels or occlusion remain unknown."""
        sub = self.res/4
        corners = np.array([[0,0],[0,1],[1,1],[1,0]])*sub
        floor = self.body['ground_z_m']
        lidar_z = scan['lidar_z_base_m']
        for key in sorted(needed):
            if key not in cache:
                x,y,level = key
                xy = self.origin+sub*np.array([x,y])
                lo,hi = floor+.05*level, floor+.05*(level+1)
                if lo-.01 <= lidar_z <= hi+.01 and not laser_sees_through(xy+corners,scan):
                    cache[key] = 'lidar'
                else:
                    clear = visible_free_boxes([(np.r_[xy,lo],np.r_[xy+sub,hi])],depth,intr,cam)
                    cache[key] = None if clear[0] else self.depth_reason((xy+[sub/2,sub/2]).tolist()+[(lo+hi)/2],depth,intr,cam)
            if cache[key]:
                return cache[key],key
        for x,y,level in saved_boxes:
            key = ('voxel',x,y,level)
            if key not in cache:
                lo = np.array([x*.05,y*.05,floor+.05*level])
                cache[key] = None if visible_free_boxes([(lo,lo+.05)],depth,intr,cam)[0] else 'saved_depth_unconfirmed'
            if cache[key]:
                return cache[key],key
        return None,None


def clear_approach(trial, start, goal, scan_points, depth_points):
    path = NearGoal.path(trial, start, goal)
    rot = rotation(start[2])
    local = [(*((np.asarray(p[:2])-start[:2])@rot), p[2]-start[2]) for p in path]
    body = trial.navigation_body
    points = np.asarray(depth_points, dtype=float).reshape(-1, 3)
    if not len(points) or not len(scan_points):
        return False
    # Cover the between-sample translation and rotation as well as the 2 cm depth voxels.
    reach = np.linalg.norm(body['padded_outline_xy_m'], axis=1).max()
    margin = body['clearance_m'] + .005 + reach*math.radians(1.5)
    for poly, (low, high) in zip(body['component_footprints'], body['component_z_base_m'], strict=True):
        heights = points[:, 2]
        band = points[(heights >= low-margin-.01) & (heights <= high+margin+.01)]
        for cloud, allowance in ((scan_points, margin), (band, margin+.015)):
            if swept_path_obstacle(cloud, local, footprint=poly, margin=allowance,
                                   self_mask=body['self_mask_xy_m']):
                return False
    return True


class CabinetReview:
    """Review while parked. Reconfirm the selected change with a newer sensor pair."""
    def __init__(self):
        self.pending = None
        self.last_negative = None
        self.last_scan = self.last_depth = -math.inf

    def observe(self, state, args, saved, depth, stamp, intr, camera_to_base):
        import time
        from home_robot_tasks.stationary_localization import fresh_stationary
        from tools.run.cabinet_navigation import front_view_goals, cabinet_gap_reached

        now = time.monotonic()
        with state.lock:
            face_sample = state.cabinet_face
            if (not state.nav2_control.get('review_cabinet') or state.cabinet_review is not None
                    or state.nav2_control.get('motion_active') or not state.scan_history
                    or not fresh_stationary(state.stationary_localization, now)
                    or not face_sample or face_sample['stamp'] != state.pose_stamp
                    or state.scan_stamp != state.pose_stamp or not state.sensors_ready(now)):
                return
            scan = state.scan_history[-1]
            if (scan['observed_s'] != state.scan_stamp or scan['observed_s'] <= self.last_scan
                    or stamp <= self.last_depth or abs(scan['observed_s']-stamp) > .25):
                return
            self.last_scan, self.last_depth = scan['observed_s'], stamp
            pose = list(state.pose)
            face = face_sample['face']
            laser_points, depth_points = state.scan_points, state.depth_points
        cam = np.eye(4)
        cam[:2, :2] = rotation(pose[2])
        cam[:2, 3] = pose[:2]
        cam = cam @ camera_to_base
        grid = np.asarray(saved['data']).reshape(saved['height'], saved['width'])
        import json
        from collections import Counter
        folder = Path(args.area).parent
        if self.pending is None:
            sections = {int(k): v for k,v in json.loads((folder/'upper-sections.json').read_text()).items()}
            scene = list(scene_bands(folder/state.trial.navigation_body['saved_depth_scene']))
            evidence = PathEvidence(grid, saved['origin'][:2], saved['resolution'], state.trial.navigation_body,
                                    sections, scene, pose, face)
            goals = front_view_goals(state.trial,pose,face,args.cabinet_route,geometric_only=True)
            rejected = Counter()
            cache = {}
            examples = {}
            for goal in goals:
                cells, needed, saved_boxes, unknown = evidence.requirements(goal)
                reason, key = evidence.visible(needed,saved_boxes,scan,depth,intr,cam,cache)
                if reason and reason not in examples:
                    examples[reason] = dict(box=key, goal=goal)
                if not reason and unknown:
                    reason = 'unknown_map'
                if not reason and not clear_approach(state.trial,pose,goal,laser_points,depth_points):
                    reason = 'live_obstacle'
                if reason:
                    rejected[reason] += 1
                    continue
                self.pending = evidence, goal, cells, needed, saved_boxes
                return
            if self.last_negative is None:
                self.last_negative = dict(rejected)
                return
            result = dict(goal=None, pose=pose, rejected=dict(rejected), previous=self.last_negative, examples=examples,
                          scan_stamp=self.last_scan, depth_stamp=stamp)
        else:
            evidence, goal, cells, needed, saved_boxes = self.pending
            if not in_reviewed_path(evidence.pose,goal,pose) or not cabinet_gap_reached(goal,face,state.trial.navigation_body):
                self.pending = None
                return
            reason, key = evidence.visible(needed,saved_boxes,scan,depth,intr,cam,{})
            if not reason and not clear_approach(state.trial,pose,goal,laser_points,depth_points):
                reason = 'live_obstacle'
            result = dict(goal=None, pose=evidence.pose, rejected={}, scan_stamp=self.last_scan, depth_stamp=stamp)
            if reason:
                result['rejected'][reason] = 1
            else:
                # Nav2 may propose a path on this local map. Only commands inside
                # the measured tube may reach the mobile controller.
                area = dict(state.trial.area)
                feasible = area['feasible_by_yaw'].copy()
                area['feasible_by_yaw'] = feasible
                for x,y,a in NearGoal.path(state.trial,evidence.pose,goal):
                    ix,iy = np.floor((np.array([x,y])-evidence.origin)/evidence.res).astype(int)
                    if not (0 <= ix < feasible.shape[2] and 0 <= iy < feasible.shape[1]):
                        raise ValueError('재검사 경로가 지도 밖입니다.')
                    feasible[round(a/math.tau*len(feasible)) % len(feasible),iy,ix] = True
                fused = grid.copy()
                if len(cells):
                    fused[cells[:,1],cells[:,0]] = 0
                result.update(goal=goal, checked_boxes=len(needed), planning_cells=cells.tolist())
                with state.lock:
                    now = time.monotonic()
                    if not state.sensors_ready(now) or now-stamp>.8 or now-scan['observed_s']>.35:
                        return
                    state.apply_navigation_update(dict(revision=f'cabinet-{stamp:.6f}', area=area,
                        map=dict(resolution=saved['resolution'],origin=saved['origin']),fused=fused))
        with state.lock:
            state.cabinet_review = result
        state.record.write('cabinet_live_path_review', **result)
