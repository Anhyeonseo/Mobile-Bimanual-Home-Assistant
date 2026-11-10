"""Collision geometry from a single rectified exposure, in metres in the fitted surface frame."""
from dataclasses import dataclass
from itertools import product

import numpy as np
from scipy.ndimage import distance_transform_edt, label
from scipy.spatial import ConvexHull, QhullError

from .mask_cloud import MIN_POINTS, SUPPORT_MARGIN_M
from .plane import backproject
from .target_ownership import associate
from .obstacle_envelopes import separate_target_padding

CELL_M = .005
PLANE_BAND_M = .005
MAX_ISOLATED_HOLE_M = .002
# A conservative solid under the measured tabletop, deeper than the Mini 1 arm's reach.
SUPPORT_BOTTOM_M = -1.5
CORNERS = np.array(list(product((False, True), repeat=3)))


class InsufficientScene(ValueError):
    """The current exposure lacks the geometry needed for this check."""


def _support_transform(plane):
    normal = np.asarray(plane.normal)
    if (not np.isfinite([*normal, *plane.point, plane.rms_m, plane.inlier_fraction]).all()
            or not np.isclose(np.linalg.norm(normal), 1.) or plane.inlier_fraction < .10
            or plane.rms_m > PLANE_BAND_M):
        raise InsufficientScene('UNRESOLVED_SUPPORT_PLANE')
    x = np.eye(3)[np.argmin(abs(normal))]
    x = x - (x @ normal) * normal
    x /= np.linalg.norm(x)
    pose = np.eye(4)
    pose[:3, :3] = [x, np.cross(normal, x), normal]
    pose[:3, 3] = -pose[:3, :3] @ plane.point
    return pose


def _column_boxes(xy, bottoms, tops):
    """Merge neighbouring cells with identical vertical bounds into runs. Never bridge a gap."""
    if not len(xy):
        return []
    order = np.lexsort((xy[:, 0], xy[:, 1], bottoms, tops))
    xy, bottoms, tops = xy[order], bottoms[order], tops[order]
    breaks = ((np.diff(xy[:, 0]) != 1) | (np.diff(xy[:, 1]) != 0)
              | (np.diff(bottoms) != 0) | (np.diff(tops) != 0))
    starts = np.r_[0, 1 + np.flatnonzero(breaks)]
    ends = np.r_[starts[1:] - 1, len(xy) - 1]
    lows = np.column_stack((xy[starts] * CELL_M, bottoms[starts]))
    highs = np.column_stack(((xy[ends] + 1) * CELL_M, tops[ends]))
    return np.stack((lows, highs), axis=1).tolist()


def measured_obstacle_boxes(points, padding_m=.001, *, target_vertices=None):
    """Nominal envelope around actual returns, not rounded-up 5 mm cell extents.

    The 5 mm grid groups returns only. Above-table objects remain solid down to
    the support; below-table returns remain local. Padding is an offline model
    assumption, not a measured camera-error bound or a free-space guarantee.
    """
    if not 0<=padding_m<=.005:raise ValueError('bounded nominal padding required')
    result=[]; refinements=[]
    for above in (True,False):
        subset=points[points[:,2]>0] if above else points[points[:,2]<0]
        if not len(subset):continue
        keys=np.floor(subset[:,:2 if above else 3]/CELL_M).astype(int)
        groups,inverse=np.unique(keys,axis=0,return_inverse=True)
        low=np.full((len(groups),3),np.inf);high=np.full((len(groups),3),-np.inf)
        np.minimum.at(low,inverse,subset);np.maximum.at(high,inverse,subset)
        low-=padding_m;high+=padding_m
        if above:low[:,2]=0.
        bounds=np.stack((low,high),axis=1)
        if target_vertices is not None:
            target_vertices=np.asarray(target_vertices)
            nearby=np.all(high>=target_vertices.min(axis=0),axis=1)&np.all(low<=target_vertices.max(axis=0),axis=1)
            for index in np.flatnonzero(nearby):
                samples=subset[inverse==index]
                pieces=separate_target_padding(bounds[index],samples,target_vertices)
                if pieces is not None:
                    refinements.append(dict(box_index=len(result)+int(index),meshes=pieces,
                        measured_returns=len(samples),all_returns_preserved=True))
        result.extend(bounds.tolist())
    return result if target_vertices is None else (result,refinements)


@dataclass
class PickScene:
    support_from_optical: np.ndarray
    support_boxes: list
    obstacle_boxes: list
    target_bounds: np.ndarray
    depth: np.ndarray
    intr: dict
    target_pixels: np.ndarray
    nominal_obstacle_boxes: list
    target_boundary_points: int = 0
    support_from_payload: object = None
    payload_size_m: object = None
    payload_mesh: object = None
    target_body_points: int = 0
    ownership_evidence: object = None
    ownership_assessment: object = None
    uncertain_target_pixels: object = None
    obstacle_padding_refinements: object = None

    def describe(self):
        return dict(frame='capture_support', support_from_optical=self.support_from_optical.tolist(),
                    preferred_obstacle_envelope='point_bounds',
                    cell_m=CELL_M, support_boxes_m=self.support_boxes, obstacle_boxes_m=self.obstacle_boxes,
                    target_bounds_m=self.target_bounds.tolist(), target_pixels=int(self.target_pixels.sum()),
                    missing_depth_pixels=int(np.count_nonzero(self.depth == 0)),
                    visible_payload_extent_only=True, motion_authorized=False,
                    nominal_obstacle_boxes_m=self.nominal_obstacle_boxes,nominal_obstacle_padding_m=.001,
                    nominal_obstacle_padding_refinements=self.obstacle_padding_refinements or [],
                    padding_refinement_policy='subtract_known_target_volume_preserve_all_world_returns',
                    target_boundary_association=dict(method='observed_surface_connectivity',
                        points=self.target_boundary_points),
                    target_ownership=self.ownership_evidence,
                    ownership_assessment=self.ownership_assessment,
                    ambiguous_boundary=dict(points=int(np.count_nonzero(self.uncertain_target_pixels)),
                        policy='retained_occupied_not_proven_independent_obstacle'),
                    support_from_payload=self.support_from_payload,payload_size_m=self.payload_size_m,payload_mesh=self.payload_mesh,
                    target_body_association=dict(kind='flat_elongated_observed_footprint',points=self.target_body_points))

    def _background_surrounds_holes(self, u, v, far):
        """An isolated pixel may use eight measured, mutually consistent returns behind the entire volume."""
        offsets = np.array([(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)])
        h, w = self.depth.shape
        inside = (u > 0) & (u < w-1) & (v > 0) & (v < h-1)
        supported = np.zeros(len(u), dtype=bool)
        if inside.any():
            ring = self.depth[v[inside, None]+offsets[:, 0], u[inside, None]+offsets[:, 1]]
            behind = np.all(np.isfinite(ring) & (ring > far[inside, None] + PLANE_BAND_M), axis=1)
            footprint = far[inside] / min(self.intr['fx'], self.intr['fy'])
            supported[inside] = behind & (np.ptp(ring, axis=1) <= PLANE_BAND_M) & (footprint <= MAX_ISOLATED_HOLE_M)
        return supported

    def check_visible(self, low, high, *, depth_gaps=None):
        """Test the image rays crossing this box for missing depth or a foreground occluder."""
        r, t = self.support_from_optical[:3, :3], self.support_from_optical[:3, 3]
        corners = (np.where(CORNERS, high, low) - t) @ r
        if np.any(corners[:, 2] <= 0):
            raise InsufficientScene('GRIPPER_OR_PAYLOAD_OUT_OF_VIEW')
        intr = self.intr
        focal = np.array([intr['fx'], intr['fy']])
        center = np.array([intr['ppx'], intr['ppy']])
        uv = corners[:, :2] / corners[:, 2:] * focal + center
        h, w = self.depth.shape
        if uv.min(axis=0).min() < 0 or np.any(uv.max(axis=0) > [w-1, h-1]):
            raise InsufficientScene('GRIPPER_OR_PAYLOAD_OUT_OF_VIEW')
        u0, v0 = np.ceil(uv.min(axis=0)).astype(int)
        u1, v1 = np.floor(uv.max(axis=0)).astype(int)
        v, u = np.mgrid[v0:v1+1, u0:u1+1]
        rays = np.stack(((u-center[0])/focal[0], (v-center[1])/focal[1], np.ones_like(u)), axis=-1) @ r.T
        # Ray parameter is optical Z. Parallel rays only intersect slabs containing the camera origin.
        parallel = abs(rays) < 1e-12
        safe = np.where(parallel, 1., rays)
        a, b = (low-t)/safe, (high-t)/safe
        near = np.where(parallel, -np.inf, np.minimum(a, b)).max(axis=-1)
        far = np.where(parallel, np.inf, np.maximum(a, b)).min(axis=-1)
        crosses = (far >= np.maximum(0., near)) & ~np.any(parallel & ((t < low) | (t > high)), axis=-1)
        if not crosses.any():
            raise InsufficientScene('GRIPPER_OR_PAYLOAD_DEPTH_UNKNOWN')
        depths = self.depth[v, u]
        valid = np.isfinite(depths) & (depths > 0)
        # A target may occlude its own original contact volume, but not distant space behind it.
        touches_target = np.all((high >= self.target_bounds[0]) & (low <= self.target_bounds[1]))
        exempt = self.target_pixels[v, u] & touches_target
        hidden = crosses & valid & ~exempt & (depths < near - PLANE_BAND_M)
        if np.any(hidden):
            raise InsufficientScene('GRIPPER_OR_PAYLOAD_OCCLUDED')
        missing = crosses & ~valid
        if missing.any():
            supported = self._background_surrounds_holes(u[missing], v[missing], far[missing])
            unresolved = int(np.count_nonzero(~supported))
            if unresolved:
                if depth_gaps is None:
                    raise InsufficientScene('GRIPPER_OR_PAYLOAD_DEPTH_UNKNOWN')
                depth_gaps.append(dict(missing_pixels=unresolved, checked_pixels=int(crosses.sum())))


def build_pick_scene(depth, intr, plane, mask, target_cloud, *, protected=None):
    """Carve only returns associated with the accepted target's coherent 3D cluster."""
    pose = _support_transform(plane)
    valid = depth > 0
    optical = backproject(depth, intr, valid)
    points = optical @ pose[:3, :3].T + pose[:3, 3]
    if target_cloud is None or len(target_cloud) < MIN_POINTS:
        raise InsufficientScene('NO_COHERENT_TARGET_VOLUME')
    protected=np.zeros_like(mask) if protected is None else np.asarray(protected,dtype=bool)
    ownership=associate(depth,intr,mask,optical,points[:,2],target_cloud,plane.rms_m,protected=protected)
    target=ownership.target[valid].copy()
    boundary_count=int(np.count_nonzero(target & ~mask[valid]))
    if target.sum() < MIN_POINTS:
        raise InsufficientScene('NO_COHERENT_TARGET_VOLUME')
    selected = points[target]
    # Align the carried envelope with the observed footprint instead of using
    # a support-axis box whose empty corners can collide with nearby surfaces.
    _,axes=np.linalg.eigh(np.cov(selected[:,:2].T));long=axes[:,-1]
    if long[np.argmax(abs(long))]<0:long=-long
    rotation=np.array([[long[0],-long[1],0],[long[1],long[0],0],[0,0,1.]])
    local=selected@rotation;payload_low=local.min(axis=0);payload_high=local.max(axis=0)
    payload_low[2]=0.;payload_low[:2]-=.001;payload_high[:2]+=.001
    payload_pose=np.eye(4);payload_pose[:3,:3]=rotation;payload_pose[:3,3]=rotation@((payload_low+payload_high)/2)
    footprint=local[ConvexHull(local[:,:2]).vertices,:2];n=len(footprint);middle=(payload_low+payload_high)/2
    footprint_equations=ConvexHull(local[:,:2]).equations
    # Extrude the observed convex footprint, not its empty bounding-box corners.
    # A one-micrometre numerical separation avoids FCL reporting exact resting
    # contact as penetration. This is not physical clearance or sensor accuracy.
    vertices=np.vstack([np.c_[footprint,np.full(n,z)]-middle for z in (.000001,payload_high[2])])
    faces=[]
    for i in range(1,n-1):faces.extend([[0,i+1,i],[n,n+i,n+i+1]])
    for i in range(n):j=(i+1)%n;faces.extend([[i,j,n+j],[i,n+j,n+i]])
    payload_mesh=dict(vertices=vertices.tolist(),faces=faces,kind='observed_footprint_prism')
    # A thin elongated rigid target has side returns below its top cluster.
    # Returns inside its observed footprint/body
    # cannot simultaneously be a separate world obstacle. Keep them as carried
    # material; do not apply this convex-body inference to tall/concave targets.
    spread=np.linalg.eigvalsh(np.cov(selected[:,:2].T));body_count=0
    if payload_high[2]<=.03 and spread[-1]>=4*max(spread[0],1e-12):
        hull=ConvexHull(local[:,:2]);all_local=points@rotation
        in_footprint=np.all(all_local[:,:2]@hull.equations[:,:2].T+hull.equations[:,2]<=1e-10,axis=1)
        # The payload is already modelled as a solid prism. Its observed
        # interior cannot also be a world obstacle merely because its side
        # projects outside SAM. Conversely, a measured hole in that footprint
        # forbids inferring unmasked material as part of a solid body.
        interior=np.all(all_local[:,:2]@hull.equations[:,:2].T+hull.equations[:,2]<-.002,axis=1)
        # A table return under a noisy 3-D rim is not by itself a hole in
        # the object. Require a gap inside the RGB silhouette's convex extent
        # as well (closed holes and open concavities), with measured support.
        vv,uu=np.nonzero(valid);mv,mu=np.nonzero(mask)
        image_hull=ConvexHull(np.c_[mu,mv])
        image_interior=np.all(np.c_[uu,vv]@image_hull.equations[:,:2].T+image_hull.equations[:,2]<-1.,axis=1)
        image_gap=distance_transform_edt(~mask)[valid]>=1.5
        holes=np.zeros_like(mask);holes[valid]=interior&image_interior&image_gap&(abs(points[:,2])<=SUPPORT_MARGIN_M)
        components,count=label(holes)
        hole_sizes=np.bincount(components.ravel())[1:]
        solid_consistent=not ((count and hole_sizes.max()>=4) or np.any(protected[valid]&in_footprint))
        body=(~target)&~protected[valid]&(mask[valid]|solid_consistent)&in_footprint&(points[:,2]>SUPPORT_MARGIN_M)&(points[:,2]<=payload_high[2])
        body_count=int(body.sum());target|=body
        ownership.evidence['solid_footprint_supported']=bool(solid_consistent)
    low, high = selected.min(axis=0), selected.max(axis=0)
    low[:2] -= .001
    high[:2] += .001
    low[2] = 0.
    # Require nearby observed support around the target, not an infinite extension of a distant plane.
    target_pixels = np.zeros_like(mask)
    target_pixels[valid] = target
    ownership.evidence.update(total_target_points=int(target.sum()),body_points=body_count,
        unresolved_rim_points=int((ownership.uncertain_boundary&~target_pixels).sum()))
    pixel_radius = max(intr['fx'], intr['fy']) * .06 / np.median(optical[target, 2])
    near_mask = distance_transform_edt(~target_pixels)[valid] <= pixel_radius
    support = ~target & (abs(points[:, 2]) <= PLANE_BAND_M)
    cells = np.unique(np.floor(points[support, :2] / CELL_M).astype(int), axis=0)
    nearby_cells = np.unique(np.floor(points[support & near_mask, :2] / CELL_M).astype(int), axis=0)
    centers = (nearby_cells + .5) * CELL_M
    local = centers[np.all((centers >= low[:2]-.06) & (centers <= high[:2]+.06), axis=1)]
    if len(local) < 4:
        raise InsufficientScene('NO_LOCAL_SUPPORT_AT_TARGET')
    try:
        hull = ConvexHull(local)
    except QhullError as exc:
        raise InsufficientScene('NO_LOCAL_SUPPORT_AT_TARGET') from exc
    middle = (low[:2] + high[:2])/2
    if np.any(hull.equations[:, :2] @ middle + hull.equations[:, 2] > 0):
        raise InsufficientScene('NO_LOCAL_SUPPORT_AT_TARGET')
    target_cells = np.floor(selected[:, :2] / CELL_M).astype(int)
    footprint = np.unique(np.vstack((cells, target_cells)), axis=0)
    support_boxes = _column_boxes(footprint, np.full(len(footprint), SUPPORT_BOTTOM_M), np.zeros(len(footprint)))

    # Keep all measured non-target points, including unnamed objects and outliers inside the SAM mask.
    clutter = points[~(support | target)]
    # A conflicting scene is an observation/model problem, not proof that the
    # arm cannot reach. Preserve the returns and identify a bounded reobserve
    # region before spending time on IK or generating hardware commands.
    all_local=points@rotation
    in_payload=(np.all(all_local[:,:2]@footprint_equations[:,:2].T+footprint_equations[:,2]<=1e-10,axis=1)
                &(points[:,2]>=.000001)&(points[:,2]<=payload_high[2]))
    conflicting=in_payload&~(support|target)
    assessment=dict(status='CONSISTENT',world_returns_inside_payload=int(conflicting.sum()))
    if conflicting.any():
        vv,uu=np.nonzero(valid);x=uu[conflicting];y=vv[conflicting]
        assessment.update(status='TARGET_WORLD_OWNERSHIP_UNRESOLVED',
            next_action='recapture_target_boundary',retryable=True,
            roi_xyxy=[max(0,int(x.min())-8),max(0,int(y.min())-8),
                      min(depth.shape[1],int(x.max())+9),min(depth.shape[0],int(y.max())+9)])
    grid = np.floor(clutter / CELL_M).astype(int)
    above = grid[clutter[:, 2] > 0]
    columns, indices = np.unique(above[:, :2], axis=0, return_inverse=True)
    top = np.zeros(len(columns), dtype=int)
    np.maximum.at(top, indices, above[:, 2] + 1)
    obstacles = _column_boxes(columns, np.zeros(len(columns)), top*CELL_M)
    # A lower floor seen past the table does not occupy all the space up to the tabletop.
    below = np.unique(grid[clutter[:, 2] < 0], axis=0)
    obstacles.extend(_column_boxes(below[:, :2], below[:, 2]*CELL_M, (below[:, 2]+1)*CELL_M))
    payload_vertices=np.asarray(payload_mesh['vertices'])@payload_pose[:3,:3].T+payload_pose[:3,3]
    nominal,refinements=measured_obstacle_boxes(clutter,target_vertices=payload_vertices)
    return PickScene(pose, support_boxes, obstacles, np.stack((low, high)), depth, intr, target_pixels,
                     nominal,boundary_count,payload_pose.tolist(),(payload_high-payload_low).tolist(),payload_mesh,body_count,
                     ownership.evidence,assessment,ownership.uncertain_boundary&~target_pixels,refinements)
