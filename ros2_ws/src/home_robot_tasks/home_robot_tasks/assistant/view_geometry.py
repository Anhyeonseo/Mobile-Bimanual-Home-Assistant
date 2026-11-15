"""Calibrated image-frustum and measured-obstacle screening of an alignment view."""
import numpy as np
from .alignment_contracts import cloud
from .placement import Check
from ..s2_geometry import rigid


def view_check(points_map_m, map_from_optical, intrinsics, *, obstacles_map_m=(),
               minimum_visible_fraction=.65, image_margin_px=12, depth_margin_m=.015):
    points=cloud(points_map_m)
    transform=np.linalg.inv(rigid(map_from_optical))
    camera=points@transform[:3,:3].T+transform[:3,3]
    width,height=intrinsics['width'],intrinsics['height']
    fx,fy,cx,cy=(float(intrinsics[k]) for k in ('fx','fy','ppx','ppy'))
    if (width<=0 or height<=0 or min(fx,fy)<=0 or not np.isfinite([fx,fy,cx,cy]).all()
            or not 0<minimum_visible_fraction<=1): raise ValueError('calibrated camera bounds required')
    def project(p):
        z=np.maximum(p[:,2],1e-9)
        return np.column_stack((p[:,0]/z*fx+cx,p[:,1]/z*fy+cy))
    uv=project(camera)
    visible=(camera[:,2]>.1)&(camera[:,2]<3)&(uv[:,0]>=image_margin_px)&(uv[:,0]<width-image_margin_px)&(uv[:,1]>=image_margin_px)&(uv[:,1]<height-image_margin_px)
    obstacles=np.asarray(obstacles_map_m,float)
    if obstacles.size:
        obstacles=cloud(obstacles)@transform[:3,:3].T+transform[:3,3]
        good=obstacles[:,2]>.1
        pixels=project(obstacles[good]); depths=obstacles[good,2]
        # Pixel-bin z-buffer. Unknown pixels are not called measured free space;
        # this is a view proposal, and the next real capture remains mandatory.
        from scipy.spatial import cKDTree
        tree=cKDTree(pixels)
        for i in np.flatnonzero(visible):
            ids=tree.query_ball_point(uv[i],3.)
            if ids and min(depths[ids])<camera[i,2]-depth_margin_m: visible[i]=False
    fraction=float(visible.mean())
    return Check(fraction>=minimum_visible_fraction, 'projected_visible' if fraction>=minimum_visible_fraction else 'out_of_view_or_occluded',
                 cost=1-fraction, mode='measured')


def observation_views(surface_points_m, current_xy_yaw, *, stand_off_m=.45, maximum_views=3):
    """Diverse surface-facing positions; callers still check navigation/height/view."""
    points=cloud(surface_points_m,3); center=np.median(points,axis=0)
    lo,hi=points[:,:2].min(axis=0),points[:,:2].max(axis=0)
    candidates=[]
    for xy in ((lo[0]-stand_off_m,center[1]),(hi[0]+stand_off_m,center[1]),
               (center[0],lo[1]-stand_off_m),(center[0],hi[1]+stand_off_m)):
        yaw=float(np.arctan2(center[1]-xy[1],center[0]-xy[0]))
        candidates.append((float(np.linalg.norm(np.asarray(xy)-current_xy_yaw[:2])),(*map(float,xy),yaw)))
    return [p for _,p in sorted(candidates)[:maximum_views]]
