"""Geometric D415 mounting check, not evidence of obstacle detection."""
import math
import numpy as np
from .navigation import number
from .fetch import InvalidTask


def frustum_coverage(points_body, *, height_m, pitch_down_rad, camera_x_m,
                     width,height,fx,fy,cx,cy,minimum_depth_m,maximum_depth_m):
    for v in (height_m,pitch_down_rad,camera_x_m,fx,fy,cx,cy,minimum_depth_m,maximum_depth_m):number(v,'camera geometry')
    points=np.asarray(points_body,float)
    if (points.ndim!=2 or points.shape[1]!=3 or len(points)>100000 or not np.isfinite(points).all()
            or type(width) is not int or type(height) is not int or min(width,height,fx,fy)<=0
            or not 0<=cx<width or not 0<=cy<height or not 0<minimum_depth_m<maximum_depth_m):
        raise InvalidTask('invalid camera coverage inputs')
    c,s=math.cos(pitch_down_rad),math.sin(pitch_down_rad)
    p=points-np.array([camera_x_m,0,height_m])
    # x right, y down, z forward in optical frame. Positive pitch looks down.
    optical=np.column_stack((-p[:,1],-s*p[:,0]-c*p[:,2],c*p[:,0]-s*p[:,2]))
    z=optical[:,2]
    u=fx*optical[:,0]/np.where(z!=0,z,1)+cx
    v=fy*optical[:,1]/np.where(z!=0,z,1)+cy
    inside=(z>=minimum_depth_m)&(z<=maximum_depth_m)&(u>=0)&(u<width)&(v>=0)&(v<height)
    return inside.tolist()
