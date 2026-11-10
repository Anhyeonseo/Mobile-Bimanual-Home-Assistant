"""Observed footprint centrality; never a measured centre of mass or grip force."""
import numpy as np
from scipy.spatial import ConvexHull, QhullError


def footprint_reference(points, normal):
    """Area centroid avoids favouring the densely sampled/near end of an object."""
    points=np.asarray(points,float);normal=np.asarray(normal,float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<3 or not np.isfinite(points).all():
        return None
    if normal.shape!=(3,) or not np.isfinite(normal).all() or not np.isclose(np.linalg.norm(normal),1.):
        raise ValueError('unit support normal required')
    x=np.eye(3)[np.argmin(abs(normal))];x-=normal*(x@normal);x/=np.linalg.norm(x)
    axes=np.column_stack((x,np.cross(normal,x)))
    origin=points.mean(axis=0);xy=(points-origin)@axes
    try:polygon=xy[ConvexHull(xy).vertices]
    except QhullError:return None
    following=np.roll(polygon,-1,axis=0)
    cross=polygon[:,0]*following[:,1]-following[:,0]*polygon[:,1]
    area2=float(cross.sum())
    if abs(area2)<1e-12:return None
    center=((polygon+following)*cross[:,None]).sum(axis=0)/(3*area2)
    return dict(center=origin+axes@center,vertices=origin+polygon@axes.T,normal=normal)


def contact_centrality(tip,closing_axis,reference):
    """Distance along the footprint transverse to the pinch line, scale-normalized."""
    if reference is None:return dict(status='UNRESOLVED',measured_com=False)
    lateral=np.cross(reference['normal'],closing_axis);length=np.linalg.norm(lateral)
    if length<1e-8:return dict(status='UNRESOLVED',measured_com=False)
    lateral/=length
    offset=abs(float((reference['center']-tip)@lateral))
    radius=max(float(np.max(abs((reference['vertices']-reference['center'])@lateral))),1e-6)
    return dict(status='FOOTPRINT_PROXY',center_m=reference['center'].tolist(),
        center_basis='observed_convex_footprint_area_centroid',
        transverse_offset_m=offset,transverse_radius_m=radius,normalized_offset=offset/radius,
        measured_com=False,force_closure_verified=False)
