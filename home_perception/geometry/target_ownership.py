"""Associate observed RGB-D material with a target without filling unseen space.

SAM supplies seeds, not a hard 3-D silhouette. Only adjacent, measured surfaces
can extend those seeds across a bounded image rim. A depth discontinuity, table
return or missing pixel breaks connectivity. Unresolved returns remain occupied.
The limits below are development association budgets, not calibrated sensor errors.
"""
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


@dataclass
class TargetOwnership:
    target: np.ndarray
    uncertain_boundary: np.ndarray
    evidence: dict


def associate(depth, intr, mask, optical, heights, clean_target, plane_rms_m, *, protected=None):
    valid=np.isfinite(depth)&(depth>0)
    if mask.shape!=depth.shape or len(optical)!=int(valid.sum()) or len(heights)!=len(optical):
        raise ValueError('aligned mask, measured depth and point rows required')
    if not np.isfinite(plane_rms_m) or not 0<=plane_rms_m<=.005:
        raise ValueError('bounded observed support residual required')
    protected=np.zeros_like(mask) if protected is None else np.asarray(protected,dtype=bool)
    if protected.shape!=mask.shape or np.any(protected&mask):
        raise ValueError('distinct protected instance pixels must not overlap the selected mask')
    target=np.zeros(depth.shape,bool);uncertain=np.zeros_like(target)
    if len(clean_target)==0:
        return TargetOwnership(target,uncertain,dict(method='observed_surface_connectivity',seed_points=0))
    distance=cKDTree(clean_target).query(optical)[0]
    inside=mask[valid]&(heights>.003)
    seeds=inside&(distance<=.005)
    if not seeds.any():
        return TargetOwnership(target,uncertain,dict(method='observed_surface_connectivity',seed_points=0))

    # Adapt the rim to image sampling and measured plane residual rather than
    # rejecting a whole object at a fixed metric distance to its denoised top.
    focal=min(intr['fx'],intr['fy'])
    spacing=optical[:,2]/focal
    radius=np.clip(2*plane_rms_m+2*spacing,.003,.006)
    image_distance=distance_transform_edt(~mask)[valid]
    rim=(image_distance<=6)&(image_distance*spacing<=radius)
    top=float(np.percentile(heights[seeds],99))
    eligible=(mask[valid]|rim)&(heights>.003)&(heights<=top+np.minimum(.003,2*plane_rms_m+.001))
    eligible|=seeds
    eligible&=~protected[valid]
    pixel_index=np.full(depth.shape,-1,dtype=int)
    valid_index=np.flatnonzero(valid)
    rows=np.flatnonzero(eligible)
    pixel_index.flat[valid_index[rows]]=np.arange(len(rows))
    vertices=optical[rows]
    edges=[]
    for a,b in ((pixel_index[:-1,:],pixel_index[1:,:]),(pixel_index[:,:-1],pixel_index[:,1:])):
        present=(a>=0)&(b>=0);left,right=a[present],b[present]
        local_spacing=np.minimum(vertices[left,2],vertices[right,2])/focal
        limit=np.clip(2*plane_rms_m+2*local_spacing,.0025,.006)
        connected=np.linalg.norm(vertices[left]-vertices[right],axis=1)<=limit
        edges.append((left[connected],right[connected]))
    source=np.concatenate([e[0] for e in edges]);dest=np.concatenate([e[1] for e in edges])
    graph=coo_matrix((np.ones(len(source)),(source,dest)),shape=(len(rows),len(rows))).tocsr()
    _,labels=connected_components(graph,directed=False)
    seed_labels=np.unique(labels[seeds[rows]])
    owned=np.isin(labels,seed_labels)
    target.flat[valid_index[rows[owned]]]=True
    # A grazing side may be far away in 3-D despite being one image pixel from
    # the silhouette. Failing the association radius does not establish a new
    # independent object. Keep such returns occupied, but retain their ambiguity.
    uncertain.flat[valid_index]=(image_distance<=6)&~target[valid]&(heights>.003)&(heights<=top+.003)
    return TargetOwnership(target,uncertain,dict(method='observed_surface_connectivity',
        seed_points=int(seeds.sum()),associated_rim_points=int((target&~mask).sum()),
        unresolved_rim_points=int(uncertain.sum()),maximum_pixels=6,
        radius_m_range=[float(radius.min()),float(radius.max())],
        radius_m_cap=.006,seed_distance_m=.005,adjacency=4,
        protected_instance_pixels=int(protected.sum()),
        missing_depth_filled=False,unresolved_policy='retain_as_occupied',
        sensor_accuracy_calibrated=False))
