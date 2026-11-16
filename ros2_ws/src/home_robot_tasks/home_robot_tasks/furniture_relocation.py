"""Appearance and measured geometry association with an accepted map landmark.

References are immutable recorded RGB-D evidence. A missing match never erases
memory. This estimates identity/location; it does not establish free space.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .assistant.contracts import document
from .assistant.world import WorldStore
from .recorded_semantics import register_native_depth
from .s2_geometry import rigid


def image_points(depth, intrinsics, camera_to_map):
    matrix=np.asarray(camera_to_map,dtype=float);rigid(matrix)
    yy,xx=np.indices(depth.shape)
    fx,fy,cx,cy=[intrinsics[k] for k in ('fx','fy','ppx','ppy')]
    if not np.isfinite([fx,fy,cx,cy]).all() or min(fx,fy)<=0:
        raise ValueError('finite calibrated intrinsics required')
    xyz=np.stack(((xx-cx)*depth/fx,(yy-cy)*depth/fy,depth),axis=-1)
    return xyz@matrix[:3,:3].T+matrix[:3,3]


def fit_relocation(source, target):
    """Reject collapsed/repeated features before fitting a rigid XY motion."""
    import cv2
    a,b=np.asarray(source,float),np.asarray(target,float)
    if a.shape!=b.shape or a.ndim!=2 or a.shape[1]!=3 or len(a)<8:
        return None
    if not np.isfinite(a).all() or not np.isfinite(b).all():return None
    matrix,mask=cv2.estimateAffinePartial2D(a[:,:2].copy(),b[:,:2].copy(),
        method=cv2.RANSAC,ransacReprojThreshold=.04,maxIters=1500,confidence=.99)
    if matrix is None or mask is None:return None
    scale=float(np.linalg.norm(matrix[:,0]));keep=mask.ravel().astype(bool)
    if not .9<=scale<=1.1 or keep.sum()<8 or keep.mean()<.55:return None
    a,b=a[keep],b[keep]
    # Multiple SIFT orientations at one pixel are not independent geometry.
    if min(len(np.unique(np.rint(x/.01),axis=0)) for x in (a,b))<6:return None
    if min(np.linalg.eigvalsh(np.cov(x[:,:2].T))[0] for x in (a,b))<.0001:return None
    if min(np.linalg.norm(np.ptp(x[:,:2],axis=0)) for x in (a,b))<.15:return None
    ac,bc=a[:,:2].mean(0),b[:,:2].mean(0)
    u,_,vt=np.linalg.svd((a[:,:2]-ac).T@(b[:,:2]-bc));rotation=vt.T@u.T
    if np.linalg.det(rotation)<0:return None
    translation=bc-rotation@ac
    residual=np.linalg.norm(a[:,:2]@rotation.T+translation-b[:,:2],axis=1)
    dz=b[:,2]-a[:,2]
    if np.sqrt(np.mean(residual**2))>.035 or np.quantile(abs(dz),.9)>.06:return None
    if np.linalg.norm(translation)>3.:return None
    transform=np.eye(4);transform[:2,:2]=rotation;transform[:2,3]=translation
    return dict(matrix=transform.tolist(),inliers=int(keep.sum()),
        inlier_fraction=float(keep.mean()),rms_m=float(np.sqrt(np.mean(residual**2))),
        scale_check=scale,method='appearance_geometry_reidentification')


class FurnitureRelocalizer:
    def __init__(self, folder):
        import cv2
        self.folder=Path(folder);self.reference=json.loads((self.folder/'reference.json').read_text())
        raw=(self.folder/'features.npz').read_bytes()
        if hashlib.sha256(raw).hexdigest()!=self.reference['features_sha256']:
            raise ValueError('furniture reference hash mismatch')
        self.features=dict(np.load(self.folder/'features.npz',allow_pickle=False))
        self.detector=cv2.SIFT_create(nfeatures=1600,contrastThreshold=.015)

    def locate(self, color, native, metadata, camera_to_map):
        import cv2
        depth=register_native_depth(native,metadata)
        points=image_points(depth,metadata['color_intrinsics'],camera_to_map)
        valid=(depth>=.15)&(depth<=4.)
        mask=cv2.erode(valid.astype('uint8')*255,np.ones((3,3),np.uint8))
        keys,des=self.detector.detectAndCompute(color,mask)
        if des is None or len(keys)<8:
            return self.locate_planar(color,native,metadata,camera_to_map)
        uv=np.rint([k.pt for k in keys]).astype(int)
        xyz=points[uv[:,1],uv[:,0]];ok=valid[uv[:,1],uv[:,0]]
        des,xyz=des[ok],xyz[ok]
        if len(des)<8:return self.locate_planar(color,native,metadata,camera_to_map)
        matcher=cv2.BFMatcher();proposals=[]
        for view in self.reference['views']:
            name=str(view);old=self.features[name+'_des'];origin=self.features[name+'_xyz']
            back={m.queryIdx:m.trainIdx for m,n in matcher.knnMatch(des,old,k=2)
                  if m.distance<.8*n.distance}
            pairs=[m for m,n in matcher.knnMatch(old,des,k=2)
                   if m.distance<.75*n.distance and back.get(m.trainIdx)==m.queryIdx]
            if len(pairs)<8:continue
            found=fit_relocation([origin[m.queryIdx] for m in pairs],[xyz[m.trainIdx] for m in pairs])
            if found:proposals.append(dict(found,reference_view=view))
        if not proposals:return self.locate_planar(color,native,metadata,camera_to_map)
        return self._select(proposals)

    def _select(self, proposals):
        if not proposals:return None
        anchor=np.asarray(self.reference['anchor_xyz_m'])
        def center(p):
            m=np.asarray(p['matrix']);return (m[:3,:3]@anchor+m[:3,3])
        best=max(proposals,key=lambda p:p['inliers'])
        if any(np.linalg.norm(center(p)-center(best))>.12 for p in proposals):return None
        return dict(best,furniture_id=self.reference['furniture_id'],
                    anchor_xyz_m=center(best).tolist(),reference_revision=self.reference['revision'])

    def locate_planar(self,color,native,metadata,camera_to_map):
        """Perspective-normalized appearance, confirmed against measured shape.

        Pure shape cannot disambiguate identical furniture. Keep mutual texture
        matches, independent 3-D support and the original rigid-fit checks.
        """
        import cv2
        from scipy.spatial import cKDTree
        from .furniture_geometry import orthographic_features
        views=self.reference.get('planar_views',[])
        if not views:return None
        surfaces=orthographic_features(color,native,metadata,camera_to_map,
            height_m=float(np.median([v['height_m'] for v in views])))
        matcher=cv2.BFMatcher();proposals=[]
        for surface in surfaces:
            des,xyz=surface['des'],surface['xyz']
            for view in views:
                name=view['name'];old=self.features[name+'_des'];origin=self.features[name+'_xyz']
                if len(old)<8:continue
                back={m.queryIdx:m.trainIdx for m,n in matcher.knnMatch(des,old,k=2)
                      if m.distance<.8*n.distance}
                pairs=[m for m,n in matcher.knnMatch(old,des,k=2)
                       if m.distance<.75*n.distance and back.get(m.trainIdx)==m.queryIdx]
                if len(pairs)<8:continue
                found=fit_relocation([origin[m.queryIdx] for m in pairs],
                                     [xyz[m.trainIdx] for m in pairs])
                if not found:continue
                matrix=np.asarray(found['matrix']);boundary=self.features[name+'_boundary']
                boundary=boundary@matrix[:2,:2].T+matrix[:2,3]
                distance=cKDTree(boundary).query(surface['boundary_xy_m'])[0]
                # Compare only the measured current outline, excluding crop
                # edges. Occluded original edges need not be visible now.
                fraction=float(np.mean(distance<.06))
                if fraction<.7:continue
                proposals.append(dict(found,reference_view=view['view'],
                    appearance_projection='measured_horizontal_surface',
                    shape_boundary_inlier_fraction=fraction,
                    shape_boundary_median_m=float(np.median(distance))))
        return self._select(proposals)


class FurnitureLocationMemory:
    """Persist only repeated, geometrically consistent observations.

    A new clock starts with no votes, but the last verified location survives.
    Writes use the existing atomic WorldStore; absent detections do not write.
    """
    def __init__(self, reference, store, clock_id):
        self.reference=reference;self.store=store;self.clock_id=clock_id
        self.binding=hashlib.sha256(document(reference).encode()).hexdigest()
        self.key=reference['furniture_id'];self.last_stamp=None;self.votes=[]
        self.saved=store.record('furniture_last_location',self.key)
        if self.saved and self.saved['reference_binding']!=self.binding:
            raise ValueError('saved furniture reference differs')

    def observe(self, proposal, *, capture_id, observed_s, now_s, localization_qualified):
        if (not localization_qualified or not math.isfinite(observed_s) or
                not 0<=now_s-observed_s<=5. or
                self.last_stamp is not None and observed_s<=self.last_stamp):
            raise ValueError('fresh localized furniture exposure required')
        self.last_stamp=observed_s
        if proposal is None:self.votes=[];return None
        if (proposal['furniture_id']!=self.key or
                proposal['reference_revision']!=self.reference['revision'] or
                proposal['method']!='appearance_geometry_reidentification'):
            raise ValueError('registered furniture identity required')
        matrix=np.asarray(proposal['matrix']);rigid(matrix)
        anchor=matrix[:3,:3]@np.asarray(self.reference['anchor_xyz_m'])+matrix[:3,3]
        if not np.allclose(anchor,proposal['anchor_xyz_m'],atol=1e-6,rtol=0):
            raise ValueError('furniture anchor differs from measured transform')
        if self.votes:
            previous=self.votes[-1]
            yaw=lambda p:math.atan2(p['matrix'][1][0],p['matrix'][0][0])
            turn=(yaw(proposal)-yaw(previous[2])+math.pi)%(2*math.pi)-math.pi
            if (observed_s-previous[0]>1.5 or abs(turn)>.15 or
                    math.dist(proposal['anchor_xyz_m'],previous[2]['anchor_xyz_m'])>.08):self.votes=[]
        if any(v[1]==capture_id for v in self.votes):raise ValueError('capture reused')
        self.votes.append((observed_s,capture_id,proposal));self.votes=self.votes[-8:]
        if len(self.votes)<3 or observed_s-self.votes[0][0]<2.:return None
        result=dict(proposal,reference_binding=self.binding,clock_id=self.clock_id,
            capture_id=capture_id,observed_s=observed_s,
            evidence_capture_ids=[v[1] for v in self.votes],identity_verified=True,
            occupancy_clearance_authorized=False)
        self.store.save_record('furniture_last_location',self.key,result)
        self.saved=result
        return result
