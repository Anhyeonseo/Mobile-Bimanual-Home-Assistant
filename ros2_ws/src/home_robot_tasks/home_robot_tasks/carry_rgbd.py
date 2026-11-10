"""Image/depth producer for the stationary pick monitor, without device I/O.

Track the same textured surface patch; do not redetect another object when it
is hidden. Optical flow is checked in both directions. All selected points must
survive, so changing a visible subset cannot shift the reported centroid.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np

from .s2_geometry import rigid


@dataclass(frozen=True)
class PatchObservation:
    sequence: int
    observed_s: float
    xyz_base_m: tuple
    error_bound_m: float
    patch_id: str


class SurfacePatchTracker:
    def __init__(self, color, depth_m, mask, intrinsics, base_from_optical, *,
                 patch_id, observed_s, delta_error_floor_m):
        if (not patch_id or not math.isfinite(delta_error_floor_m)
                or not 0 < delta_error_floor_m <= .02):
            raise ValueError('explicit patch identity and depth-delta error budget required')
        self.intr = dict(intrinsics)
        if (self.intr.get('model') not in ('distortion.none','distortion.brown_conrady','distortion.inverse_brown_conrady')
                or any(abs(x)>1e-10 for x in self.intr.get('coeffs',()))
                or min(self.intr['fx'],self.intr['fy'])<=0):
            raise ValueError('rectified aligned color-depth intrinsics required')
        self.transform = rigid(base_from_optical)
        self.floor, self.patch_id = delta_error_floor_m, patch_id
        self.gray = self._gray(color)
        if mask.shape != self.gray.shape or depth_m.shape != mask.shape or mask.dtype != np.bool_:
            raise ValueError('target mask and metric depth must match the color exposure')
        interior=cv2.erode(mask.astype(np.uint8),np.ones((5,5),np.uint8))
        interior[(~np.isfinite(depth_m)) | (depth_m<=0)] = 0
        features=cv2.goodFeaturesToTrack(self.gray,maxCorners=80,qualityLevel=.02,
                                         minDistance=4,mask=interior,blockSize=5)
        if features is None or len(features)<6:
            raise ValueError('NO_VISIBLE_TEXTURE_PATCH')
        xy=features.reshape(-1,2)
        # A compact fixed set avoids using changing object silhouettes as points.
        distances=np.linalg.norm(xy[:,None,:]-xy[None,:,:],axis=2)
        center=int(np.argmin(np.sort(distances,axis=1)[:,5]))
        indices=np.argsort(distances[center])[:6]
        self.points=xy[indices].reshape(-1,1,2).astype(np.float32)
        if not math.isfinite(observed_s) or observed_s < 0:
            raise ValueError('valid exposure timestamp required')
        self.sequence=0;self.stamp=float(observed_s);self.failed=False
        self.initial=self._measure(depth_m,observed_s)

    @staticmethod
    def _gray(color):
        if not isinstance(color,np.ndarray) or color.dtype!=np.uint8 or color.ndim!=3 or color.shape[2]!=3:
            raise ValueError('BGR uint8 exposure required')
        return cv2.cvtColor(color,cv2.COLOR_BGR2GRAY)

    def _measure(self,depth_m,stamp):
        if depth_m.shape!=self.gray.shape:
            raise ValueError('aligned depth shape changed')
        points=[];noise=[]
        for u,v in self.points.reshape(-1,2):
            x,y=int(round(float(u))),int(round(float(v)))
            if not 2<=x<depth_m.shape[1]-2 or not 2<=y<depth_m.shape[0]-2:
                raise ValueError('PATCH_LEFT_IMAGE')
            z=depth_m[y-2:y+3,x-2:x+3]
            z=z[np.isfinite(z)&(z>0)]
            if len(z)<20: raise ValueError('PATCH_DEPTH_MISSING')
            median=float(np.median(z))
            if np.ptp(z)>.01: raise ValueError('PATCH_DEPTH_EDGE')
            noise.append(float(np.median(abs(z-median)))*1.4826)
            points.append([(u-self.intr['ppx'])*median/self.intr['fx'],
                           (v-self.intr['ppy'])*median/self.intr['fy'],median])
        xyz=self.transform[:3,:3]@np.mean(points,axis=0)+self.transform[:3,3]
        return PatchObservation(self.sequence,float(stamp),tuple(map(float,xyz)),
                                max(self.floor,max(noise)),self.patch_id)

    def update(self,color,depth_m,stamp):
        if self.failed: raise ValueError('PATCH_TRACKING_LOST')
        try:
            if not math.isfinite(stamp) or not 0<stamp-self.stamp<=.2:
                raise ValueError('PATCH_FRAME_STALE')
            gray=self._gray(color)
            if gray.shape!=self.gray.shape: raise ValueError('camera shape changed')
            next_points,status,_=cv2.calcOpticalFlowPyrLK(self.gray,gray,self.points,None,winSize=(21,21),maxLevel=3)
            if next_points is None or not np.all(status): raise ValueError('PATCH_OCCLUDED')
            back,back_status,_=cv2.calcOpticalFlowPyrLK(gray,self.gray,next_points,None,winSize=(21,21),maxLevel=3)
            if (back is None or not np.all(back_status) or not np.isfinite(next_points).all()
                    or np.max(np.linalg.norm(back-self.points,axis=2))>.75
                    or np.max(np.linalg.norm(next_points-self.points,axis=2))>40):
                raise ValueError('PATCH_FLOW_INCONSISTENT')
            self.gray=gray;self.points=next_points
            self.sequence+=1;self.stamp=float(stamp)
            return self._measure(depth_m,stamp)
        except (ValueError,cv2.error):
            self.failed=True
            raise


class SceneValidationError(ValueError):
    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


class StationarySceneCheck:
    """Compare a fresh fixed-camera exposure with the planned scene before ARM."""
    def __init__(self, color, depth_m, mask, *, maximum_background_depth_m=None):
        if maximum_background_depth_m is not None and (not math.isfinite(maximum_background_depth_m) or maximum_background_depth_m <= 0):
            raise ValueError('positive finite scene depth bound required')
        self.maximum_depth = maximum_background_depth_m
        self.color=SurfacePatchTracker._gray(color)
        self.depth=np.asarray(depth_m,dtype=float).copy()
        self.target=np.asarray(mask,dtype=bool).copy()
        self.background=~cv2.dilate(mask.astype(np.uint8),np.ones((21,21),np.uint8)).astype(bool)

    def compare(self,color,depth_m,*,background_only=False,target_only=False):
        if background_only and target_only:raise ValueError('one observation region required')
        gray=SurfacePatchTracker._gray(color)
        if gray.shape!=self.color.shape or depth_m.shape!=self.depth.shape:
            raise ValueError('scene stream geometry changed')
        checks={}
        for name,area in (('target',self.target),('background',self.background)):
            if background_only and name=='target':continue
            if target_only and name=='background':continue
            valid=area&(self.depth>0)&(depth_m>0)&np.isfinite(depth_m)
            coverage=float(valid.sum()/max(1,area.sum()))
            if valid.sum()<30:
                checks[name]=dict(unchanged=False,valid_fraction=coverage);continue
            depth_area=area
            entrants=0
            largest_entrant=0
            if name=='background' and self.maximum_depth is not None:
                old_near=(self.depth>0)&np.isfinite(self.depth)&(self.depth<=self.maximum_depth)
                new_near=(depth_m>0)&np.isfinite(depth_m)&(depth_m<=self.maximum_depth)
                # Either exposure may bring an obstacle into the reachable slab.
                # Do not scope solely by the old background or by screen rows.
                depth_area=area&(old_near|new_near)
                # A recovered missing depth sample is not measured motion from
                # far to near. It still counts in coverage, never as free space.
                nearer=area&(self.depth>0)&np.isfinite(self.depth)&new_near&(self.depth-depth_m>.006)
                entrants=int(nearer.sum())
                if entrants:
                    # A small connected entrant matters even inside the existing
                    # workspace. Isolated stereo speckles across the whole image
                    # must not add up to one object. Preserve thin/diagonal edges;
                    # no erosion or morphological opening is applied.
                    _,_,components,_=cv2.connectedComponentsWithStats(nearer.astype(np.uint8),connectivity=8)
                    largest_entrant=int(components[1:,cv2.CC_STAT_AREA].max())
            depth_valid=valid&depth_area
            depth_coverage=float(depth_valid.sum()/max(1,depth_area.sum()))
            depth95=float(np.quantile(abs(depth_m[depth_valid]-self.depth[depth_valid]),.95)) if depth_valid.any() else 0.
            # A retreating background surface leaves the original, more occupied
            # collision scene conservative. Never change that scene or allow this
            # rule for the target. Invalid/missing depth is still unknown.
            nearer95=(float(np.quantile(np.maximum(self.depth[depth_valid]-depth_m[depth_valid],0.),.95))
                      if depth_valid.any() else 0.)
            checked95=nearer95 if name=='background' and self.maximum_depth is not None else depth95
            depth_ok=(not depth_area.any() or (depth_valid.sum()>=30 and depth_coverage>=.7 and checked95<=.006))
            image_change=float(np.mean(abs(gray[valid].astype(float)-self.color[valid].astype(float))>35))
            checks[name]=dict(unchanged=bool(coverage>=.7 and depth_ok and image_change<=.1 and largest_entrant<30),
                              valid_fraction=coverage,depth_difference95_m=depth95,image_changed_fraction=image_change,
                              depth_checked_pixels=int(depth_valid.sum()),depth_valid_fraction=depth_coverage,
                              entering_workspace_pixels=entrants,largest_entering_component_pixels=largest_entrant,
                              nearer_depth_difference95_m=nearer95)
        return dict(unchanged=all(v['unchanged'] for v in checks.values()),regions=checks)
