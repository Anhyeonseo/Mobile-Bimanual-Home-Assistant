"""RGB-D support-plane checks for an explicitly selected image patch.

No placement position is inferred from a room name. The plane test does not
establish the compliance/strength or full clearance of a sofa or mattress.
"""
import math
import numpy as np
from .fetch import InvalidTask
from .navigation import number


class SurfaceSampler:
    def __init__(self, regions):
        self.regions=dict(regions)
        for roi in self.regions.values():
            if (not isinstance(roi,(list,tuple)) or len(roi)!=4 or any(type(v) is not int for v in roi)
                    or not 0<=roi[0]<roi[2] or not 0<=roi[1]<roi[3]):
                raise InvalidTask("explicit image rectangle required")

    def __call__(self,pair,parameters):
        place=parameters.get('destination_place')
        if place is None:return ()
        if place not in self.regions:raise InvalidTask("unregistered destination ROI")
        x0,y0,x1,y1=self.regions[place]
        if x1>=pair.frame.width or y1>=pair.frame.height:raise InvalidTask("surface ROI outside image")
        points=[]
        for v in np.linspace(y0,y1,16).round().astype(int):
            for u in np.linspace(x0,x1,16).round().astype(int):
                z=float(pair.depth_m[v,u])
                if math.isfinite(z) and z>0:points.append((int(u),int(v),z))
        return tuple(dict.fromkeys(points))


class SurfaceObserver:
    def __init__(self,camera,motion,clock,*,maximum_residual_m,minimum_span_m,
                 maximum_tilt_rad,destination_radius_m,minimum_samples=20):
        self.camera,self.motion,self.clock=camera,motion,clock
        self.residual=number(maximum_residual_m,'plane residual')
        self.span=number(minimum_span_m,'observed patch span')
        self.tilt=number(maximum_tilt_rad,'plane tilt')
        self.radius=number(destination_radius_m,'destination radius')
        if (not 0<self.residual<=.05 or not 0<self.radius<=.5 or not 0<self.span<=1
                or not 0<self.tilt<math.pi/2 or type(minimum_samples) is not int or not 6<=minimum_samples<=256):
            raise InvalidTask("explicit bounded plane/placement thresholds required")
        self.minimum=minimum_samples

    def __call__(self,skill,parameters,observation):
        frame,info=observation;samples=info.get('surface_samples_uv_depth')
        if not isinstance(samples,(list,tuple)) or not self.minimum<=len(samples)<=256:
            raise InvalidTask("insufficient observed surface")
        now=self.clock();motion=self.motion()
        self.camera.validate_frame(frame,motion,now_s=now,requested_s=frame.observed_s)
        p=self.camera.policy;points=[];intr=frame.intrinsics
        for u,v,z in samples:
            u,v,z=(number(x,'surface sample') for x in (u,v,z))
            if not 0<=u<frame.width or not 0<=v<frame.height:raise InvalidTask("surface pixel outside image")
            if p.minimum_depth_m<=z<=p.maximum_depth_m:
                points.append(((u-intr.cx)*z/intr.fx,(v-intr.cy)*z/intr.fy,z))
        if len(points)<self.minimum:raise InvalidTask("insufficient valid surface depth")
        matrix=np.asarray(frame.root_from_camera,float)
        points=np.asarray(points)@matrix[:3,:3].T+matrix[:3,3]
        center=np.median(points,axis=0)
        _,singular,axes=np.linalg.svd(points-center,full_matrices=False)
        normal=axes[-1]
        if normal[2]<0:normal=-normal
        distance=np.abs((points-center)@normal)
        # Do not trim away obstructions to claim a clear support surface.
        if (np.max(distance)>self.residual or singular[1]<1e-8
                or math.acos(min(1.,float(normal[2])))>self.tilt):
            raise InvalidTask("surface not a sufficiently flat upward support")
        local=(points-center)@axes[:2].T
        if min(np.ptp(local,axis=0))<self.span:raise InvalidTask("surface coverage too small")
        result=dict(position_m=tuple(map(float,center)),frame_id=frame.root_frame,
            observed_s=frame.observed_s,capture_id=frame.capture_id,pose_revision=frame.pose_revision,
            calibration_id=frame.calibration_id,surface_normal=tuple(map(float,normal)),
            surface_residual_m=float(max(distance)),object_at_destination=False)
        if skill=='verify_delivery':
            detections=[d for d in info['detections'] if d.object_id==parameters['object_id'] and d.confidence>=.7]
            if len(detections)!=1:raise InvalidTask("one independently observed destination object required")
            obj=self.camera.locate(frame,motion,detections[0].samples_uv_depth,now_s=now,requested_s=frame.observed_s)
            delta=np.asarray(obj['position_m'])-center;height=float(delta@normal)
            result['object_at_destination']=bool(-self.residual<=height<=self.radius
                and np.linalg.norm(delta-height*normal)<=self.radius)
        return result
