"""Measured horizontal surfaces and perspective-normalized furniture appearance.

Shape is a location cue, not proof that two identical tables are the same item.
Cropped image edges are excluded; returned feature positions retain real depth.
These observations never authorize removing occupied map cells on their own.
"""

import numpy as np

from .furniture_relocation import image_points
from .recorded_semantics import register_native_depth


def planar_surfaces(color, native, metadata, camera_to_map, *, height_m,
                    height_tolerance_m=.06, minimum_pixels=1000):
    """Extract separate horizontal components at a registered surface height."""
    import cv2
    depth=register_native_depth(native,metadata)
    if depth.shape!=color.shape[:2]:raise ValueError('registered image sizes differ')
    xyz=image_points(depth,metadata['color_intrinsics'],camera_to_map)
    valid=np.isfinite(depth)&(depth>.15)&(depth<4.)
    mask=(valid&(abs(xyz[:,:,2]-height_m)<height_tolerance_m)).astype('uint8')
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
    result=[]
    for index in range(1,count):
        if stats[index,cv2.CC_STAT_AREA]<minimum_pixels:continue
        component=(labels==index).astype('uint8')
        # Close tiny depth holes only; coordinates always come from real returns.
        shell=cv2.morphologyEx(component,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
        contours,_=cv2.findContours(shell,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        if not contours:continue
        uv=max(contours,key=cv2.contourArea).reshape(-1,2)
        h,w=depth.shape
        real=(uv[:,0]>3)&(uv[:,0]<w-4)&(uv[:,1]>3)&(uv[:,1]<h-4)
        real&=component[uv[:,1],uv[:,0]]>0
        all_points=xyz[component>0];height=float(np.median(all_points[:,2]))
        z=xyz[uv[:,1],uv[:,0],2];real&=abs(z-height)<.045
        boundary=xyz[uv[real,1],uv[real,0],:2]
        boundary=_unique(boundary)
        if len(boundary)<30:continue
        # A tilted surface is not the registered horizontal support surface.
        cloud=all_points[::max(1,len(all_points)//2000)]
        design=np.c_[cloud[:,:2],np.ones(len(cloud))]
        plane=np.linalg.lstsq(design,cloud[:,2],rcond=None)[0]
        residual=cloud[:,2]-design@plane
        if np.linalg.norm(plane[:2])>.12 or np.quantile(abs(residual),.9)>.025:continue
        result.append(dict(boundary_xy_m=boundary,cloud_xyz_m=cloud,
            height_m=height,plane_rms_m=float(np.sqrt(np.mean(residual**2))),
            clipped=bool(np.any(~((uv[:,0]>3)&(uv[:,0]<w-4)&(uv[:,1]>3)&(uv[:,1]<h-4)))),
            boundary_pixels=uv[real],mask=component,pixel_count=int(stats[index,cv2.CC_STAT_AREA])))
    return result


def _unique(points, resolution=.01):
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=2 or not np.isfinite(points).all():
        raise ValueError('finite XY points required')
    _,indices=np.unique(np.rint(points/resolution),axis=0,return_index=True)
    return points[np.sort(indices)]


def orthographic_features(color, native, metadata, camera_to_map, *, height_m):
    """Use a measured plane to remove perspective, retain measured XYZ per key.

    The view plane only selects image sampling coordinates. Returned feature
    positions come from registered depth, not an invented perfectly flat table.
    """
    import cv2
    surfaces=planar_surfaces(color,native,metadata,camera_to_map,height_m=height_m)
    depth=register_native_depth(native,metadata)
    points=image_points(depth,metadata['color_intrinsics'],camera_to_map)
    tf=np.asarray(camera_to_map,float);intrinsics=metadata['color_intrinsics']
    detector=cv2.SIFT_create(nfeatures=2000,contrastThreshold=.008)
    results=[];resolution=.003
    for surface in surfaces:
        cloud=points[surface['mask']>0];lo=cloud[:,:2].min(0)-.02;hi=cloud[:,:2].max(0)+.02
        shape=np.ceil((hi-lo)[::-1]/resolution).astype(int)
        if min(shape)<10 or np.prod(shape)>2000000:continue
        yy,xx=np.indices(tuple(shape))
        plane=np.stack((lo[0]+xx*resolution,lo[1]+yy*resolution,
                        np.full(xx.shape,surface['height_m'])),axis=-1)
        cam=(plane-tf[:3,3])@tf[:3,:3]
        if np.any(cam[:,:,2]<=.05):continue
        u=(cam[:,:,0]/cam[:,:,2]*intrinsics['fx']+intrinsics['ppx']).astype('float32')
        v=(cam[:,:,1]/cam[:,:,2]*intrinsics['fy']+intrinsics['ppy']).astype('float32')
        view=cv2.remap(color,u,v,cv2.INTER_LINEAR)
        mask=cv2.remap(surface['mask']*255,u,v,cv2.INTER_NEAREST)
        mask=cv2.erode(mask,np.ones((7,7),np.uint8))
        keys,des=detector.detectAndCompute(view,mask)
        if des is None or len(keys)<8:continue
        uv=np.rint([k.pt for k in keys]).astype(int)
        source_u=np.rint(u[uv[:,1],uv[:,0]]).astype(int)
        source_v=np.rint(v[uv[:,1],uv[:,0]]).astype(int)
        good=(source_u>=0)&(source_u<depth.shape[1])&(source_v>=0)&(source_v<depth.shape[0])
        indices=np.flatnonzero(good);source_u,source_v=source_u[good],source_v[good]
        good_depth=surface['mask'][source_v,source_u]>0
        indices=indices[good_depth];source_u,source_v=source_u[good_depth],source_v[good_depth]
        if len(indices)<8:continue
        results.append(dict(des=des[indices],xyz=points[source_v,source_u],
            uv=uv[indices],image=view,height_m=surface['height_m'],
            boundary_xy_m=surface['boundary_xy_m'],clipped=surface['clipped']))
    return results
