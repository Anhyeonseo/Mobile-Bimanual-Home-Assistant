"""Ray-rendered held geometry viewed from different camera/object placements.

Ground-truth labels come from ray/solid intersection, not from the ownership
implementation. These are synthetic geometry regressions, not D415 accuracy.
"""
import numpy as np
import pytest
from scipy.ndimage import binary_erosion
from scipy.spatial.transform import Rotation

from home_perception.geometry.pick_scene import build_pick_scene
from home_perception.geometry.plane import Plane, backproject


def render(pitch, yaw, offset, erosion):
    angle=np.deg2rad(pitch)
    camera=np.array([0.,-.65*np.cos(angle),.65*np.sin(angle)])
    forward=-camera/np.linalg.norm(camera)
    right=np.cross(forward,[0.,0.,1.]);right/=np.linalg.norm(right)
    rotation=np.column_stack((right,np.cross(forward,right),forward))
    intr=dict(fx=330.,fy=330.,ppx=160.,ppy=120.)
    v,u=np.indices((240,320))
    rays=np.stack(((u-160)/330,(v-120)/330,np.ones_like(u)),axis=-1)@rotation.T
    depth=-camera[2]/rays[:,:,2]
    depth=np.where(depth>0,depth,np.inf)
    labels=np.zeros_like(u)
    object_rotation=Rotation.from_euler('z',yaw,degrees=True).as_matrix()
    for label,center,size in ((1,[offset,0.,.01],[.14,.035,.02]),(2,[offset,.075,.02],[.035,.035,.04])):
        origin=(camera-center)@object_rotation
        direction=rays@object_rotation
        with np.errstate(divide='ignore',invalid='ignore'):
            a=(-np.array(size)/2-origin)/direction
            b=(np.array(size)/2-origin)/direction
        near=np.max(np.minimum(a,b),axis=-1);far=np.min(np.maximum(a,b),axis=-1)
        hit=(near>0)&(near<=far)&(near<depth)
        depth[hit]=near[hit];labels[hit]=label
    depth[~np.isfinite(depth)]=0.
    plane=Plane(rotation.T@np.array([0.,0.,1.]),-rotation.T@camera,.0005, .9)
    mask=binary_erosion(labels==1,iterations=erosion)
    cloud=backproject(depth,intr,mask)
    cloud=cloud[plane.height(cloud)>.003]
    return depth,intr,plane,mask,cloud,labels


@pytest.mark.parametrize('pitch,yaw,offset',[
    (30,0,-.08),(30,45,0.),(30,90,.08),
    (45,0,0.),(45,45,.08),(45,90,-.08),
    (60,0,.08),(60,45,-.08),(60,90,0.)])
@pytest.mark.parametrize('erosion',[1,2])
def test_surface_ownership_across_viewpoints(pitch,yaw,offset,erosion,record_property):
    depth,intr,plane,mask,cloud,labels=render(pitch,yaw,offset,erosion)
    scene=build_pick_scene(depth,intr,plane,mask,cloud,protected=labels==2)
    assert scene.ownership_assessment['status']=='CONSISTENT'
    assert not np.any(scene.target_pixels&(labels!=1))
    heights=np.zeros_like(depth);heights[depth>0]=plane.height(backproject(depth,intr,depth>0))
    body=(labels==1)&(heights>.005)
    # A one-pixel grazing side can be disconnected in measured 3-D. Record
    # recall instead of pretending this fixture proves perfect segmentation.
    # Every ambiguous return stays occupied and is explicitly reported.
    missed=body&~scene.target_pixels
    record_property('visible_target_recall',float(scene.target_pixels[body].mean()))
    record_property('unassociated_target_returns',int(missed.sum()))
    assert np.all(scene.uncertain_target_pixels[missed])
    world=backproject(depth,intr,missed)
    support=world@scene.support_from_optical[:3,:3].T+scene.support_from_optical[:3,3]
    boxes=np.asarray(scene.nominal_obstacle_boxes)
    for point in support:
        assert np.any(np.all((point>=boxes[:,0])&(point<=boxes[:,1]),axis=1))
    assert np.any(labels==2) and not np.any(scene.target_pixels[labels==2])
