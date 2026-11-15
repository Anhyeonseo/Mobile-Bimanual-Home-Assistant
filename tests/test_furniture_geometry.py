import hashlib
import json

import cv2
import numpy as np
import pytest

from home_robot_tasks.furniture_geometry import planar_surfaces,orthographic_features
from home_robot_tasks.furniture_relocation import FurnitureRelocalizer


def scene(seed=4,clip=False):
    h,w=240,320
    rng=np.random.default_rng(seed)
    texture=cv2.GaussianBlur(rng.integers(0,256,(h,w),dtype=np.uint8),(5,5),0)
    color=cv2.cvtColor(texture,cv2.COLOR_GRAY2BGR)
    y,x=np.indices((h,w));mask=((x-(35 if clip else 160))/125)**2+((y-120)/90)**2<1
    raw=np.where(mask,650,0).astype('uint16')
    intr=dict(width=w,height=h,fx=250.,fy=250.,ppx=160.,ppy=120.,coeffs=[0.]*5)
    meta=dict(native_depth_intrinsics=intr,color_intrinsics=intr,depth_scale_m=.001,
        depth_to_color=dict(rotation_column_major=np.eye(3).ravel().tolist(),translation_m=[0.]*3))
    tf=np.diag([1.,-1.,-1.,1.]);tf[2,3]=1.
    return color,raw,meta,tf


def test_clipped_camera_border_is_not_a_furniture_edge():
    color,raw,meta,tf=scene(clip=True)
    surfaces=planar_surfaces(color,raw,meta,tf,height_m=.35)
    assert len(surfaces)==1 and surfaces[0]['clipped']
    pixels=surfaces[0]['boundary_pixels']
    assert pixels[:,0].min()>3 and pixels[:,0].max()<316
    assert surfaces[0]['height_m']==pytest.approx(.35)


def test_features_keep_measured_depth_and_empty_depth_is_not_filled():
    color,raw,meta,tf=scene()
    raw[80:140,120:190]=640  # measured 10 mm relief on the same surface
    surfaces=orthographic_features(color,raw,meta,tf,height_m=.35)
    xyz=surfaces[0]['xyz']
    assert np.ptp(xyz[:,2])==pytest.approx(.01)
    assert set(np.round(xyz[:,2],3))=={.35,.36}
    assert not orthographic_features(color,raw*0,meta,tf,height_m=.35)
    assert not orthographic_features(color,raw,meta,tf,height_m=.8)


def reference(tmp_path):
    color,raw,meta,tf=scene()
    s=orthographic_features(color,raw,meta,tf,height_m=.35)[0]
    np.savez(tmp_path/'features.npz',p_des=s['des'],p_xyz=s['xyz'],p_boundary=s['boundary_xy_m'])
    ref=dict(features_sha256=hashlib.sha256((tmp_path/'features.npz').read_bytes()).hexdigest(),
        views=[],planar_views=[dict(name='p',view=0,height_m=.35)],
        anchor_xyz_m=[0,0,.35],furniture_id='table',revision='test')
    (tmp_path/'reference.json').write_text(json.dumps(ref))
    return FurnitureRelocalizer(tmp_path),color,raw,meta,tf


def test_planar_fallback_recovers_translation_without_weakening_rigid_fit(tmp_path):
    matcher,color,raw,meta,tf=reference(tmp_path)
    tf[:2,3]=[.3,-.1]
    p=matcher.locate(color,raw,meta,tf)
    assert p is not None
    assert p['anchor_xyz_m']==pytest.approx([.3,-.1,.35],abs=.005)
    assert p['appearance_projection']=='measured_horizontal_surface'
    assert p['inliers']>=8 and p['shape_boundary_inlier_fraction']>.9


def test_same_shape_with_different_or_missing_texture_is_not_same_identity(tmp_path):
    matcher,_,raw,meta,tf=reference(tmp_path)
    other=scene(seed=805)[0]
    assert matcher.locate(other,raw,meta,tf) is None
    assert matcher.locate(np.full_like(other,180),raw,meta,tf) is None


def test_disagreeing_reference_placements_are_not_saved(tmp_path):
    matcher,color,raw,meta,tf=reference(tmp_path)
    matcher.reference['planar_views'].append(dict(name='q',view=1,height_m=.35))
    for suffix in ('des','xyz','boundary'):
        matcher.features['q_'+suffix]=matcher.features['p_'+suffix].copy()
    matcher.features['q_xyz'][:,:2]+=[.4,0]
    matcher.features['q_boundary']+=[.4,0]
    assert matcher.locate(color,raw,meta,tf) is None
