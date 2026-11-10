"""Conservative optical-depth bound of both arms and a carried object.

Sum of joint translations plus collision-mesh radius encloses every rotation,
not just the selected trajectory. This scopes background depth revalidation;
it does not remove planning obstacles or alter any collision clearance.
"""
from itertools import product
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def arm_radius(urdf, meshes, *, joint_roots=()):
    robot = ET.parse(urdf).getroot()
    incoming = {j.find('child').get('link'): j for j in robot.findall('joint')}
    lengths = {}

    def distance(link):
        if link not in incoming or link in joint_roots:
            return 0.
        if link not in lengths:
            joint = incoming[link]
            xyz = np.fromstring(joint.find('origin').get('xyz', '0 0 0'), sep=' ')
            lengths[link] = distance(joint.find('parent').get('link')) + np.linalg.norm(xyz)
        return lengths[link]

    radius = 0.
    record = np.dtype([('normal', '<3f4'), ('vertices', '<9f4'), ('attribute', '<u2')])
    for link in robot.findall('link'):
        for collision in link.findall('collision'):
            shape = collision.find('geometry/mesh')
            if shape is None:
                raise ValueError('mesh collision geometry required for arm reach envelope')
            raw = (Path(meshes) / Path(shape.get('filename')).name).read_bytes()
            count = int.from_bytes(raw[80:84], 'little')
            if len(raw) != 84 + 50 * count or count == 0:
                raise ValueError('invalid binary collision STL')
            vertices = np.frombuffer(raw, record, offset=84)['vertices'].reshape(-1, 3)
            scale = np.fromstring(shape.get('scale', '1 1 1'), sep=' ')
            origin = collision.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
            vertices = (vertices * scale) @ Rotation.from_euler('xyz', rpy).as_matrix().T + xyz
            radius = max(radius, distance(link.get('name')) + float(np.linalg.norm(vertices, axis=1).max()))
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError('positive finite arm reach envelope required')
    return radius


def background_depth_guard(arms, profile, grasp, urdf, meshes):
    radius = arm_radius(urdf, meshes)
    if grasp.get('payload_mesh'):
        vertices = np.asarray(grasp['payload_mesh']['vertices'], float)
    else:
        vertices = np.asarray(list(product((-1., 1.), repeat=3))) * np.asarray(grasp['object_size_m']) / 2
    transform = np.asarray(grasp['tcp_from_object'])
    payload_radius = float(np.linalg.norm(vertices @ transform[:3, :3].T + transform[:3, 3], axis=1).max())
    optical = np.linalg.inv(np.asarray(profile['base_from_optical']))
    bounds = {}
    for side, arm in arms.items():
        chain = sum(np.linalg.norm(o[:3, 3]) for o, _ in arm.joints) + np.linalg.norm(arm.tcp[:3, 3])
        bound = max(radius, chain + payload_radius)
        bounds[side] = float((optical @ arm.mount)[2, 3] + bound)
    # Additional observation margin, not a new collision allowance.
    margin = .05
    return dict(kind='full_arm_and_payload_reach_v1', maximum_background_depth_m=max(bounds.values()) + margin,
                per_arm_optical_depth_bound_m=bounds, official_arm_radius_m=radius,
                payload_radius_from_tcp_m=payload_radius, observation_margin_m=margin,
                outside_depth_scope='RGB change still checked; nearer entrants remain checked')


def route_background_depth_guard(arms, profile, grasp, urdf, meshes, program):
    """Bound the actual checked route, including both arms, every jaw angle,
    payload geometry and interpolation between wire samples.

    Collision vertices are enclosed by spheres about their nearest five-axis
    joint origin. Branches such as the moving finger retain their whole radius.
    This narrows the observation scope only; it never removes planning boxes.
    """
    from .s2_geometry import JOINTS, UPSTREAM, model_from_control, digest
    from dataclasses import asdict

    robot=ET.parse(urdf).getroot()
    joints={j.get('name'):j for j in robot.findall('joint')}
    roots={joints[UPSTREAM[0]].find('parent').get('link')}
    roots.update(joints[name].find('child').get('link') for name in UPSTREAM[:5])
    local_radius=arm_radius(urdf,meshes,joint_roots=roots)
    if grasp.get('payload_mesh'):
        vertices=np.asarray(grasp['payload_mesh']['vertices'],float)
    else:
        vertices=np.asarray(list(product((-1.,1.),repeat=3)))*np.asarray(grasp['object_size_m'])/2
    transform=np.asarray(grasp['tcp_from_object'])
    payload_radius=float(np.linalg.norm(vertices@transform[:3,:3].T+transform[:3,3],axis=1).max())
    optical=np.linalg.inv(np.asarray(profile['base_from_optical']))
    bounds={side:-np.inf for side in arms};interpolation={side:0. for side in arms}
    previous={};samples=0
    for stage in program.stages:
        for point in stage.points:
            samples+=1
            for side,offset in (('left',0),('right',6)):
                arm=arms[side]
                q=np.asarray([model_from_control(float(point.positions_rad[offset+i]),
                    profile['arms'][side]['alignment'][n]) for i,n in enumerate(JOINTS[:5])])
                pose,centers=arm.fk(q)
                depths=(optical@np.c_[centers,np.ones(len(centers))].T)[2]
                bound=max(float(depths.max())+local_radius,
                          float((optical@pose)[2,3])+payload_radius)
                bounds[side]=max(bounds[side],bound)
                if side in previous:
                    # Triangle inequality: sum joint angle excursions times a
                    # bound on the distance of any vertex from any joint. This
                    # also encloses the non-linear FK between sampled endpoints.
                    chain=sum(np.linalg.norm(o[:3,3]) for o,_ in arm.joints)
                    reach=2*chain+max(local_radius,np.linalg.norm(arm.tcp[:3,3])+payload_radius)
                    interpolation[side]=max(interpolation[side],float(reach*np.abs(q-previous[side]).sum()))
                previous[side]=q
    if samples<2 or not np.isfinite(list(bounds.values())+list(interpolation.values())).all():
        raise ValueError('finite checked route required for scene scope')
    margin=.05
    maximum=max(bounds[s]+interpolation[s] for s in arms)+margin
    if maximum<=0:raise ValueError('route depth bound must be positive')
    return dict(kind='checked_route_and_payload_depth_v1',maximum_background_depth_m=maximum,
        per_arm_optical_depth_bound_m=bounds,per_arm_interpolation_margin_m=interpolation,
        official_link_radius_m=local_radius,payload_radius_from_tcp_m=payload_radius,
        observation_margin_m=margin,checked_samples=samples,program_sha256=digest(asdict(program)),
        outside_depth_scope='RGB change still checked; geometry entering the checked route depth is checked',
        collision_scene_changed=False)
