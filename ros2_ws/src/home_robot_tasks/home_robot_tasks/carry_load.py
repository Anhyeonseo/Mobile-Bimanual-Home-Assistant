"""Geometric payload-load proxies; not measured torque or payload capacity.

Gravity torque is reported per kg of carried mass. A missing COM uses the
geometric object-frame origin explicitly, not an invented measured mass/COM.
Arm self-weight, acceleration, grip friction and drivetrain limits are absent.
"""
from dataclasses import dataclass
from itertools import product
import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection
from scipy.spatial.transform import Rotation
from .s2_geometry import rigid


@dataclass(frozen=True)
class CarriedGeometry:
    tcp_from_object: np.ndarray
    size_m: tuple
    mesh: object = None
    com_object_m: tuple | None = None

    def __post_init__(self):
        rigid(self.tcp_from_object)
        size=np.asarray(self.size_m,float)
        if size.shape!=(3,) or not np.isfinite(size).all() or np.any(size<=0):raise ValueError('positive payload dimensions required')
        if self.com_object_m is not None and (np.shape(self.com_object_m)!=(3,) or not np.isfinite(self.com_object_m).all()):
            raise ValueError('finite supplied payload COM required')

    @property
    def vertices(self):
        if self.mesh is not None:return np.asarray(self.mesh['vertices'],float)
        return np.array(list(product((-1.,1.),repeat=3)))*np.asarray(self.size_m)/2


def payload_load(arm,q,payload):
    pose=arm.mount.copy();origins=[];axes=[]
    for (origin,axis),angle in zip(arm.joints,q,strict=True):
        pose=pose@origin;origins.append(pose[:3,3].copy());axes.append(pose[:3,:3]@axis)
        pose[:3,:3]=pose[:3,:3]@Rotation.from_rotvec(axis*angle).as_matrix()
    tcp=pose@arm.tcp;obj=tcp@rigid(payload.tcp_from_object)
    com=obj[:3,3]+obj[:3,:3]@np.asarray(payload.com_object_m if payload.com_object_m is not None else (0.,0.,0.))
    origins=np.asarray(origins);axes=np.asarray(axes);r=com-origins
    torque=np.abs(np.einsum('ij,ij->i',axes,np.cross(r,[0.,0.,-9.80665])))
    vertices=payload.vertices@obj[:3,:3].T+obj[:3,3]
    return dict(com_basis='supplied' if payload.com_object_m is not None else 'geometric_center_proxy',
        com_base_m=com.tolist(),shoulder_horizontal_lever_m=float(np.linalg.norm(r[1,:2])),
        elbow_horizontal_lever_m=float(np.linalg.norm(r[2,:2])),
        payload_gravity_torque_per_kg_nm=torque.tolist(),
        maximum_horizontal_lever_m=float(np.max(np.linalg.norm(r[:,:2],axis=1))),
        shoulder_worst_envelope_lever_m=float(np.max(np.linalg.norm(vertices[:,:2]-origins[1,:2],axis=1))),
        forward_extent_from_arm_root_m=float(np.max(vertices[:,0]-arm.mount[0,3])),
        lowest_payload_from_arm_root_m=float(np.min(vertices[:,2]-arm.mount[2,3])),
        minimum_joint_limit_margin_rad=float(np.min(np.minimum(np.asarray(q)-arm.lo,arm.hi-np.asarray(q)))),
        includes_arm_weight_or_dynamics=False,physical_load_validated=False)


def grasp_balance(base_from_tcp,payload):
    """Balance about the jaw closing line, not about the asymmetric TCP origin.

    The line is TCP X. Moving the reference along that line (e.g. from the
    fixed finger to the pad midpoint) must not change the balance estimate.
    A missing COM uses the same explicit object-envelope-centre proxy as load.
    """
    pose=rigid(base_from_tcp);relative=rigid(payload.tcp_from_object)
    center=np.asarray(payload.com_object_m if payload.com_object_m is not None else (0.,0.,0.))
    r=relative[:3,3]+relative[:3,:3]@center
    offset=float(np.linalg.norm(r[1:]))
    vertices=(payload.vertices-center)@relative[:3,:3].T
    radius=max(float(np.max(np.linalg.norm(vertices[:,1:],axis=1))),1e-6)
    gravity_tcp=pose[:3,:3].T@np.array([0.,0.,-9.80665])
    return dict(com_basis='supplied' if payload.com_object_m is not None else 'geometric_center_proxy',
        center_to_closing_line_m=offset,transverse_radius_m=radius,normalized_offset=offset/radius,
        pinch_axis_gravity_torque_per_kg_nm=abs(float(np.cross(r,gravity_tcp)[0])),
        center_base_m=(pose@relative@np.r_[center,1.])[:3].tolist(),
        physical_balance_validated=False,friction_or_force_capacity_known=False)


def inflated_payload_mesh(payload,margin_m):
    """Conservative convex face-offset probe, not a measured Euclidean distance.

    The offset polytope contains a sphere dilation of the convex envelope;
    passing collision checks gives a model clearance lower bound. Concavities
    are filled for the probe only; original collision geometry is unchanged.
    """
    if not 0<margin_m<=.05:raise ValueError('bounded clearance probe required')
    vertices=payload.vertices;hull=ConvexHull(vertices)
    halfspaces=np.unique(np.round(hull.equations,12),axis=0);halfspaces[:,-1]-=margin_m
    outer=HalfspaceIntersection(halfspaces,vertices.mean(axis=0)).intersections
    shell=ConvexHull(outer)
    return dict(vertices=outer.tolist(),faces=shell.simplices.tolist(),kind='convex_clearance_probe')
