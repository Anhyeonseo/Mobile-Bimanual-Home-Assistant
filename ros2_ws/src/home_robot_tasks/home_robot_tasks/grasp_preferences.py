"""Grasp posture preferences and an explicit tabletop envelope; no collision override."""
from dataclasses import dataclass
import numpy as np
from .s2_geometry import rigid


@dataclass(frozen=True)
class TabletopGraspPolicy:
    # Soft preferences leave moderately tilted alternatives available. These
    # are planning choices, not experimentally established contact limits.
    preferred_approach_deg: float = 30.
    preferred_closing_deg: float = 10.
    maximum_approach_deg: float = 50.
    maximum_closing_deg: float = 20.
    preferred_balance_offset: float = .10
    maximum_balance_offset: float = .35

    def __post_init__(self):
        for preferred,maximum in ((self.preferred_approach_deg,self.maximum_approach_deg),
                                  (self.preferred_closing_deg,self.maximum_closing_deg)):
            if not np.isfinite([preferred,maximum]).all() or not 0<preferred<=maximum<90:
                raise ValueError('finite ordered tabletop posture angles required')
        if (not np.isfinite([self.preferred_balance_offset,self.maximum_balance_offset]).all()
                or not 0<self.preferred_balance_offset<=self.maximum_balance_offset<=1):
            raise ValueError('finite ordered normalized balance offsets required')


def contact_posture(pose, policy):
    rotation=rigid(pose)[:3,:3]
    approach=float(np.rad2deg(np.arccos(np.clip(-rotation[2,2],-1,1))))
    closing=float(np.rad2deg(np.arcsin(np.clip(abs(rotation[2,0]),0,1))))
    return dict(approach_from_down_deg=approach,closing_line_from_horizontal_deg=closing,
        accepted=approach<=policy.maximum_approach_deg+1e-8 and closing<=policy.maximum_closing_deg+1e-8,
        cost=(approach/policy.preferred_approach_deg)**2+(closing/policy.preferred_closing_deg)**2)


def contact_roll_interval(pose, policy, limit=np.pi/3):
    """Intersect the original roll freedom with the tabletop envelope.

    Rotating about TCP X preserves both contact positions and closing slope.
    Restrict the search itself so a rejected tilted solution cannot hide a
    reachable moderate one. Neither the contact point nor the base moves.
    """
    rotation=rigid(pose)[:3,:3]
    if abs(rotation[2,0])>np.sin(np.deg2rad(policy.maximum_closing_deg))+1e-12:return None
    a,b=-rotation[2,2],rotation[2,1]
    radius=float(np.hypot(a,b));threshold=np.cos(np.deg2rad(policy.maximum_approach_deg))
    if radius<threshold:return None
    center=float(np.arctan2(b,a));half=float(np.arccos(np.clip(threshold/radius,-1,1)))
    intervals=[(max(-limit,center+shift-half),min(limit,center+shift+half)) for shift in (-2*np.pi,0,2*np.pi)]
    intervals=[(lo,hi) for lo,hi in intervals if hi-lo>1e-10]
    return max(intervals,key=lambda bounds:bounds[1]-bounds[0]) if intervals else None


def grasp_preferences(arm, start, candidate, policy=None):
    size=np.asarray(candidate.object_size_m,float)
    order=np.argsort(size)
    # A distinct long axis justifies a straight-across preference. Nearly round
    # objects get no arbitrary principal-axis preference; no label is consulted.
    elongation=float(size[order[-1]]/size[order[-2]])
    confidence=float(np.clip(elongation-1.,0.,1.))
    longitudinal=float(abs(candidate.tcp_from_object[0,order[-1]]))
    obliquity=float(np.arcsin(np.clip(longitudinal,0.,1.)))
    q=np.asarray(candidate.pregrasp_model_rad)
    reference=float(q[4]-arm.anchor[4])
    travel=float(q[4]-start[4])
    cost=confidence*(obliquity/np.deg2rad(30))**2
    cost+=.25*(travel/(np.pi/2))**2
    posture=None;joint_margin=None
    if policy is None:
        cost+=(reference/(np.pi/2))**2
    else:
        posture=contact_posture(candidate.base_from_grasp_tcp,policy)
        normalized=(q-(arm.lo+arm.hi)/2)/((arm.hi-arm.lo)/2)
        cost+=posture['cost']+.25*float(np.mean(normalized**4))
        joint_margin=float(np.rad2deg(np.min(np.minimum(q-arm.lo,arm.hi-q))))
    return dict(cost=float(cost),long_axis_obliquity_deg=float(np.rad2deg(obliquity)),
        long_axis_preference_weight=confidence,wrist_from_home_deg=float(np.rad2deg(reference)),
        wrist_travel_deg=float(np.rad2deg(travel)),hard_angle_filter=policy is not None,
        contact_posture=posture,minimum_joint_limit_margin_deg=joint_margin,
        contact_or_force_proven=False)
