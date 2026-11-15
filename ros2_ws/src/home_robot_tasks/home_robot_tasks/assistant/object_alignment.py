"""Object-shape / five-joint inverse reach proposals BEFORE fine grasp inference.

Input is a segmented, calibrated cloud in the floor-referenced map frame, not
an object name or a six-DoF grasp. FK witnesses propose useful base/lift poses;
they are NOT arm goals, contact proofs, collision checks or motion authority.
The observation strategy selects final-view requirements. Fine grasp execution
needs either a new exposure or independently validated target tracking, plus
the existing full-STL route checks. Partial surfaces remain unknown.
"""
from collections import Counter
from dataclasses import asdict, dataclass
import math

import numpy as np
from scipy.stats import qmc

from .contracts import Context, Stamp, finite, vector
from .placement import Check
from ..fetch import InvalidTask
from ..grasp_preferences import TabletopGraspPolicy, contact_posture
from ..navigation import identifier
from ..s2_geometry import digest


@dataclass(frozen=True)
class AlignmentPolicy:
    joint_samples: int = 512
    arm_samples: int = 32
    directions: int = 12
    maximum_regions: int = 24
    maximum_results: int = 16
    opening_reserve_m: float = .004
    section_half_length_m: float = .025
    section_half_height_m: float = .012
    minimum_points: int = 12

    def __post_init__(self):
        for name, low, high in (('joint_samples',32,4096),('arm_samples',1,128),
                               ('directions',4,36),('maximum_regions',1,64),
                               ('maximum_results',1,64),('minimum_points',6,100)):
            value = getattr(self, name)
            if type(value) is not int or not low <= value <= high:
                raise ValueError('bounded alignment policy required: '+name)
        for name in ('opening_reserve_m','section_half_length_m','section_half_height_m'):
            finite(getattr(self,name), name, .001, .1)


def shape_regions(points, support_z_m, width_range_m, policy=AlignmentPolicy()):
    """Observed local widths and directions; no antipodal or mass inference.

    Several central/offset sections handle nonuniform bodies. Round objects keep
    all directions; an elongated object's narrow cross-sections naturally fit
    more often. Over-wide *local* sections do not reject the whole object.
    """
    points = np.asarray(points, float)
    if (points.ndim != 2 or points.shape[1] != 3 or not 24 <= len(points) <= 20000
            or not np.isfinite(points).all()):
        raise ValueError('24..20000 finite segmented map points required')
    support = finite(support_z_m, 'support height', 0, 3)
    lower, upper = vector([float(v) for v in width_range_m],2,'jaw width range')
    if not 0 < lower < upper <= .3:
        raise ValueError('ordered physical jaw width range required')
    # The caller must remove the support/background in its original segmentation.
    # Do not silently turn floor or below-support samples into target evidence.
    if points[:,2].min() < support-.002:
        raise ValueError('target cloud crosses support plane')
    center = (points.min(axis=0)+points.max(axis=0))/2
    xy = points[:,:2]-center[:2]
    values, axes = np.linalg.eigh(np.cov(xy.T))
    if values[-1] < 1e-8:
        raise ValueError('unresolved target extent')
    principal = math.atan2(axes[1,-1],axes[0,-1])
    elongation_weight=float(np.clip(math.sqrt(values[-1]/max(values[0],1e-12))-1,0,1))
    radius = max(float(np.linalg.norm(np.ptp(xy,axis=0)))/2, .01)
    regions, rejected, seen = [], Counter(), set()
    for angle in principal+np.arange(policy.directions)*math.pi/policy.directions:
        closing = np.array([math.cos(angle), math.sin(angle)])
        along = np.array([-closing[1],closing[0]])
        a, b = xy@closing, xy@along
        for fraction in (.5,.25,.75):
            offset = float(np.quantile(b,fraction))
            for z in np.unique(np.quantile(points[:,2],[.3,.65,.9])):
                mask = ((abs(b-offset)<=policy.section_half_length_m) &
                        (abs(points[:,2]-z)<=policy.section_half_height_m))
                if mask.sum() < policy.minimum_points:
                    rejected['insufficient_section'] += 1
                    continue
                lo,hi = float(a[mask].min()),float(a[mask].max())
                width = hi-lo
                if not lower <= width or width+policy.opening_reserve_m > upper:
                    rejected['observed_section_width'] += 1
                    continue
                anchor = np.r_[center[:2]+closing*(lo+hi)/2+along*np.median(b[mask]),
                               np.median(points[mask,2])]
                key = tuple(np.round(np.r_[anchor,closing],4))
                if key in seen: continue
                seen.add(key)
                # Bounding-shape centrality is a preference, NOT center of mass.
                centrality = float(np.linalg.norm(anchor[:2]-center[:2])/radius)
                obliquity=math.asin(float(np.clip(abs(closing@axes[:,-1]),0,1)))
                direction_cost=elongation_weight*(obliquity/math.radians(30))**2
                regions.append(dict(anchor_map_m=anchor.tolist(),closing_xy=closing.tolist(),
                    observed_width_m=width,points=int(mask.sum()),centrality_cost=centrality,
                    long_axis_preference_weight=elongation_weight,long_axis_xy=axes[:,-1].tolist(),direction_cost=direction_cost,
                    cost=centrality+direction_cost+.15*(width/upper)**2))
    # Keep direction diversity rather than filling the budget with one section.
    regions.sort(key=lambda r:r['cost'])
    buckets = {}
    for r in regions:
        k = tuple(np.round(r['closing_xy'],5))
        buckets.setdefault(k,[]).append(r)
    ordered=[]
    while buckets and len(ordered)<policy.maximum_regions:
        for k in list(buckets):
            ordered.append(buckets[k].pop(0))
            if not buckets[k]: del buckets[k]
            if len(ordered)==policy.maximum_regions: break
    return ordered,dict(rejected)


def contact_direction_review(arm, candidate, *, maximum_parallel_error_deg=10.):
    """Signed stock-finger geometry at a five-joint witness; no motion permit.

    Strongly elongated observed shapes require an approach projection parallel
    to the shape axis. Round shapes retain direction freedom. Fixed/moving
    sides stay distinct, even though the observed contact line is undirected.
    Opening is about the fixed finger, never about the nominal pinch midpoint.
    """
    maximum_parallel_error_deg=finite(maximum_parallel_error_deg,'parallel angle',1,30)
    region=candidate['observed_region'];width=region['observed_width_m']
    q=np.asarray(vector(candidate['fk_witness_model_rad'],5,'witness'))
    if np.any(q<arm.lo) or np.any(q>arm.hi):raise ValueError('witness outside joint range')
    x,y,yaw=vector(candidate['base_xy_yaw'],3,'base')
    co,si=math.cos(yaw),math.sin(yaw);R=np.array([[co,-si,0],[si,co,0],[0,0,1.]])
    height=finite(candidate['lift_travel_m'],'lift',0,arm.profile['lift']['travel_m'])
    tcp=arm.fk(q)[0];stock=tcp@arm.tcp_from_stock
    offset=(height-arm.lift_m)*np.asarray(arm.profile['lift']['axis_base'])
    origin=R@(stock[:3,3]+offset)+[x,y,-arm.profile['ground_z_m']]
    closing=-R@stock[:3,0];approach=R@tcp[:3,2]
    fixed=origin+closing*arm.jaw.fixed_face_m
    moving=fixed+closing*width;center=(fixed+moving)/2
    confidence=region.get('long_axis_preference_weight',0.)
    long_axis=region.get('long_axis_xy');parallel=None
    if long_axis is not None:
        axis=np.asarray(vector(long_axis,2,'shape axis'))
        if abs(np.linalg.norm(axis)-1)>1e-5:raise ValueError('unit shape axis required')
        projection=np.linalg.norm(approach[:2])
        parallel=(math.degrees(math.acos(float(np.clip(abs(approach[:2]@axis)/projection,0,1))))
                  if projection>1e-5 else 90.)
    constrained=confidence>=.5
    accepted=(not constrained or parallel is not None and parallel<=maximum_parallel_error_deg)
    return dict(accepted=bool(accepted),shape_direction_constrained=constrained,
        approach_parallel_error_deg=parallel,maximum_parallel_error_deg=maximum_parallel_error_deg,
        approach_map=approach.tolist(),fixed_to_moving_map=closing.tolist(),
        fixed_contact_map_m=fixed.tolist(),moving_contact_map_m=moving.tolist(),
        contact_center_map_m=center.tolist(),shape_axis_xy=long_axis,
        fixed_finger_side='negative_closing_half',opening_anchor='fixed_finger',
        cost=confidence*(parallel/5.)**2 if parallel is not None else 0.,
        motion_authorized=False,collision_checked=False)


def reach_comfort_review(arm, candidate):
    """Soft extension/joint-margin ranking; not a measured payload capability."""
    q=np.asarray(candidate['fk_witness_model_rad'])
    tcp,chain=arm.fk(q);stock=tcp@arm.tcp_from_stock
    width=candidate['observed_region']['observed_width_m']
    center=stock[:3,3]-stock[:3,0]*arm.jaw.center_from_fixed_m(width)
    # Shoulder lift pivot to contact, excluding the mounting pedestal and
    # replacing the virtual TCP endpoint with the physical contact reference.
    links=np.vstack([chain[2:-1],center])
    length=float(np.linalg.norm(np.diff(links,axis=0),axis=1).sum())
    distance=float(np.linalg.norm(center-links[0]))
    ratio=distance/max(length,1e-9)
    margin=float(np.min(np.minimum(q-arm.lo,arm.hi-q)/(arm.hi-arm.lo)))
    return dict(shoulder_contact_distance_m=distance,chain_length_m=length,
        extension_ratio=ratio,joint_margin_fraction=margin,
        cost=2.*(max(0.,ratio-.75)/.25)**2+.02/(margin+.02),
        policy='soft_reach_and_joint_margin_preference',payload_verified=False)


def contact_entry_review(arm, candidate, support_z_m, *, distance_m=.04, opening_reserve_m=.004):
    """Local approach IK and complete finger STL vs support, not a full arm route.

    Reuses bounded Cartesian five-joint IK (1 mm / 2 degree limits). The fixed
    finger is the opening anchor. Last 2 mm may touch the support, but forward
    travel may not drag along it. Other objects and the full arm still require
    the existing fresh-scene collision planner after base alignment.
    """
    from ..carry_cartesian import bounded_cartesian
    q=np.asarray(candidate['fk_witness_model_rad']);region=candidate['observed_region']
    width=region['observed_width_m'];opening=width+opening_reserve_m
    if not arm.jaw.accepts(opening):return dict(accepted=False,reason='opening_reserve_outside_stock_jaw')
    finite(distance_m,'approach stroke',.01,.08)
    tcp=arm.fk(q)[0];pre=tcp.copy();pre[:3,3]-=distance_m*tcp[:3,2]
    try:path=bounded_cartesian(arm,q,pre)[::-1]
    except ValueError as e:return dict(accepted=False,reason='five_joint_entry:'+str(e))
    offset=(candidate['lift_travel_m']-arm.lift_m)*np.asarray(arm.profile['lift']['axis_base'])
    support=np.eye(4);support[2,3]=support_z_m+arm.profile['ground_z_m']-offset[2]
    opening_angle=float(np.interp(opening,arm.jaw.gaps,arm.jaw.angles))
    minimum=float('inf');final_minimum=None
    for step,joints in enumerate(path):
        actual=arm.fk(joints)[0];stock=actual@arm.tcp_from_stock
        distances=arm.jaw.plane_clearance(stock,opening_angle,support)
        clearance=min(v['clearance_m'] for v in distances.values())
        remaining=np.linalg.norm(actual[:3,3]-tcp[:3,3])
        if clearance < (-.002 if remaining<=.002 else 0.):
            return dict(accepted=False,reason='stock_finger_drags_on_support',clearance_m=clearance)
        minimum=min(minimum,clearance);final_minimum=clearance
    # Moving-finger closing sweep is sampled in its real hinge coordinate.
    closed_minimum=float('inf')
    for a in np.linspace(float(np.interp(width,arm.jaw.gaps,arm.jaw.angles)),opening_angle,17):
        d=arm.jaw.plane_clearance(tcp@arm.tcp_from_stock,float(a),support)
        closed_minimum=min(closed_minimum,*(v['clearance_m'] for v in d.values()))
    if closed_minimum<-.002:return dict(accepted=False,reason='closing_sweep_crosses_support',clearance_m=closed_minimum)
    return dict(accepted=True,reason='five_joint_entry_and_stock_support_samples',
        opening_anchor='fixed_finger',open_width_m=opening,contact_width_m=width,
        entry_model_rad=path[0].tolist(),contact_model_rad=q.tolist(),
        path_model_rad=path.tolist(),samples=len(path),stroke_m=distance_m,
        minimum_open_finger_clearance_m=minimum,final_open_clearance_m=final_minimum,
        minimum_closing_clearance_m=closed_minimum,
        full_arm_and_scene_collision_checked=False,hidden_object_geometry_checked=False,
        continuous_collision_proof=False,motion_authorized=False)


def comfortable_samples(arm, policy=AlignmentPolicy()):
    """Reuse calibrated SO101 FK/ranges, retaining joint and reach diversity."""
    sampler=qmc.Sobol(5,scramble=True,seed=7)
    unit=sampler.random_base2(math.ceil(math.log2(policy.joint_samples)))[:policy.joint_samples]
    # Avoid sampling exactly on joint stops. This is a soft search allocation;
    # lack of a witness is inconclusive, never a proof of physical unreachability.
    joints=np.vstack([arm.anchor,(arm.lo+arm.hi)/2,
                      arm.lo+(.05+.90*unit)*(arm.hi-arm.lo)])
    results=[]
    posture_policy=TabletopGraspPolicy()
    for q in joints:
        pose=arm.fk(q)[0]
        posture=contact_posture(pose,posture_policy)
        if not posture['accepted']: continue
        margin=float(np.min(np.minimum(q-arm.lo,arm.hi-q)/(arm.hi-arm.lo)))
        cost=(posture['cost']+.2*float(np.mean(((q-(arm.lo+arm.hi)/2)/((arm.hi-arm.lo)/2))**4))
              +.25*((q[4]-arm.anchor[4])/(math.pi/2))**2)
        results.append(dict(pose=pose,q=q,margin=margin,cost=cost,posture=posture))
    results.sort(key=lambda r:r['cost'])
    diverse=[];bins=set()
    for r in results:
        key=tuple(np.round(r['pose'][:3,3]/.03).astype(int))
        if key in bins: continue
        bins.add(key);diverse.append(r)
        if len(diverse)==policy.arm_samples:break
    return diverse


class ObjectAlignmentPlanner:
    """Model proposals, optionally screened by existing route/view/STL adapters.

    screen(candidate) must return navigation, view and configuration Check
    objects. Configuration includes the proposed base/lift transition and body
    envelope; it cannot be replaced by this module's FK witness. Fine pick
    motion is deliberately deferred until after a NEW capture and GraspNet.
    """
    def __init__(self, arms, policy=AlignmentPolicy()):
        if not arms or set(arms)-{'left','right'}:raise ValueError('installed arms required')
        self.arms,self.policy=arms,policy
        self.samples={name:comfortable_samples(arm,policy) for name,arm in arms.items()}

    def plan(self, *, points_map_m, support_z_m, stamp, context, now_s, scene_revision,
             current_xy_yaw, current_lift_m, screen=None, maximum_age_s=2):
        if not isinstance(stamp,Stamp) or not isinstance(context,Context):
            raise InvalidTask('stamped map observation required')
        stamp.check(context,now_s,maximum_age_s)
        identifier(scene_revision,'scene_revision')
        current=np.asarray(vector(current_xy_yaw,3,'current base'))
        height=finite(current_lift_m,'lift travel',0,2)
        accepted=[];rejected=Counter();evaluated=0
        for side,arm in self.arms.items():
            if context.calibration_id!=arm.profile['calibration_id']:
                raise InvalidTask('alignment_calibration_mismatch')
            lift_axis=np.asarray(arm.profile['lift']['axis_base'],float)
            if lift_axis.shape!=(3,) or not np.isfinite(lift_axis).all() or lift_axis[2]<.9:
                raise ValueError('upward lift model required')
            travel=float(arm.profile['lift']['travel_m'])
            if height>travel:raise ValueError('current lift outside model')
            regions,failures=shape_regions(points_map_m,support_z_m,
                                          (arm.jaw.gaps[0],arm.jaw.gaps[-1]),self.policy)
            rejected.update(failures)
            for region_index,region in enumerate(regions):
                anchor=np.asarray(region['anchor_map_m']);axis=np.asarray(region['closing_xy'])
                for sample_index,sample in enumerate(self.samples[side]):
                    tcp=sample['pose'];closing=-tcp[:2,0]
                    heading=math.atan2(axis[1],axis[0])-math.atan2(closing[1],closing[0])
                    h=arm.lift_m+(anchor[2]+arm.profile['ground_z_m']-tcp[2,3])/lift_axis[2]
                    if not 0<=h<=travel:
                        rejected['lift_outside_model']+=2;continue
                    for flip in (0,math.pi):
                        yaw=math.atan2(math.sin(heading+flip),math.cos(heading+flip))
                        c,s=math.cos(yaw),math.sin(yaw)
                        rotation=np.array([[c,-s],[s,c]])
                        xy=anchor[:2]-rotation@(tcp[:2,3]+(h-arm.lift_m)*lift_axis[:2])
                        evaluated+=1
                        # FK anchor is a coarse task-region reference. It is NOT
                        # the asymmetric stock fingers' physical contact midpoint.
                        terms=dict(shape=region['cost'],posture=sample['cost'],
                            travel=float(np.linalg.norm(xy-current[:2])),
                            yaw=abs(math.atan2(math.sin(yaw-current[2]),math.cos(yaw-current[2])))/math.pi,
                            lift=abs(h-height)/travel)
                        candidate=dict(candidate_id=f'alignment-{side}-{region_index}-{sample_index}-{int(bool(flip))}',
                            arm=side,base_xy_yaw=[*map(float,xy),yaw],lift_travel_m=float(h),
                            observed_region=region,fk_witness_model_rad=sample['q'].tolist(),
                            joint_margin_fraction=sample['margin'],posture=sample['posture'],
                            cost_terms=terms,cost=sum(terms.values()),checks={})
                        if screen is not None:
                            checks=screen(candidate)
                            if not isinstance(checks,dict) or set(checks)!={'navigation','view','configuration'}:
                                raise InvalidTask('complete prealignment screening required')
                            for key,check in checks.items():
                                if not isinstance(check,Check):raise InvalidTask('typed prealignment check required')
                                check.validate()
                                candidate['checks'][key]=asdict(check)
                            bad=[k+':'+v.reason for k,v in checks.items() if not v.valid or v.mode=='unverified']
                            if bad:rejected.update(bad);continue
                            candidate['cost']+=checks['navigation'].cost
                        accepted.append(candidate)
        accepted.sort(key=lambda x:(x['cost'],x['candidate_id']))
        # Keep alternatives spread across feasible base poses, not duplicate FK branches.
        result=[];occupied=set()
        for c in accepted:
            x,y,yaw=c['base_xy_yaw']
            key=(c['arm'],round(x/.04),round(y/.04),round(yaw/math.radians(10)),round(c['lift_travel_m']/.03))
            if key in occupied:continue
            occupied.add(key);result.append(c)
            if len(result)==self.policy.maximum_results:break
        return dict(schema_version=1,status=('SCREENED_PROPOSALS' if screen else 'MODEL_PROPOSALS') if result else 'NO_SAMPLED_PROPOSAL',
            context=context.to_dict(),capture_id=stamp.capture_id,observed_s=stamp.observed_s,
            scene_revision=scene_revision,policy=asdict(self.policy),
            model_sha256={k:digest(dict(profile=v.profile,lo=v.lo.tolist(),hi=v.hi.tolist(),
                                       tcp=v.tcp.tolist(),jaw_gaps=v.jaw.gaps.tolist(),
                                       joints=[(o.tolist(),a.tolist()) for o,a in v.joints])) for k,v in self.arms.items()},
            cloud_sha256=digest(np.asarray(points_map_m,float).tolist()),support_z_m=support_z_m,
            candidates=result,evaluated=evaluated,rejected=dict(rejected),
            motion_authorized=False,executable=False,requires_reobservation=True,
            fine_grasp_generated=False,physical_contact_or_force_verified=False,
            limitations=['observed local spans only; hidden shape and mass unknown',
                         'FK anchor is not physical contact midpoint',
                         'sampled tabletop search is not a reachability proof',
                         'fresh GraspNet and full STL pick/retreat checks required after alignment'])


def alignment_reach_sensitivity(arm, candidate, *, translation_m=.03, yaw_rad=.1):
    """Offline ranking evidence for arrival error, never a grasp/motion permit.

    At the nominal pose and 26 nearby poses, resolve the same observed contact
    centre and closing axis with five joints. Reobservation must replace this
    old target before grasping. Samples are not a continuous reachability proof,
    and neither arm collision nor calibration/observation error is covered.
    """
    from scipy.optimize import least_squares
    translation_m=finite(translation_m,'arrival translation',0,.1)
    yaw_rad=finite(yaw_rad,'arrival yaw',0,.3)
    base=np.asarray(vector(candidate['base_xy_yaw'],3,'candidate base'))
    height=finite(candidate['lift_travel_m'],'candidate lift',0,arm.profile['lift']['travel_m'])
    region=candidate['observed_region']
    target=np.asarray(vector(region['anchor_map_m'],3,'contact anchor'))
    axis=np.r_[vector(region['closing_xy'],2,'closing axis'),0.]
    if abs(np.linalg.norm(axis)-1)>1e-5:raise ValueError('unit closing axis required')
    width=finite(region['observed_width_m'],'observed width',arm.jaw.gaps[0],arm.jaw.gaps[-1])
    q0=np.asarray(vector(candidate['fk_witness_model_rad'],5,'witness'))
    if np.any(q0<arm.lo) or np.any(q0>arm.hi):raise ValueError('witness outside joint range')
    delta=(height-arm.lift_m)*np.asarray(arm.profile['lift']['axis_base'])
    policy=TabletopGraspPolicy(); rows=[]
    offsets=[np.zeros(2)]+[translation_m*np.array([math.cos(a),math.sin(a)])
                           for a in np.arange(8)*math.pi/4]
    for xy in offsets:
        for dyaw in (0.,-yaw_rad,yaw_rad):
            base_sample=base+np.r_[xy,dyaw];x,y,yaw=base_sample
            co,si=math.cos(yaw),math.sin(yaw)
            rotation=np.array([[co,-si,0],[si,co,0],[0,0,1.]])
            def state(q):
                tcp=arm.fk(q)[0];stock=tcp@arm.tcp_from_stock
                center=stock[:3,3]-stock[:3,0]*arm.jaw.center_from_fixed_m(width)+delta
                return (rotation@center+[x,y,-arm.profile['ground_z_m']],
                        -rotation@stock[:3,0],tcp)
            # Either sign is the same undirected closing line. Keep the nominal
            # finger side, so a solver cannot obtain a fit by flipping fingers.
            directed_axis=axis*(1 if state(q0)[1]@axis>=0 else -1)
            def residual(q):
                center,closing,tcp=state(q)
                approach=math.acos(float(np.clip(-tcp[2,2],-1,1)))
                return np.r_[(center-target)/.002,(closing-directed_axis)/.04,
                    max(0.,approach-math.radians(policy.maximum_approach_deg))/.04,
                    .005*(q-q0)]
            fit=least_squares(residual,np.clip(q0,arm.lo+1e-8,arm.hi-1e-8),
                bounds=(arm.lo,arm.hi),max_nfev=80)
            center,closing,tcp=state(fit.x)
            error=float(np.linalg.norm(center-target))
            angle=math.acos(float(np.clip(closing@directed_axis,-1,1)))
            posture=contact_posture(tcp,policy)
            margin=float(np.rad2deg(np.min(np.minimum(fit.x-arm.lo,arm.hi-fit.x))))
            rows.append(dict(offset_xy_yaw=np.r_[xy,dyaw].tolist(),
                position_residual_m=error,closing_residual_rad=angle,
                fk_witness_model_rad=fit.x.tolist(),
                minimum_joint_margin_deg=margin,posture=posture,
                accepted=bool(error<=.002 and angle<=math.radians(5) and posture['accepted'])))
    passed=[r for r in rows if r['accepted']]
    return dict(mode='sampled_kinematic_sensitivity',samples=len(rows),passed=len(passed),
        nominal_passed=rows[0]['accepted'],translation_m=translation_m,yaw_rad=yaw_rad,
        minimum_joint_margin_deg=min((r['minimum_joint_margin_deg'] for r in passed),default=None),
        rows=rows,motion_authorized=False,collision_checked=False,
        physical_accuracy_verified=False,requires_fresh_capture=True)


class FixedHeightAlignmentPlanner(ObjectAlignmentPlanner):
    """Production prealignment: lookup height first, physical finger section centre.

    The bounded height alternatives are explored only after the preceding height
    has no checked route. Projection adjusts the five arm joints at a fixed lift;
    it never creates an arm command. Screens must check path, observation strategy and full body.
    """
    def plan_fixed(self, scene, *, current_xy_yaw, current_lift_m, height_rule,
                   screen, now_s, maximum_screened=64, seed_candidates=()):
        from scipy.optimize import least_squares
        scene.fresh(scene.stamp.context, now_s)
        if not callable(screen) or not 1 <= maximum_screened <= 256:
            raise InvalidTask('bounded current-scene alignment screening required')
        current = np.asarray(vector(current_xy_yaw, 3, 'current base'))
        failures = Counter(); checked = 0
        extra={side:[] for side in self.arms}
        if len(seed_candidates)>16:raise InvalidTask('bounded prior candidate seeds required')
        for prior in seed_candidates:
            side=prior['arm']
            if side not in self.arms:raise InvalidTask('unknown seed arm')
            arm=self.arms[side];q=np.asarray(vector(prior['fk_witness_model_rad'],5,'prior seed'))
            if np.any(q<arm.lo) or np.any(q>arm.hi):raise InvalidTask('seed outside joint limits')
            extra[side].append(dict(q=q))
        h0 = height_rule.setting(scene.support_z_m)['target_travel_from_bottom_m']
        for offset in (0., .01, -.01, .02, -.02, .04, -.04):
            height = h0+offset
            proposals = []
            for side, arm in self.arms.items():
                if scene.stamp.context.calibration_id != arm.profile['calibration_id']:
                    raise InvalidTask('alignment calibration mismatch')
                travel = arm.profile['lift']['travel_m']
                if not 0 <= height <= travel: continue
                regions, rejected = shape_regions(scene.points_map_m, scene.support_z_m,
                    (arm.jaw.gaps[0], arm.jaw.gaps[-1]), self.policy)
                failures.update(rejected)
                lift_axis = np.asarray(arm.profile['lift']['axis_base'])
                for ri, region in enumerate(regions):
                    target = np.asarray(region['anchor_map_m'])
                    width = region['observed_width_m']
                    def contact(q):
                        # The configured TCP can be virtual. Always resolve the
                        # stock STL frame before finding the asymmetric midpoint.
                        pose = arm.fk(q)[0] @ arm.tcp_from_stock
                        center = pose[:3, 3]-pose[:3, 0]*arm.jaw.center_from_fixed_m(width)
                        return pose, center+(height-arm.lift_m)*lift_axis
                    for si, sample in enumerate(extra[side]+self.samples[side]):
                        seed = np.clip(sample['q'], arm.lo+1e-7, arm.hi-1e-7)
                        def residual(q):
                            return np.r_[(contact(q)[1][2]-arm.profile['ground_z_m']-target[2])/.002,
                                         contact(q)[0][2,0]/.02, .02*(q-seed)]
                        fit = least_squares(residual, seed, bounds=(arm.lo+1e-8, arm.hi-1e-8), max_nfev=25)
                        pose, center = contact(fit.x)
                        if abs(center[2]-arm.profile['ground_z_m']-target[2]) > .002:
                            failures['fixed_height_contact_residual'] += 1; continue
                        posture = contact_posture(arm.fk(fit.x)[0], TabletopGraspPolicy())
                        if not posture['accepted']: continue
                        axis = region['closing_xy']; closing = -pose[:2, 0]
                        if np.linalg.norm(closing) < .5: continue
                        heading = math.atan2(axis[1], axis[0])-math.atan2(closing[1], closing[0])
                        for flip in (0., math.pi):
                            yaw = math.atan2(math.sin(heading+flip), math.cos(heading+flip))
                            c,s = math.cos(yaw), math.sin(yaw)
                            xy = target[:2]-np.array([[c,-s],[s,c]])@center[:2]
                            cost = (region['cost']+posture['cost']+float(np.linalg.norm(xy-current[:2]))
                                +abs(math.atan2(math.sin(yaw-current[2]), math.cos(yaw-current[2])))/math.pi
                                +abs(height-current_lift_m)/travel)
                            candidate=dict(candidate_id=f'fixed-{side}-{offset:g}-{ri}-{si}-{int(bool(flip))}',
                                arm=side, base_xy_yaw=[*map(float,xy),yaw], lift_travel_m=height,
                                observed_region=region, fk_witness_model_rad=fit.x.tolist(),
                                contact_reference='stock_stl_section_midpoint', cost=cost,
                                posture=posture, height_lookup_m=h0, checks={})
                            direction=contact_direction_review(arm,candidate)
                            if not direction['accepted']:
                                failures['approach_not_parallel_to_observed_shape']+=1;continue
                            candidate['contact_direction']=direction
                            candidate['reach_comfort']=reach_comfort_review(arm,candidate)
                            candidate['cost']+=direction['cost']+candidate['reach_comfort']['cost']
                            proposals.append(candidate)
            proposals.sort(key=lambda c:(c['cost'],c['candidate_id']))
            accepted=[]; bins=set()
            for candidate in proposals:
                x,y,yaw=candidate['base_xy_yaw']
                key=(round(x/.03),round(y/.03),round(yaw/math.radians(10)))
                if key in bins: continue
                bins.add(key)
                if len(bins)>maximum_screened: break
                entry=contact_entry_review(self.arms[candidate['arm']],candidate,scene.support_z_m,
                    opening_reserve_m=self.policy.opening_reserve_m)
                candidate['entry_review']=entry
                if not entry['accepted']:
                    failures[entry['reason']]+=1;continue
                checks=screen(candidate); checked+=1
                if not isinstance(checks,dict) or set(checks)!={'navigation','view','configuration'}:
                    raise InvalidTask('complete prealignment screening required')
                for k,v in checks.items():
                    if not isinstance(v,Check): raise InvalidTask('typed prealignment check required')
                    v.validate()
                    candidate['checks'][k]=asdict(v)
                bad=[k+':'+v.reason for k,v in checks.items() if not v.valid or v.mode=='unverified']
                if bad: failures.update(bad); continue
                candidate['cost']+=checks['navigation'].cost
                accepted.append(candidate)
            if accepted:
                accepted.sort(key=lambda c:c['cost'])
                return dict(status='SCREENED_PROPOSALS', candidates=accepted[:self.policy.maximum_results],
                    capture_id=scene.stamp.capture_id, cloud_sha256=scene.cloud_sha256,
                    context=scene.stamp.context.to_dict(), scene_revision=scene.scene_revision,
                    model_revision=scene.model_revision, checked=checked, rejected=dict(failures),
                    motion_authorized=False, fine_grasp_generated=False)
        return dict(status='NO_SAMPLED_PROPOSAL', candidates=[], checked=checked,
                    rejected=dict(failures), motion_authorized=False, fine_grasp_generated=False)

    def plan_place_fixed(self, scene, held, placements, *, current_xy_yaw, current_lift_m,
                         height_rule, screen, now_s):
        """Inverse reach for the MEASURED attachment, not a new pinch candidate."""
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation
        scene.fresh(scene.stamp.context,now_s)
        held.check(scene.stamp.context,scene.object_id,now_s,scene.model_revision)
        h0=height_rule.setting(scene.support_z_m)['target_travel_from_bottom_m']
        current=np.asarray(current_xy_yaw,float); rejected=Counter()
        for delta in (0.,.01,-.01,.02,-.02,.04,-.04):
            h=h0+delta; proposals=[]
            for side,arm in self.arms.items():
                if not 0<=h<=arm.profile['lift']['travel_m']:continue
                for pi,placement in enumerate(placements[:8]):
                    desired=np.asarray(placement['map_from_object'])@np.linalg.inv(held.tcp_from_object)
                    for si,sample in enumerate(self.samples[side]):
                        seed=np.r_[np.clip(sample['q'],arm.lo+1e-7,arm.hi-1e-7),current[2]]
                        def residual(values):
                            pose=arm.fk(values[:5])[0].copy()
                            pose[:3,3]+=(h-arm.lift_m)*np.asarray(arm.profile['lift']['axis_base'])
                            rz=Rotation.from_euler('z',values[5]).as_matrix()
                            return np.r_[(pose[2,3]-arm.profile['ground_z_m']-desired[2,3])/.002,
                                Rotation.from_matrix(desired[:3,:3].T@rz@pose[:3,:3]).as_rotvec()/.04,
                                .01*(values[:5]-seed[:5])]
                        fit=least_squares(residual,seed,bounds=(np.r_[arm.lo+1e-8,-np.inf],np.r_[arm.hi-1e-8,np.inf]),max_nfev=35)
                        if np.linalg.norm(residual(fit.x)[:4])>1.:continue
                        yaw=math.atan2(math.sin(fit.x[5]),math.cos(fit.x[5]))
                        pose=arm.fk(fit.x[:5])[0].copy()
                        pose[:3,3]+=(h-arm.lift_m)*np.asarray(arm.profile['lift']['axis_base'])
                        rz=Rotation.from_euler('z',yaw).as_matrix()
                        xy=desired[:2,3]-(rz@pose[:3,3])[:2]
                        proposals.append(dict(candidate_id=f'place-{side}-{delta:g}-{pi}-{si}',arm=side,
                            base_xy_yaw=[*map(float,xy),yaw],lift_travel_m=h,placement=placement,
                            fk_witness_model_rad=fit.x[:5].tolist(),height_lookup_m=h0,
                            cost=float(np.linalg.norm(xy-current[:2]))+abs(h-current_lift_m),checks={}))
            accepted=[]
            for c in sorted(proposals,key=lambda p:p['cost'])[:64]:
                checks=screen(c)
                if set(checks)!={'navigation','view','configuration'}:raise InvalidTask('complete place alignment screening required')
                for k,v in checks.items():
                    if not isinstance(v,Check):raise InvalidTask('typed place screening required')
                    v.validate();c['checks'][k]=asdict(v)
                if any(not v.valid or v.mode=='unverified' for v in checks.values()):
                    rejected['place_alignment_screen']+=1;continue
                c['cost']+=checks['navigation'].cost;accepted.append(c)
            if accepted:return dict(status='SCREENED_PROPOSALS',candidates=sorted(accepted,key=lambda c:c['cost'])[:16],motion_authorized=False)
        return dict(status='NO_SAMPLED_PROPOSAL',candidates=[],rejected=dict(rejected),motion_authorized=False)
