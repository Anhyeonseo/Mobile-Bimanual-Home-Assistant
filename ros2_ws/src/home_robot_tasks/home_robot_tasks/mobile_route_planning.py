"""Worker-side adapters for existing GraspNet, SO101 binding and MoveIt routes.

All inputs come from the newly exposed frame and measured robot state. No
hardware is opened here. A deployment supplies its isolated planning services;
execution still belongs to the runtime's single resident arm/stop owner.
"""
from dataclasses import dataclass, replace
import numpy as np
from .carry_binding import bind_grasp, BoundPayload
from .carry_planning import plan_pick_carry, plan_place_carry, PlanRejected
from .grasp_preferences import TabletopGraspPolicy
from .place_geometry import place_candidates
from .task_route_program import compile_task_route
from .s2_geometry import JOINTS, model_from_control, rigid


@dataclass(frozen=True)
class CaptureRouteInput:
    arm: object
    planner: object
    bounds: object
    anchor_control_rad: tuple
    installed_jaw: object
    review: dict
    group_metadata: dict
    payload: BoundPayload | None = None
    grasp_inputs: dict | None = None
    held: object = None
    held_candidate: object = None
    support_grid: object = None
    empty_transport_model_rad: object = None
    hold_control: float | None = None
    open_control: float | None = None
    contact_control_interval: tuple | None = None

    def __post_init__(self):
        from copy import deepcopy
        for name in ('review','group_metadata','grasp_inputs','payload','held','held_candidate','support_grid'):
            object.__setattr__(self,name,deepcopy(getattr(self,name)))
        object.__setattr__(self,'anchor_control_rad',tuple(self.anchor_control_rad))
        if len(self.anchor_control_rad)!=12 or not np.isfinite(self.anchor_control_rad).all():
            raise PlanRejected('measured twelve-axis snapshot required')


class MobileRoutePlanner:
    def __init__(self, *, grasp_generator=None):
        # Imported lazily: pure prealignment never loads or calls GraspNet.
        self.grasp_generator=grasp_generator

    def __call__(self,request):
        data=request.prepared['route']
        if not isinstance(data,CaptureRouteInput):raise PlanRejected('capture route input required')
        scene=request.scene;arm=data.arm
        if (data.review.get('capture_id')!=scene.stamp.capture_id or data.review.get('source_sha256')!=scene.source_sha256
                or data.review.get('calibration_id')!=scene.stamp.context.calibration_id
                or arm.lift_m!=scene.lift_m or data.review.get('lift_height_m')!=scene.lift_m):
            raise PlanRejected('fresh capture, model and actual lift must match')
        index=0 if arm.side=='left' else 6
        start=np.array([model_from_control(float(data.anchor_control_rad[index+i]),arm.profile['arms'][arm.side]['alignment'][n])
                        for i,n in enumerate(JOINTS[:5])])
        if request.skill=='pick':
            if data.payload is None or data.grasp_inputs is None:raise PlanRejected('fresh segmented RGB-D and payload required')
            if self.grasp_generator is None:
                from home_perception.grasp.graspnet import candidates
                generator=candidates
            else:generator=self.grasp_generator
            group={**data.group_metadata,**generator(**data.grasp_inputs)}
            candidates=[];previews={}
            for i in range(min(32,len(group.get('refined_grasps',[])))):
                for sign in (1,-1):
                    try:
                        candidate,preview=bind_grasp(arm,data.review,group,i,data.payload,data.installed_jaw,
                            jaw_sign=sign,grasp_policy=TabletopGraspPolicy(),approach_mode='vertical',close_allowance_m=.005)
                        candidates.append(candidate);previews[candidate.candidate_id]=preview
                    except (ValueError,PlanRejected):continue
            plan=plan_pick_carry(arm,data.planner,start,candidates,scene_id=scene.stamp.capture_id,
                                 grasp_policy=TabletopGraspPolicy(),stop_after_first=True)
            if plan.status!='OFFLINE_ROUTE_FOUND':raise PlanRejected('no complete fresh pick/carry route: '+str(plan.failures[-3:]))
            preview=previews[plan.candidate_id]
            gap=preview.get('measured_gap_preview',preview.get('nominal_gap_preview'))
            sign=arm.profile['arms'][arm.side]['alignment']['gripper']['raw_to_control_sign']
            open_control=sign*(gap['entry_raw']-2048)*2*np.pi/4096
            close_control=sign*(gap['lower_search_raw']-2048)*2*np.pi/4096
            envelope=data.installed_jaw.contact_envelope(preview['closure'])
            upper=sign*(envelope['upper_contact_raw']-2048)*2*np.pi/4096
            return compile_task_route('pick',plan,scene=scene,arm=arm,anchor_control_rad=data.anchor_control_rad,
                                      bounds=data.bounds,open_control=open_control,close_control=close_control,contact_control_interval=(close_control,upper))
        if request.skill!='place':raise PlanRejected('explicit pick or place required')
        if data.held is None or data.held_candidate is None or data.support_grid is None:
            raise PlanRejected('measured held transform and new empty support required')
        data.held.check(scene.stamp.context,scene.object_id,request.configuration[2],scene.model_revision)
        if data.held_candidate.candidate_id!=data.held.grasp_id:
            raise PlanRejected('measured attachment belongs to another grasp')
        grid=data.support_grid
        if grid.stamp!=scene.stamp or grid.scene_revision!=scene.scene_revision or grid.surface_id!=scene.surface_id:
            raise PlanRejected('support must be from this destination capture')
        if data.open_control is None or data.hold_control is None:raise PlanRejected('calibrated open and current hold command required')
        held=replace(data.held_candidate,scene_id=scene.stamp.capture_id,tcp_from_object=data.held.tcp_from_object.copy(),
            base_from_grasp_tcp=arm.fk(start)[0],object_size_m=data.held.size_m,payload_mesh=data.held.payload_mesh)
        if held.object_constraint is not None:
            held=replace(held,object_constraint=replace(held.object_constraint,tcp_from_object=data.held.tcp_from_object.copy()))
        x,y,yaw=scene.base_xy_yaw;c,s=np.cos(yaw),np.sin(yaw)
        map_from_base=np.array([[c,-s,0,x],[s,c,0,y],[0,0,1,-arm.profile['ground_z_m']],[0,0,0,1.]])
        base_from_map=np.linalg.inv(map_from_base)
        rows,cols=grid.empty.shape
        origin=np.array(grid.origin_xy); upper=origin+[cols*grid.resolution_m,rows*grid.resolution_m]
        surface_corners=np.array([[a,b,0,1] for a in (origin[0],upper[0]) for b in (origin[1],upper[1])])
        base_corners=(base_from_map@grid.map_from_surface@surface_corners.T).T[:,:3]
        bounds=np.array([base_corners.min(axis=0),base_corners.max(axis=0)])
        for proposal in place_candidates(grid,data.held,now_s=request.configuration[2],maximum_results=2):
            plan=plan_place_carry(arm,data.planner,start,held,scene_id=scene.stamp.capture_id,
                base_from_object=base_from_map@rigid(proposal['map_from_object']),support_bounds_m=bounds,
                support_observation_id=scene.stamp.capture_id,support_observed_s=scene.stamp.observed_s,
                now_s=request.configuration[2],empty_transport_model_rad=data.empty_transport_model_rad)
            if plan.status=='OFFLINE_ROUTE_FOUND':
                return compile_task_route('place',plan,scene=scene,arm=arm,anchor_control_rad=data.anchor_control_rad,
                    bounds=data.bounds,open_control=data.open_control,close_control=data.hold_control,contact_control_interval=data.contact_control_interval)
        raise PlanRejected('no complete empty-support place/release/retreat route')


class MobileAlignmentPlanner:
    def __init__(self,planner):self.planner=planner

    def __call__(self,request):
        p=request.prepared
        common=dict(current_xy_yaw=request.configuration[0],current_lift_m=request.configuration[1],
                    height_rule=p['height_rule'],screen=p['alignment_screen'],now_s=request.configuration[2])
        if request.skill=='align_for_pick':return self.planner.plan_fixed(request.scene,**common)
        held=p['held'];grid=p['support_grid']
        options=place_candidates(grid,held,now_s=request.configuration[2])
        return self.planner.plan_place_fixed(request.scene,held,options,**common)
