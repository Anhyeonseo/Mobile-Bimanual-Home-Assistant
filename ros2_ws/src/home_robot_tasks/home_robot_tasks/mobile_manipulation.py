"""Host assembly of automatic alignment and complete manipulation.

The injected acquisition/state/navigation/MoveIt services are the existing
runtime services. Construction never connects devices or activates a task.
"""
from .assistant.prealignment_binding import PrealignmentPolicy
from .mobile_route_planning import MobileAlignmentPlanner, MobileRoutePlanner
from .task_route_program import TaskRoutePort
from .manipulation_port import PlanningContext
from .fetch import InvalidTask
from .execution import Feedback
from dataclasses import dataclass


@dataclass(frozen=True)
class GeometryEvidence:
    object_id: str
    observed_s: float
    scene_revision: str
    attached: bool
    detached: bool
    probe_clearance_verified: bool
    probe_follow_verified: bool
    placement_support_verified: bool

    def check(self,object_id,scene_revision,now):
        if (self.object_id!=object_id or self.scene_revision!=scene_revision
                or not 0<=now-self.observed_s<=.5
                or any(type(getattr(self,k)) is not bool for k in
                       ('attached','detached','probe_clearance_verified','probe_follow_verified','placement_support_verified'))
                or self.attached and self.detached):
            raise InvalidTask('fresh independent geometry evidence required')


def contact_guard(payload,geometry_evidence,scene_revision,clock):
    """ResidentRouteBuilder contact hook; never derived from a command ACK."""
    def check(envelope,snapshot):
        now=clock();proof=payload.proof(now)
        if not 0<=now-proof.observed_s<=.5:return False
        if payload.object_id!=envelope.object_id:return False
        if not (proof.conditions.get('probe_contact_live') or proof.conditions.get('load_retained')):return False
        if envelope.release_goal_rad is not None:
            observed=geometry_evidence(envelope.object_id,'release',now)
            if not isinstance(observed,GeometryEvidence):return False
            observed.check(envelope.object_id,scene_revision(),now)
            return observed.attached and observed.placement_support_verified
        return True
    return check


class TaskRouteContext:
    def __init__(self, *, current_context, lift_height, model_revision, retained_object, context_identity):
        self.current,self.lift,self.model,self.held=current_context,lift_height,model_revision,retained_object
        self.identity=context_identity

    def __call__(self,program,stage,now):
        c=self.current()
        if (not isinstance(c,PlanningContext) or not 0<=now-c.observed_s<=.5
                or not c.base_stopped or not c.lift_stopped or c.capture_id!=program.capture_id
                or c.calibration_id!=program.calibration_id or self.model()!=program.model_revision
                or program.context is None or program.context!=self.identity()
                or abs(self.lift()-program.lift_m)>.003):
            raise InvalidTask('route observation or actual platform changed')
        if (stage.attached and stage.name!='probe') or stage.name=='release':
            import numpy as np
            from scipy.spatial.transform import Rotation
            held=self.held()
            if held is None or held.object_id!=program.object_id or held.grasp_id!=program.resolved_grasp.candidate_id or held.context!=program.context or held.model_revision!=program.model_revision or not 0<=now-held.observed_s<=.5:
                raise InvalidTask('fresh measured retained attachment required')
            expected=program.resolved_grasp.tcp_from_object
            change=np.linalg.inv(expected)@held.tcp_from_object
            if np.linalg.norm(change[:3,3])>.003 or Rotation.from_matrix(change[:3,:3]).magnitude()>.05:
                raise InvalidTask('measured attachment changed; held route must be replanned')
        return c


class MobileManipulationBindings:
    """Pass policy and manipulation_factory into build_runtime.

    prepare(scene,operation,skill,now) returns an immutable acquisition snapshot:
      align: height_rule, alignment_screen, checked transport; for place also
             support_grid and HeldObject;
      pick/place: CaptureRouteInput in route, from that exact RGB-D exposure.
    The screen uses AlignmentScreen (SE2 path + full configuration + view).
    No detector/class-name-based fixed base or grasp pose is accepted here.
    """
    def __init__(self, *, shape_planner, scene_for, configuration, epoch, prepare,
                 state, scene_port, context_for, scene_command, geometry_evidence, loaded_snapshot, check_loaded, grasp_generator=None):
        self.state=state
        self.geometry_evidence=geometry_evidence
        self.transport_program=None
        self.policy=PrealignmentPolicy(scene_for=scene_for,configuration=configuration,epoch=epoch,
            prepare=prepare,alignment_planner=MobileAlignmentPlanner(shape_planner),
            route_planner=MobileRoutePlanner(grasp_generator=grasp_generator),
            navigation_plan=lambda candidate,request:candidate['navigation_plan'],loaded_transport=self.loaded_transport,
            program_sink=self.accept_program,reset_transport=self.reset_transport)
        self.scene_port,self.context_for,self.scene_command=scene_port,context_for,scene_command
        self.loaded_snapshot,self.check_loaded=loaded_snapshot,check_loaded
        self.loaded_ports=[]

    def manipulation_factory(self,*,motion,arm,monitor,clock,catalog):
        # 'motion' is the old endpoint-only ManipulationPort. Full checked stage
        # trajectories go straight to its SAME resident arm, never another writer.
        from .loaded_carry_planning import LoadedCarryPort
        def evidence(goal,phase,now):
            base=monitor(goal,phase,now)
            observed=self.geometry_evidence(catalog.object_id,phase,now)
            if not isinstance(observed,GeometryEvidence):raise InvalidTask('independent task geometry evidence missing')
            observed.check(catalog.object_id,catalog.scene(),now)
            return Feedback(goal,min(base.observed_s,observed.observed_s),base.status,
                {**base.conditions,'object_attached':observed.attached,'object_detached':observed.detached,
                 'probe_clearance_verified':observed.probe_clearance_verified,
                 'probe_follow_verified':observed.probe_follow_verified,
                 'placement_support_verified':observed.placement_support_verified},base.reason)
        tail=LoadedCarryPort(arm,self.scene_port,snapshot=self.loaded_snapshot,check_current=self.check_loaded,
            context_for=self.context_for,scene_command=self.scene_command,monitor=evidence,clock=clock,worker=self.policy.worker,program_sink=self.accept_program)
        self.loaded_ports.append(tail)
        return TaskRoutePort(arm,self.scene_port,context_for=self.context_for,
                             scene_command=self.scene_command,monitor=evidence,clock=clock,loaded_tail=tail)

    def accept_program(self,program):
        if program.operation in ('loaded','place'):
            self.state.bind_transport_program(program)
            self.transport_program=program

    def reset_transport(self):
        self.state.reset_transport_target()
        self.transport_program=None

    def loaded_transport(self,parameters,now):
        from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
        p=self.transport_program
        if p is None or p.operation!='loaded' or p.object_id!=parameters['object_id']:
            raise InvalidTask('measured loaded carry target required')
        stage=p.stages[-1]
        context=self.context_for(p,stage,now)
        goal=stage.points[-1].positions_rad
        return dict(arm=dict(trajectory=(TimedJointPoint(50,goal),TimedJointPoint(100,goal)),
                             context=context,contact_envelope=p.contact_envelope),height_um=round(p.lift_m*1e6))

    def runtime_options(self,alignment_port):
        return dict(placement_policy=self.policy,manipulation_factory=self.manipulation_factory,
                    alignment=alignment_port)

    def close(self):
        self.policy.close()
        for port in self.loaded_ports:port.close()
