"""Compile checked pick/carry AND place routes for the existing resident arm.

No controller access. Model paths retain their exact corners, the opposite arm
and the single-turn wire convention. Runtime evidence owns grasp/release truth.
"""
from dataclasses import dataclass, replace
import numpy as np
from .carry_program import CarryStage, _control, _wire_interior, _split_probe, validate_program_winding
from .carry_trajectory import retime
from .s2_geometry import JOINTS
from .fetch import InvalidTask
from .workflow_ports import Phase, SequencePort
from .fetch_composition import ResolvedPort


@dataclass(frozen=True)
class TaskRouteProgram:
    operation: str
    object_id: str
    capture_id: str
    calibration_id: str
    model_revision: str
    lift_m: float
    stages: tuple
    resolved_grasp: object
    contact_envelope: object = None
    context: object = None


def compile_task_route(operation, plan, *, scene, arm, anchor_control_rad, bounds,
                       open_control, close_control, contact_control_interval=None):
    expected = (['open_geometry','pregrasp','approach','close_geometry','lift','carry'] if operation=='pick'
                else ['lift','carry'] if operation=='loaded' else ['place_preplace','place_lower','release_geometry','place_retreat','empty_transport'])
    if operation not in ('pick','place','loaded') or plan.status!='OFFLINE_ROUTE_FOUND' or [s.label for s in plan.stages]!=expected:
        raise InvalidTask('complete checked pick/carry or place route required')
    if scene.lift_m != arm.lift_m: raise InvalidTask('actual fixed lift required')
    previous=np.asarray(anchor_control_rad,float).copy()
    if previous.shape!=(12,) or not np.isfinite(previous).all(): raise InvalidTask('measured bimanual anchor required')
    side=arm.side; index=0 if side=='left' else 6
    if operation in ('place','loaded'):
        if contact_control_interval is None or close_control is None:raise InvalidTask('measured contact interval required')
        lo,hi=sorted(contact_control_interval)
        if not lo-1e-6<=previous[index+5]<=hi+1e-6:raise InvalidTask('measured contact outside loaded envelope')
        previous[index+5]=close_control
    intervals=[]
    for i,n in enumerate(JOINTS[:5]):
        low,high=sorted((_control(arm.profile,side,n,arm.lo[i]),_control(arm.profile,side,n,arm.hi[i])))
        intervals.append((max(low,bounds.minimum[index+i]),min(high,bounds.maximum[index+i])))
    compiled=[]; last=None
    paths=[]
    for stage in plan.stages:
        path=np.asarray(stage.joints,float)
        if path.ndim!=2 or path.shape[1]!=5 or not 1<=len(path)<=20000 or not np.isfinite(path).all():
            raise InvalidTask('bounded five-axis checked path required')
        if last is not None and np.max(abs(last-path[0]))>.001: raise InvalidTask('broken whole-route continuity')
        last=path[-1]
        if stage.label=='lift' and operation=='pick':
            probe,remainder=_split_probe(path,arm.fk)
            paths.extend([('probe',probe,close_control,True),('lift',remainder,close_control,True)])
        else:
            names={'open_geometry':'open','close_geometry':'close','release_geometry':'release'}
            attached=stage.label in ('lift','carry','place_preplace','place_lower')
            jaw=close_control if stage.label in ('close_geometry','lift','carry','place_preplace','place_lower') else open_control
            paths.append((names.get(stage.label,stage.label),path,jaw,attached))
    for name,path,jaw,attached in paths:
        controls=[]
        for row in path:
            q=previous.copy()
            q[index:index+5]=[_wire_interior(_control(arm.profile,side,n,float(v)),*interval)
                for n,v,interval in zip(JOINTS[:5],row,intervals)]
            q[index+5]=jaw;controls.append(q)
        # The measured anchor and every numerical bridge are included. The
        # resident route validator checks these too, in the current full scene.
        if np.max(abs(np.array(controls[0])[index:index+5]-previous[index:index+5]))>.02:
            raise InvalidTask('route start differs from measured or preceding anchor')
        points,timing=retime(np.vstack([previous,controls]),bounds,dwell_s=2. if name=='close' else 0.)
        compiled.append(CarryStage(name,points,timing,attached))
        previous=np.asarray(points[-1].positions_rad)
    validate_program_winding(compiled,arm.profile)
    resolved=plan.resolved_grasp if operation in ('pick','loaded') else plan.released_grasp
    from .arm_routes import ContactEnvelope
    contact=None
    if contact_control_interval is not None:
        contact=ContactEnvelope(index+5,*sorted(contact_control_interval),close_control,scene.object_id,scene.stamp.capture_id)
    return TaskRouteProgram(operation,scene.object_id,scene.stamp.capture_id,scene.stamp.context.calibration_id,
                            scene.model_revision,scene.lift_m,tuple(compiled),resolved,contact,scene.stamp.context)


class TaskRoutePort:
    """Checked route stages on the SAME arm owner, with independent evidence.

    context_for checks actual state/current scene for each stage. scene_command
    builds the existing attachment/detachment command from measured evidence.
    Neither callback may infer a successful grasp from a completed trajectory.
    """
    def __init__(self, arm, scene, *, context_for, scene_command, monitor, clock, loaded_tail=None):
        self.arm,self.scene,self.context_for,self.scene_command=arm,scene,context_for,scene_command
        self.clock=clock
        self.loaded_tail=loaded_tail
        self.sequence=SequencePort(self._build,monitor,clock)

    def ready(self): return self.sequence.ready() and self.arm.ready()
    def start(self,g,p): self.sequence.start(g,p)
    def poll(self,g): return self.sequence.poll(g)
    def cancel(self,g): self.sequence.cancel(g)
    def phase_safe(self,g): return self.sequence.phase_safe(g)
    @property
    def owner(self): return self.sequence.owner

    def _build(self,parameters):
        if set(parameters)!={'program'} or not isinstance(parameters['program'],TaskRouteProgram):
            raise InvalidTask('typed checked task program required')
        program=parameters['program']; phases=[]
        stationary=('hardware_ready','base_stopped','lift_stopped')
        held=('load_retained','object_attached')
        released=('release_verified','object_detached')
        for stage in program.stages:
            if program.operation=='pick' and stage.name in ('lift','carry') and self.loaded_tail is not None:
                if stage.name=='lift':
                    phases.append(Phase('pick_measured_carry',self.loaded_tail,{'grasp_program':program},
                        stationary+held,stationary+held,('arm_in_transport_pose','arm_clear','load_retained'),240.))
                continue
            name=program.operation+'_'+stage.name
            before=during=after=()
            if program.operation in ('pick','loaded'):
                if stage.name=='open': before=('gripper_empty',);after=('gripper_open','gripper_empty')
                elif stage.name in ('pregrasp','approach'): before=during=('gripper_open','gripper_empty')
                elif stage.name=='close': after=('contact_plausible',)
                elif stage.name=='probe': before=('contact_plausible',);during=('probe_contact_live','probe_clearance_verified');after=('probe_follow_verified','grasp_verified','load_retained')
                else: before=during=held
            else:
                if stage.name in ('place_preplace','place_lower'): before=during=held
                elif stage.name=='release': before=(*held,'placement_support_verified');during=('object_attached','placement_support_verified');after=('release_verified',)
                else: before=during=released
            if stage.name in ('carry','empty_transport'): after=(*after,'arm_in_transport_pose','arm_clear')
            def resolve(_,args,now,s=stage):
                context=self.context_for(program,s,now)
                result={'trajectory':s.points,'context':context}
                if s.attached or s.name=='release':
                    if program.contact_envelope is None:raise InvalidTask('calibrated contact envelope required for loaded motion')
                    result['contact_envelope']=(replace(program.contact_envelope,release_goal_rad=s.points[-1].positions_rad[program.contact_envelope.axis]) if s.name=='release' else program.contact_envelope)
                return result
            port=ResolvedPort(name,self.arm,resolve,self.clock)
            duration=stage.points[-1].offset_ms/1000+5.
            phases.append(Phase(name,port,{},stationary+before,stationary+during,after,min(duration,300.)))
            if stage.name in ('probe','release'):
                attach=stage.name=='probe'; label='attach' if attach else 'detach'
                conditions=('load_retained',) if attach else ('release_verified',)
                after_scene=('object_attached',) if attach else ('object_detached',)
                def resolve_scene(_,args,now,action=label):
                    return self.scene_command(action,program,now)
                port=ResolvedPort(label,self.scene,resolve_scene,self.clock)
                phases.append(Phase(program.operation+'_'+label,port,{},stationary+conditions,
                                    stationary+conditions,after_scene,5.))
        return tuple(phases)
