"""Replan the loaded tail from the measured attachment after the lift probe."""
from dataclasses import dataclass, replace
from types import SimpleNamespace
import numpy as np
from .carry_load import CarriedGeometry
from .carry_posture import carry_goals, checked_joints
from .carry_planning import PickCarryPlan, PlanRejected
from .task_route_program import compile_task_route, TaskRoutePort
from .async_resolution import AsyncResolvedPort
from .s2_geometry import JOINTS, model_from_control
from .fetch import InvalidTask


@dataclass(frozen=True)
class LoadedSnapshot:
    route_input: object
    scene: object
    retained_object: object
    epoch: object
    observed_s: float
    remaining_lift_m: float = .016


def plan_measured_carry(snapshot):
    d=snapshot.route_input;arm=d.arm;held=snapshot.retained_object
    held.check(snapshot.scene.stamp.context,snapshot.scene.object_id,snapshot.observed_s,snapshot.scene.model_revision)
    if not .004<=snapshot.remaining_lift_m<=.05:raise PlanRejected('bounded remaining lift required')
    index=0 if arm.side=='left' else 6
    start=checked_joints(arm,[model_from_control(float(d.anchor_control_rad[index+i]),arm.profile['arms'][arm.side]['alignment'][n])
                              for i,n in enumerate(JOINTS[:5])])
    if (d.held_candidate is None or d.held_candidate.candidate_id!=held.grasp_id
            or d.held_candidate.calibration_id!=arm.profile['calibration_id']
            or snapshot.scene.stamp.context.calibration_id!=arm.profile['calibration_id']):
        raise PlanRejected('verified contact candidate and calibration required')
    candidate=replace(d.held_candidate,base_from_grasp_tcp=arm.fk(start)[0],tcp_from_object=held.tcp_from_object.copy(),
        object_size_m=held.size_m,payload_mesh=held.payload_mesh,scene_id=snapshot.scene.stamp.capture_id)
    if candidate.object_constraint:
        candidate=replace(candidate,object_constraint=replace(candidate.object_constraint,tcp_from_object=held.tcp_from_object.copy()))
    target=candidate.base_from_grasp_tcp.copy();target[2,3]+=snapshot.remaining_lift_m
    lift=d.planner.cartesian(start,target,candidate,attached=True,label='lift')
    goals=carry_goals(arm,lift.end,candidate.base_from_grasp_tcp,object_constraint=candidate.object_constraint,
                     payload=CarriedGeometry(candidate.tcp_from_object,candidate.object_size_m,candidate.payload_mesh))
    failures=[]
    for goal in goals:
        try:
            carry=d.planner.connect(lift.end,goal.model_rad,candidate,attached=True,label='carry')
            plan=PickCarryPlan('OFFLINE_ROUTE_FOUND',candidate_id=candidate.candidate_id,
                stages=[lift,carry],carry_goal=goal,resolved_grasp=candidate)
            return compile_task_route('loaded',plan,scene=snapshot.scene,arm=arm,anchor_control_rad=d.anchor_control_rad,
                bounds=d.bounds,open_control=d.open_control,close_control=d.hold_control,
                contact_control_interval=d.contact_control_interval)
        except PlanRejected as exc:failures.append(str(exc))
    raise PlanRejected('no loaded carry route from measured attachment: '+str(failures[-3:]))


class LoadedCarryPort(AsyncResolvedPort):
    def __init__(self,arm,scene_port,*,snapshot,check_current,context_for,scene_command,monitor,clock,worker=None,program_sink=None):
        self.capture=snapshot;self.check=check_current
        class Policy:
            def snapshot(inner,skill,parameters,catalog,now):
                value=snapshot(parameters['grasp_program'],now)
                if not isinstance(value,LoadedSnapshot):raise InvalidTask('measured loaded snapshot required')
                value.retained_object.check(value.scene.stamp.context,value.scene.object_id,now,value.scene.model_revision)
                if not 0<=now-value.observed_s<=.5:raise InvalidTask('stale loaded state')
                return value
            def compute(inner,request):return plan_measured_carry(request)
            def check_pending(inner,request,catalog,now):
                if check_current(request,now) is not True:raise InvalidTask('loaded planning state changed')
            def commit(inner,request,result,catalog,now):
                inner.check_pending(request,catalog,now)
                if (result.operation!='loaded' or result.object_id!=request.scene.object_id
                        or result.capture_id!=request.scene.stamp.capture_id):raise InvalidTask('loaded route identity mismatch')
                if program_sink is not None:program_sink(result)
                return {'program':result}
        child=TaskRoutePort(arm,scene_port,context_for=context_for,scene_command=scene_command,monitor=monitor,clock=clock)
        super().__init__('measured_carry',child,Policy(),SimpleNamespace(resolve=None),clock,worker=worker)
