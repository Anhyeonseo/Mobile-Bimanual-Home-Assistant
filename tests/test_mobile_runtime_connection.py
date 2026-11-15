"""Full existing application with new async policy/route ports, synthetic devices.

Real SO101 inverse reach and native full-mesh routes have separate tests. This
fixture isolates orchestration, evidence gates and every cancellation boundary.
"""
import json
import time
from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from home_robot_tasks.offline_stack import OfflineStack, SyntheticPort
from home_robot_tasks.assistant.prealignment_binding import PrealignmentPolicy
from home_robot_tasks.assistant.alignment_contracts import TaskScene
from home_robot_tasks.assistant.contracts import Context, Stamp
from home_robot_tasks.task_route_program import TaskRoutePort
from home_robot_tasks.manipulation_port import PlanningContext
from home_robot_tasks.execution import Feedback
from test_mobile_task_pipeline import route_program

ROOT=Path(__file__).resolve().parents[1]
read=lambda name:json.loads((ROOT/'config'/name).read_text())


class ConnectedFixture(OfflineStack):
    def joint_placement_policy(self,map_document):
        self.new_pipeline_events=[]
        ctx=Context(map_document['map_id'],map_document['revision'],self.camera.policy.calibration_id,'fixture-boot')
        def observed(target,operation,object_id):
            x,y=np.meshgrid(np.linspace(0,.16,20),np.linspace(0,.04,12))
            return TaskScene(object_id,'sofa' if operation=='pick' else 'bedroom_drop_zone',
                Stamp(target['observed_s'],ctx,target['capture_id']),self.scene_revision,'fixture-model',target['pose_revision'],
                self.capture_poses[target['capture_id']],self.height/1e6,
                np.c_[x.ravel(),y.ravel(),np.full(x.size,.82)],.8,{'fixture':'a'*64})
        def alignment(r):
            self.new_pipeline_events.append((r.skill,r.scene.stamp.capture_id))
            x,y,yaw=r.configuration[0]
            return {'status':'SCREENED_PROPOSALS','candidates':[dict(base_xy_yaw=[x+.05,y,yaw],lift_travel_m=.5,arm='left')]}
        def route(r):
            self.new_pipeline_events.append((r.skill,r.scene.stamp.capture_id))
            return replace(route_program(r.skill),object_id=r.scene.object_id,capture_id=r.scene.stamp.capture_id,
                           calibration_id=ctx.calibration_id,model_revision='fixture-model')
        return PrealignmentPolicy(scene_for=observed,
            configuration=lambda:(tuple(self.xy_yaw),self.height/1e6,self.now,self.velocity==(0.,0.,0.) and self.lift_speed==0),
            epoch=lambda:(self.boot,self.scene_revision),prepare=lambda *a:{'transport':read('fetch_stack.simulation.json')['catalog']['transport']},
            alignment_planner=alignment,route_planner=route,navigation_plan=lambda c,r:dict(frame_id='map',
                goal=dict(x=c['base_xy_yaw'][0],y=c['base_xy_yaw'][1],yaw_rad=c['base_xy_yaw'][2])),
            loaded_transport=lambda *a:read('fetch_stack.simulation.json')['catalog']['transport'])

    def manipulation_factory(self,*,motion,arm,monitor,clock,catalog):
        plant=self
        class RouteMotion(SyntheticPort):
            def start(self,g,p):
                super().start(g,p);self.entries[g]['stage']=plant.current_stage
            def poll(self,g):
                result=super().poll(g)
                if result.state=='SUCCEEDED':
                    if self.entries[g]['stage']=='close':plant.held=True
                    if self.entries[g]['stage']=='release':plant.held=False
                return result
        route_motion=RouteMotion(self,'compiled_arm')
        def context(program,stage,now):
            self.current_stage=stage.name
            return PlanningContext(self.now,program.capture_id,self.state.motion().pose_revision,self.scene_revision,
                program.calibration_id,'base_link',(0.,)*12,True,True)
        def evidence(g,phase,now):
            f=monitor(g,phase,now)
            return Feedback(g,f.observed_s,f.status,{**f.conditions,
                'contact_plausible':self.held,'probe_contact_live':self.held,
                'probe_clearance_verified':True,'probe_follow_verified':self.held,
                'placement_support_verified':True,'object_attached':self.attached=='remote_control','object_detached':self.attached is None})
        from home_robot_tasks.async_resolution import AsyncResolvedPort
        from types import SimpleNamespace
        child=TaskRoutePort(route_motion,self.ports['scene'],context_for=context,
            scene_command=lambda action,p,t:dict(operation=action,object_id=p.object_id),monitor=evidence,clock=clock)
        tail_policy=SimpleNamespace(
            snapshot=lambda skill,parameters,catalog,now:parameters['grasp_program'],
            compute=lambda program:replace(program,operation='loaded',stages=program.stages[-2:]),
            check_pending=lambda *args:None,
            commit=lambda request,result,catalog,now:{'program':result})
        self.fixture_tail=AsyncResolvedPort('loaded',child,tail_policy,SimpleNamespace(resolve=None),clock)
        p=TaskRoutePort(route_motion,self.ports['scene'],context_for=context,loaded_tail=self.fixture_tail,
            scene_command=lambda action,program,t:dict(operation=action,object_id=program.object_id),monitor=evidence,clock=clock)
        p.history=lambda g:p.sequence.entries[g]['history']
        return p

    def tick(self):
        # Yield to the background worker; simulated runtime still advances 20 Hz.
        time.sleep(.0005)
        return super().tick()


def run_fixture(cancel_step=None):
    p=ConnectedFixture(read('fetch_stack.simulation.json'),read('home.example.json'),read('navigation_map.simulation.json'),joint_placement=True)
    try:
        result=p.run(dict(schema_version=1,request_id='mobile',operation='fetch_object',request=read('fetch_remote.example.json')),cancel_step=cancel_step)
        return p,result
    finally:
        p.assembly.catalog.placement_policy.close()
        p.fixture_tail.close()


def test_new_pipeline_runs_all_fifteen_steps_on_existing_runtime():
    p,r=run_fixture()
    assert r['status']=='SUCCEEDED',r
    assert r['completed_steps']==15 and not r['physical_task_completed'] and r['hardware_commands']==0
    assert [s for s,_ in p.new_pipeline_events]==['align_for_pick','pick','align_for_place','place']
    assert p.new_pipeline_events[0][1]!=p.new_pipeline_events[1][1]
    assert p.new_pipeline_events[2][1]!=p.new_pipeline_events[3][1]
    assert not p.held and p.attached is None
    phases=[h['phase'] for h in r['manipulation_stages']]
    assert phases.index('pick_probe')<phases.index('pick_attach')<phases.index('pick_measured_carry')
    assert phases.index('place_release')<phases.index('place_detach')<phases.index('place_empty_transport')


@pytest.mark.parametrize('step',range(15))
def test_new_pipeline_cancels_each_stage_without_replaying_worker(step):
    p,r=run_fixture(step)
    assert r['status']=='CANCELLED',r
    assert r['stop_requests']==1


def test_ros_assembly_connects_new_policy_and_checked_alignment_without_second_writer(monkeypatch):
    from types import SimpleNamespace as NS
    from home_robot_tasks import ros_stack
    state=object();alignment=object();factory=object();policy=object();seen={}
    mobile=NS(state=state,runtime_options=lambda a:dict(alignment=a,manipulation_factory=factory,placement_policy=policy))
    for name in ('RosRGBDSource','InferencePort','MoveItPlanPort'):
        monkeypatch.setattr(ros_stack,name,lambda *a,**k:NS(close=lambda:None))
    monkeypatch.setattr(ros_stack,'EvidencePublisher',lambda *a:object())
    def build(**kwargs):seen.update(kwargs);return NS()
    monkeypatch.setattr(ros_stack,'build_runtime',build)
    args=dict(node=None,tf_buffer=None,world={},navigation_map=None,catalog_document={},state=NS(motion_at=lambda t:None),
        payload=None,camera=NS(policy=None),detect=lambda a:None,moveit_bridge=NS(request=None,map=None),
        arm_adapter=None,arm_route_builder=None,lift_client=None,stop_client=None,evidence_client=None,
        clock=lambda:1.,scene_revision=lambda:'s',surface_observer=None,rgbd_topics={},depth_scale_m=.001,navigation_port=object())
    mobile.state=args['state'];args['navigation_port']=alignment
    result=ros_stack.build_ros_runtime(**args,mobile_bindings=mobile,alignment_port=alignment)
    assert seen['placement_policy'] is policy and seen['manipulation_factory'] is factory and seen['alignment'] is alignment
    assert result.mobile_bindings is mobile and callable(seen['arm'])  # factory only; not connected


def test_ros_assembly_refuses_mobile_without_alignment_before_acquiring_devices():
    from types import SimpleNamespace as NS
    from home_robot_tasks import ros_stack
    from home_robot_tasks.fetch import InvalidTask
    kwargs={k:None for k in ('node','tf_buffer','world','navigation_map','catalog_document','state','payload','camera','detect',
        'moveit_bridge','arm_adapter','arm_route_builder','lift_client','stop_client','evidence_client','clock','scene_revision',
        'surface_observer','rgbd_topics','depth_scale_m')}
    with pytest.raises(InvalidTask,match='alignment port'):
        ros_stack.build_ros_runtime(**kwargs,mobile_bindings=NS(state=None))
