"""Complete PC runtime with explicitly synthetic devices, images and mechanics.

Uses production application, robot-state, resolver, search/capture/manipulation,
phase/payload/stop coordinators. It makes NO claim about physical motion/vision.
Optional externally supplied Nav2/planning ports can replace the synthetic ones.
"""
from types import SimpleNamespace
import numpy as np
from .capture_port import ImagePair, ImageDetection, LatestRGBDSource
from .manipulation_port import PlanningContext
from .navigation import NavigationMap
from .payload_monitor import PayloadMonitor, GripMeasurement, PayloadObservation
from .perception import CameraIntrinsics
from .robot_state import RobotObservation, RobotStateStore
from .runtime_factory import build_runtime
from .skill_ports import PortResult
from .pick_place import PickPlacePort
from .execution import Feedback
from .evidence_producer import EvidenceProducer, SourceSample, FIELDS
from .surface_observer import SurfaceSampler, SurfaceObserver
from .stationary_rgbd import StationaryRGBD, CapturePolicy, RGBDFrame


class SyntheticPort:
    def __init__(self, plant, role):
        self.plant,self.role = plant,role
        self.entries = {};self.starts=[]

    def ready(self):
        return not any(e['state']=='RUNNING' for e in self.entries.values())

    def start(self,g,p):
        if not self.ready() or g in self.entries: raise RuntimeError('synthetic port owned')
        self.entries[g] = dict(state='RUNNING',polls=0,parameters=p)
        self.starts.append((g,p))
        if self.role == 'lift': self.plant.lift_speed = .1
        if self.role == 'navigation': self.plant.velocity = (.1,0.,0.)

    def poll(self,g):
        e=self.entries[g]
        if e['state']=='RUNNING':
            e['polls']+=1
            if e['polls']>=2:
                e['state']='SUCCEEDED';p=e['parameters']
                if self.role=='navigation':
                    self.plant.xy_yaw=(p['goal']['x'],p['goal']['y'],p['goal']['yaw_rad']);self.plant.velocity=(0.,0.,0.)
                if self.role=='lift':self.plant.height=p['height_um'];self.plant.lift_speed=0.
                if self.role=='gripper': self.plant.held=p['action']=='close'
                if self.role=='scene':
                    self.plant.attached=p['object_id'] if p['operation']=='attach' else None
                    self.plant.scene_revision += '-updated'
        return PortResult(e['state'])

    def result(self,g):
        if self.role=='detector':
            if self.plant.hidden: return []
            return [ImageDetection('remote_control',.95,((20,18),(40,18),(40,30),(20,30)))]
        if self.role=='planner':return (self.entries[g]['parameters']['task']['operation'],)
        raise RuntimeError('no result for this port')

    def cancel(self,g):
        self.entries[g]['state']='CANCELLED'
        if self.role=='navigation':self.plant.velocity=(0.,0.,0.)
        if self.role=='lift':self.plant.lift_speed=0.


class SyntheticStop:
    def __init__(self,plant):self.plant=plant;self.sent=None;self.requests=0
    def request(self):
        if self.sent is None:
            self.requests+=1;self.sent=self.plant.now
            self.plant.velocity=(0.,0.,0.);self.plant.lift_speed=0.
    def status(self):
        p=self.plant
        confirmed=self.sent is not None and p.now-self.sent>=.15 and p.state.fresh()
        proof=p.payload.proof(p.now)
        confirmed=confirmed and proof.conditions['payload_safe']
        return SimpleNamespace(confirmed=confirmed,flags=255 if confirmed else 0),p.now


class OfflineStack:
    def __init__(self, config, world, map_document, *, navigation=None, planner=None, boot_id=42, guarded_navigation=False):
        if config.get('mode')!='simulation' or config.get('schema_version')!=1:
            raise ValueError('explicit simulation configuration required')
        self.now=1.;self.seq=0;self.boot=boot_id;self.xy_yaw=(1.5,1.5,0.)
        self.velocity=(0.,0.,0.);self.height=10000;self.lift_speed=0.;self.held=False
        self.hidden=False;self.sensor_loss=False;self.scene_revision='scene-1';self.attached=None
        self.clock=lambda:self.now
        self.state=RobotStateStore(config['state'],self.clock)
        self.evidence=EvidenceProducer(self.clock,boot_id=boot_id,profile_sha256='a'*64,
            maximum_age_s=config['state']['maximum_age_s'],maximum_skew_s=.05)
        self.payload=PayloadMonitor(**config['payload']);self.payload.expect('empty','remote_control',self.now)
        self.camera=StationaryRGBD(CapturePolicy(**config['camera']))
        self.surface=SurfaceObserver(self.camera,self.state.motion,self.clock,
            maximum_residual_m=.01,minimum_span_m=.1,maximum_tilt_rad=.3,destination_radius_m=.2)
        self.source=LatestRGBDSource()
        self.ports={role:SyntheticPort(self,role) for role in ('arm','lift','navigation','planner','detector','gripper','scene')}
        self.stop=SyntheticStop(self)
        self.observe()
        # Establish initial empty-gripper proof from multiple synthetic observations.
        for _ in range(4): self.now=round(self.now+.05,6);self.observe()
        self.guarded_navigation=None
        if guarded_navigation:
            if navigation is not None:raise ValueError('choose one navigation fixture')
            from .offline_navigation import OfflineNavigation
            self.guarded_navigation=OfflineNavigation(self)
            navigation=self.guarded_navigation.port
        self.assembly=build_runtime(world=world,navigation_map=NavigationMap(map_document),catalog_document=config['catalog'],
            state=self.state,payload=self.payload,camera=self.camera,source=self.source,detector=self.ports['detector'],
            navigation=navigation or self.ports['navigation'],arm=self.ports['arm'],lift=self.ports['lift'],
            planner=planner or self.ports['planner'],surface_observer=self.surface_observer,stop_client=self.stop,
            clock=self.clock,scene_revision=lambda:self.scene_revision,
            maintenance=(self.guarded_navigation.maintain,) if guarded_navigation else (),
            manipulation_factory=self.manipulation_factory,
            surface_sampler=SurfaceSampler({'bedroom_drop_zone':(24,16,40,32)}))
        self.runtime=self.assembly.runtime

    def manipulation_factory(self, *, motion, arm, monitor, clock, catalog):
        def resolve(stage, parameters, now):
            # Explicit toy geometry only. Real deployment must supply calibrated
            # pregrasp/retreat poses and fresh link/world object transforms.
            if stage.endswith(('_open', '_close')):
                return {'action': stage.rsplit('_',1)[1], 'arm':parameters['arm']}
            if stage.endswith(('_attach', '_detach')):
                return {'operation':stage.rsplit('_',1)[1], 'object_id':parameters['object_id']}
            task=dict(parameters)
            if stage.endswith(('_pregrasp', '_preplace', '_retreat')):
                task['position_m']=[*parameters['position_m'][:2],parameters['position_m'][2]+.03]
            return task
        def evidence(g, phase, now):
            f=monitor(g,phase,now)
            return Feedback(f.goal_id,f.observed_s,f.status,{**f.conditions,
                'object_attached':self.attached==catalog.object_id,
                'object_detached':self.attached is None})
        return PickPlacePort(motion,self.ports['gripper'],self.ports['scene'],resolve,evidence,clock)

    def observe(self):
        self.seq+=1
        if self.sensor_loss:return
        target = {}
        if hasattr(self, 'assembly'):
            catalog = self.assembly.catalog
            candidates = [t for t in (catalog.target, catalog.surface) if t]
            if candidates: target = max(candidates, key=lambda t: t['observed_s'])
        observation=RobotObservation(self.seq,self.boot,self.now,self.xy_yaw,self.velocity,(0.,)*12,
            self.height,self.lift_speed,True,True,True,True,True,True,True,
            bool(target),bool(target),target.get("capture_id", ""),target.get("scene_revision", ""))
        try:
            for source,keys in FIELDS.items():
                self.evidence.observe(source,SourceSample(self.seq,self.now,self.boot,'a'*64,
                    {key:getattr(observation,key) for key in keys}))
            fused=self.evidence.produce()
            if fused is not None:self.state.observe(fused)
        except Exception as error:
            self.state.fault=str(error)
            raise
        self.payload.observe_grip(GripMeasurement(self.seq,self.now,15 if self.held else 45,40 if self.held else 0,True),self.now)
        self.payload.observe_vision(PayloadObservation(self.seq,self.now,'remote_control','held' if self.held else 'empty',
            'bedroom_drop_zone',not self.held,self.camera.policy.camera_frame),self.now)
        self.payload.proof(self.now)
        p=self.camera.policy
        frame=RGBDFrame(f'frame-{self.seq}',self.now,self.state.motion().pose_revision,p.camera_frame,p.calibration_id,
            p.width,p.height,CameraIntrinsics(50,50,32,24),p.root_frame,self.now,
            ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1)),True)
        self.source.publish(ImagePair(frame,np.zeros((p.height,p.width,3),np.uint8),np.ones((p.height,p.width),np.float32),self.now))

    def surface_observer(self,skill,parameters,observation):
        return self.surface(skill,parameters,observation)

    def tick(self):
        self.now=round(self.now+.05,6)
        if self.guarded_navigation is not None:self.guarded_navigation.tick()
        self.observe()
        return self.runtime.tick()

    def run(self,request,*,cancel_step=None,fault=None):
        self.runtime.submit(request)
        task=self.runtime.app.tasks[request['request_id']]
        injected=False
        for _ in range(6200):
            if not injected and task.state=='RUNNING':
                if cancel_step is not None and task.index==cancel_step:
                    self.runtime.app.cancel(task.run_id,self.now);injected=True
                elif fault and task.step['skill']=='navigate_with_load':
                    if fault=='sensor_loss':self.sensor_loss=True
                    elif fault=='reboot':self.boot+=1
                    elif fault=='load_loss':self.held=False
                    injected=True
            try:self.tick()
            except ValueError as error:
                self.runtime.fault=str(error);self.runtime.tick()
            if task.state in task.TERMINAL or task.state=='STOP_UNCONFIRMED':break
        manipulation = self.assembly.manipulation
        stages = [h for goal in manipulation.sequence.entries for h in manipulation.history(goal)]
        return {**task.result(),'runtime_fault':self.runtime.fault,'synthetic_devices':True,
                'manipulation_stages':stages,
                'stop_requests':self.stop.requests,'maximum_tick_gap_s':self.runtime.maximum_observed_gap_s}
