"""Ideal wheel/depth fixtures for whole-runtime contract tests, never deployment.

The controller follows offline grid waypoints with toy control, NOT Nav2. Free voxels are an
explicit synthetic observation oracle, NOT D415 coverage evidence. Production
wheel conversion, feedback odometry, travel guard and session ownership run.
"""
import math
from types import SimpleNamespace
from .base_driver import BaseProfile, MobileBaseDriver
from .base_motion import OmniKinematics, Wheel
from .navigation_binding import GuardedNavigationPort, StateTravelEvidence, TravelSessionPort
from .navigation_depth import DepthProfile, DepthVolume, TravelGuard
from .skill_ports import PortResult


class OfflineNavigation:
    def __init__(self, plant):
        self.plant=plant;self.session=7;self.rates=(0,0,0,0);self.commands=[]
        self.depth_enabled=True;self.clearance_enabled=True;self.clearance=(0.,'initial',False)
        self.resets=[];self.starts=[];self.goal=None;self.parameters=None;self.cancelled=False
        self.volume=DepthVolume(DepthProfile('synthetic-depth','synthetic-calibration',.1,.1,4,0,1,.25,.5))
        self.volume.reset('initial')
        self.guard=TravelGuard(self.volume,radius_m=.2,height_m=1,margin_m=.02,reaction_s=.1,
                              braking_m_s2=1,maximum_speed_m_s=.4,maximum_yaw_rad_s=.8)
        geometry=OmniKinematics(tuple(Wheel(.2*math.cos(a),.2*math.sin(a),a+math.pi/2,.05)
                                     for a in (0,2*math.pi/3,4*math.pi/3)))
        self.evidence=StateTravelEvidence(plant.state,self.volume,lambda:self.clearance)
        self.driver=MobileBaseDriver(self,geometry,BaseProfile((100,)*3,(10,)*3,(100,)*3,.2,.1,.1),
            plant.clock,self.guard,self.evidence,lambda reason:plant.stop.request())
        self.driver.odometry.pose=plant.xy_yaw
        guarded=GuardedNavigationPort(self,self.driver,plant.clock,lambda:self.volume.epoch,
            lambda after:self.driver.observation is not None and self.driver.observation.observed_s>after
                         and all(abs(v)<1e-4 for v in self.driver.observation.twist))
        self.port=TravelSessionPort(guarded,self.guard,self.evidence,plant.clock,self.reset)

    def reset(self,epoch):
        self.volume.reset(epoch);self.clearance=(0.,epoch,False)
        self.resets.append((self.plant.now,epoch,self.plant.height))

    def status(self):
        tick=round(self.plant.now*1000)
        return SimpleNamespace(feedback_tick_ms=tick,mcu_tick_ms=tick,velocity_raw=self.rates,
                               boot_id=self.plant.boot,ready_for_motion=lambda *args:True)

    def velocity(self,rates,**parameters):
        self.commands.append((self.plant.now,rates,self.plant.height,self.volume.epoch))
        self.rates=rates
        return self.status()

    def ready(self):return self.goal is None
    def start(self,goal,parameters):
        self.goal=goal;self.parameters=parameters;self.cancelled=False
        self.starts.append((goal,parameters))
        self.path=list(parameters["path_xy"])+[parameters["goal"]];self.index=0
    def poll(self,goal):
        if self.cancelled:return PortResult('CANCELLED')
        target=self.parameters['goal'];x,y,yaw=self.plant.xy_yaw
        if self.index==len(self.path)-1 and math.hypot(target['x']-x,target['y']-y)<.025 and abs(self.parameters['goal']['yaw_rad']-yaw)<.02:
            self.goal=None
            return PortResult('SUCCEEDED')
        return PortResult('RUNNING')
    def cancel(self,goal):self.cancelled=True;self.goal=None

    def tick(self):
        p=self.plant
        if p.stop.sent is not None:self.rates=(0,0,0,0)
        if self.depth_enabled:
            x,y,_=self.driver.odometry.pose;cx,cy=math.floor(x/.1),math.floor(y/.1)
            self.volume.cells={(a,b,z):(False,p.now) for a in range(cx-12,cx+13)
                               for b in range(cy-12,cy+13) for z in range(10)}
            self.volume.last_stamp=p.now;self.volume.error=None
        if self.clearance_enabled:self.clearance=(p.now,self.volume.epoch,True)
        if self.goal is not None and self.driver.owner is not None and not self.driver.braking and not self.driver.fault:
            x,y,yaw=p.xy_yaw;t=self.path[self.index]
            if self.index<len(self.path)-1 and math.hypot(t['x']-x,t['y']-y)<.04:
                self.index+=1;t=self.path[self.index]
            dx,dy=2*(t['x']-x),2*(t['y']-y)
            scale=max(1.,math.hypot(dx,dy)/.3)
            vx,vy=dx/scale,dy/scale
            self.driver.submit((math.cos(yaw)*vx+math.sin(yaw)*vy,-math.sin(yaw)*vx+math.cos(yaw)*vy,
                                max(-.4,min(.4,self.parameters['goal']['yaw_rad']-yaw))),
                observed_s=p.now,owner=self.driver.owner,source='nav2',epoch=self.driver.epoch)
        observation=self.driver.tick()
        if observation is not None:p.xy_yaw,p.velocity=observation.pose,observation.twist
        elif p.stop.sent is not None:p.velocity=(0.,0.,0.)

    def maintain(self):
        if self.driver.fault:raise ValueError(self.driver.fault)
