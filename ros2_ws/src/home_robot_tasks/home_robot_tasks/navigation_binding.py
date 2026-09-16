"""Navigation action lifetime and shared base ownership, without auto-arming."""
from .fetch import InvalidTask
from .skill_ports import PortResult


class GuardedNavigationPort:
    def __init__(self,nav2,driver,clock,epoch,base_stopped):
        self.nav2,self.driver,self.clock,self.epoch,self.base_stopped=nav2,driver,clock,epoch,base_stopped
        self.goal=None
        self.finished_at=None

    def ready(self):
        return self.goal is None and self.driver.fault is None and self.nav2.ready()

    def start(self,goal_id,parameters):
        if not self.ready():raise InvalidTask('guarded navigation unavailable')
        self.driver.acquire(goal_id,'nav2',self.epoch())
        self.goal=goal_id
        self.finished_at=None
        try:self.nav2.start(goal_id,parameters)
        except Exception:
            self.driver._fault('navigation start failed')
            raise

    def poll(self,goal_id):
        if goal_id!=self.goal:raise InvalidTask('navigation owner mismatch')
        if self.driver.fault:return PortResult('FAILED',self.driver.fault)
        result=self.nav2.poll(goal_id)
        if result.state=='SUCCEEDED':
            if self.finished_at is None:
                self.finished_at=self.clock()
            self.driver.braking=True
            # Callback must prove measured stop acquired AFTER action completion.
            if self.clock()>self.finished_at and self.base_stopped(self.finished_at) is True:
                self.driver.release(stop_confirmed=True)
                self.goal=None
                return result
            # A Nav2 result does not prove that wheels stopped. Fresh explicit
            # zero command is intentional braking, never a refreshed old move.
            now=self.clock()
            if self.driver.command is None or now>self.driver.command[0]:
                self.driver.submit((0.,0.,0.),observed_s=now,owner=goal_id,source='nav2',epoch=self.driver.epoch)
            return PortResult('RUNNING','awaiting measured base stop')
        if result.state in ('FAILED','CANCELLED'):
            self.driver._fault('navigation '+result.state.lower())
        return result

    def cancel(self,goal_id):
        self.driver._fault('navigation cancelled')
        self.nav2.cancel(goal_id)

    def maintain(self,io):
        # Serialized by RobotRuntime with arms/lift. A fault propagates to its
        # common whole-stop backend and task cancellation, retaining admission.
        io.tick()
        if self.driver.fault:raise InvalidTask(self.driver.fault)


class StateTravelEvidence:
    """Combine measured posture and separately measured initial-body clearance.

    clearance() returns (original_monotonic_stamp, epoch, is_clear). It is not
    inferred from arm pose, LiDAR floor clearance or a successful Nav2 result.
    """
    def __init__(self,state,volume,clearance):
        self.state,self.volume,self.clearance=state,volume,clearance

    def __call__(self):
        from .navigation_depth import TravelEvidence
        from .navigation import number
        c=self.state.conditions()
        stamp,epoch,clear=self.clearance()
        stamp=number(stamp,'envelope observation time')
        if not 0<=self.state.clock()-stamp<=self.volume.p.maximum_age_s:
            raise InvalidTask('stale or future initial-envelope evidence')
        ready=all(c.get(k) is True for k in ('hardware_ready','arm_in_transport_pose',
            'lift_in_transport_position','lift_stopped','lift_homed','lift_hold_verified'))
        observed=min(stamp,self.state.latest.observed_s) if self.state.latest else stamp
        return TravelEvidence(observed,epoch,ready,c.get('localized') is True,clear)


def build_guarded_navigation(*,node,client,geometry,profile,guard,state,clearance,clock,
                             request_stop,io_parameters,reset_observations,action_name='/offline_nav2/navigate_to_pose'):
    """Wire once, then inject returned port/maintenance into build_ros_runtime.

    client is the existing single-owner MobileClient; request_stop MUST use the
    runtime's common whole-stop client. Never connect original teleop Host to
    the same bus. Depth producer.poll belongs in navigation_observations. Pass its reset method
    as reset_observations (also reset the independent clearance source).
    """
    from .base_driver import MobileBaseDriver,RosBaseIO
    from .nav2_port import Nav2Port
    evidence=StateTravelEvidence(state,guard.volume,clearance)
    driver=MobileBaseDriver(client,geometry,profile,clock,guard,evidence,lambda reason:request_stop())
    io=RosBaseIO(node,driver,**io_parameters)
    def stopped(after):
        c=state.conditions()
        return (state.latest is not None and state.latest.observed_s>after
                and c.get('base_stopped') is True and c.get('hardware_ready') is True)
    port=GuardedNavigationPort(Nav2Port(node,action_name),driver,clock,lambda:guard.volume.epoch,stopped)
    session=TravelSessionPort(port,guard,evidence,clock,reset_observations)
    return session,io,lambda:port.maintain(io)


class TravelSessionPort:
    """Stationary, bounded re-observation before granting Nav2 wheel ownership.

    reset(epoch) must flush both the depth producer and any clearance producer.
    Each acquisition gets a new epoch; only post-transition measured evidence
    may enable motion. This does not home, arm, turn to search, or recover faults.
    """
    def __init__(self, navigation, guard, evidence, clock, reset, *, timeout_s=2.):
        from .navigation import number
        self.navigation,self.guard,self.evidence,self.clock,self.reset=navigation,guard,evidence,clock,reset
        self.timeout=number(timeout_s,'travel observation timeout')
        if not 0 < self.timeout <= 10:raise InvalidTask('bounded travel observation timeout required')
        if not callable(reset):raise InvalidTask('depth and clearance reset callback required')
        self.entries={};self.owner=None;self.sequence=0

    def ready(self):
        return self.owner is None and self.navigation.ready()

    def start(self,goal_id,parameters):
        if not self.ready() or goal_id in self.entries or len(self.entries)>=128:
            raise InvalidTask('travel session owned or exhausted')
        self.sequence+=1
        epoch=f'travel-session-{self.sequence}'
        self.entries[goal_id]=dict(started=self.clock(),epoch=epoch,parameters=parameters,
                                   dispatched=False,result=None,reason='awaiting fresh travel observations')
        self.owner=goal_id
        try:self.reset(epoch)
        except Exception as error:
            self.entries[goal_id]['result']=PortResult('FAILED',str(error));self.owner=None
            raise

    def poll(self,goal_id):
        entry=self.entries[goal_id]
        if entry['result'] is not None:return entry['result']
        if entry['dispatched']:
            result=self.navigation.poll(goal_id)
        elif self.clock()-entry['started']>=self.timeout:
            result=PortResult('FAILED','travel observation timeout: '+entry['reason'])
        else:
            try:
                evidence=self.evidence()
                volume=self.guard.volume
                if (evidence.epoch!=entry['epoch'] or volume.epoch!=entry['epoch']
                        or evidence.observed_s<=entry['started'] or volume.last_stamp is None
                        or volume.last_stamp<=entry['started']):
                    raise InvalidTask('waiting for post-transition evidence')
                driver=self.navigation.driver
                observation=driver.observation
                if observation is None or observation.observed_s<=entry['started']:
                    raise InvalidTask('waiting for measured base feedback')
                if any(abs(v)>1e-4 for v in observation.twist):
                    raise InvalidTask('travel preparation requires measured stationary base')
                self.guard.check((0.,0.,0.),observation.twist,observation.pose,evidence,self.clock())
            except InvalidTask as error:
                entry['reason']=str(error)
                return PortResult('RUNNING',entry['reason'])
            # Mark attempted before calling: a partially started action must be
            # cancelled through the shared stop path if dispatch raises.
            entry['dispatched']=True
            self.navigation.start(goal_id,entry['parameters'])
            return PortResult('RUNNING')
        if result.state in ('SUCCEEDED','FAILED','CANCELLED'):
            entry['result']=result;self.owner=None
        return result

    def cancel(self,goal_id):
        entry=self.entries[goal_id]
        if entry['result'] is not None:return
        if entry['dispatched']:self.navigation.cancel(goal_id)
        else:
            entry['result']=PortResult('CANCELLED');self.owner=None
