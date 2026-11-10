"""L2 acquisition -> L3 observation. No commands, inferred poses or fake clocks.

Camera work runs independently of the serial writer. The latest observation is
bounded; dropped frames cannot extend freshness or restart a lost target track.
"""
from collections import deque
from dataclasses import dataclass
import math
import threading
import time

import numpy as np

from .carry_execution import CarryFeedback
from .carry_observation import CarryVisualMonitor
from .carry_rgbd import SurfacePatchTracker, StationarySceneCheck, SceneValidationError
from .probe_evidence import LiftSample
from .s2_geometry import rigid


@dataclass(frozen=True)
class JointExposure:
    sequence: int
    lower_s: float
    upper_s: float
    positions_rad: tuple

    @property
    def observed_s(self):return (self.lower_s+self.upper_s)/2


class ObservationNotFresh(ValueError):
    """Discard this measurement without renewing any previously valid timestamp."""


def joint_exposure(packet, sent_s, received_s):
    ages=tuple(packet.sample_age_ms)
    details=dict(status=packet.status_code,joint_count=packet.joint_count,present_mask=packet.present_mask,
                 round_trip_ms=(received_s-sent_s)*1000,sample_age_ms=ages)
    if (packet.status_code or packet.joint_count!=12 or packet.present_mask!=4095
            or len(ages)!=12 or len(packet.positions_urad)!=12
            or min(ages)<0 or received_s<sent_s):
        raise ValueError('invalid resident feedback: '+str(details))
    if received_s-sent_s>.02 or max(ages)>150:
        raise ObservationNotFresh('stale or uncertain resident feedback: '+str(details))
    # Earliest request start to latest response arrival is a conservative span;
    # do not replace it with host packet receipt time.
    return JointExposure(packet.completed_pairs,sent_s-max(ages)/1000,
                         received_s-min(ages)/1000,tuple(v/1e6 for v in packet.positions_urad))


def match_exposure(history, color_s, depth_s, now):
    if not 0<=now-min(color_s,depth_s)<=.2 or max(color_s,depth_s)>now:
        raise ValueError('camera exposure stale')
    matches=[j for j in history if max(j.upper_s,color_s,depth_s)-min(j.lower_s,color_s,depth_s)<=.05]
    if not matches:raise ValueError('no measured joints within RGB-D 50 ms window')
    return min(matches,key=lambda j:abs(j.observed_s-(color_s+depth_s)/2))


class FixedPlatformGuard:
    """Continuity relative to an explicitly established height, not auto-homing."""
    maximum_age_s = .2

    def __init__(self,height_m,metres_per_count):
        if not math.isfinite(height_m) or height_m<0 or not math.isfinite(metres_per_count) or metres_per_count==0:
            raise ValueError('established nonnegative lift height and nonzero signed scale required')
        self.height=height_m;self.scale=metres_per_count;self.origin=None
        self.tick=None;self.stamp=-1.;self.sequence=-1
        self.base_stopped=False;self.lift_stopped=False;self.current_height=height_m
        self.last_query=None
        self.accepted_query=None
        self.query_interval=None
    def update(self,packet,sent_s,received_s):
        self.last_query=dict(packet=packet,round_trip_ms=(received_s-sent_s)*1000,accepted=False)
        if (received_s<sent_s or packet['present_mask']!=15 or not packet['active']
                or len(packet['sample_age_ms'])!=4 or min(packet['sample_age_ms'])<0):
            raise ValueError('stationary platform feedback missing: '+str(self.last_query))
        # The fixed-platform guard is not a camera/joint synchronization test.
        # Use the same deadline at ingestion and consumption. A separate 180 ms
        # cutoff discarded valid 181 ms samples during reader handoff, leaving
        # the older cache to expire. Charge the full reply delay; never renew
        # the sample timestamp to receipt time. New-route headroom is checked
        # separately by route_start_ready(), not by dropping observations.
        stamp=sent_s-max(packet['sample_age_ms'])/1000
        if received_s-stamp>self.maximum_age_s:
            raise ObservationNotFresh('stationary platform feedback stale: '+str(self.last_query))
        tick=packet['firmware_tick_ms']
        if self.tick is not None:
            # Configuration is deliberately outside execution and may take
            # longer than one second. Compare the two actual clocks instead
            # of treating any such pause as a reboot. This never refreshes an
            # old sample: the per-packet age and live 200 ms guard still apply.
            elapsed=((tick-self.tick)&0xffffffff)/1000
            previous_sent,previous_received=self.query_interval
            earliest=max(0.,sent_s-previous_received-.005)
            latest=received_s-previous_sent+.005
            if elapsed>=2147483.648 or not earliest<=elapsed<=latest:
                raise ValueError('controller restarted or platform clock discontinuity: '+
                    str(dict(firmware_elapsed_s=elapsed,host_interval_s=(earliest,latest),
                             previous_tick=self.tick,current_tick=tick)))
        raw=np.asarray(packet['positions_raw'],dtype=int)
        if raw.shape!=(4,) or np.any(raw<0) or np.any(raw>4095):raise ValueError('invalid platform raw values')
        if self.origin is None:self.origin=raw.copy()
        delta=(raw-self.origin+2048)%4096-2048
        self.base_stopped=bool(np.max(abs(delta[:3]))<=4)
        self.lift_stopped=bool(abs(delta[3])<=8)
        # The coordinate profile already encodes direction: increasing raw
        # counts lower this robot's lift, so its metres_per_count is negative.
        self.current_height=self.height+float(delta[3])*self.scale
        self.tick=tick;self.sequence=packet['sequence']
        self.query_interval=(sent_s,received_s)
        self.stamp=stamp
        self.last_query['accepted']=True
        self.accepted_query=self.last_query
        if not self.base_stopped or not self.lift_stopped:raise ValueError('base or fixed lift moved')

    def require_fresh(self,now):
        age=now-self.stamp
        if not 0<=age<=self.maximum_age_s:
            raise ValueError('platform sample stale: '+str(dict(age_s=age,
                accepted_query=self.accepted_query,last_query=self.last_query)))


class MeasuredFK:
    """Numpy-only forward kinematics, exported from the same planning model."""
    def __init__(self,spec):
        self.mount=rigid(spec['mount']);self.tcp=rigid(spec['tcp'])
        self.joints=[(rigid(j['origin']),np.asarray(j['axis'],float)) for j in spec['joints']]
        self.offset=np.asarray(spec['home_model_rad'],float);self.sign=np.asarray(spec['control_to_model_sign'],float)
        if len(self.joints)!=5 or self.offset.shape!=(5,) or self.sign.shape!=(5,) or not np.isfinite(self.offset).all() or not np.all(abs(self.sign)==1):
            raise ValueError('five calibrated arm coordinates required')
        if any(a.shape!=(3,) or not np.isclose(np.linalg.norm(a),1) for _,a in self.joints):
            raise ValueError('unit joint axes required')
    def __call__(self,measured_controls):
        q=np.asarray(measured_controls[:5],float)*self.sign+self.offset
        if q.shape!=(5,) or not np.isfinite(q).all():raise ValueError('measured arm coordinates required')
        pose=self.mount.copy()
        for (origin,axis),angle in zip(self.joints,q):
            x,y,z=axis;cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            rot=np.eye(3)+math.sin(angle)*cross+(1-math.cos(angle))*(cross@cross)
            pose=pose@origin;pose[:3,:3]=pose[:3,:3]@rot
        return pose@self.tcp


class LiveCarryInputs:
    def __init__(self,program,fk,reference,mask,intrinsics,base_from_optical,
                 platform,*,clock=time.monotonic,error_budget_m=.0006,robot_sweep_mask=None,maximum_background_depth_m=None,
                 observation_policy='continuous_patch'):
        if observation_policy not in ('continuous_patch','snapshot_then_verify'):
            raise ValueError('explicit supported observation policy required')
        self.observation_policy=observation_policy
        self.admitted=False
        self.program,self.fk,self.platform,self.clock=program,fk,platform,clock
        self.reference,self.mask,self.intrinsics,self.transform=reference,mask,intrinsics,base_from_optical
        self.error_budget=error_budget_m
        self.scene=StationarySceneCheck(*reference,mask,maximum_background_depth_m=maximum_background_depth_m)
        self.background=StationarySceneCheck(*reference,mask,maximum_background_depth_m=maximum_background_depth_m)
        if robot_sweep_mask is not None:self.background.background &= ~robot_sweep_mask
        self.lock=threading.RLock();self.joints=deque(maxlen=40);self.generation=0
        self.tracker=None;self.failure=None;self.relation='unknown';self.phase='IDLE'
        self.scene_time=-1.;self.camera_time=-1.;self.latest_pair=None;self.latest_patch=None
        self.monitor=CarryVisualMonitor(probe_duration_s=next(s.timing['duration_s'] for s in program.stages if s.name=='probe'))
        self.validation=None;self.last_pair_stamp=-1.
        self.feedback_time=-1.
        self.epoch=0
    def reset_unarmed_history(self):
        """Explicit boundary after PREPARE, which resets MCU acquisition count."""
        with self.lock:
            if self.phase!='IDLE':raise ValueError('cannot reset a running observation')
            self.epoch+=1;self.joints.clear();self.latest_pair=None
            self.last_pair_stamp=-1.;self.feedback_time=-1.;self.generation+=1
    def add_joints(self,packet,sent_s,received_s):
        sample=joint_exposure(packet,sent_s,received_s)
        with self.lock:
            if self.joints and sample.sequence==self.joints[-1].sequence:
                if sample.positions_rad!=self.joints[-1].positions_rad:
                    raise ValueError('same acquisition sequence changed positions')
                return  # Cached values never acquire a new timestamp.
            if self.joints and sample.lower_s<self.joints[-1].lower_s-.002:
                raise ValueError('joint sample clock regressed')
            if sample.upper_s<self.feedback_time:
                raise ValueError('joint exposure predates previous sample')
            self.feedback_time=max(self.feedback_time,sample.observed_s)
            self.joints.append(sample);self.generation+=1
    def add_platform(self,packet,sent_s,received_s):
        with self.lock:self.platform.update(packet,sent_s,received_s);self.generation+=1
    def add_frame(self,record,images):
        if self.observation_policy=='snapshot_then_verify':
            return self._admission_frame(record,images)
        color,depth,metadata=images
        depth_m=depth.astype(float)*metadata['depth_scale_m']
        stamp=record['color_time_s'];now=self.clock()
        with self.lock:
            if self.failure or not self.joints:return
            phase=self.phase;history=tuple(self.joints);epoch=self.epoch
        # OpenCV/scene statistics stay outside the serial observer lock.
        # Only this worker owns tracker; the writer reads bounded cached data.
        try:
            validation=None
            if self.tracker is None:
                validation=self.scene.compare(color,depth_m)
                if not validation['unchanged']:raise SceneValidationError('planned scene changed; recapture/replan',validation)
                self.tracker=SurfacePatchTracker(color,depth_m,self.mask,self.intrinsics,self.transform,
                    patch_id=self.program.candidate_id+':visible-patch',observed_s=stamp,delta_error_floor_m=self.error_budget)
                patch=self.tracker.initial
            else:patch=self.tracker.update(color,depth_m,stamp)
            if phase=='IDLE':
                # Initial tracking already checked this exact exposure. A second
                # full image pass can age out the first RGB-D/joint pair on Jetson.
                if validation is None:validation=self.scene.compare(color,depth_m)
                if not validation['unchanged']:raise SceneValidationError('scene changed before admission',validation)
            else:
                background=self.background.compare(color,depth_m,background_only=True)
                if not background['unchanged']:raise SceneValidationError('visible environment changed outside robot sweep',background)
            try:j=match_exposure(history,stamp,record['depth_time_s'],now)
            except ValueError:j=None
            pose=self.fk(j.positions_rad) if j is not None else None
            with self.lock:
                if self.failure or epoch!=self.epoch:return
                if validation is not None:self.validation=validation;self.scene_time=stamp
                self.latest_patch=patch;self.camera_time=stamp;self.generation+=1
                if j is None or j.observed_s<=self.last_pair_stamp:return
                sample=LiftSample(self.program.capture_id,self.program.candidate_id,patch.patch_id,j.observed_s,stamp,
                                  tuple(pose[:3,3]),patch.xyz_base_m,patch.error_bound_m)
                self.latest_pair=(sample,pose,patch.sequence);self.last_pair_stamp=j.observed_s
                if self.monitor.reference is not None:
                    self.relation,_=self.monitor.observe(sample,pose,now)
        except (ValueError,RuntimeError) as exc:
            with self.lock:self.failure=exc;self.generation+=1

    def _admission_frame(self,record,images):
        """Confirm the selected target before executing the planned static scene.

        No patch is selected and no running image may replace the approved
        scene or renew its timestamp. Environment collisions belong to planning
        on the current RGB-D capture, not a global background pixel-equality
        interlock. This stationary mode does not monitor dynamic obstacles.
        Post-pick verification is separate.
        """
        with self.lock:
            if self.admitted or self.phase!='IDLE' or self.failure or not self.joints:return
            history=tuple(self.joints);epoch=self.epoch
        color,depth,metadata=images
        try:
            stamp=record['color_time_s']
            validation=self.scene.compare(color,depth.astype(float)*metadata['depth_scale_m'],target_only=True)
            validation.update(environment_basis='native_checked_rgbd_planning_snapshot',
                              live_background_revalidated=False,live_target_revalidated=True)
            if not validation['unchanged']:
                raise SceneValidationError('planned target changed before admission',validation)
            # Lack of a matching pair can be temporary during acquisition.
            try:j=match_exposure(history,stamp,record['depth_time_s'],self.clock())
            except ValueError:return
            with self.lock:
                if self.admitted or self.phase!='IDLE' or self.failure or epoch!=self.epoch:return
                self.validation=validation;self.scene_time=stamp;self.camera_time=stamp
                self.last_pair_stamp=j.observed_s;self.generation+=1
        except (ValueError,RuntimeError) as exc:
            with self.lock:
                if not self.admitted and self.phase=='IDLE' and epoch==self.epoch:
                    self.failure=exc;self.generation+=1

    def admit_snapshot(self):
        """Latch only a fresh successful admission, before the first motor route."""
        with self.lock:
            if self.observation_policy!='snapshot_then_verify' or self.phase!='IDLE' or self.admitted:
                raise ValueError('new idle snapshot admission required')
            self.feedback()
            self.admitted=True;self.generation+=1
    def set_phase(self,name):
        with self.lock:
            if name==self.phase:return
            if name=='probe' and self.observation_policy=='continuous_patch':
                if self.latest_pair is None:raise ValueError('no synchronized patch at probe start')
                sample,pose,_=self.latest_pair
                self.monitor.begin_probe(sample,pose,self.clock())
            self.phase=name;self.generation+=1
    def feedback(self):
        with self.lock:
            now=self.clock()
            if self.failure:raise self.failure
            if self.observation_policy=='snapshot_then_verify':
                return self._snapshot_feedback(now)
            if not self.joints or self.latest_pair is None:raise ValueError('waiting for measured RGB-D/joint pair')
            j=self.joints[-1];sample,_,sequence=self.latest_pair
            self._require_platform_fresh(now)
            if not 0<=now-min(sample.tool_time_s,sample.vision_time_s)<=.2:raise ValueError('target observation stale')
            p=self.program
            # Zero is the unavailable old-host exposure sentinel, never fresh
            # exposure evidence. Separate live scene_time renews admission.
            return CarryFeedback(self.generation,self.feedback_time,0.,p.capture_id,p.calibration_id,dict(p.source_sha256),
                j.positions_rad,self.platform.current_height,self.platform.base_stopped,self.platform.lift_stopped,
                True,True,self.relation,sequence,sample.vision_time_s,True,True,p.capture_id,p.candidate_id,self.scene_time)
    def validate_stage(self,stage,snapshot):
        # Native validation is bound by the bundle. Live admission checks the
        # unchanged scene; the route builder separately verifies measured anchor.
        f=self.feedback()
        if self.observation_policy=='snapshot_then_verify':
            return self.admitted and f.scene_current and not self.failure
        return f.scene_current and not self.failure and 0<=self.clock()-self.camera_time<=.2

    def _snapshot_feedback(self,now):
        if not self.joints or not self.validation or not self.validation['unchanged']:
            raise ValueError('waiting for measured scene admission')
        self._require_platform_fresh(now)
        if not 0<=now-self.feedback_time<=.2:raise ValueError('joint sample stale')
        if not self.admitted and (not 0<=now-self.scene_time<=.2 or self.last_pair_stamp<0):
            raise ValueError('scene admission stale')
        p=self.program;j=self.joints[-1]
        return CarryFeedback(self.generation,self.feedback_time,0.,p.capture_id,p.calibration_id,dict(p.source_sha256),
            j.positions_rad,self.platform.current_height,self.platform.base_stopped,self.platform.lift_stopped,
            True,True,'unknown',-1,-1.,True,True,p.capture_id,p.candidate_id,self.scene_time,
            observation_policy=self.observation_policy)

    def _require_platform_fresh(self,now):
        self.platform.require_fresh(now)

    def route_start_ready(self):
        """Reserve freshness through the idle-reader -> stream-reader handoff.

        The four platform slots are interleaved with arm reads (35 ms each,
        140 ms sweep). Admission near the 200 ms deadline can expire
        while the new reader starts. Wait for a completed recent sweep instead
        of extending the running 200 ms limit or resetting its timestamp.
        """
        with self.lock:
            now=self.clock()
            self.feedback()  # Keep real stale/missing/moved input failures.
            return (0<=now-self.platform.stamp<=.120 and
                    0<=now-self.feedback_time<=.100)


class ResidentObservationReader:
    """Single serial owner, acquiring cached MCU measurements at consumption.

    A heartbeat or status transaction can age earlier host-side observations.
    The supervisor calls read() after its maintenance I/O, before deciding on
    motion. This performs one bounded read per source, no wait/retry/rearm and
    no motor-bus reads. Actual stale encoders still fail inputs.feedback().
    """
    def __init__(self,transport,adapter,inputs,*,clock=time.monotonic):
        self.transport,self.adapter,self.inputs,self.clock=transport,adapter,inputs,clock
        self.deferred=[]

    def refresh(self):
        self.adapter.keepalive()
        for query,update in ((self.transport.get_feedback_snapshot,self.inputs.add_joints),
                             (self.transport.stationary_platform,self.inputs.add_platform)):
            sent=self.clock();packet=query();received=self.clock()
            try:update(packet,sent,received)
            except ObservationNotFresh as exc:
                self.deferred.append(str(exc))
                del self.deferred[:-20]
                # Never change a timestamp on rejection. Preserve the most
                # recent rejected packets rather than only startup failures.

    def read(self):
        self.refresh()
        return self.inputs.feedback()


class CameraWorker:
    def __init__(self,capture_factory,consume):
        self.factory,self.consume=capture_factory,consume
        self.stop=threading.Event();self.ready=threading.Event();self.error=None;self.frames=0
        self.thread=threading.Thread(target=self._run,name='stationary-rgbd',daemon=True)
    def _run(self):
        try:
            with self.factory() as camera:
                self.ready.set()
                while not self.stop.is_set():
                    record,images=camera.next_frame();self.consume(record,images);self.frames+=1
        except BaseException as exc:self.error=exc;self.ready.set()
    def __enter__(self):
        self.thread.start()
        if not self.ready.wait(12):raise TimeoutError('D415 startup timeout')
        if self.error:raise self.error
        return self
    def check(self):
        if self.error:raise RuntimeError('camera worker: '+str(self.error))
    def __exit__(self,*_):
        self.stop.set();self.thread.join(3)
        if self.thread.is_alive():raise RuntimeError('camera worker did not stop')
