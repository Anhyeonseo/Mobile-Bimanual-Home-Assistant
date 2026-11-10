"""Bounded D415 timestamp association for the read-only S2 observer.

SDK global_time is a host-clock conversion, not hardware synchronization proof:
https://realsenseai.github.io/librealsense/python_docs/_generated/pyrealsense2.timestamp_domain.html
"""
from datetime import datetime, timezone
import time
from . import s2_geometry as s2
from .camera_evidence import from_metadata, match_observed_identity


class GlobalClock:
    def __init__(self,*,wall=time.time,mono=time.monotonic):
        self.wall,self.mono=wall,mono
        self.offset=self._offset()
    def _offset(self):
        a=self.mono();w=self.wall();b=self.mono()
        if b-a>.002:raise ValueError('clock sampling delayed')
        return (a+b)/2-w
    def convert(self,timestamp_ms,domain):
        if domain!='timestamp_domain.global_time':raise ValueError('SDK global_time required; arrival time is not acquisition time')
        if abs(self._offset()-self.offset)>.005:raise ValueError('host wall clock stepped; restart capture')
        t=s2.number(timestamp_ms,'RGB-D timestamp',1e15)/1000+self.offset
        if not 0<=self.mono()-t<=.2:raise ValueError('stale/future camera acquisition')
        return t


def association(record,camera):
    stamps=[s2.number(camera[k],k,1e12) for k in ('color_time_s','depth_time_s')]
    for r in record['samples']:
        s2.number(r['acquired_s'],'motor time',1e12)
        if not 0<=s2.number(r['time_uncertainty_s'],'time uncertainty',.02):raise ValueError('invalid time uncertainty')
    lo=min(stamps+[r['acquired_s']-r['time_uncertainty_s'] for r in record['samples']])
    hi=max(stamps+[r['acquired_s']+r['time_uncertainty_s'] for r in record['samples']])
    span=hi-lo
    if span>.05:raise ValueError(f'RGB-D/motor association exceeds 50 ms: {span*1000:.3f} ms')
    return {'acquisition_span_s':span,'camera_timestamp_basis':'SDK global_time converted to host monotonic',
            'physical_hardware_sync_accuracy_verified':False}


class Capture:
    """Owns only camera streaming/options; restores changed options on exit."""
    def __init__(self,profile,reference_evidence=None,*,retain_frame=False):
        self.expected=profile['camera_device'];self.pipeline=None;self.started=False;self.saved=[];self.buffer=[]
        self.reference=reference_evidence
        self.last_matched=None
        self.retain_frame=retain_frame
        self.images={}
        if self.reference is not None and self.reference['camera_device']!=self.expected:
            raise ValueError('camera reference does not match profile')
    def __enter__(self):
        import pyrealsense2 as rs
        self.rs=rs
        try:
            ctx=rs.context()
            devices=[d for d in ctx.query_devices() if 'D415' in d.get_info(rs.camera_info.name)]
            if len(devices)!=1:raise ValueError('exactly one D415 required')
            d=devices[0]
            self.device={k:d.get_info(v) for k,v in [('name',rs.camera_info.name),('serial',rs.camera_info.serial_number),('firmware',rs.camera_info.firmware_version),('usb',rs.camera_info.usb_type_descriptor)]}
            for sensor in d.query_sensors():
                if sensor.supports(rs.option.global_time_enabled):
                    self.saved.append((sensor,sensor.get_option(rs.option.global_time_enabled)))
                    sensor.set_option(rs.option.global_time_enabled,1.)
            config=rs.config();config.enable_device(self.device['serial'])
            config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
            dims = (1280,720) if self.reference is None else (
                self.reference['profiles']['depth']['width'], self.reference['profiles']['depth']['height'])
            if dims not in ((1280,720), (640,360)):
                raise ValueError('unsupported D415 measurement profile')
            config.enable_stream(rs.stream.depth,*dims,rs.format.z16,30)
            self.pipeline=rs.pipeline(ctx);self.active=self.pipeline.start(config);self.started=True
            self.align=rs.align(rs.stream.color);self.clock=GlobalClock()
            # SDK device->OS conversion needs warmup. No motor reference exists yet.
            valid=0
            for _ in range(90):
                frames=self.pipeline.wait_for_frames(1000)
                if all(str(f.get_frame_timestamp_domain())=='timestamp_domain.global_time' for f in (frames.get_color_frame(),frames.get_depth_frame())):valid+=1
            if valid<30:raise ValueError('camera global timestamp conversion did not settle')
            self._record(frames)
            return self
        except BaseException:
            self.__exit__(None,None,None)
            raise
    def __exit__(self,exc_type,*_):
        errors=[]
        if self.started:
            try:self.pipeline.stop()
            except RuntimeError as e:errors.append(str(e))
        for sensor,value in reversed(self.saved):
            try:sensor.set_option(self.rs.option.global_time_enabled,value)
            except RuntimeError as e:errors.append(str(e))
        self.saved=[];self.pipeline=None;self.started=False
        if errors and exc_type is None:raise RuntimeError('camera cleanup: '+'; '.join(errors))
    @staticmethod
    def _intr(i):
        return {k:getattr(i,k) for k in ('width','height','fx','fy','ppx','ppy','coeffs')}|{'model':str(i.model)}
    def _record(self,frames):
        rs=self.rs;depth=frames.get_depth_frame();color=frames.get_color_frame()
        aligned=self.align.process(frames).get_depth_frame()
        native=depth.profile.as_video_stream_profile();cp=color.profile.as_video_stream_profile()
        extr=native.get_extrinsics_to(cp)
        frame=lambda f:{'number':f.get_frame_number(),'timestamp_ms':f.get_timestamp(),'timestamp_domain':str(f.get_frame_timestamp_domain())}
        m={'device':self.device,'coordinate_frame':'color_camera_optical_frame','measurement_depth_file':'depth_aligned.png',
           'alignment':'depth to color via rs.align','captured_at_utc':datetime.now(timezone.utc).isoformat(),
           'color_intrinsics':self._intr(cp.get_intrinsics()),'native_depth_intrinsics':self._intr(native.get_intrinsics()),
           'aligned_depth_intrinsics':self._intr(aligned.profile.as_video_stream_profile().get_intrinsics()),
           'color_capture':{'width':cp.width(),'height':cp.height(),'fps':cp.fps(),'format':'bgr8'},
           'depth_capture':{'width':native.width(),'height':native.height(),'fps':native.fps(),'format':'z16'},
           'depth_scale_m':self.active.get_device().first_depth_sensor().get_depth_scale(),
           'depth_to_color':{'rotation_column_major':extr.rotation,'translation_m':extr.translation},
           'color_frame':frame(color),'depth_frame':frame(depth)}
        checked=from_metadata(m)
        equivalence=None
        if self.reference is not None:equivalence=match_observed_identity(checked,self.reference)
        elif checked['camera_device']!=self.expected:raise ValueError('D415 device/stream/calibration changed')
        record={'camera_device':self.expected,'observed_camera_device':checked['camera_device'],
                'calibration_equivalence':equivalence,'source_frame':'color_camera_optical_frame',
                'capture_id':'rgbd-'+str(color.get_frame_number()),'color_frame':m['color_frame'],'depth_frame':m['depth_frame'],
                'color_time_s':self.clock.convert(color.get_timestamp(),str(color.get_frame_timestamp_domain())),
                'depth_time_s':self.clock.convert(depth.get_timestamp(),str(depth.get_frame_timestamp_domain()))}
        self.buffer.append(record);self.buffer=self.buffer[-3:]
        if self.retain_frame:
            import numpy as np
            self.images[record['capture_id']]=(np.asanyarray(color.get_data()).copy(),
                                               np.asanyarray(aligned.get_data()).copy(),m)
            self.images={c['capture_id']:self.images[c['capture_id']] for c in self.buffer if c['capture_id'] in self.images}
        return record
    def retain_matched(self, camera):
        images=self.images[camera['capture_id']]
        self.retain_frame=False
        self.images.clear()
        return images
    def next_frame(self, timeout_ms=1000):
        """Continuous consumer API; exposure time stays separate from arrival."""
        if not self.started or not self.retain_frame:
            raise ValueError('retaining camera stream required')
        record=self._record(self.pipeline.wait_for_frames(timeout_ms))
        return record, self.images[record['capture_id']]
    def match(self,motors):
        center=sum(r['acquired_s'] for r in motors['samples'])/len(motors['samples'])
        deadline=time.monotonic()+.1
        last_error=None
        # USB delivery is later than exposure. The first available frameset
        # can predate the motor scan; consume until a matching exposure arrives.
        # Never relabel delivery time as exposure or widen the 50 ms bound.
        while True:
            candidates=sorted((c for c in self.buffer if c['capture_id']!=self.last_matched),
                              key=lambda c:abs(c['color_time_s']-center))
            for candidate in candidates:
                try:checked=association(motors,candidate)
                except ValueError as e:
                    last_error=e;continue
                self.last_matched=candidate['capture_id']
                return {**candidate,**checked}
            remaining=deadline-time.monotonic()
            if remaining<=0:
                raise ValueError('no matching RGB-D exposure within 100 ms: '+str(last_error))
            self._record(self.pipeline.wait_for_frames(max(1,int(remaining*1000))))
