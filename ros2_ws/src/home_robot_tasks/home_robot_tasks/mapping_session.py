"""Native-depth ingestion into the existing scan/map/semantic persistence path."""
from copy import deepcopy
import hashlib
from .assistant.scanning import ScanManager
from .navigation_depth import DepthVolume
from .map_bundle import export_scan,align_to_depth_lattice
from .fetch import InvalidTask


class MappingSession:
    def __init__(self,store,context,profile,envelope,*,session_id,name,map_document,mode='replay'):
        if profile.calibration!=context.calibration_id or profile.world_frame!=context.frame_id:
            raise InvalidTask('mapping profile/context mismatch')
        self.scan=ScanManager(store,context);self.volume=DepthVolume(profile);self.envelope=envelope
        self.session_id=session_id;self.frames=0
        self.scan.start(session_id,name,mode=mode)
        self.replace_map(map_document)

    def replace_map(self,map_document):
        source=deepcopy(map_document)
        r=self.volume.p.resolution_m
        if (abs(map_document['resolution_m']-r)>1e-9 or
                any(abs(map_document['origin'][k]/r-round(map_document['origin'][k]/r))>1e-6 for k in ('x','y'))):
            from .assistant.contracts import document
            revision='lattice-'+hashlib.sha256(document(dict(source=source,resolution=r)).encode()).hexdigest()[:24]
            map_document=align_to_depth_lattice(source,resolution_m=r,revision=revision)
        value=self.scan.ingest_map(self.session_id,map_document)
        if value.get('source_slam_map')!=source:
            value['source_slam_map']=source
            self.scan.store.save_record('scan',self.session_id,value)
        epoch=self.scan.depth_epoch(self.session_id)
        if epoch!=self.volume.epoch:
            self.volume.reset(epoch);self.frames=0
        return value

    def depth(self,image,intrinsics,camera_to_map,*,observed_s,now_s,frame,calibration_id,clock_id,motion_qualified):
        if clock_id!=self.scan.context.clock_id:raise InvalidTask('depth clock domain mismatch')
        self.volume.integrate(image,intrinsics,camera_to_map,observed_s=observed_s,now_s=now_s,
            frame=frame,calibration=calibration_id,epoch=self.volume.epoch,motion_qualified=motion_qualified)
        self.frames+=1

    def finish(self,*,now_s,output):
        if not self.frames:raise InvalidTask('native depth observations required')
        self.scan.build_navigation(self.session_id,self.volume,self.envelope,now_s=now_s)
        self.scan.review(self.session_id);self.scan.save(self.session_id)
        return export_scan(self.scan,self.session_id,output)

    def rgbd(self, mapper, rgb, aligned_depth, intrinsics, *, stamp, transform, xy_m, now_s):
        """Worker-side semantic annotation; native depth collision input is separate."""
        from .assistant.contracts import Context
        expected = Context(self.scan.context.map_id,
            self.scan.get(self.session_id)['map_document']['revision'],
            self.scan.context.calibration_id, self.scan.context.clock_id, self.scan.context.frame_id)
        expected.check(mapper.memory.context)
        if mapper.memory.store is not self.scan.store:
            raise InvalidTask('shared mapping store required')
        result = mapper.observe(rgb, aligned_depth, intrinsics, stamp=stamp,
                                transform=transform, now_s=now_s)
        self.scan.observe(self.session_id, dict(stamp=stamp.to_dict(), xy_m=list(xy_m),
            categories=result['provenance']['categories'], surface_ids=[]), now_s=now_s)
        return result
