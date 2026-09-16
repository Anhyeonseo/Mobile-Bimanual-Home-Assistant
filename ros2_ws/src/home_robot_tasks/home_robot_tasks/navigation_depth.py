"""Bounded, timestamped depth occupancy for the fixed travel envelope.

Uses native rectified depth (not RGB-aligned/cropped depth). Unknown voxels are
not free. Obstacles persist until a later valid depth ray clears them; a planar
laser cannot clear upper-body voxels. This is a discretized perception model,
not proof that D415 detects every material or withstands chassis vibration.
"""
from dataclasses import dataclass
import itertools
import math
import numpy as np

from .fetch import InvalidTask
from .navigation import number


@dataclass(frozen=True)
class DepthProfile:
    frame: str
    calibration: str
    resolution_m: float
    minimum_depth_m: float
    maximum_depth_m: float
    minimum_height_m: float
    maximum_height_m: float
    maximum_age_s: float
    minimum_valid_fraction: float
    maximum_voxels: int = 150000
    maximum_ray_steps: int = 500000
    world_frame: str = 'odom'

    def __post_init__(self):
        values = (self.resolution_m, self.minimum_depth_m, self.maximum_depth_m,
                  self.minimum_height_m, self.maximum_height_m, self.maximum_age_s,
                  self.minimum_valid_fraction)
        for v in values:
            number(v, 'depth profile')
        if (not isinstance(self.world_frame,str) or not self.world_frame
                or not self.frame or not self.calibration or not .01 <= self.resolution_m <= .25
                or not 0 < self.minimum_depth_m < self.maximum_depth_m <= 10
                or not 0 <= self.minimum_height_m < self.maximum_height_m <= 3
                or not 0 < self.maximum_age_s <= 1
                or not 0 < self.minimum_valid_fraction <= 1
                or type(self.maximum_voxels) is not int or not 100 <= self.maximum_voxels <= 500000):
            raise InvalidTask('invalid depth profile')
        if type(self.maximum_ray_steps) is not int or not 100 <= self.maximum_ray_steps <= 2000000:
            raise InvalidTask('invalid depth work budget')


class DepthVolume:
    def __init__(self, profile):
        self.p = profile
        self.cells = {}  # cell -> (occupied, original observation time)
        self.last_stamp = None
        self.epoch = None
        self.error = 'no depth observation'

    def reset(self, epoch):
        if not isinstance(epoch, str) or not epoch or epoch==self.epoch:
            raise InvalidTask('navigation epoch required')
        self.epoch = epoch
        self.cells.clear()
        self.last_stamp = None
        self.error = 'awaiting new travel observation'

    def key(self, point):
        return tuple(math.floor(float(v) / self.p.resolution_m) for v in point)

    def _ray(self, start, end):
        """Voxel traversal, excluding endpoint. Ties advance all crossed axes."""
        r = self.p.resolution_m
        a,b=tuple(start),tuple(end)
        cell=[math.floor(v/r) for v in a]
        target=tuple(math.floor(v/r) for v in b)
        delta=[b[i]-a[i] for i in range(3)]
        step=[1 if v>0 else -1 if v<0 else 0 for v in delta]
        tmax,tdelta=[math.inf]*3,[math.inf]*3
        for i in range(3):
            if step[i]:
                edge=(cell[i]+(step[i]>0))*r
                tmax[i]=(edge-a[i])/delta[i]
                tdelta[i]=r/abs(delta[i])
        for _ in range(sum(abs(target[i]-cell[i]) for i in range(3))+1):
            if tuple(cell)==target:return
            yield tuple(cell)
            crossing=min(tmax)+1e-12
            for i in range(3):
                if tmax[i]<=crossing:
                    cell[i]+=step[i]
                    tmax[i]+=tdelta[i]

    def integrate(self, depth_m, intrinsics, camera_to_world, *, observed_s,
                  now_s, frame, calibration, epoch, motion_qualified):
        try:
            p = self.p
            stamp, now = number(observed_s, 'depth time'), number(now_s, 'clock')
            if (frame != p.frame or calibration != p.calibration or epoch != self.epoch
                    or type(motion_qualified) is not bool or not motion_qualified
                    or not 0 <= now-stamp <= p.maximum_age_s
                    or (self.last_stamp is not None and stamp <= self.last_stamp)):
                raise InvalidTask('depth identity, age, motion quality or sequence invalid')
            depth = np.asarray(depth_m, dtype=float)
            transform = np.asarray(camera_to_world, dtype=float)
            if (depth.ndim != 2 or not 1 <= depth.size <= 1280*720
                    or transform.shape != (4,4) or not np.isfinite(transform).all()
                    or not np.allclose(transform[3], [0,0,0,1])
                    or not np.allclose(transform[:3,:3].T@transform[:3,:3], np.eye(3), atol=1e-5)
                    or not np.isclose(np.linalg.det(transform[:3,:3]), 1, atol=1e-5)):
                raise InvalidTask('invalid depth image or rigid exposure transform')
            fx, fy, cx, cy = (number(v, 'intrinsics') for v in intrinsics)
            if fx <= 0 or fy <= 0 or not 0 <= cx < depth.shape[1] or not 0 <= cy < depth.shape[0]:
                raise InvalidTask('invalid depth intrinsics')
            valid = np.isfinite(depth) & (depth >= p.minimum_depth_m) & (depth <= p.maximum_depth_m)
            if valid.mean() < p.minimum_valid_fraction:
                raise InvalidTask('insufficient valid depth')
            # All accepted pixels are used. Decimation, if needed, must preserve
            # nearest obstacles and match the supplied intrinsics upstream.
            vv, uu = np.nonzero(valid)
            z = depth[vv,uu]
            xyz = np.column_stack(((uu-cx)*z/fx, (vv-cy)*z/fy, z))
            points = xyz@transform[:3,:3].T + transform[:3,3]
            origin = transform[:3,3]
            hits, cleared = set(), set()
            low = math.floor(p.minimum_height_m/p.resolution_m)
            high = math.ceil(p.maximum_height_m/p.resolution_m)
            # Deduplicate endpoints before traversal, bounding CPU by voxel count.
            voxel_keys=np.floor(points/p.resolution_m).astype(np.int64)
            unique,indices=np.unique(voxel_keys,axis=0,return_index=True)
            if len(indices)>p.maximum_voxels:
                raise InvalidTask('depth observation exceeds voxel budget')
            endpoints={tuple(int(v) for v in key):points[i] for key,i in zip(unique,indices)}
            optical_forward=tuple(float(v) for v in transform[:3,2])
            origin_tuple=tuple(float(v) for v in origin)
            work = 0
            for key, point in endpoints.items():
                if low <= key[2] < high:
                    hits.add(key)
                for cell in self._ray(origin_tuple, tuple(float(v) for v in point)):
                    work += 1
                    if work > p.maximum_ray_steps:
                        raise InvalidTask('depth traversal work budget exceeded')
                    if low <= cell[2] < high:
                        # Never clear the stereo near blind region.
                        optical_z=sum(optical_forward[i]*((cell[i]+.5)*p.resolution_m-origin_tuple[i]) for i in range(3))
                        if optical_z >= p.minimum_depth_m + math.sqrt(3)*p.resolution_m:
                            cleared.add(cell)
                    if len(cleared) > p.maximum_voxels or len(hits) > p.maximum_voxels:
                        raise InvalidTask('depth ray budget exceeded')
            if len(self.cells.keys() | cleared | hits) > p.maximum_voxels:
                raise InvalidTask('depth map budget exceeded; explicit local reset required')
            # A hit wins over every clear ray within one observation.
            for cell in cleared-hits:
                self.cells[cell] = (False, stamp)
            for cell in hits:
                self.cells[cell] = (True, stamp)
            self.last_stamp, self.error = stamp, None
        except Exception as error:
            self.error = str(error)
            raise

    def fresh(self, now):
        return (self.error is None and self.last_stamp is not None
                and 0 <= now-self.last_stamp <= self.p.maximum_age_s)

    def projected_obstacles(self):
        return frozenset((x,y) for (x,y,z),(hit,stamp) in self.cells.items() if hit)

    def free(self, cell, now):
        value = self.cells.get(cell)
        return (value is not None and not value[0]
                and 0 <= now-value[1] <= self.p.maximum_age_s)


@dataclass(frozen=True)
class TravelEvidence:
    observed_s: float
    epoch: str
    ready: bool
    localized: bool
    envelope_clear: bool


class TravelGuard:
    """Conservative circular travel envelope; rejects unknown stopping corridor.

Initial envelope must be independently checked collision-free when entering
travel mode. A circle contains all yaw orientations, including arm/payload.
Movement uses the union of measured and proposed stopping corridors. No claim
of full-body shape optimisation or reliable detection of transparent objects.
"""
    def __init__(self, volume, *, radius_m, height_m, margin_m, reaction_s,
                 braking_m_s2, maximum_speed_m_s, maximum_yaw_rad_s):
        self.volume = volume
        for v in (radius_m,height_m,margin_m,reaction_s,braking_m_s2,
                  maximum_speed_m_s,maximum_yaw_rad_s):
            number(v,'travel bound')
        if (min(radius_m,height_m,reaction_s,braking_m_s2,maximum_speed_m_s,maximum_yaw_rad_s)<=0
                or margin_m < 0 or radius_m+margin_m > 2
                or height_m > volume.p.maximum_height_m):
            raise InvalidTask('invalid travel envelope')
        self.radius, self.height, self.margin = radius_m,height_m,margin_m
        self.reaction, self.braking = reaction_s,braking_m_s2
        self.speed, self.yaw = maximum_speed_m_s,maximum_yaw_rad_s

    def check(self, command, measured, xy_yaw, evidence, now):
        vectors = [tuple(number(v,'travel motion') for v in vec) for vec in (command,measured,xy_yaw)]
        if any(len(v)!=3 for v in vectors):
            raise InvalidTask('three travel components required')
        command,measured,pose = vectors
        if (not isinstance(evidence,TravelEvidence) or evidence.epoch != self.volume.epoch
                or any(type(v) is not bool or not v for v in
                       (evidence.ready,evidence.localized,evidence.envelope_clear))
                or not 0 <= now-number(evidence.observed_s,'travel time') <= self.volume.p.maximum_age_s
                or not self.volume.fresh(now)):
            raise InvalidTask('travel posture, localization or depth unavailable')
        if math.hypot(*command[:2]) > self.speed+1e-9 or abs(command[2]) > self.yaw+1e-9:
            raise InvalidTask('travel speed outside profile')
        r = self.volume.p.resolution_m
        radius = self.radius+self.margin+math.sqrt(2)*r
        origin = np.asarray(pose[:2])
        # Unknown space outside the initial, independently verified envelope
        # is checked across the full configured height, not just the camera ray.
        work = 0
        checked_columns = set()
        for twist in (command,measured):
            speed = math.hypot(*twist[:2])
            if speed <= 1e-9:
                continue
            age = now-self.volume.last_stamp
            duration = self.reaction+age+speed/self.braking
            # Straight measured/proposed directions plus a lateral bound for
            # any yaw rate within the configured/measured maximum during stop.
            # This also encloses transient curves when yaw braking is unknown.
            reach = speed*duration
            turn_bound = min(2*reach, .5*speed*max(self.yaw,abs(twist[2]))*duration**2)
            corridor_radius = radius+turn_bound
            steps = max(1, math.ceil(reach/(r/2)))
            if steps > 2000:
                raise InvalidTask('stopping corridor exceeds budget')
            for i in range(1,steps+1):
                t = duration*i/steps
                dx,dy = t*twist[0],t*twist[1]
                c,s = math.cos(pose[2]),math.sin(pose[2])
                centre = origin + (c*dx-s*dy,s*dx+c*dy)
                lo,hi = np.floor((centre-corridor_radius)/r).astype(int),np.ceil((centre+corridor_radius)/r).astype(int)
                for x,y in itertools.product(range(lo[0],hi[0]+1),range(lo[1],hi[1]+1)):
                    if (x,y) in checked_columns:continue
                    px,py=(x+.5)*r,(y+.5)*r
                    if math.hypot(px-centre[0],py-centre[1])>corridor_radius or math.hypot(px-origin[0],py-origin[1])<=self.radius:
                        continue
                    checked_columns.add((x,y))
                    for z in range(math.floor(self.volume.p.minimum_height_m/r),math.ceil(self.height/r)):
                        work += 1
                        if work > 100000:
                            raise InvalidTask('travel validation work budget exceeded')
                        if not self.volume.free((x,y,z),now):
                            raise InvalidTask('stopping corridor occupied or unobserved')
        return command
