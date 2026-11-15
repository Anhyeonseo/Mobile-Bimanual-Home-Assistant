"""Supervised, short saved-map goal trial using existing geometry and clients.

This commissioning controller measures goal progress with LiDAR registration.
Raw output is bounded; it is not a calibrated SI-speed or full Nav2 acceptance.
Construction, preview and readiness never authorize wheel output.
"""
import math
import hashlib
import numpy as np
from .base_motion import OmniKinematics, Wheel
from .fetch import InvalidTask


def angle(a):
    return (a+math.pi)%(2*math.pi)-math.pi


class NearGoal:
    WAITING = ('WAITING_SENSOR', 'WAITING_OBSTACLE', 'WAITING_BROWSER')
    def __init__(self, geometry, area, *, distance=.2, raw_limit=50, forward_pattern=None, navigation_body=None):
        if not 0.1 <= distance <= .2 or type(raw_limit) is not int or not 1 <= raw_limit <= 50:
            raise InvalidTask('bounded supervised trial required')
        self.geometry=OmniKinematics(tuple(Wheel(**{k:w[k] for k in
            ('x_m','y_m','rolling_angle_rad','radius_m')}) for w in geometry['wheels']))
        self.area=area;self.distance=distance;self.limit=raw_limit
        self.navigation_body=navigation_body
        if forward_pattern is not None:
            if tuple(forward_pattern)!=(-200,200,0) or geometry.get('wheel_order')!=[7,8,9]:
                raise InvalidTask('reviewed teleop forward pattern and wheel IDs required')
            forward_pattern=tuple(forward_pattern)
        self.forward_pattern=forward_pattern
        self.departure_since=None;self.departure_stamp=None;self.departure_samples=0
        self.start_pose=self.goal=None;self.state='PREVIEW';self.started=None
        self.last_pose_stamp=None;self.arrived=0;self.bad_since=None
        self.resume_required=False;self.recovery_since=None;self.recovery_stamp=None
        self.recovery_samples=0;self.recovery_ready=False
        self.active_timeout_s=90

    def map_free(self, pose):
        a=self.area;r=float(a['resolution_m']);x,y=np.floor((np.asarray(pose[:2])-a['origin_xy_m'])/r).astype(int)
        grid=a['feasible_by_yaw'];yaw=int(round((pose[2]%(2*math.pi))/(2*math.pi/grid.shape[0])))%grid.shape[0]
        return 0<=x<grid.shape[2] and 0<=y<grid.shape[1] and bool(grid[yaw,y,x])

    def path(self, start, goal):
        length=math.dist(start[:2],goal[:2]);turn=angle(goal[2]-start[2])
        steps=max(1,math.ceil(length/.01),math.ceil(abs(turn)/math.radians(3)))
        return [tuple(start[i]+(turn if i==2 else goal[i]-start[i])*t/steps for i in range(3)) for t in range(steps+1)]

    def plan(self, pose):
        if self.state not in ('PREVIEW','READY'):raise InvalidTask('trial already consumed')
        if len(pose)!=3 or not all(math.isfinite(x) for x in pose):raise InvalidTask('invalid localization')
        goal=(pose[0]+self.distance*math.cos(pose[2]),pose[1]+self.distance*math.sin(pose[2]),pose[2])
        if not all(self.map_free(p) for p in self.path(pose,goal)):
            raise InvalidTask('saved RGB-D/body map blocks this path')
        self.start_pose=tuple(pose);self.goal=goal;self.state='READY'
        return self.path(pose,goal)

    def start(self, now, pose, *, operator_confirmed):
        if self.state!='READY' or operator_confirmed is not True:raise InvalidTask('explicit start required')
        if math.dist(pose[:2],self.start_pose[:2])>.05 or abs(angle(pose[2]-self.start_pose[2]))>.15:
            raise InvalidTask('preview pose changed; refresh preview')
        if self.forward_pattern is not None:
            # The operator requested body-forward travel, not a correction to
            # an older noisy preview. Recheck the newly anchored 20 cm corridor.
            goal=(pose[0]+self.distance*math.cos(pose[2]),pose[1]+self.distance*math.sin(pose[2]),pose[2])
            if not all(self.map_free(p) for p in self.path(pose,goal)):
                raise InvalidTask('current forward corridor is blocked; refresh preview')
            self.start_pose=tuple(pose);self.goal=goal
        self.state='RUNNING';self.started=now

    def pause(self, now, reason):
        if self.state not in ('RUNNING', *self.WAITING):return
        if self.state=='RUNNING':self.bad_since=now
        if reason not in self.WAITING:raise InvalidTask('invalid pause reason')
        if reason=='WAITING_BROWSER':self.resume_required=True
        if self.state!=reason:
            self.recovery_since=None;self.recovery_stamp=None;self.recovery_samples=0
            self.recovery_ready=False
        self.state=reason;self.arrived=0

    def route_valid(self, pose):
        return (pose is not None and len(pose)==3 and all(math.isfinite(x) for x in pose)
                and math.dist(pose[:2],self.start_pose[:2])<=self.distance+.10
                and all(self.map_free(p) for p in self.path(pose,self.goal)))

    def resume(self, now, pose, pose_stamp, *, operator_confirmed, sensors_fresh, corridor_clear):
        if (self.state not in self.WAITING or operator_confirmed is not True
                or not self.recovery_ready or not sensors_fresh or not corridor_clear
                or not 0<=now-pose_stamp<=.5 or not self.route_valid(pose)):
            raise InvalidTask('새 관측과 경로 확인 후 이어하기를 눌러 주세요')
        # Paused time does not consume the bounded active travel budget.
        self.started+=now-self.bad_since
        self.state='RUNNING';self.bad_since=None;self.resume_required=False
        self.recovery_ready=False;self.recovery_since=None;self.recovery_samples=0
        self.recovery_stamp=None;self.arrived=0;self.last_pose_stamp=None

    def admit_command(self, now, pose, pose_stamp, *, sensors_fresh, corridor_clear):
        if self.state not in ('RUNNING', *self.WAITING):return False
        if self.state=='RUNNING' and now-self.started>self.active_timeout_s:raise InvalidTask('goal trial timeout')
        if not sensors_fresh or not 0<=now-pose_stamp<=.5:
            self.pause(now,'WAITING_SENSOR')
        elif not corridor_clear:
            self.pause(now,'WAITING_OBSTACLE')
        if self.state in self.WAITING:
            if now-self.bad_since>=2:self.resume_required=True
            good=sensors_fresh and 0<=now-pose_stamp<=.5 and corridor_clear and self.route_valid(pose)
            if not good:
                self.recovery_since=None;self.recovery_samples=0;self.recovery_stamp=None
                self.recovery_ready=False
            else:
                if self.recovery_since is None:self.recovery_since=now
                if pose_stamp!=self.recovery_stamp:
                    self.recovery_samples+=1;self.recovery_stamp=pose_stamp
                self.recovery_ready=now-self.recovery_since>=.3 and self.recovery_samples>=3
                if self.recovery_ready and not self.resume_required:
                    self.resume(now,pose,pose_stamp,operator_confirmed=True,
                                sensors_fresh=True,corridor_clear=True)
            # Recovery itself never replays the pre-pause command.
            return False
        if not self.route_valid(pose):raise InvalidTask('current path outside reviewed map or trial envelope')
        return True

    def command(self, now, pose, pose_stamp, *, sensors_fresh, corridor_clear):
        if not self.admit_command(now,pose,pose_stamp,sensors_fresh=sensors_fresh,corridor_clear=corridor_clear):
            return (0,0,0,0)
        if self.forward_pattern is not None:
            c,s=math.cos(self.start_pose[2]),math.sin(self.start_pose[2])
            px,py=pose[0]-self.start_pose[0],pose[1]-self.start_pose[1]
            lateral=-s*px+c*py
            departed=abs(lateral)>.06 or abs(angle(pose[2]-self.start_pose[2]))>math.radians(15) or c*px+s*py<-.04
            if departed:
                if self.departure_since is None:self.departure_since=now
                if pose_stamp!=self.departure_stamp:self.departure_samples+=1;self.departure_stamp=pose_stamp
                if self.departure_samples>=3 and now-self.departure_since>=.3:
                    raise InvalidTask('전진 경로 이탈 · 바퀴 방향·위치 관측 확인 필요')
                return (0,0,0,0)
            self.departure_since=None;self.departure_stamp=None;self.departure_samples=0
        dx,dy=self.goal[0]-pose[0],self.goal[1]-pose[1];yaw_error=angle(self.goal[2]-pose[2])
        if math.hypot(dx,dy)<=.04 and abs(yaw_error)<=math.radians(8):
            if pose_stamp!=self.last_pose_stamp:self.arrived+=1
            self.last_pose_stamp=pose_stamp
            if self.arrived>=3:self.state='SETTLING'
            return (0,0,0,0)
        self.arrived=0;self.last_pose_stamp=pose_stamp
        if self.forward_pattern is not None:
            if c*px+s*py>=self.distance:
                raise InvalidTask('전진 거리 도달 · 목표 오차 확인 필요')
            peak=max(abs(v) for v in self.forward_pattern)
            # Preserve the proven direction ratio; do not mix tiny lateral/yaw
            # corrections or infer calibrated m/s from these raw values.
            return tuple(round(v/peak*self.limit) for v in self.forward_pattern)+(0,)
        c,s=math.cos(pose[2]),math.sin(pose[2]);rates=self.geometry.wheel_rates((c*dx+s*dy,-s*dx+c*dy,.25*yaw_error))
        peak=max(abs(v) for v in rates)
        if peak<1e-9:return (0,0,0,0)
        amplitude=self.limit*min(1.,max(.3,math.hypot(dx,dy)/.12))
        return tuple(round(v/peak*amplitude) for v in rates)+(0,)

    def finish(self, stop_confirmed):
        if self.state!='SETTLING' or stop_confirmed is not True:raise InvalidTask('measured stop required')
        self.state='SUCCEEDED'

    def cancel(self):
        if self.state not in ('SUCCEEDED','FAILED','CANCELLED'):self.state='CANCELLED'



class TurnDriveTrial(NearGoal):
    """Short map-checked turn then drive using the tested teleop wheel patterns.

    No simultaneous yaw/translation mixer. Live position closes each phase;
    obstacles retain the goal and use the same zero-output/resume state machine.
    """
    allow_blocked_start=True
    def __init__(self,geometry,area,*,patterns,distance=.4,turn_degrees=-30,raw_limit=75,navigation_body=None):
        if not .2<=distance<=.6 or not 15<=abs(turn_degrees)<=45 or not 50<=raw_limit<=100:
            raise InvalidTask('short turn/drive route required')
        if (tuple(patterns['forward'][1])!=(-200,200,0) or
            tuple(patterns['rotate-left'][1])!=(200,200,200) or
            tuple(patterns['rotate-right'][1])!=(-200,-200,-200) or geometry.get('wheel_order')!=[7,8,9]):
            raise InvalidTask('reviewed three-wheel teleop patterns required')
        super().__init__(geometry,area,navigation_body=navigation_body)
        self.distance=distance;self.limit=raw_limit;self.turn=math.radians(turn_degrees)
        self.patterns=patterns;self.phase='TURNING';self.active_timeout_s=180
        self.aligned=0;self.aligned_stamp=None;self.transition_until=0

    def path(self,start,goal):
        heading=math.atan2(goal[1]-start[1],goal[0]-start[0]) if math.dist(start[:2],goal[:2])>.04 else goal[2]
        pivot=(*start[:2],heading);end=(*goal[:2],heading)
        return NearGoal.path(self,start,pivot)+NearGoal.path(self,pivot,end)[1:]

    def plan(self,pose):
        if self.state not in ('PREVIEW','READY'):raise InvalidTask('trial already consumed')
        if len(pose)!=3 or not all(math.isfinite(x) for x in pose):raise InvalidTask('invalid localization')
        heading=angle(pose[2]+self.turn)
        goal=(pose[0]+self.distance*math.cos(heading),pose[1]+self.distance*math.sin(heading),heading)
        path=self.path(pose,goal)
        if not all(self.map_free(p) for p in path):raise InvalidTask('회전·이동 경로에 지도 장애물이 있습니다')
        self.start_pose=tuple(pose);self.goal=goal;self.state='READY';return path

    def start(self,now,pose,*,operator_confirmed):
        super().start(now,pose,operator_confirmed=operator_confirmed)
        self.phase='TURNING';self.aligned=0;self.aligned_stamp=None

    def route_valid(self,pose):
        return (pose is not None and len(pose)==3 and all(math.isfinite(x) for x in pose)
            and math.dist(pose[:2],self.start_pose[:2])<=self.distance+.15
            and all(self.map_free(p) for p in self.path(pose,self.goal)))

    def command(self,now,pose,pose_stamp,*,sensors_fresh,corridor_clear):
        if not self.admit_command(now,pose,pose_stamp,sensors_fresh=sensors_fresh,corridor_clear=corridor_clear):
            self.aligned=0;self.aligned_stamp=None
            return (0,0,0,0)
        distance=math.dist(pose[:2],self.goal[:2])
        if distance<=.04:
            error=angle(self.goal[2]-pose[2])
            if abs(error)<=math.radians(8):
                if pose_stamp!=self.last_pose_stamp:self.arrived+=1
                self.last_pose_stamp=pose_stamp
                if self.arrived>=3:self.state='SETTLING'
                return (0,0,0,0)
            self.arrived=0;self.phase='FINAL_TURNING'
            pattern=self.patterns['rotate-left' if error>0 else 'rotate-right'][1]
            return tuple(round(v/max(map(abs,pattern))*50) for v in pattern)+(0,)
        if self.phase=='FINAL_TURNING':self.phase='TURNING';self.aligned=0
        self.arrived=0;self.last_pose_stamp=pose_stamp
        heading=math.atan2(self.goal[1]-pose[1],self.goal[0]-pose[0]);error=angle(heading-pose[2])
        if self.phase=='DRIVING' and abs(error)>math.radians(12):
            self.phase='TURNING';self.aligned=0;self.aligned_stamp=None
            self.transition_until=now+.3;return (0,0,0,0)
        if now<self.transition_until:return (0,0,0,0)
        if self.phase=='TURNING':
            if abs(error)<=math.radians(5):
                if pose_stamp!=self.aligned_stamp:self.aligned+=1
                self.aligned_stamp=pose_stamp
                if self.aligned>=3:self.phase='DRIVING';self.transition_until=now+.3
                return (0,0,0,0)
            self.aligned=0;self.aligned_stamp=None
            key='rotate-left' if error>0 else 'rotate-right';limit=50
        else:key='forward';limit=self.limit
        pattern=self.patterns[key][1];peak=max(map(abs,pattern))
        return tuple(round(v/peak*limit) for v in pattern)+(0,)


def swept_path_obstacle(points_base,path_base,*,footprint,margin=.03,return_hits=False):
    """Full fixed body swept through sampled translations AND rotations."""
    p=np.asarray(points_base,dtype=float)
    if p.size==0:return [] if return_hits else False
    p=p.reshape(-1,p.shape[-1]);xy=p[:,:2]
    if not np.isfinite(xy).all():
        if return_hits:raise ValueError('nonfinite obstacle observation')
        return True
    poly=np.asarray(footprint,float);edges=np.roll(poly,-1,axis=0)-poly
    normals=np.column_stack((edges[:,1],-edges[:,0]))/np.linalg.norm(edges,axis=1)[:,None]
    bounds=np.sum(normals*poly,axis=1)
    p=p[~np.all(xy@normals.T<=bounds+1e-12,axis=1)]
    inflated=[]
    for i in range(len(poly)):
        pair=np.stack((normals[i-1],normals[i]))
        if abs(np.linalg.det(pair))>1e-12:
            inflated.append(np.linalg.solve(pair,[bounds[i-1]+margin,bounds[i]+margin]))
    radius=max(float(np.max(np.linalg.norm(poly,axis=1))),max((float(np.linalg.norm(v)) for v in inflated),default=math.inf))
    for x,y,yaw in path_base:
        c,s=math.cos(yaw),math.sin(yaw)
        delta=p[:,:2]-[x,y]
        near=np.sum(delta*delta,axis=1)<=(radius+1e-12)**2
        if not np.any(near):continue
        q=delta[near]@np.array([[c,-s],[s,c]])
        hits=np.all(q@normals.T<=bounds+margin+1e-12,axis=1)
        if np.any(hits):return p[near][hits][:8].tolist() if return_hits else True
    return [] if return_hits else False

def validate_navigation_body(body, area, map_path):
    """Reject a stale clearance artifact before opening any hardware connection."""
    poly=np.asarray(body['footprint_xy_m'],float)
    if (body.get('mode')!='fixed_lift_and_mount_projection' or body.get('articulated_arms_included') is not False
            or body.get('lift_position_m')!=0 or poly.ndim!=2 or poly.shape[1]!=2 or len(poly)<3
            or not np.isfinite(poly).all()):
        raise InvalidTask('fixed lift/mount geometry required')
    edges=np.roll(poly,-1,axis=0)-poly
    cross=edges[:,0]*np.roll(edges,-1,axis=0)[:,1]-edges[:,1]*np.roll(edges,-1,axis=0)[:,0]
    if np.any(cross < -1e-10) or not np.any(cross>0) or np.any(np.linalg.norm(edges,axis=1)<1e-10):
        raise InvalidTask('counterclockwise convex body required')
    expected=np.asarray(area.get('navigation_footprint_xy_m',[]))
    if expected.shape!=poly.shape or not np.allclose(expected,poly,atol=1e-9,rtol=0):
        raise InvalidTask('rebuild movement area with this fixed lift footprint')
    clearance=float(body['clearance_m'])
    if not math.isfinite(clearance) or clearance<0 or abs(float(area.get('clearance_m',-1))-clearance)>1e-9:
        raise InvalidTask('map/live body clearance mismatch')
    if str(area.get('source_map_sha256',''))!=hashlib.sha256(map_path.read_bytes()).hexdigest():
        raise InvalidTask('movement area belongs to a different occupancy map')
    return body


def swept_obstacle(points_base, delta_base, *, body=(-.1541,.2003,-.2134,.2115), margin=.03, footprint=None, return_hits=False):
    """Check translated fixed body, with normalized edge distances for clearance.

    Initial body interior is excluded as self; no unseen voxel becomes free.
    Operator-cleared short corridor is required separately for this trial only.
    """
    p=np.asarray(points_base,dtype=float)
    if p.size==0:return [] if return_hits else False
    p=p.reshape(-1,p.shape[-1]);xy=p[:,:2]
    if not np.isfinite(xy).all():
        if return_hits:raise ValueError('nonfinite obstacle observation')
        return True
    if footprint is None:
        x0,x1,y0,y1=body
        poly=np.array([[x0,y0],[x1,y0],[x1,y1],[x0,y1]])
    else:poly=np.asarray(footprint,float)
    edges=np.roll(poly,-1,axis=0)-poly
    normals=np.column_stack((edges[:,1],-edges[:,0]))/np.linalg.norm(edges,axis=1)[:,None]
    bounds=np.sum(normals*poly,axis=1)
    p=p[~np.all(xy@normals.T<=bounds+1e-12,axis=1)];xy=p[:,:2]
    for t in np.linspace(0,1,max(2,math.ceil(math.hypot(*delta_base)/.01)+1)):
        q=xy-np.asarray(delta_base)*t
        hits=np.all(q@normals.T<=bounds+margin+1e-12,axis=1)
        if np.any(hits):return p[hits][:8].tolist() if return_hits else True
    return [] if return_hits else False


def voxel_centers(points, resolution=.02):
    """Deduplicate bounded local voxels without sorting full point rows."""
    points=np.asarray(points,dtype=float).reshape(-1,3)
    if not len(points):return np.empty((0,3))
    cells=np.floor(points/resolution).astype(np.int64)
    low=cells.min(axis=0);shape=cells.max(axis=0)-low+1
    if math.prod(shape)>1_000_000:raise ValueError('local depth extent exceeded')
    ids=np.ravel_multi_index((cells-low).T,tuple(shape))
    occupied=np.flatnonzero(np.bincount(ids))
    return (np.column_stack(np.unravel_index(occupied,tuple(shape)))+low+.5)*resolution
