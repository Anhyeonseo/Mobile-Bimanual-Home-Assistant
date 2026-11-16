"""Bounded holonomic SE(2) alignment with swept-body sampling.

The pose checker must include the base, folded/loaded body, RGB-D height
obstacles and unknown map cells. A clear centre point is not a valid checker.
"""
import heapq
import math
import numpy as np
from .assistant.placement import Check
from .fetch import InvalidTask


def wrap(a):return math.atan2(math.sin(a),math.cos(a))


def alignment_path(start,goal,pose_free,*,resolution_m=.05,yaw_bins=24,radius_m=2.,maximum_expansions=15000):
    start=np.asarray(start,float);goal=np.asarray(goal,float)
    if (start.shape!=(3,) or goal.shape!=(3,) or not np.isfinite([start,goal]).all()
            or not callable(pose_free) or not .02<=resolution_m<=.1 or not 8<=yaw_bins<=72
            or not .1<=radius_m<=3 or not 1<=maximum_expansions<=50000):
        raise InvalidTask('bounded local alignment search required')
    step=2*math.pi/yaw_bins
    def pose(k):return np.array([start[0]+k[0]*resolution_m,start[1]+k[1]*resolution_m,wrap(start[2]+k[2]*step)])
    cache={}
    def free(p):
        key=tuple(np.round(p,6))
        if key not in cache:cache[key]=pose_free(tuple(map(float,p))) is True
        return cache[key]
    def sweep(a,b):
        dyaw=wrap(b[2]-a[2]); n=max(1,math.ceil(np.linalg.norm(b[:2]-a[:2])/.01),math.ceil(abs(dyaw)/math.radians(3)))
        return all(free(np.r_[a[:2]+(b[:2]-a[:2])*u,wrap(a[2]+dyaw*u)]) for u in np.linspace(0,1,n+1))
    if np.linalg.norm(goal[:2]-start[:2])>radius_m or not free(start) or not free(goal):
        return None
    def heuristic(p):return float(np.linalg.norm(p[:2]-goal[:2]))+.15*abs(wrap(p[2]-goal[2]))
    root=(0,0,0);cost={root:0.};parent={};queue=[(heuristic(start),0.,root)]
    moves=[(x,y,0) for x,y in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1))]+[(0,0,1),(0,0,-1)]
    for _ in range(maximum_expansions):
        if not queue:break
        _,g,k=heapq.heappop(queue)
        if g!=cost.get(k):continue
        p=pose(k)
        if np.linalg.norm(p[:2]-goal[:2])<=resolution_m*1.5 and abs(wrap(p[2]-goal[2]))<=step and sweep(p,goal):
            route=[goal.tolist(),p.tolist()]
            while k in parent:k=parent[k];route.append(pose(k).tolist())
            route.reverse()
            return dict(poses_xy_yaw=route,cost_m=g+heuristic(p),sweep_translation_m=.01,sweep_rotation_rad=math.radians(3),continuous_collision_proof=False)
        for dx,dy,dt in moves:
            key=(k[0]+dx,k[1]+dy,(k[2]+dt)%yaw_bins);next_pose=pose(key)
            if np.linalg.norm(next_pose[:2]-start[:2])>radius_m:continue
            candidate=g+math.hypot(dx,dy)*resolution_m+.15*abs(dt*step)
            if candidate>=cost.get(key,float('inf')) or not sweep(p,next_pose):continue
            cost[key]=candidate;parent[key]=k
            heapq.heappush(queue,(candidate+heuristic(next_pose),candidate,key))
    return None


class AlignmentScreen:
    def __init__(self,*,start,pose_free,configuration_check,view_check,frame_id='map',mode='measured',
                 observation_strategy='recapture_at_goal'):
        self.start,self.pose_free,self.configuration_check,self.view_check=start,pose_free,configuration_check,view_check
        self.frame_id,self.mode=frame_id,mode
        if observation_strategy not in ('recapture_at_goal','stored_target'):
            raise InvalidTask('explicit alignment observation strategy required')
        self.observation_strategy=observation_strategy
        if mode not in ('measured','simulation'):raise InvalidTask('explicit screen provenance required')

    def __call__(self,candidate):
        configuration=self.configuration_check(candidate)
        view=self.view_check(candidate)
        if not isinstance(configuration,Check) or not isinstance(view,Check):raise InvalidTask('typed full body and camera checks required')
        configuration.validate();view.validate()
        candidate['observation_strategy']=self.observation_strategy
        candidate['projected_view']={'valid':view.valid,'reason':view.reason,'cost':view.cost}
        if self.observation_strategy=='stored_target':
            # Camera visibility at the grasp pose is not a reach constraint.
            # This selects a base route only; target tracking evidence and a
            # complete grasp path are separate requirements before execution.
            candidate['requires_target_tracking']=True
            view=Check(True,'final_visibility_not_required_for_stored_target',mode=self.mode)
        route=None
        if configuration.valid and view.valid:
            route=alignment_path(self.start,candidate['base_xy_yaw'],lambda pose:self.pose_free(pose,candidate))
        if route:
            x,y,yaw=candidate['base_xy_yaw']
            candidate['navigation_plan']=dict(frame_id=self.frame_id,goal={'x':x,'y':y,'yaw_rad':yaw},**route)
        return dict(configuration=configuration,view=view,
                    navigation=Check(route is not None,'checked_swept_route' if route else 'no_local_swept_route',
                                     cost=route['cost_m'] if route else 0,mode=self.mode))
