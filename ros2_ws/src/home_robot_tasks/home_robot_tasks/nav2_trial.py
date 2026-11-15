"""Nav2 commissioning on the existing sensor worker and single MCU owner.

Planning is permitted while parked. FollowPath is sent only after the web
operator starts; no serial device, motor enable or reset is owned here.
"""
import math
import time
from pathlib import Path

import numpy as np

from .base_motion import TeleopCommandModel
from .navigation_trial import NearGoal, TurnDriveTrial, angle
from .fetch import InvalidTask
from .stationary_localization import fresh_stationary


def recoverable_nav2_result(stage,code):
    # Jazzy nav2_msgs FollowPath/ComputePathToPose Result constants. Normal TF,
    # progress and blocked-route failures wait/replan, not a firmware STOP latch.
    return code in ({102,103,104,105,106,107} if stage=='follow' else {202,205,206,207,208})


def alignment_goal_matches(path,goal):
    """Never substitute the planner's nearby free target for approved alignment."""
    return bool(path and len(path[-1])==3 and len(goal)==3
        and all(math.isfinite(v) for v in (*path[-1],*goal))
        and math.dist(path[-1][:2],goal[:2])<=.005)


class Nav2Trial(TurnDriveTrial):
    def __init__(self, geometry, area, *, patterns, navigation_body, command_profile,
                 controller_maximum_raw, distance=.4, turn_degrees=30, target_pose=None, manipulation_alignment=False, fine_alignment=False):
        if not .2<=distance<=1.2:
            raise ValueError('bounded Nav2 distance required')
        super().__init__(geometry,area,patterns=patterns,navigation_body=navigation_body,
                         distance=min(distance,.6),turn_degrees=turn_degrees)
        self.distance=distance
        self.model=TeleopCommandModel(command_profile)
        if type(controller_maximum_raw) is not int or not 0<controller_maximum_raw<=200:
            raise ValueError('explicit installed controller wheel limit required')
        self.model.limit=min(self.model.limit,controller_maximum_raw)
        self.nav={};self.required_map_revision=None;self.phase='NAV2';self.motion_since=math.inf
        self.active_timeout_s=max(180,450*distance)
        self.collision_hold=False
        if type(manipulation_alignment) is not bool or (manipulation_alignment and target_pose is None):
            raise ValueError('alignment requires an explicit map goal')
        self.manipulation_alignment=manipulation_alignment
        if fine_alignment and not manipulation_alignment:
            raise ValueError('fine alignment requires manipulation alignment')
        self.fine_alignment=fine_alignment
        self.stationary_report=None;self.fine_verified=False;self.verification_started=None
        self.verification_result=None
        self.maximum_turn=math.radians(90 if manipulation_alignment else 45)
        self.arrival_xy=.01 if fine_alignment else .03 if manipulation_alignment else .12
        self.arrival_yaw=math.radians(1) if fine_alignment else .1 if manipulation_alignment else .2
        self.target_pose=None if target_pose is None else tuple(target_pose)
        if self.target_pose is not None and (len(self.target_pose)!=3 or
                not all(math.isfinite(v) for v in self.target_pose)):
            raise ValueError('finite map target pose required')

    def path(self,start,goal):
        if self.target_pose is not None:
            # A straight segment is not a route around saved furniture.
            # Display only the route returned by the actual Nav2 planner.
            if self.nav.get('goal')==list(goal):return self.nav.get('path') or []
            return []
        return super().path(start,goal)

    def plan(self,pose):
        if self.target_pose is None:return super().plan(pose)
        if self.state not in ('PREVIEW','READY'):raise InvalidTask('trial already consumed')
        if len(pose)!=3 or not all(math.isfinite(v) for v in pose):raise InvalidTask('invalid localization')
        goal=self.target_pose
        if self.fine_alignment and (math.dist(pose[:2],goal[:2])>.15 or abs(angle(goal[2]-pose[2]))>math.radians(10)):
            raise InvalidTask('미세 정렬은 정지 위치에서 15cm·10° 이내만 가능합니다')
        # Resuming an approved map goal can leave less than 20 cm (or only
        # final yaw). Do not require moving farther away to re-admit it.
        if math.dist(pose[:2],goal[:2])>self.distance:
            raise InvalidTask('map target exceeds bounded trial distance')
        if abs(angle(goal[2]-pose[2]))>self.maximum_turn:
            raise InvalidTask('map target exceeds bounded trial turn')
        if not self.map_free(goal):
            raise InvalidTask('목적지에 지도 장애물이 있습니다')
        self.start_pose=tuple(pose);self.goal=goal;self.state='READY'
        return self.path(pose,goal)

    def start(self,now,pose,*,operator_confirmed):
        if self.fine_alignment and not fresh_stationary(self.stationary_report,now):
            raise InvalidTask('정지 위치 관측을 모으는 중입니다')
        super().start(now,pose,operator_confirmed=operator_confirmed)
        self.motion_since=now;self.phase='NAV2'

    def finish(self, stop_confirmed):
        super().finish(stop_confirmed)
        if self.fine_alignment and not self.fine_verified:
            self.state='ALIGNMENT_REVIEW'

    def resume(self,now,*args,**kwargs):
        super().resume(now,*args,**kwargs);self.motion_since=now
        self.collision_hold=False;self.phase='NAV2'

    def backend_ready(self):
        return (self.nav.get('ready') is True and self.nav.get('goal')==list(self.goal or ())
            and (self.required_map_revision is None or self.nav.get('map_revision')==self.required_map_revision)
                and 0<=time.monotonic()-self.nav.get('observed_s',-1)<=.5
                and 0<=time.monotonic()-self.nav.get('odom_s',-1)<=.5
                and 0<=time.monotonic()-self.nav.get('depth_s',-1)<=.8)

    def route_valid(self,pose):
        # Nav2 is allowed to steer around an obstacle rather than being forced
        # onto the original straight preview. Keep the bounded trial envelope.
        # Navigation collision authority is DWB's full fixed-body footprint on
        # the fused LiDAR/RGB-D map, plus the independent live swept-body gate.
        # The offline centre/yaw lattice screens destination candidates only;
        # snapping every live pose to it adds a second, discontinuous veto.
        return (pose is not None and self.start_pose is not None
                and math.dist(pose[:2],self.start_pose[:2])<=self.distance+.2)

    def sweep(self,twist):
        # One second at the actually bounded command, plus the fixed 3 cm body
        # margin. No long straight-line gate that prevents Nav2 local avoidance.
        raw=self.model.raw_command(twist);vx,vy,w=self.model.expected_twist(raw)
        result=[]
        for t in np.linspace(0,1,11):
            a,b=(math.sin(w*t)/w,(1-math.cos(w*t))/w) if abs(w)>1e-9 else (t,0.)
            result.append((a*vx-b*vy,b*vx+a*vy,w*t))
        return result

    def obstacle_sweep(self,twist):
        # One second at commissioning speed is under 1 cm. Keep a 15 cm
        # early-stop translation zone; pure rotation uses the original sweep.
        path=self.sweep(twist)
        distance=math.hypot(*path[-1][:2])
        scale=max(1.,.15/distance) if distance>1e-5 else 1.
        return [(x*scale,y*scale,yaw) for x,y,yaw in path]

    def command(self,now,pose,pose_stamp,*,sensors_fresh,corridor_clear):
        nav=self.nav
        if self.fine_alignment and self.state=='VERIFYING_ALIGNMENT':
            report=self.stationary_report
            if (sensors_fresh and fresh_stationary(report,now)
                    and report['observed_s']-report.get('span_s',0)>=self.verification_started):
                measured=report['pose']
                xy=math.dist(measured[:2],self.goal[:2]);yaw=abs(angle(measured[2]-self.goal[2]))
                self.fine_verified=xy<=self.arrival_xy and yaw<=self.arrival_yaw
                self.verification_result=dict(report,goal=list(self.goal),xy_error_m=xy,yaw_error_rad=yaw,
                    within_fine_tolerance=self.fine_verified)
                self.state='SETTLING'
            elif now-self.verification_started>=8:
                self.verification_result=dict(within_fine_tolerance=False,reason='stationary localization not stable within 8s')
                self.state='SETTLING'
            return (0,0,0,0)
        current=(nav.get('goal')==list(self.goal or ()) and 0<=now-nav.get('observed_s',-1)<=.5
                 and (self.required_map_revision is None or nav.get('map_revision')==self.required_map_revision))
        sensors_fresh=(sensors_fresh and current and 0<=now-nav.get('odom_s',-1)<=.5
                       and 0<=now-nav.get('depth_s',-1)<=.8)
        if current and nav.get('error') and self.state in ('RUNNING',*self.WAITING):
            raise InvalidTask('Nav2: '+nav['error'])
        if current and nav.get('wait_reason') and self.state=='RUNNING':
            self.pause(now,'WAITING_OBSTACLE')
        immediate_obstacle=sensors_fresh and not corridor_clear
        if self.state in self.WAITING:corridor_clear=corridor_clear and self.backend_ready()
        if (self.state in ('RUNNING',*self.WAITING) and sensors_fresh and 0<=now-pose_stamp<=.5
                and nav.get('goal')==list(self.goal) and 0<=now-nav.get('observed_s',-1)<=.5
                and nav.get('result')=='SUCCEEDED'):
            if self.fine_alignment:
                self.state='VERIFYING_ALIGNMENT';self.verification_started=now
                return (0,0,0,0)
            if math.dist(pose[:2],self.goal[:2])>self.arrival_xy or abs(angle(pose[2]-self.goal[2]))>self.arrival_yaw:
                raise InvalidTask('Nav2 result disagrees with measured destination')
            self.state='SETTLING';return (0,0,0,0)
        # Planner obstacles can be routed around. Fresh endpoints in the
        # immediate body sweep stop first, independently of the planner.
        # A zero twist or disappearing endpoint cannot automatically restart.
        if self.state in ('RUNNING',*self.WAITING) and immediate_obstacle:
            self.pause(now,'WAITING_OBSTACLE')
            self.collision_hold=True;self.resume_required=True;self.phase='OBSTACLE_STOP'
        if not self.admit_command(now,pose,pose_stamp,sensors_fresh=sensors_fresh,corridor_clear=corridor_clear):
            return (0,0,0,0)
        if nav.get('goal')!=list(self.goal) or not 0<=now-nav.get('observed_s',-1)<=.5:
            return (0,0,0,0)
        if nav.get('error'):raise InvalidTask('Nav2: '+nav['error'])
        stamp=nav.get('command_s',-1)
        if (not nav.get('ready') or not nav.get('executing') or stamp<=self.motion_since or not 0<=now-stamp<=.25):
            return (0,0,0,0)
        twist=nav['twist'];raw=self.model.raw_command(twist)
        return tuple(round(v) for v in raw)+(0,)


def obstacle_grid(points, footprint, *, size=100, resolution=.025,pose=(0.,0.,0.),origin=None,shape=None):
    """Independent fresh depth endpoint layer; unseen cells stay unknown."""
    grid=np.full((size,size) if shape is None else shape,-1,np.int8)
    p=np.asarray(points,dtype=float).reshape(-1,3)[:,:2]
    if not np.isfinite(p).all():raise ValueError('nonfinite depth points')
    poly=np.asarray(footprint);edges=np.roll(poly,-1,axis=0)-poly
    normals=np.column_stack((edges[:,1],-edges[:,0]))/np.linalg.norm(edges,axis=1)[:,None]
    bounds=np.sum(normals*poly,axis=1)
    p=p[~np.all(p@normals.T<=bounds+1e-12,axis=1)]
    c,s=math.cos(pose[2]),math.sin(pose[2])
    p=p@np.array([[c,s],[-s,c]])+np.asarray(pose[:2])
    origin=(-size*resolution/2,)*2 if origin is None else origin
    indices=np.floor((p-origin)/resolution).astype(int)
    indices=indices[np.all((indices>=0)&(indices<np.array([grid.shape[1],grid.shape[0]])),axis=1)]
    if len(indices):grid[indices[:,1],indices[:,0]]=100
    return grid


def offset_convex_footprint(points, distance):
    """Euclidean edge clearance, unlike Nav2's axis-wise sign padding.

    Use the same half-planes as the continuous swept-body check. Axis-wise
    padding expands diagonal corners by up to sqrt(2) times the requested gap.
    """
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=2 or len(points)<3 or not np.isfinite(points).all() or not math.isfinite(distance) or distance<0:
        raise ValueError('finite convex footprint and nonnegative clearance required')
    edges=np.roll(points,-1,axis=0)-points
    if np.any(np.linalg.norm(edges,axis=1)<1e-10):raise ValueError('duplicate footprint edge')
    normals=np.column_stack((edges[:,1],-edges[:,0]))/np.linalg.norm(edges,axis=1)[:,None]
    bounds=np.sum(normals*points,axis=1)
    if np.any(points@normals.T>bounds+1e-9):raise ValueError('counterclockwise convex footprint required')
    result=[]
    for i in range(len(points)):
        pair=np.stack((normals[i-1],normals[i]))
        if abs(np.linalg.det(pair))>1e-12:
            result.append(np.linalg.solve(pair,[bounds[i-1]+distance,bounds[i]+distance]).tolist())
    return result


def nav2_trial_config(template, root, namespace='/s4_nav2_trial', *, map_target=False, manipulation_alignment=False, fine_alignment=False):
    """Keep the reviewed omni/full-lift parameters; use existing LiDAR pose TF."""
    import copy
    config=copy.deepcopy(template)
    # Exact namespace rewrite includes topics and nested costmap node names.
    import json
    config=json.loads(json.dumps(config).replace('/offline_nav2',namespace))
    config.pop(namespace+'/amcl',None)
    controller=config[namespace+'/controller_server']['ros__parameters']
    controller['enable_stamped_cmd_vel']=True
    policy=json.loads((Path(root)/'config/home_assistant/s4_unified.policy.json').read_text())
    command=json.loads((Path(root)/'config/home_assistant/s4_base.command-candidate.json').read_text())
    limit=policy['maximum_wheel_speed_raw']
    if type(limit) is not int or not 0<limit<=command['maximum_raw']:
        raise ValueError('installed controller wheel limit exceeds command model')
    scale=limit/command['maximum_raw']
    dwb=controller['FollowPath']
    if manipulation_alignment:
        if not map_target:raise ValueError('alignment requires an explicit map target')
        config[namespace+'/planner_server']['ros__parameters']['GridBased']['tolerance']=0.
        controller['goal_checker'].update(xy_goal_tolerance=.025,yaw_goal_tolerance=.07)
        dwb['xy_goal_tolerance']=.025
    for key in ('min_vel_x','max_vel_x','min_vel_y','max_vel_y','max_vel_theta',
                'min_speed_xy','max_speed_xy','min_speed_theta',
                'acc_lim_x','acc_lim_y','acc_lim_theta','decel_lim_x','decel_lim_y','decel_lim_theta'):
        if key in dwb:dwb[key]*=scale
    if manipulation_alignment:
        # Align-14 repeatedly selected raw 12/23 below usable rolling output.
        # Reject tiny candidates INSIDE DWB, before trajectory/collision scoring;
        # never amplify its chosen command after the footprint check. Pure yaw
        # remains possible because DWB admits XY OR yaw above its lower bound.
        # Keep the installed maximums and measured-arrival tolerances unchanged.
        dwb['min_speed_xy']=min(.0045,dwb['max_speed_xy'])
        dwb['min_speed_theta']=min(.025,dwb['max_vel_theta'])
    if fine_alignment:
        if not manipulation_alignment:raise ValueError('fine alignment requires manipulation alignment')
        controller['goal_checker'].update(xy_goal_tolerance=.008,yaw_goal_tolerance=math.radians(.8),stateful=False)
        dwb['xy_goal_tolerance']=.008
    # Preserve projected distance at the installed lower speed, so a trajectory
    # still crosses costmap cells and can make progress toward its goal.
    dwb['sim_time']/=scale
    if map_target:
        # A three-wheel omni base can translate toward a rear map target
        # without first pointing its nose there. Keep the same speed cap,
        # footprint critics and final-yaw checker; score path distance at
        # the base rather than a point in front of it.
        dwb['min_vel_x']=-dwb['max_vel_x']
        dwb['PathAlign.scale']=0.
        dwb['GoalAlign.scale']=0.
        # Holonomic travel needs an exact zero-yaw sample and a preference
        # against gratuitous spinning, without forbidding the final turn.
        dwb['vx_samples']=13;dwb['vtheta_samples']=21
        dwb['critics'].append('Twirling');dwb['Twirling.scale']=10.
        # Score progress along the next portion of the detour, not only the
        # far endpoint across the obstacle (which can reward standing still).
        dwb['forward_prune_distance']=.5
        # DWB keeps the first point within prune_distance, then ends the local
        # path at forward_prune_distance. A larger rear threshold can retain a
        # passed point outside the forward window and produce an empty path.
        dwb['prune_distance']=.45
        # A one-cell lateral departure to follow a curved detour must cost
        # less than the one-cell progress it makes toward the local goal.
        dwb['PathDist.scale']=16.
    local=config[namespace+'/local_costmap/local_costmap']['ros__parameters']
    local.update(use_maximum=True,track_unknown_space=False,
                 plugins=['static_layer','obstacle_layer','depth_layer','inflation_layer'])
    local['static_layer']=dict(plugin='nav2_costmap_2d::StaticLayer',map_topic=namespace+'/map',
                               map_subscribe_transient_local=True)
    local['depth_layer']=dict(plugin='nav2_costmap_2d::StaticLayer',map_topic=namespace+'/depth_grid',
        map_subscribe_transient_local=True,subscribe_to_updates=False,footprint_clearing_enabled=False)
    global_map=config[namespace+'/global_costmap/global_costmap']['ros__parameters']
    # Jazzy StaticLayer reads this at the costmap root, not per plugin.
    global_map['use_maximum']=True
    global_map['plugins']=['static_layer','obstacle_layer','depth_layer','inflation_layer']
    global_map['depth_layer']=dict(local['depth_layer'],map_topic=namespace+'/global_depth_grid')
    # DWB rasterizes the padded footprint against occupied costmap cells.
    # Keep the physical 3 cm clearance once. An additional half-cell diagonal
    # here expands it to 6.5 cm before DWB's cell collision test and can reject
    # every departure from a physically clear parked pose near furniture.
    # Live lidar/depth swept-body checks still use the physical margin and
    # 15 cm stopping zone; neither obstacle data nor the body is reduced.
    for costmap in (local,global_map):
        distance=costmap['footprint_padding']
        costmap['footprint']=json.dumps(offset_convex_footprint(json.loads(costmap['footprint']),distance))
        costmap['footprint_padding']=0.  # already offset once along edge normals
    return config


class Nav2SensorBridge:
    """ROS-only peer of the original sensor worker. Never owns the MCU bus."""
    def __init__(self,node,state,args,root):
        import json,subprocess,yaml
        from ament_index_python.packages import get_package_prefix
        from rclpy.action import ActionClient
        from nav2_msgs.action import ComputePathToPose
        from nav_msgs.msg import OccupancyGrid,Odometry
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import TwistStamped
        from rclpy.qos import QoSProfile,DurabilityPolicy
        from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
        from .nav2_port import Nav2FollowPathPort
        self.node,self.state,self.args,self.root=node,state,args,Path(root)
        self.namespace='/s4_nav2_trial';self.processes=[];self.logs=[]
        self.activated=False;self.lifecycle=None;self.life_future=None;self.life_index=0
        self.life_started=time.monotonic();self.plan_future=None;self.planned_goal=None;self.path=None
        self.goal=None;self.active=None;self.sequence=0;self.cancelling=False;self.error=None
        self.result=None;self.command_s=-1.;self.twist=(0.,0.,0.);self.last_pose=self.last_depth=-1.
        self.last_motion_s=-math.inf;self.last_motion_twist=(0.,0.,0.)
        self.retry_after=0.;self.wait_reason=None;self.last_plan_request=-math.inf
        self.map_revision=None;self.plan_revision=None
        self.follow=Nav2FollowPathPort(node,self.namespace+'/follow_path')
        self.planner=ActionClient(node,ComputePathToPose,self.namespace+'/compute_path_to_pose')
        self.plan_type=ComputePathToPose
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub=node.create_publisher(OccupancyGrid,self.namespace+'/map',q)
        self.depth_pub=node.create_publisher(OccupancyGrid,self.namespace+'/depth_grid',q)
        self.global_depth_pub=node.create_publisher(OccupancyGrid,self.namespace+'/global_depth_grid',q)
        self.scan_pub=node.create_publisher(LaserScan,self.namespace+'/scan',10)
        self.odom_pub=node.create_publisher(Odometry,self.namespace+'/odom',10)
        self.tf=TransformBroadcaster(node);self.static_tf=StaticTransformBroadcaster(node)
        self.command_sub=node.create_subscription(TwistStamped,self.namespace+'/cmd_vel',self.receive,1)
        saved=json.loads(args.map.read_text());area=dict(np.load(args.area))
        data=np.array(saved['data']).reshape(saved['height'],saved['width'])
        self.global_grid_geometry=dict(shape=data.shape,resolution=saved['resolution'],origin=tuple(saved['origin']))
        if area['rgbd_obstacles'].shape!=data.shape:raise ValueError('saved depth/map grids differ')
        data[area['rgbd_obstacles']]=100
        self.map=self.grid_message(data,saved['resolution'],saved['origin'],'map')
        self.map_pub.publish(self.map)
        config=nav2_trial_config(yaml.safe_load((root/'config/nav2.alohamini.candidate.yaml').read_text()),root,
                                map_target=state.trial.target_pose is not None,
                                manipulation_alignment=state.trial.manipulation_alignment, fine_alignment=state.trial.fine_alignment)
        params=args.output/'nav2-params.yaml';params.write_text(yaml.safe_dump(config))
        self.node.create_timer(1.,lambda:self.map_pub.publish(self.map))
        from geometry_msgs.msg import TransformStamped
        identity=TransformStamped();identity.header.frame_id='map';identity.child_frame_id='odom'
        identity.header.stamp=node.get_clock().now().to_msg();identity.transform.rotation.w=1.
        review=json.loads(args.visibility.read_text())['visibility']
        laser=TransformStamped();laser.header.frame_id='base_footprint';laser.child_frame_id='lidar_a'
        laser.header.stamp=identity.header.stamp
        laser.transform.translation.x,laser.transform.translation.y,laser.transform.translation.z=review['base_from_lidar_translation_m']
        laser.transform.rotation.z=math.sin(review['yaw_rad']/2);laser.transform.rotation.w=math.cos(review['yaw_rad']/2)
        self.static_tf.sendTransform([identity,laser])
        try:
            for package,name in [('nav2_planner','planner_server'),('nav2_controller','controller_server')]:
                log=(args.output/('nav2-'+name+'.log')).open('w');self.logs.append(log)
                self.processes.append(subprocess.Popen([str(Path(get_package_prefix(package))/'lib'/package/name),
                    '--ros-args','-r','__ns:='+self.namespace,'--params-file',str(params)],stdout=log,stderr=subprocess.STDOUT))
        except Exception:self.close();raise

    def grid_message(self,data,resolution,origin,frame):
        from nav_msgs.msg import OccupancyGrid
        m=OccupancyGrid();m.header.frame_id=frame;m.header.stamp=self.node.get_clock().now().to_msg()
        m.info.resolution=float(resolution);m.info.width=data.shape[1];m.info.height=data.shape[0]
        m.info.origin.position.x=float(origin[0]);m.info.origin.position.y=float(origin[1]);m.info.origin.orientation.w=1.
        m.data=data.ravel().astype(int).tolist();return m

    def stamp(self,monotonic):
        from rclpy.time import Time
        age=time.monotonic()-monotonic
        return Time(nanoseconds=self.node.get_clock().now().nanoseconds-round(age*1e9)).to_msg()

    def receive(self,msg):
        if self.active is None or self.cancelling:return
        age=(self.node.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if (msg.header.frame_id!='base_footprint' or not 0<=age<=.25 or
                any(v!=0 for v in (msg.twist.linear.z,msg.twist.angular.x,msg.twist.angular.y))):return
        values=(msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z)
        if not all(math.isfinite(v) for v in values):return
        stamp=time.monotonic()-age
        if stamp<=self.command_s:return
        self.command_s=stamp;self.twist=values
        if any(abs(v)>1e-5 for v in values):
            self.last_motion_s=stamp;self.last_motion_twist=values

    def collision_twist(self,now):
        # Nav2 zero output must not erase the approached corridor before the
        # sensor worker sees an intrusion. This is never used as drive output.
        if any(abs(v)>1e-5 for v in self.twist):return self.twist
        if 0<=now-getattr(self,'last_motion_s',-math.inf)<=.5:return self.last_motion_twist
        return (0.,0.,0.)

    def publish_scan(self,scan):
        from sensor_msgs.msg import LaserScan
        m=LaserScan();m.header.frame_id='lidar_a';m.header.stamp=self.stamp(scan['observed_s'])
        for key in ('angle_min','angle_increment','time_increment','scan_time'):setattr(m,key,float(scan[key]))
        m.angle_max=m.angle_min+359*m.angle_increment;m.range_min=.25;m.range_max=3.5
        m.ranges=list(scan['ranges']);self.scan_pub.publish(m)

    def publish_pose_depth(self):
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TransformStamped
        with self.state.lock:
            pose=self.state.pose;at=self.state.pose_stamp
            depth=self.state.depth_points;depth_at=self.state.depth_stamp
            wheel=self.state.wheel_observation
            history=list(self.state.pose_history)
        if pose is None:return
        if at!=self.last_pose:
            m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_footprint';m.header.stamp=self.stamp(at)
            m.pose.pose.position.x,m.pose.pose.position.y=pose[:2]
            m.pose.pose.orientation.z=math.sin(pose[2]/2);m.pose.pose.orientation.w=math.cos(pose[2]/2)
            if wheel and 0<=time.monotonic()-wheel['observed_s']<=.35:
                m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z=wheel['twist']
            else:
                # Never invent a stationary velocity from missing wheel data.
                return
            for i in (0,7,35):m.pose.covariance[i]=.0025;m.twist.covariance[i]=.01
            self.odom_pub.publish(m)
            tf=TransformStamped();tf.header=m.header;tf.child_frame_id=m.child_frame_id
            tf.transform.translation.x,tf.transform.translation.y=pose[:2];tf.transform.rotation=m.pose.pose.orientation
            self.tf.sendTransform(tf);self.last_pose=at
        if depth_at!=self.last_depth and 0<=time.monotonic()-depth_at<=.8:
            # Project using the pose at exposure, not the newest pose. If the
            # next scan has not arrived, defer this frame rather than extrapolate.
            if len(history)<2 or not history[0][0]<=depth_at<=history[-1][0]:return
            from .s4_recorded_mapping import interpolate
            rows=np.asarray([[t,*p] for t,p in history]);rows[:,3]=np.unwrap(rows[:,3])
            q=interpolate(rows,[depth_at])[0]
            origin=tuple(np.floor((np.asarray(q[:2])-1.25)/.025)*.025)
            data=obstacle_grid(depth,self.state.trial.navigation_body['footprint_xy_m'],pose=q,origin=origin)
            m=self.grid_message(data,.025,origin,'odom')
            m.header.stamp=self.stamp(depth_at);self.depth_pub.publish(m);self.last_depth=depth_at
            # Keep exactly the saved global map geometry; a moving small grid
            # would resize the non-rolling master costmap. Unknown depth cells
            # cannot clear the saved occupancy or the independent LiDAR layer.
            geometry=self.global_grid_geometry
            data=obstacle_grid(depth,self.state.trial.navigation_body['footprint_xy_m'],pose=q,**geometry)
            m=self.grid_message(data,geometry['resolution'],geometry['origin'],'map')
            m.header.stamp=self.stamp(depth_at);self.global_depth_pub.publish(m)

    def activate(self):
        from lifecycle_msgs.srv import ChangeState
        if self.activated:return
        steps=[(name,transition) for name in ('planner_server','controller_server') for transition in (1,3)]
        if time.monotonic()-self.life_started>40:raise TimeoutError('Nav2 lifecycle preparation timed out')
        if self.life_future is not None:
            if not self.life_future.done():return
            if not self.life_future.result().success:raise RuntimeError('Nav2 lifecycle rejected')
            self.node.destroy_client(self.lifecycle);self.lifecycle=self.life_future=None;self.life_index+=1
        if self.life_index==len(steps):self.activated=True;return
        name,transition=steps[self.life_index]
        if self.lifecycle is None:self.lifecycle=self.node.create_client(ChangeState,self.namespace+'/'+name+'/change_state')
        if self.lifecycle.service_is_ready():
            r=ChangeState.Request();r.transition.id=transition;self.life_future=self.lifecycle.call_async(r)

    def request_plan(self,goal):
        self.plan_revision=getattr(self,'map_revision',None)
        request=self.plan_type.Goal();request.goal.header.frame_id='map'
        request.goal.header.stamp=self.node.get_clock().now().to_msg()
        request.goal.pose.position.x,request.goal.pose.position.y=goal[:2]
        request.goal.pose.orientation.z=math.sin(goal[2]/2);request.goal.pose.orientation.w=math.cos(goal[2]/2)
        request.planner_id='GridBased';request.use_start=False
        self.planned_goal=list(goal);self.path=None;self.plan_future=self.planner.send_goal_async(request)
        self.plan_accepted=False
        self.last_plan_request=time.monotonic()

    def apply_navigation_update(self):
        with self.state.lock:update=getattr(self.state,'navigation_update',None)
        if update is None or update['revision']==self.map_revision:return
        geometry=self.global_grid_geometry
        if (update['fused'].shape!=geometry['shape'] or update['map']['resolution']!=geometry['resolution']
                or tuple(update['map']['origin'])!=geometry['origin']):
            raise ValueError('updated navigation map lattice changed')
        self.map=self.grid_message(update['fused'],geometry['resolution'],geometry['origin'],'map')
        self.map_pub.publish(self.map);self.map_revision=update['revision']
        self.planned_goal=None;self.path=None;self.command_s=-1.;self.twist=(0.,0.,0.)
        if self.active is not None and not self.cancelling:
            self.follow.cancel(self.active);self.cancelling=True
        self.wait_reason='가구 위치 반영 · 경로 재계산'

    def tick(self):
        try:
            if self.error is not None:
                # A terminal error cannot send another action while the host
                # is still receiving the failure through the sensor queue.
                with self.state.lock:
                    self.state.nav2_feedback=dict(ready=False,observed_s=time.monotonic(),goal=self.goal,error=self.error,executing=False)
                return
            if any(p.poll() is not None for p in self.processes):raise RuntimeError('Nav2 process exited')
            self.apply_navigation_update();self.publish_pose_depth();self.activate()
            with self.state.lock:control=dict(self.state.nav2_control)
            goal=control.get('goal');execute=control.get('execute',False)
            if goal!=self.goal:
                self.goal=goal;self.result=None;self.error=None;self.wait_reason=None;self.retry_after=0.
            if self.active is not None:
                result=self.follow.poll(self.active)
                if not execute and not self.cancelling:
                    self.follow.cancel(self.active);self.cancelling=True;self.command_s=-1.;self.twist=(0.,0.,0.)
                if result.state!='RUNNING':
                    if not self.cancelling:
                        if result.state=='SUCCEEDED':self.result='SUCCEEDED'
                        elif recoverable_nav2_result('follow',self.follow.entries[self.active].get('error_code',0)):
                            self.wait_reason='Nav2 경로 재계산 대기';self.retry_after=time.monotonic()+1.
                            self.planned_goal=None;self.path=None
                        else:
                            entry=self.follow.entries[self.active]
                            raise RuntimeError('Nav2 follow path %s: %s' %
                                (entry.get('error_code','unknown'),entry.get('error_message') or result.reason or result.state))
                    else:self.planned_goal=None
                    self.active=None;self.cancelling=False;self.command_s=-1.;self.twist=(0.,0.,0.)
            if (self.activated and goal and self.active is None and self.plan_future is None
                    and (self.planned_goal!=goal or
                         (not execute and self.result is None and time.monotonic()-self.last_plan_request>=1.))
                    and self.planner.server_is_ready() and self.last_pose>=0
                    and time.monotonic()>=self.retry_after and self.state.sensors_ready(time.monotonic())):
                self.request_plan(goal)
            if self.plan_future is not None and self.plan_future.done():
                value=self.plan_future.result()
                if not self.plan_accepted:
                    if not value.accepted:
                        if self.plan_revision!=self.map_revision:self.plan_future=None;self.planned_goal=None
                        else:raise RuntimeError('Nav2 planner rejected goal')
                    else:self.plan_future=value.get_result_async();self.plan_accepted=True
                else:
                    self.plan_future=None
                    if self.plan_revision!=self.map_revision:
                        self.path=None;self.planned_goal=None
                    elif value.status!=4 or value.result.error_code:
                        if not recoverable_nav2_result('plan',value.result.error_code):raise RuntimeError('Nav2 planner could not find a route')
                        self.wait_reason='Nav2 통과 경로 대기';self.retry_after=time.monotonic()+1.;self.planned_goal=None
                    else:
                        self.path=[[p.pose.position.x,p.pose.position.y,2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in value.result.path.poses]
                        if len(self.path)<2:raise RuntimeError('Nav2 returned an empty path')
                        self.path[-1][2]=goal[2];self.wait_reason=None
                        if self.state.trial.manipulation_alignment and not alignment_goal_matches(self.path,goal):
                            self.path=None;self.planned_goal=None;self.retry_after=time.monotonic()+1.
                            self.wait_reason='승인한 정렬 목표까지 경로 계산 대기'
            ready=(self.activated and self.path is not None and self.planned_goal==goal and self.follow.ready()
                   and self.last_depth>=0 and self.last_pose>=0)
            if execute and ready and self.active is None and self.result is None:
                self.sequence+=1;self.active='web-nav2-'+str(self.sequence)
                self.follow.start(self.active,dict(frame_id='map',poses_xy_yaw=self.path))
                self.command_s=-1.;self.twist=(0.,0.,0.)
            with self.state.lock:
                self.state.nav2_feedback=dict(ready=bool(ready),observed_s=time.monotonic(),goal=goal,
                    map_revision=getattr(self,'map_revision',None),
                    odom_s=self.last_pose,depth_s=self.last_depth,
                    executing=self.active is not None and not self.cancelling,result=self.result,error=self.error,
                    command_s=self.command_s,twist=list(self.twist),path=self.path,sequence=self.sequence,wait_reason=self.wait_reason)
                self.state.nav2_feedback['collision_twist']=list(self.collision_twist(time.monotonic()))
        except Exception as error:
            self.error=str(error)
            if self.active is not None and not self.cancelling:self.follow.cancel(self.active);self.cancelling=True
            with self.state.lock:self.state.nav2_feedback=dict(ready=False,observed_s=time.monotonic(),goal=self.goal,error=self.error,executing=False)

    def close(self):
        import signal,subprocess
        if self.active is not None:
            try:self.follow.cancel(self.active)
            except Exception:pass
        for process in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=4)
                except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=3)
        for log in self.logs:log.close()
