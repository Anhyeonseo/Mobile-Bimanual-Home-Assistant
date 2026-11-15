#!/usr/bin/env python3
"""S4 supervised 20 cm map-goal trial. Starts parked; browser Start is required.

Reuses recorded map/body clearance, native D415, corrected LDS driver, existing
ICP, unified hold/evidence/mobile/STOP clients and append-only session recording.
This is a bounded commissioning trial, not general autonomous Nav2 acceptance.
"""
import argparse
from pulse_mobile_base import PATTERNS
from dataclasses import asdict,replace
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import hashlib,hmac,json,math,os,secrets,signal,subprocess,sys,threading,time
import multiprocessing,queue
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks.navigation_trial import NearGoal,TurnDriveTrial,swept_path_obstacle,swept_obstacle,angle,voxel_centers,validate_navigation_body
from home_robot_tasks.nav2_trial import Nav2Trial,Nav2SensorBridge
from home_robot_tasks.stationary_localization import StationaryScanWindow,fresh_stationary
from home_robot_tasks.s4_recorded_mapping import align,transform,scan_points
from base_teleop_web import SessionRecord
import prepare_unified_controller as commissioning


def registered_lidar_sweep(scan,pose,mount):
    """Keep capture clock and installation offset in separate values."""
    sweep=dict(scan);c,s=math.cos(pose[2]),math.sin(pose[2])
    mount_offset=mount['base_from_lidar_translation_m']
    sweep['lidar_xy_yaw']=[pose[0]+c*mount_offset[0]-s*mount_offset[1],
                          pose[1]+s*mount_offset[0]+c*mount_offset[1],pose[2]+mount['yaw_rad']]
    sweep['ranges']=[v if v is not None and math.isfinite(v) else None for v in scan['ranges']]
    return sweep


class State:
    def __init__(self,trial,record):
        self.lock=threading.RLock();self.trial=trial;self.record=record
        self.pose=None;self.pose_stamp=-1.;self.scan_points=[];self.scan_stamp=-1.
        self.scan_error='라이다 준비 중';self.scan_warning=None
        self.depth_points=[];self.depth_stamp=-1.;self.depth_error='카메라 준비 중'
        self.depth_floor=None
        self.controller_ready=False;self.controller='연결 준비 중';self.pose_history=[]
        self.path=[];self.request=None;self.owner=None;self.last_browser=-1.;self.finished=False
        self.nonzero=0;self.stop_confirmed=False;self.error=None;self.closed=threading.Event()
        self.boot=None;self.lift=None
        self.async_corridor=False;self.corridor_observation=None
        self.nav2_control={};self.nav2_feedback={};self.wheel_observation=None
        self.acquisition_reference=None;self.acquisition_samples=[]
        self.furniture=None
        self.navigation_update=None;self.navigation_revision=None;self.scan_history=[]
        self.navigation_required=False
        self.stationary_window=StationaryScanWindow();self.stationary_localization=None

    def apply_navigation_update(self,update):
        if update is None or update['revision']==self.navigation_revision:return
        with self.lock:
            if update['area']['feasible_by_yaw'].shape!=self.trial.area['feasible_by_yaw'].shape:
                raise ValueError('navigation update lattice changed')
            self.trial.area=update['area'];self.navigation_update=update
            self.navigation_revision=update['revision']
            if isinstance(self.trial,Nav2Trial):
                self.trial.required_map_revision=update['revision'];self.path=[]

    def sensors_ready(self,now):
        return (self.pose is not None and self.scan_error is None and self.depth_error is None
                and 0<=now-self.pose_stamp<=.5 and 0<=now-self.scan_stamp<=.35
                and 0<=now-self.depth_stamp<=.8)

    def motion_sensors_ready(self,now):
        return self.sensors_ready(now) and (not self.async_corridor or self.corridor_hits() is not None)

    def observe_scan(self,points,stamp):
        # Collision evidence is independent of map registration. A passerby may
        # spoil ICP, but their fresh endpoints must still stop an intersecting path.
        points=np.asarray(points,dtype=float)
        if points.ndim!=2 or points.shape[1]!=2 or not np.isfinite(points).all():
            raise ValueError('라이다 유효 좌표 부족')
        with self.lock:
            if stamp<=self.scan_stamp:raise ValueError('라이다 시각 역전')
            self.scan_points=points.tolist();self.scan_stamp=stamp
            self.scan_error=None if len(points)>=60 else '라이다 유효 영역 부족'
        if len(points)<60:raise ValueError('라이다 유효 영역 부족')

    def observe_localization(self,candidate,stamp,fit,inliers,tracking_support=None):
        candidate=np.asarray(candidate,dtype=float)
        if candidate.shape!=(3,) or not np.isfinite(candidate).all() or not math.isfinite(fit):
            raise ValueError('위치 추정 값 확인 필요')
        support=tracking_support or {}
        # A newly placed box occludes mapped walls. Continue an already acquired
        # pose only when many spatially distributed wall matches still agree;
        # never use this exception to acquire an unknown pose or bridge a gap.
        distributed=(fit>=.65 and support.get('matched_endpoints',0)>=120
            and support.get('rmse_m',math.inf)<=.04
            and support.get('minor_spread_m',0)>=.1)
        tracking_ok=self.pose is not None and 0<stamp-self.pose_stamp<=.5 and distributed
        # The launch pose is a reviewed hypothesis, not an odometry reset.
        # At acquisition, demand several agreeing map matches near that seed
        # when a foreground object already obscures some of the saved walls.
        wheel=self.wheel_observation
        parked=(self.nav2_control.get('execute') is False and wheel is not None
            and 0<=stamp-wheel['observed_s']<=.35
            and all(abs(v)<.001 for v in wheel['twist']))
        reacquire=self.pose is not None and stamp-self.pose_stamp>.5 and parked
        if (self.pose is None and self.acquisition_reference is not None or reacquire) and fit<.75:
            ref=self.pose if reacquire else self.acquisition_reference
            previous=self.acquisition_samples[-1] if self.acquisition_samples else None
            consistent=(distributed and math.dist(candidate[:2],ref[:2])<=.15
                and abs(angle(candidate[2]-ref[2]))<=.15)
            if not consistent:self.acquisition_samples=[]
            elif previous is not None and (not 0<stamp-previous[0]<=.5
                    or math.dist(candidate[:2],previous[1][:2])>.05
                    or abs(angle(candidate[2]-previous[1][2]))>.05):
                self.acquisition_samples=[(stamp,candidate.tolist())]
            else:self.acquisition_samples.append((stamp,candidate.tolist()))
            samples=self.acquisition_samples[-5:];self.acquisition_samples=samples
            tracking_ok=consistent and len(samples)>=3 and stamp-samples[0][0]>=.3
        if (fit<.75 and not tracking_ok) or inliers<60:raise ValueError('지도 대조 품질 부족')
        with self.lock:
            if stamp<=self.pose_stamp:raise ValueError('위치 추정 시각 역전')
            if self.pose is not None and (np.linalg.norm(candidate[:2]-np.asarray(self.pose[:2]))>.15 or
                    abs(angle(candidate[2]-self.pose[2]))>.25):
                raise ValueError('위치 추정 변화가 큼')
            stabilized=self.stationary_window.observe(candidate,stamp,revision=self.navigation_revision)
            self.stationary_localization=dict(self.stationary_window.report)
            self.pose=stabilized;self.pose_stamp=stamp;self.scan_warning=None
            self.acquisition_samples=[]
            self.pose_history.append((stamp,self.pose));self.pose_history=self.pose_history[-100:]

    def scan_rejected(self,error):
        # Retain original source timestamps. Rejected frames never extend the
        # 350 ms collision-scan or 500 ms accepted-localization lifetime.
        with self.lock:self.scan_warning=str(error)

    def corridor_hits(self):
        if not self.async_corridor:
            return self.compute_corridor_hits(self.pose,self.trial.goal,self.scan_points,self.depth_points)
        # Geometry runs in the existing sensor process, never under the serial
        # owner's lock. Results retain their original observation timestamps.
        result=self.corridor_observation
        if (result is None or self.trial.goal is None or
                tuple(result['goal'])!=tuple(self.trial.goal) or
                result['pose_stamp']!=self.pose_stamp or result['scan_stamp']!=self.scan_stamp or
                result['depth_stamp']!=self.depth_stamp or not self.sensors_ready(time.monotonic())):
            return None
        if isinstance(self.trial,Nav2Trial) and result.get('nav_twist')!=self.nav2_feedback.get('collision_twist',self.nav2_feedback.get('twist',[0.,0.,0.])):
            return None
        return result['hits']

    def compute_corridor_hits(self,pose,goal,scan_points,depth_points,nav_twist=None):
        if pose is None or goal is None:return None
        if isinstance(self.trial,Nav2Trial):
            twist=self.nav2_feedback.get('collision_twist',self.nav2_feedback.get('twist',(0.,0.,0.))) if nav_twist is None else nav_twist
            path=self.trial.obstacle_sweep(twist)
            options=dict(footprint=self.trial.navigation_body['footprint_xy_m'],margin=self.trial.navigation_body['clearance_m'],return_hits=True)
            return dict(lidar=swept_path_obstacle(scan_points,path,**options),depth=swept_path_obstacle(depth_points,path,**options))
        if isinstance(self.trial,TurnDriveTrial):
            a=pose[2];c,s=math.cos(a),math.sin(a)
            path=[]
            for x,y,yaw in self.trial.path(pose,goal):
                dx,dy=x-pose[0],y-pose[1]
                path.append((c*dx+s*dy,-s*dx+c*dy,angle(yaw-a)))
            options=dict(footprint=self.trial.navigation_body['footprint_xy_m'],margin=self.trial.navigation_body['clearance_m'],return_hits=True)
            return dict(lidar=swept_path_obstacle(scan_points,path,**options),depth=swept_path_obstacle(depth_points,path,**options))
        delta=np.asarray(goal[:2])-pose[:2];a=pose[2]
        local=(math.cos(a)*delta[0]+math.sin(a)*delta[1],-math.sin(a)*delta[0]+math.cos(a)*delta[1])
        body=self.trial.navigation_body
        options={} if body is None else dict(footprint=body['footprint_xy_m'],margin=body['clearance_m'])
        return dict(lidar=swept_obstacle(scan_points,local,return_hits=True,**options),
                    depth=swept_obstacle(depth_points,local,return_hits=True,**options))

    def corridor_clear(self):
        hits=self.corridor_hits()
        return hits is not None and not any(hits.values())

    def snapshot(self):
        with self.lock:
            now=time.monotonic();sensors=self.sensors_ready(now)
            hits=self.corridor_hits();clear=hits is not None and not any(hits.values())
            return dict(state=self.trial.state,pose=self.pose,goal=self.trial.goal,path=self.path,
                ready=self.controller_ready and (not self.navigation_required or self.navigation_revision is not None) and sensors and bool(self.path) and (not isinstance(self.trial,Nav2Trial) or self.trial.backend_ready()) and (clear or (hits is not None and getattr(self.trial,'allow_blocked_start',False))) and not self.finished
                    and self.trial.state=='READY' and self.request is None
                    and (not getattr(self.trial,'fine_alignment',False) or fresh_stationary(self.stationary_localization,now)),
                resume_ready=self.controller_ready and sensors and clear and not self.finished
                    and self.trial.state in NearGoal.WAITING and self.trial.recovery_ready and self.request is None,
                stationary_localization=self.stationary_localization,
                fine_alignment=getattr(self.trial,'fine_alignment',False),
                fine_alignment_result=getattr(self.trial,'verification_result',None),
                resume_required=self.trial.resume_required,navigation_body=self.trial.navigation_body,
                controller=self.controller,scan=self.scan_error or ('라이다 갱신 대기' if
                    now-self.pose_stamp>.5 or now-self.scan_stamp>.35 else
                    '최근 위치 유지 · 라이다 재대조 중' if self.scan_warning else '라이다 위치 수신 중'),
                scan_warning=self.scan_warning,
                depth=self.depth_error or ('깊이 영상 수신 중' if now-self.depth_stamp<=.8 else '깊이 영상 갱신 대기'),
                scan_age=round(now-self.scan_stamp,2),localization_age=round(now-self.pose_stamp,2),depth_age=round(now-self.depth_stamp,2),
                corridor_clear=clear,obstacle_evidence=hits,boot=self.boot,lift_mm=self.lift,
                obstacles_base=(self.scan_points+ [p[:2] for p in self.depth_points[::max(1,len(self.depth_points)//300)]]) if sensors else [],
                depth_floor=self.depth_floor,
                nonzero_commands=self.nonzero,stop_confirmed=self.stop_confirmed,error=self.error or self.nav2_feedback.get('error'),
                navigation_backend='Nav2 planner/DWB' if isinstance(self.trial,Nav2Trial) else 'bounded trial',
                nav2=self.nav2_feedback if isinstance(self.trial,Nav2Trial) else None,
                phase=getattr(self.trial,'phase',None),
                corridor_calculation_ms=(self.corridor_observation or {}).get('calculation_ms'),
                furniture=self.furniture,
                navigation_revision=self.navigation_revision,
                scope=(self.nav2_scope() if isinstance(self.trial,Nav2Trial) else '오른쪽 30° 방향 전환 후 약 40cm 이동 · 장애물은 대기 후 이어하기' if isinstance(self.trial,TurnDriveTrial) else '현장 관찰 아래 전방 20cm 목적지 이동 · 시작 전 바퀴 출력 없음'))

    def nav2_scope(self):
        t=self.trial
        if t.target_pose is not None:
            if t.start_pose is None:return 'Nav2 · 지정 목적지 경로 확인 중 · 시작 전 이동 없음'
            distance=math.dist(t.start_pose[:2],t.goal[:2])*100
            turn=math.degrees(angle(t.goal[2]-t.start_pose[2]))
            return f'Nav2 · 지도 목적지 약 {distance:.0f}cm · 최종 방향 {"왼쪽" if turn>=0 else "오른쪽"} {abs(turn):.0f}° · 시작 전 이동 없음'
        return f'Nav2 · 왼쪽 앞 약 {t.distance*100:.0f}cm 목적지 · 시작 버튼을 누르기 전에는 이동하지 않음'

    def action(self,data):
        with self.lock:
            owner=data.get('owner');action=data.get('action');now=time.monotonic()
            if not isinstance(owner,str) or not 16<=len(owner)<=64:raise ValueError('invalid browser identity')
            if self.owner is not None and self.owner!=owner and (self.trial.state in ('RUNNING','SETTLING') or
                    (self.trial.state in NearGoal.WAITING and now-self.last_browser<=3)):
                raise ValueError('다른 화면에서 시험 중')
            if action=='heartbeat':
                if owner==self.owner:self.last_browser=now
                return
            if action=='plan':
                if self.request is not None:raise ValueError('요청 처리 중입니다')
                if not self.sensors_ready(now):raise ValueError('현재 센서 수신을 기다려 주세요')
                self.path=self.trial.plan(self.pose);self.error=None
                return
            if action=='start':
                if data.get('confirmed') is not True:raise ValueError('표시 위치·경로·받침 확인이 필요합니다')
                if not self.snapshot()['ready'] or self.request is not None:raise ValueError('아직 시작할 수 없습니다')
                # Reserve the one action before queuing it to the sole serial owner.
                if self.trial.state!='READY':raise ValueError('이미 시작한 시험입니다')
                self.owner=owner;self.last_browser=now;self.request='start'
            elif action=='resume':
                if data.get('confirmed') is not True or not self.snapshot()['resume_ready']:
                    raise ValueError('새 관측과 경로 확인 후 이어하기를 눌러 주세요')
                self.owner=owner;self.last_browser=now;self.request='resume'
            elif action=='cancel':
                self.request='cancel';self.owner=owner;self.last_browser=now
            else:raise ValueError('unknown action')


def sensor_worker(state,args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from home_robot_tasks.s2_rgbd_acquisition import Capture
    from home_robot_tasks.s4_sensor_frames import load_sensor_frames
    from home_robot_tasks.navigation_depth import native_endpoints,measured_floor_heights
    from scipy.spatial import cKDTree
    saved=json.loads(args.map.read_text());grid=np.array(saved['data']).reshape(saved['height'],saved['width'])
    yy,xx=np.nonzero(grid>=65);target=np.array(saved['origin'])+(np.column_stack((xx,yy))+.5)*saved['resolution']
    tree=cKDTree(target);review=json.loads(args.visibility.read_text())['visibility']
    mount=transform([*review['base_from_lidar_translation_m'][:2],review['yaw_rad']])
    seed=np.asarray(args.initial_pose,dtype=float);latest=[None]
    state.acquisition_reference=seed.tolist()
    rclpy.init();node=rclpy.create_node('s4_near_goal_observer');offset=node.get_clock().now().nanoseconds/1e9-time.monotonic()
    def scan(m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9-offset
        latest[0]=dict(observed_s=stamp,angle_min=m.angle_min,angle_increment=m.angle_increment,
                       time_increment=m.time_increment,scan_time=m.scan_time,ranges=list(m.ranges))
    node.create_subscription(LaserScan,'/lds_commissioning/scan',scan,qos_profile_sensor_data)
    driver=None;camera=None;last=-1.;camera_to_base=None;ground=None;nav2=None
    depth_thread=None;furniture_thread=None;furniture_queue=queue.Queue(maxsize=1)
    def furniture_worker():
        from home_robot_tasks.furniture_relocation import FurnitureRelocalizer,FurnitureLocationMemory
        from home_robot_tasks.assistant.world import WorldStore
        from check_lds01_driver import RGBDArchive
        from home_robot_tasks.s4_recorded_mapping import interpolate
        store=None;layer=None;compiler=None;compile_requested=threading.Event()
        def compile_maps():
            revision=None
            while not state.closed.is_set():
                if not compile_requested.wait(.2):continue
                compile_requested.clear()
                try:
                    bundle=layer.compile()
                    if state.closed.is_set():return
                    if bundle['revision']==revision:continue
                    # One pointer swaps the map and orientation-specific body
                    # checks together. Compilation never owns the motor port.
                    state.apply_navigation_update(bundle);revision=bundle['revision']
                    state.record.write('furniture_navigation_update',**{k:v for k,v in bundle.items() if k not in ('map','map_bytes','area','fused')})
                    target=args.output/('navigation-'+revision);target.mkdir(exist_ok=True)
                    (target/'occupancy.json').write_bytes(bundle['map_bytes'])
                    np.savez_compressed(target/'movement-area.npz',**bundle['area'])
                except Exception as error:
                    state.record.write('furniture_navigation_error',error=str(error))
        try:
            matcher=FurnitureRelocalizer(args.furniture_reference)
            store=WorldStore(args.furniture_memory)
            memory=FurnitureLocationMemory(matcher.reference,store,'jetson-'+str(os.getpid()))
            if getattr(args,'furniture_navigation',None):
                from home_robot_tasks.furniture_navigation import FurnitureNavigationLayer
                layer=FurnitureNavigationLayer(args.furniture_navigation,store)
                if layer.config['map_sha256']!=hashlib.sha256(args.map.read_bytes()).hexdigest():
                    raise ValueError('navigation evidence belongs to another source map')
                if layer.config['reference_revision']!=matcher.reference['revision']:
                    raise ValueError('navigation/reference binding mismatch')
                compiler=threading.Thread(target=compile_maps,daemon=True);compiler.start()
                compile_requested.set()
            archive=RGBDArchive(args.output,1.)
            with state.lock:state.furniture=dict(status='관측 대기',last_location=memory.saved,reference_anchor=matcher.reference['anchor_xyz_m'],reference_geometry=matcher.reference['observed_geometry_xyz_m'],navigation_map_updated=False)
            while not state.closed.is_set():
                try:record,color,native,metadata=furniture_queue.get(timeout=.2)
                except queue.Empty:continue
                stamp=record['depth_time_s'];history=[]
                # Await the next accepted pose, never extrapolate a moving camera.
                until=time.monotonic()+.5
                while not state.closed.is_set() and time.monotonic()<until:
                    with state.lock:history=list(state.pose_history)
                    if len(history)>=2 and history[0][0]<=stamp<=history[-1][0]:break
                    state.closed.wait(.02)
                if len(history)<2 or not history[0][0]<=stamp<=history[-1][0]:continue
                rows=np.asarray([[t,*p] for t,p in history]);rows[:,3]=np.unwrap(rows[:,3])
                j=int(np.searchsorted(rows[:,0],stamp))
                if not 0<j<len(rows) or rows[j,0]-rows[j-1,0]>.5:continue
                pose=interpolate(rows,[stamp])[0]
                tf={v['child']:np.asarray(v['matrix']) for v in load_sensor_frames(ROOT,depth_to_color=metadata['depth_to_color'])['transforms']}
                world=np.eye(4);world[:2,:2]=transform(pose)[:2,:2];world[:2,3]=pose[:2]
                world[2,3]=-tf['base_footprint'][2,3]
                camera_to_map=world@tf['color_camera_optical_frame']
                archive.add(record,color,native,metadata)
                proposal=matcher.locate(color,native,metadata,camera_to_map)
                try:
                    saved=memory.observe(proposal,capture_id=record['capture_id'],observed_s=stamp,
                        now_s=time.monotonic(),localization_qualified=True)
                except ValueError as error:
                    state.record.write('furniture_frame_rejected',capture_id=record['capture_id'],error=str(error))
                    continue
                state.record.write('furniture_observation',capture_id=record['capture_id'],observed_s=stamp,
                    camera_to_map=camera_to_map.tolist(),proposal=proposal,saved=saved)
                if saved and layer:
                    with state.lock:scans=list(state.scan_history)
                    scan=min(scans,key=lambda s:abs(s['observed_s']-stamp)) if scans else None
                    if scan is not None and abs(scan['observed_s']-stamp)>.2:scan=None
                    try:
                        changed=layer.observe(saved_location=saved,native=native,metadata=metadata,camera_to_map=camera_to_map,
                            scan=scan,lidar_xy_yaw=None if scan is None else scan['lidar_xy_yaw'],
                            capture_id=record['capture_id'],observed_s=stamp,now_s=time.monotonic())
                        state.record.write('furniture_navigation_observation',capture_id=record['capture_id'],
                            observed_s=stamp,scan=scan,revision=layer.state['revision'],changed=changed)
                        if changed:compile_requested.set()
                    except ValueError as error:state.record.write('furniture_navigation_rejected',error=str(error))
                with state.lock:state.furniture=dict(status='위치 저장' if saved else '위치 확인 중' if proposal else '탁자 재관측 중',last_location=memory.saved,reference_anchor=matcher.reference['anchor_xyz_m'],reference_geometry=matcher.reference['observed_geometry_xyz_m'],navigation_map_updated=state.navigation_revision is not None)
        except Exception as e:
            # Semantic association is not a motor/control dependency.
            state.record.write('furniture_error',error=str(e))
            with state.lock:state.furniture=dict(status='가구 관측 확인 필요',error=str(e))
        finally:
            if compiler:compiler.join(timeout=1)
            if store:store.close()
    def depth_worker():
        nonlocal camera_to_base,ground
        last_furniture=-math.inf
        while not state.closed.is_set():
            began=time.monotonic()
            try:
                record,images=camera.next_frame();metadata=images[2];stamp=record['depth_time_s']
                if not 0<=time.monotonic()-stamp<=.8:raise ValueError('깊이 영상 지연')
                if camera_to_base is None:
                    tf=load_sensor_frames(ROOT,depth_to_color=metadata['depth_to_color'])['transforms']
                    transforms={v['child']:np.array(v['matrix']) for v in tf}
                    camera_to_base=transforms['color_camera_optical_frame']@transforms['depth_camera_optical_frame']
                    ground=transforms['base_footprint'][2,3]
                depth=camera.native_depth_images[record['capture_id']].astype(float)*metadata['depth_scale_m']
                intr=metadata['native_depth_intrinsics'];valid=np.isfinite(depth)&(depth>=.15)&(depth<=3.5)
                if valid.mean()<.25:raise ValueError('깊이 유효 영역 부족')
                points=native_endpoints(depth,valid,(intr['fx'],intr['fy'],intr['ppx'],intr['ppy']),camera_to_base)
                # Same physical floor and maximum body height as the reviewed map.
                # All pixels are projected; retain only endpoints that can
                # intersect this <=30 cm trial's complete body sweep.
                floor_height,floor=measured_floor_heights(points,ground)
                points=points[(floor_height>.04)&(points[:,2]<ground+1.12)&(np.abs(points[:,0])<1.)&(np.abs(points[:,1])<1.)]
                points=voxel_centers(points)
                with state.lock:
                    state.depth_points=points.tolist();state.depth_stamp=stamp;state.depth_error=None
                    state.depth_floor=floor
                if getattr(args,'furniture_reference',None) and stamp-last_furniture>=1.:
                    try:
                        furniture_queue.put_nowait((record,images[0].copy(),camera.native_depth_images[record['capture_id']].copy(),metadata))
                        last_furniture=stamp
                    except queue.Full:pass
            except Exception as e:
                with state.lock:state.depth_error=str(e)
            state.closed.wait(max(0.,.15-(time.monotonic()-began)))
    try:
        driver=subprocess.Popen([str(args.driver),'--ros-args','-r','__ns:=/lds_commissioning','-p','port:='+args.lidar_port,'-p','frame_id:=lidar_a'],stdout=args.driver_log,stderr=subprocess.STDOUT)
        from home_robot_tasks.camera_evidence import load_capture_profile
        profile,reference=load_capture_profile(ROOT,getattr(args,'camera_mode','wide'))
        state.record.write('camera_profile',mode=getattr(args,'camera_mode','wide'),profiles=reference['profiles'],
            camera_device=reference['camera_device'])
        camera=Capture(profile,reference_evidence=reference,retain_frame=True,retain_native_depth=True);camera.__enter__()
        depth_thread=threading.Thread(target=depth_worker,daemon=True);depth_thread.start()
        if getattr(args,'furniture_reference',None):
            furniture_thread=threading.Thread(target=furniture_worker,daemon=True);furniture_thread.start()
        if getattr(args,'nav2',False):nav2=Nav2SensorBridge(node,state,args,ROOT)
        while not state.closed.is_set():
            rclpy.spin_once(node,timeout_sec=.01)
            for _ in range(5):rclpy.spin_once(node,timeout_sec=0)
            if driver.poll() is not None:raise RuntimeError('라이다 실행 종료')
            now=time.monotonic();s=latest[0]
            if s and s['observed_s']!=last:
                last=s['observed_s']
                try:
                    if not 0<=now-last<=.35 or len(s['ranges'])!=360 or abs(s['angle_min'])>1e-6 or abs(s['angle_increment']-math.pi/180)>1e-6 or s['time_increment']>=0:
                        raise ValueError('라이다 시각·배열 확인 필요: age=%.3f increment=%.6f'%(now-last,s['time_increment']))
                    s=dict(s);s['ranges']=list(s['ranges'])
                    for i in review['masked_ray_indices']:s['ranges'][i]=math.nan
                    if nav2:nav2.publish_scan(s)
                    points=scan_points(s,mount,None)
                    state.observe_scan(points,last)
                    registration_scan,registration_seed=state.stationary_window.prepare(s,seed,state.wheel_observation,
                        execute=state.nav2_control.get('execute',False),now=now)
                    registration_points=scan_points(registration_scan,mount,None)
                    matrix,quality=align(registration_points,target,transform(registration_seed));candidate=np.array([matrix[0,2],matrix[1,2],math.atan2(matrix[1,0],matrix[0,0])])
                    world=points@matrix[:2,:2].T+matrix[:2,2];distances=tree.query(world)[0];matched=distances<=.1;fit=float(np.mean(matched))
                    spread=float(np.sqrt(max(0,np.linalg.eigvalsh(np.cov(points[matched].T))[0]))) if matched.sum()>=3 else 0.
                    support=dict(matched_endpoints=int(matched.sum()),minor_spread_m=spread,rmse_m=quality['rmse_m'])
                    state.observe_localization(candidate,last,fit,quality['inliers'],support)
                    seed=candidate
                    with state.lock:
                        if state.pose_stamp==last:
                            # Masked rays and the accepted capture-time pose,
                            # never the accumulated SLAM grid, prove free space.
                            sweep=registered_lidar_sweep(s,state.pose,review)
                            state.scan_history.append(sweep);state.scan_history=state.scan_history[-20:]
                        if state.trial.state=='PREVIEW':
                            try:state.path=state.trial.plan(state.pose)
                            except ValueError as e:state.error=str(e)
                    state.record.write('localization',pose=state.pose,raw_pose=candidate.tolist(),stationary=state.stationary_localization,observed_s=last,fit=fit,quality=quality,tracking_support=support)
                except Exception as e:
                    state.scan_rejected(e)
            # Publish the scan/accepted pose just processed above, including
            # the new history sample needed to bracket depth exposures. Running
            # Nav2 first unnecessarily carried the previous scan through its
            # next costmap/action update and consumed the freshness budget.
            if nav2:nav2.tick()
    except Exception as e:
        state.record.write('sensor_error',error=str(e))
        with state.lock:state.scan_error=str(e);state.depth_error=str(e)
    finally:
        state.closed.set()
        if nav2:nav2.close()
        if depth_thread:depth_thread.join(timeout=2)
        if furniture_thread:furniture_thread.join(timeout=6)
        if camera:camera.__exit__(None,None,None)
        if driver:
            driver.send_signal(signal.SIGINT)
            try:driver.wait(timeout=5)
            except subprocess.TimeoutExpired:driver.terminate();driver.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()


_SENSOR_FIELDS=('pose','pose_stamp','scan_points','scan_stamp','scan_error','scan_warning','depth_points','depth_stamp','depth_error','depth_floor','nav2_feedback','furniture','navigation_update','stationary_localization')

def make_trial(args,geometry,area,body):
    if getattr(args,'nav2',False):
        profile=json.loads((ROOT/'config/home_assistant/s4_base.command-candidate.json').read_text())
        policy=json.loads((ROOT/'config/home_assistant/s4_unified.policy.json').read_text())
        return Nav2Trial(geometry,area,patterns=PATTERNS,navigation_body=body,command_profile=profile,
                         controller_maximum_raw=policy['maximum_wheel_speed_raw'],
                         distance=getattr(args,'nav2_distance',.4),target_pose=getattr(args,'nav2_goal',None),
                         manipulation_alignment=getattr(args,'manipulation_alignment',False),fine_alignment=getattr(args,'fine_alignment',False))
    return TurnDriveTrial(geometry,area,patterns=PATTERNS,navigation_body=body) if args.turn_drive else NearGoal(geometry,area,forward_pattern=PATTERNS['forward'][1],navigation_body=body)

def sensor_process_main(options,outbox,goals,closed):
    args=argparse.Namespace(**options)
    args.driver_log=(args.output/'lidar.log').open('a')
    record=SessionRecord(args.output)
    geometry=json.loads((ROOT/'config/home_assistant/s4_base.nominal.json').read_text())
    body=json.loads((ROOT/'config/home_assistant/navigation_body.fixed.json').read_text())
    area=dict(np.load(args.area))
    trial=make_trial(args,geometry,area,body)
    local=State(trial,record);local.closed=closed;local.navigation_required=bool(getattr(args,'furniture_navigation',None))
    def publish():
        goal=None;last_key=None;cached=None;navigation_ack=None
        while not closed.is_set():
            try:
                while True:
                    control=goals.get_nowait()
                    if isinstance(control,dict):
                        goal=control.get('goal')
                        navigation_ack=control.get('navigation_revision')
                        with local.lock:local.nav2_control=control;local.wheel_observation=control.get('wheel')
                    else:goal=control
            except queue.Empty:pass
            with local.lock:sample={k:getattr(local,k) for k in _SENSOR_FIELDS}
            if sample['navigation_update'] is not None and sample['navigation_update']['revision']==navigation_ack:
                sample['navigation_update']=None
            key=(None if goal is None else tuple(goal),sample['pose_stamp'],sample['scan_stamp'],sample['depth_stamp'],
                 tuple(sample['nav2_feedback'].get('collision_twist',sample['nav2_feedback'].get('twist',()))))
            if key!=last_key:
                cached=None
                if goal is not None and sample['pose'] is not None:
                    at=time.monotonic()
                    try:
                        twist=sample['nav2_feedback'].get('collision_twist',sample['nav2_feedback'].get('twist',[0.,0.,0.]))
                        hits=local.compute_corridor_hits(sample['pose'],goal,sample['scan_points'],sample['depth_points'],nav_twist=twist)
                        cached=dict(goal=goal,hits=hits,pose_stamp=sample['pose_stamp'],
                            scan_stamp=sample['scan_stamp'],depth_stamp=sample['depth_stamp'],
                            nav_twist=twist,
                            calculation_ms=(time.monotonic()-at)*1000)
                    except Exception as e:record.write('corridor_error',error=str(e))
                last_key=key
            sample['corridor_observation']=cached
            try:outbox.put_nowait(sample)
            except queue.Full:pass
            closed.wait(.02)
    sender=threading.Thread(target=publish,daemon=True);sender.start()
    try:sensor_worker(local,args)
    finally:
        closed.set();sender.join(timeout=1);record.close();args.driver_log.close()


def sensor_bridge_worker(state,args):
    # Camera projection/SDK startup must not block the MCU heartbeat owner.
    # Source timestamps survive the process boundary; queued data never gains age.
    context=multiprocessing.get_context('spawn');outbox=context.Queue(maxsize=1);goals=context.Queue(maxsize=1);closed=context.Event()
    options={k:v for k,v in vars(args).items() if k!='driver_log'}
    process=context.Process(target=sensor_process_main,args=(options,outbox,goals,closed),daemon=True)
    process.start()
    try:
        sent_goal=None
        while not state.closed.is_set():
            with state.lock:
                goal=state.trial.goal
                payload=dict(goal=None if goal is None else list(goal),execute=state.trial.state=='RUNNING',wheel=state.wheel_observation,navigation_revision=state.navigation_revision)
            try:goals.put_nowait(payload);sent_goal=None if goal is None else tuple(goal)
            except queue.Full:pass
            try:sample=outbox.get(timeout=.2)
            except queue.Empty:
                if not process.is_alive():raise RuntimeError('센서 관측 프로세스 종료: '+str(process.exitcode))
                continue
            with state.lock:
                for key in (*_SENSOR_FIELDS,'corridor_observation'):
                    if key!='navigation_update':setattr(state,key,sample[key])
                state.apply_navigation_update(sample.get('navigation_update'))
                if isinstance(state.trial,Nav2Trial):
                    state.trial.nav=state.nav2_feedback
                    state.trial.stationary_report=state.stationary_localization
                    if state.nav2_feedback.get('goal')==list(state.trial.goal or ()) and state.nav2_feedback.get('path'):
                        state.path=state.nav2_feedback['path']
                if state.trial.state=='PREVIEW' and state.sensors_ready(time.monotonic()):
                    try:state.path=state.trial.plan(state.pose)
                    except ValueError as e:state.error=str(e)
    finally:
        closed.set();process.join(timeout=7)
        if process.is_alive():process.terminate();process.join(timeout=2)
        outbox.close();goals.close()


def observe_stopped(state, observe, stop, transport):
    # Execution has ended; keep observing instead of abandoning
    # the physical stop after a two-second host deadline.
    last_note=last_diagnostic=-1.;confirmed_logged=False
    while not state.closed.is_set():
        now=time.monotonic()
        with state.lock:
            cancel=state.request=='cancel'
            if cancel:state.request=None
        if cancel:
            try:
                stop.request()
                state.record.write('stop_requested_again',source='operator')
            except Exception as e:state.record.write('stop_request_unconfirmed',error=str(e))
        try:observe(allow_stop=True)
        except Exception as e:
            if now-last_note>=2:
                state.record.write('stop_observation_wait',error=str(e));last_note=now
        try:
            proof,_=stop.status()
            with state.lock:
                state.stop_confirmed=proof.base_confirmed
                state.controller=('바퀴 정지 확인 · 팔·리프트 시작 상태 유지' if proof.base_confirmed else
                    '정지 미확인 · 움직이면 모터 전원 차단 · 받침 유지')
            if proof.base_confirmed and not confirmed_logged:
                state.record.write('final_stop',proof=asdict(proof));confirmed_logged=True
        except Exception as e:
            with state.lock:state.stop_confirmed=False;state.controller='정지 확인 대기 · 통신 재조회 중 · 받침 유지'
            if now-last_note>=2:
                state.record.write('stop_unconfirmed',error=str(e));last_note=now
        if now-last_diagnostic>=2:
            try:state.record.write('stop_diagnostics',runtime=transport.stationary_runtime())
            except Exception as e:state.record.write('diagnostics_error',error=str(e))
            last_diagnostic=now
        state.closed.wait(.05)


def reference_supported_bottom(observe,mobile,lift,*,clock=time.monotonic,sleep=time.sleep):
    """Wait for fresh stopped feedback, then renew ownership until hold settles.

    The 200 ms lift command lease must remain alive during HOLD_PENDING.
    No downward homing or position move is sent here.
    """
    deadline=clock()+3;stable_since=None
    while True:
        ls=observe();feedback=mobile.status()
        # The aggregate feedback timestamp is the oldest axis in the sweep.
        # A healthy parked sweep can be 100..120 ms old while the independently
        # checked lift sample is current. Use the existing 250 ms observation
        # budget; keep the zero-speed window and MCU reference admission intact.
        if ls.flags&4 and feedback.feedback_is_fresh(250) and feedback.velocity_raw[3]==0:
            # The MCU's initial wheel-zero position proof needs >=400 ms.
            # Do not enter HOLD_PENDING on a lone quiet speed byte before that
            # existing proof has had time to become authoritative. Keep fresh
            # observations flowing; no motor command is sent during this wait.
            if stable_since is None:stable_since=clock()
            if clock()-stable_since>=.45:break
        else:stable_since=None
        if clock()>deadline:raise TimeoutError('리프트 원점 등록 전 정지 관측 대기')
        sleep(.02)
    lift.reference_bottom(1,operator_confirmed=True)
    deadline=clock()+3
    while True:
        ls=observe()
        if commissioning.supported_lift_is_holding(ls):return
        if clock()>deadline:raise TimeoutError('리프트 유지 확인 시간 초과')
        if ls.flags&2:lift.keepalive()
        sleep(.02)


def arm_observed_stationary(observe,mobile,*,clock=time.monotonic,sleep=time.sleep,on_deferred=None,transport_only=False):
    """Wait parked, including a correlated rejection before any session exists.

    Feedback may change between STATUS and ARM. Only a healthy, same-boot
    rejection proving the session was never armed can return to this wait.
    Never retry ambiguous responses, accepted sessions or motion commands.
    """
    from so101_arm_bridge.mobile_client import MobileReplyError
    from so101_arm_bridge.mobile_wire import MobileStatus
    deadline=clock()+3
    quiet_since=None;last_tick=None;samples=0
    while True:
        if clock()>=deadline:raise TimeoutError('출발 전 정지 관측 대기 시간 초과')
        observation=observe()
        if not transport_only and not commissioning.supported_lift_is_holding(observation):
            raise RuntimeError('리프트 유지 상태 확인 실패')
        feedback=mobile.status()
        if feedback.feedback_is_fresh(100) and not any(feedback.velocity_raw):
            if quiet_since is None:quiet_since=clock()
            if feedback.feedback_tick_ms!=last_tick:
                last_tick=feedback.feedback_tick_ms;samples+=1
            if samples>=3 and clock()-quiet_since>=.1:
                if clock()>=deadline:raise TimeoutError('출발 전 정지 관측 대기 시간 초과')
                try:
                    mobile.arm(1)
                    return
                except MobileReplyError as exc:
                    d=exc.diagnostics;r=MobileStatus(**d['response']);c=d.get('command',{})
                    remaining=(c.get('valid_until_ms',r.mcu_tick_ms)-r.mcu_tick_ms)&0xffffffff
                    unarmed_speed_race=(
                        d['expected_kind']==1 and d['expected_sequence']==0
                        and r.kind==129 and r.request_sequence==0
                        and mobile.boot_id is not None and r.boot_id==mobile.boot_id
                        and r.state==0 and r.session==0 and r.sequence==0 and r.reason==0
                        and r.flags==(51 if transport_only else 23) and r.capabilities&7==7 and r.feedback_is_fresh(100)
                        and any(r.velocity_raw) and c.get('opcode')==1
                        and c.get('session')==1 and c.get('sequence')==0
                        and tuple(c.get('velocity_raw',()))==(0,0,0,0) and 0<remaining<=200)
                    if not unarmed_speed_race:raise
                    if on_deferred is not None:on_deferred(d)
                    quiet_since=None;last_tick=None;samples=0
        else:quiet_since=None;last_tick=None;samples=0
        sleep(.02)


def enter_base_transport(observe,evidence,*,clock=time.monotonic,sleep=time.sleep):
    """Initial posture is real; busy read slots may defer admission while parked."""
    deadline=clock()+2
    while clock()<deadline:
        status=observe()
        if not commissioning.supported_lift_is_holding(status):
            raise RuntimeError('출발 전 리프트 유지 확인 실패')
        if evidence.enter_transport():return
        sleep(.005)
    raise TimeoutError('이동 모드 전환 대기 시간 초과')


def controller_worker(state,args):
    import serial
    from so101_arm_bridge.backend_lease import acquire_backend_lease
    from so101_arm_bridge.serial_port import open_exclusive_serial
    from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2,MobileV2Exchange
    from so101_arm_bridge.unified_commissioning import CommissionClient,FIRMWARE
    from so101_arm_bridge.unified_arm_adapter import UnifiedArmAdapter
    from so101_arm_bridge.mobile_client import MobileClient
    from so101_arm_bridge.lift_wire import LiftClient
    from so101_arm_bridge.robot_evidence import RobotEvidenceClient
    from so101_arm_bridge.system_stop import SystemStopClient
    baseline=json.loads(args.baseline.read_text());powered=False;armed=False;stopped=False
    arm_queries=[]
    deadline=time.monotonic()+20
    while not state.closed.is_set():
        with state.lock:
            if state.sensors_ready(time.monotonic()):break
        if time.monotonic()>deadline:raise TimeoutError('실시간 센서 시작 확인 시간 초과')
        time.sleep(.05)
    if state.closed.is_set():return
    with acquire_backend_lease('stm32',189),open_exclusive_serial(serial,args.port,921600,.02) as port:
        t=StreamValidationTransportV2(port,response_timeout_s=3)
        try:
            hello=t.enter_binary_mode()
            if hello.firmware_version!=FIRMWARE:raise ValueError('제어기 버전 불일치')
            if any(getattr(hello,k)!=baseline['hello'][k] for k in ('left_calibration_hash','right_calibration_hash')):raise ValueError('보정값 불일치')
            if not args.prepare_supported:
                with state.lock:state.controller='조회 연결 확인 · 자세 유지 준비 필요';state.boot=CommissionClient(t).query().boot_id
                state.record.write('inspection',hello=asdict(hello),motion_commands=0)
                while not state.closed.wait(.25):
                    with state.lock:
                        if state.request:state.request=None;state.error='현재는 조회 준비 화면입니다'
                return
            if args.resume_prepared:
                prepared=commissioning.inspect(t,baseline=baseline)
                current=prepared['commission_final']
                if current['flags']!=17 or current['reason']:
                    raise ValueError('resume requires verified prepared torque-OFF state')
            else:
                released=commissioning.release_boot_supported(t,baseline=baseline,supported_empty=True)
                state.record.write('supported_release',result=released)
                prepared=commissioning.inspect(t,prepare=True,supported=True,empty=True,baseline=baseline)
            state.record.write('registered',result=prepared)
            preview=UnifiedArmAdapter(t,ROOT/'config/home_assistant/arm_ranges.development.json')
            adapter=UnifiedArmAdapter(t,ROOT/'config/home_assistant/arm_ranges.development.json',authorized_limits=replace(preview._limits,hardware_authorized=True),motion_authorized=True)
            adapter.prepare();mobile=MobileClient(MobileV2Exchange(t),lambda:int(time.monotonic()*1000));mobile.synchronize()
            lift=LiftClient(mobile);stop=SystemStopClient(t,mobile.boot_id,time.monotonic)
            evidence=RobotEvidenceClient(t,mobile,time.monotonic,maximum_age_ms=250)
            powered=True;activation=CommissionClient(t).activate_zero(supported=True,empty_grippers=True)
            state.record.write('zero_power_activation',result=asdict(activation))
            armed=True;adapter.start_current_pose_hold('navigation-ready');anchor=adapter.prepared_state.positions_urad
            end=time.monotonic()+3
            while adapter.state.value!='ready':
                if time.monotonic()>end:raise TimeoutError('현재 자세 유지 준비 시간 초과')
                adapter.poll('navigation-ready');time.sleep(.02)
            last_lift_log=[-1.,None]
            def observe(allow_stop=False):
                h=t.heartbeat()
                if h.stop_latched and not allow_stop:
                    from so101_arm_bridge.stationary_telemetry import describe_runtime_fault
                    runtime=t.stationary_runtime();state.record.write('stop_diagnostics',runtime=runtime)
                    raise RuntimeError(describe_runtime_fault(runtime))
                if evidence.transport_only:return None  # heartbeat only; resident holding torque is unchanged
                sent=time.monotonic();snapshot=t.get_feedback_snapshot();received=time.monotonic()
                arm_queries.append(dict(sent_s=sent,received_s=received,snapshot=asdict(snapshot)))
                del arm_queries[:-16]
                commissioning.measured_hold_source(snapshot,anchor,adapter._limits,sent,received)
                evidence.publish_mcu_snapshot(snapshot,sent,received,arms_safe=True,lift_hold=False,payload_safe=True)
                ls=lift.status()
                observed=time.monotonic();signature=(ls.state,ls.fault,ls.height_um,ls.flags)
                if observed-last_lift_log[0]>=1 or signature!=last_lift_log[1]:
                    state.record.write('lift_transport_observation',status=asdict(ls))
                    last_lift_log[:]=[observed,signature]
                with state.lock:state.lift=ls.height_um/1000;state.boot=mobile.boot_id
                return ls
            reference_supported_bottom(observe,mobile,lift)
            enter_base_transport(observe,evidence)
            state.record.write('base_transport_admitted',boot=mobile.boot_id,
                initial_arm_observed_ms=evidence.last_observed,
                posture_monitoring='initial_only',post_arrival='fresh_recommission_required')
            with state.lock:state.controller_ready=True;state.controller='현재 자세 유지 · 시작 대기'
            state.record.write('parked_ready',boot=mobile.boot_id,nonzero_commands=0,arm_anchor=list(anchor))
            wheel_scales=None;last_wheel_query=-1.
            if isinstance(state.trial,Nav2Trial):
                pending=json.loads((ROOT/'config/home_assistant/s4_commissioning.pending.json').read_text())
                wheel_scales=pending['measurement_inputs']['wheel_geometry_and_raw_speed_scale']['candidate_raw_per_encoder_rad_s']
                if len(wheel_scales)!=3 or not all(math.isfinite(v) and v>0 for v in wheel_scales):raise ValueError('wheel feedback scale missing')
            def wheel_observed(result,sent):
                if wheel_scales is None:return
                age=((result.mcu_tick_ms-result.feedback_tick_ms)&0xffffffff)/1000
                if not result.feedback_is_fresh(250):return
                twist=state.trial.geometry.body_twist(tuple(v/s for v,s in zip(result.velocity_raw[:3],wheel_scales)))
                with state.lock:state.wheel_observation=dict(observed_s=sent-age,twist=twist,
                    feedback_tick_ms=result.feedback_tick_ms,boot_id=result.boot_id)
            zero_since=None;previous=time.monotonic();stop_until=None
            while not state.closed.is_set():
                if time.monotonic()-previous>.2:raise RuntimeError('제어 통신 주기 지연')
                previous=time.monotonic();ls=observe(allow_stop=stopped)
                if not evidence.transport_only and not commissioning.supported_lift_is_holding(ls):raise RuntimeError('리프트 유지 상태 확인 실패')
                with state.lock:
                    start_base=False;action=state.request;state.request=None
                    if action=='start':
                        try:
                            if not state.snapshot()['ready']:raise ValueError('시작 직전 관측이 바뀜 · 준비 확인 후 다시 시작')
                            state.trial.start(time.monotonic(),state.pose,operator_confirmed=True)
                        except ValueError as e:state.error=str(e)
                        else:
                            state.path=state.trial.path(state.trial.start_pose,state.trial.goal)
                            start_base=True;state.error=None;state.controller='목적지 이동 중'
                            state.record.write('explicit_start',pose=state.pose,goal=state.trial.goal)
                    elif action=='resume':
                        try:
                            state.trial.resume(time.monotonic(),state.pose,state.pose_stamp,operator_confirmed=True,
                                sensors_fresh=state.motion_sensors_ready(time.monotonic()),corridor_clear=state.corridor_clear())
                            state.error=None;state.record.write('explicit_resume',pose=state.pose,goal=state.trial.goal)
                        except ValueError as e:state.error=str(e)
                    elif action=='cancel':state.trial.cancel()
                    now=time.monotonic();trial_state=state.trial.state
                    if trial_state in ('RUNNING','VERIFYING_ALIGNMENT',*NearGoal.WAITING) and now-state.last_browser>3:
                        state.trial.pause(now,'WAITING_BROWSER')
                    if state.trial.state in ('RUNNING','VERIFYING_ALIGNMENT',*NearGoal.WAITING):
                        values=state.trial.command(now,state.pose,state.pose_stamp,sensors_fresh=state.motion_sensors_ready(now),corridor_clear=state.corridor_clear())
                    else:values=(0,0,0,0)
                    current=state.trial.state
                    if current!=trial_state:
                        state.record.write('trial_state',state=current,goal=state.trial.goal,
                            scan_age=now-state.scan_stamp,pose_age=now-state.pose_stamp,depth_age=now-state.depth_stamp,
                            corridor_available=state.corridor_hits() is not None,
                            nav2_observation_age=now-state.nav2_feedback.get('observed_s',now),
                            nav2_pose_age=now-state.nav2_feedback.get('odom_s',now),
                            nav2_depth_age=now-state.nav2_feedback.get('depth_s',now),
                            nav2_map_revision=state.nav2_feedback.get('map_revision'),
                            required_map_revision=state.navigation_revision,
                            corridor_calculation_ms=(state.corridor_observation or {}).get('calculation_ms'))
                    phase=getattr(state.trial,'phase',None)
                    if phase!=getattr(state,'recorded_phase',None):
                        state.record.write('route_phase',phase=phase,pose=state.pose);state.recorded_phase=phase
                    if current in NearGoal.WAITING:
                        reason={'WAITING_SENSOR':'위치·깊이 관측 대기','WAITING_OBSTACLE':'경로 장애물 대기','WAITING_BROWSER':'화면 연결 대기'}[current]
                        if current=='WAITING_OBSTACLE' and isinstance(state.trial,Nav2Trial) and not state.trial.collision_hold:
                            reason=state.trial.nav.get('wait_reason') or '주행 경로 계산 대기'
                        if getattr(state.trial,'collision_hold',False):reason='가까운 진행 경로 장애물 감지'
                        state.controller='일시정지 · '+reason+(' · 확인 후 이어하기' if state.trial.resume_required else '')
                    elif current=='VERIFYING_ALIGNMENT':state.controller='바퀴 영속도 유지 · 정지 위치 여러 스캔 확인 중'
                    elif current=='RUNNING':state.controller='방향 맞추는 중' if getattr(state.trial,'phase',None) in ('TURNING','FINAL_TURNING') else '목적지 이동 중'
                if start_base:
                    # Admission/preview can take time. Publish a fresh posture
                    # observation immediately before arming, not before planning.
                    arm_observed_stationary(observe,mobile,
                        on_deferred=lambda d:state.record.write('unarmed_admission_deferred',diagnostics=d),transport_only=True)
                    # Admission may wait while parked. The next cycle obtains
                    # current sensor/path evidence before any nonzero command.
                    values=(0,0,0,0);previous=time.monotonic()
                if mobile.session is not None and not stopped:
                    sent=time.monotonic()
                    result=mobile.velocity(values,lifetime_ms=150)
                    wheel_observed(result,sent)
                    if not result.ready_for_motion(mobile.session,250):raise RuntimeError('바퀴 명령 상태 확인 실패')
                    with state.lock:
                        state.nonzero+=bool(any(values));trial_state=state.trial.state
                    state.record.write('base_command',raw=list(values),reply=asdict(result))
                    if trial_state=='SETTLING':
                        age=(result.mcu_tick_ms-result.feedback_tick_ms)&0xffffffff
                        # Send the formal STOP after a fresh zero-command interval.
                        # Firmware owns measured stop proof (including the
                        # position window); speed bytes alone are not proof.
                        if age<250 and not any(values):
                            if zero_since is None:zero_since=result.feedback_tick_ms
                            if (result.feedback_tick_ms-zero_since)&0xffffffff>=400:
                                stop.request();stopped=True;stop_until=time.monotonic()+3
                        else:zero_since=None
                elif wheel_scales is not None and time.monotonic()-last_wheel_query>=.1:
                    sent=time.monotonic();wheel_observed(mobile.status(),sent);last_wheel_query=sent
                if trial_state=='CANCELLED' and not stopped:
                    stop.request();stopped=True;stop_until=time.monotonic()+3
                if stopped:
                    proof,_=stop.status()
                    with state.lock:
                        state.stop_confirmed=proof.base_confirmed
                        if proof.base_confirmed and not state.finished:
                            if state.trial.state=='SETTLING':state.trial.finish(True)
                            state.finished=True;state.controller_ready=False;state.controller=('바퀴 정지 확인 · 미세 정렬 재확인 필요' if state.trial.state=='ALIGNMENT_REVIEW' else '바퀴 정지 확인 · 팔·리프트 시작 상태 유지')
                            state.record.write('finished',state=state.trial.state,proof=asdict(proof),nonzero_commands=state.nonzero,fine_alignment_result=getattr(state.trial,'verification_result',None))
                    if not state.finished and time.monotonic()>stop_until:raise RuntimeError('바퀴 정지 확인 시간 초과')
                time.sleep(.01)
        except Exception as e:
            with state.lock:state.error=str(e);state.controller_ready=False;state.controller='준비 중단 · 받침 유지';state.trial.state='FAILED';state.finished=True
            state.record.write('controller_error',error=str(e),diagnostics=getattr(e,'diagnostics',None),
                recent_arm_queries=arm_queries)
        finally:
            if armed:
                try:
                    if not stopped:
                        try:stop.request()
                        except Exception as e:state.record.write('stop_request_unconfirmed',error=str(e))
                    observe_stopped(state,observe,stop,t)
                except Exception as e:state.record.write('stop_unconfirmed',error=str(e))
                try:state.record.write('stop_diagnostics',runtime=t.stationary_runtime())
                except Exception as e:state.record.write('diagnostics_error',error=str(e))
                # Read cached bus failures only after the stop procedure; never
                # delay STOP or require debugger access to retain this evidence.
                try:state.record.write('servo_feedback_failures',failures=t.stationary_feedback_failures())
                except Exception as e:state.record.write('servo_feedback_failures_unavailable',error=str(e))
            elif powered:state.record.write('power_state_unconfirmed',supports_required=True)


PAGE='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>S4 지도 이동 시험</title><style>
body{font:17px system-ui;background:#f3f5f8;color:#142333;max-width:1000px;margin:36px auto;padding:0 24px}h1{font-size:28px}.panel{background:white;padding:20px;border-radius:16px;margin:16px 0}canvas{width:100%;height:480px;background:#f0f2f4;border-radius:12px}button{padding:15px 24px;border:0;border-radius:10px;background:#165cde;color:white;font-size:17px;margin:10px 10px 0 0}button:disabled{opacity:.4}#stop{background:#bc253c}.bad{color:#a42c22}label{display:block;margin-top:15px}small{color:#536171}#status{font-weight:700}</style>
<h1>S4 · 가까운 목적지 이동</h1><p>지도에 표시한 경로로 이동하고 멈춥니다. 경로에 물체가 있으면 출발하지 않고 기다립니다.</p>
<div class="panel"><div id="status">연결 중</div><p id="detail"></p><canvas id="map"></canvas><small>파란 화살표: 라이다 추정 위치 · 초록 선/점: 예정 경로와 목표. 지도 대조 위치는 실제 위치와 확인해 주세요.</small></div>
<div class="panel"><label><input type="checkbox" id="confirm"> 표시 위치와 앞쪽 경로가 맞고, 전원선 여유·본체와 함께 움직이는 받침이 유지됩니다.</label><button id="plan">현재 위치로 경로 갱신</button><button id="start" disabled>시험 시작</button><button id="resume" disabled>이어하기</button><button id="stop">정지·종료</button><p id="error" class="bad"></p><small>빈 집게·리프트 최하단의 짧은 현장 시험입니다. 일반 자율주행·집기 성공 판정은 포함하지 않습니다.</small></div>
<script>
const token=location.hash.slice(1),owner=crypto.randomUUID(),canvas=document.querySelector('canvas'),ctx=canvas.getContext('2d');let state={},grid=null,busy=false,gridRevision=null;
async function post(action,extra={}){const r=await fetch('/action',{method:'POST',headers:{'Authorization':'Bearer '+token,'Content-Type':'application/json'},body:JSON.stringify({owner,action,...extra})});const d=await r.json();if(!r.ok)throw Error(d.error||'요청 실패');return d;}
async function poll(){try{const r=await fetch('/status',{headers:{'Authorization':'Bearer '+token},cache:'no-store'});if(!r.ok)throw Error('접속 주소 확인 필요');state=await r.json();if(state.navigation_revision&&state.navigation_revision!==gridRevision){const m=await fetch('/map',{headers:{'Authorization':'Bearer '+token}});if(m.ok){grid=await m.json();gridRevision=grid.navigation_revision;}}document.querySelector('#status').textContent=state.controller;document.querySelector('#detail').textContent=state.scope+(state.fine_alignment?' · 미세 정렬 (1cm·1° 추정 기준)':'')+' · '+state.scan+' · '+state.depth+' · 리프트 '+(state.lift_mm??'—')+'mm'+(state.stationary_localization?' · '+(state.stationary_localization.ready?'정지 위치 관측 안정':'정지 위치 관측 수집 중'):'')+(state.furniture?' · 탁자: '+state.furniture.status+' (위치 관측·저장 시험)':'');document.querySelector('#error').textContent=state.error||(!state.corridor_clear&&state.goal?'경로에 장애물이 있습니다 · 시작해도 이동 없이 대기합니다':'');document.querySelector('#start').disabled=busy||!state.ready||!document.querySelector('#confirm').checked;document.querySelector('#resume').disabled=busy||!state.resume_ready||!document.querySelector('#confirm').checked;document.querySelector('#plan').disabled=!['PREVIEW','READY'].includes(state.state);draw();if(['RUNNING','VERIFYING_ALIGNMENT','SETTLING','WAITING_SENSOR','WAITING_OBSTACLE','WAITING_BROWSER'].includes(state.state))await post('heartbeat');}catch(e){document.querySelector('#status').textContent='연결 확인 중';document.querySelector('#start').disabled=true;document.querySelector('#resume').disabled=true;}setTimeout(poll,300);}
function draw(){if(!grid)return;canvas.width=1000;canvas.height=480;const scale=Math.min(950/grid.width,440/grid.height),ox=(1000-scale*grid.width)/2,oy=(480-scale*grid.height)/2;ctx.fillStyle='#f0f2f4';ctx.fillRect(0,0,1000,480);for(let y=0;y<grid.height;y++)for(let x=0;x<grid.width;x++){const v=grid.data[y*grid.width+x];ctx.fillStyle=v<0?'#c8ced5':v>=65?'#293746':'#fff';ctx.fillRect(ox+x*scale,oy+(grid.height-y-1)*scale,scale+.2,scale+.2);}const point=p=>[ox+(p[0]-grid.origin[0])/grid.resolution*scale,oy+(grid.height-(p[1]-grid.origin[1])/grid.resolution)*scale];if(state.pose&&state.obstacles_base){const p=state.pose,c=Math.cos(p[2]),s=Math.sin(p[2]);ctx.fillStyle='#e87924';for(const [a,b] of state.obstacles_base){const [x,y]=point([p[0]+c*a-s*b,p[1]+s*a+c*b]);ctx.fillRect(x-1.5,y-1.5,3,3);}}if(state.furniture){const f=state.furniture;for(const [name,m,color] of [['원래 탁자',null,'#777'],['확인한 새 위치',f.last_location?.matrix,'#9d36c8']]){if(name==='확인한 새 위치'&&!m)continue;ctx.fillStyle=color;for(const v of f.reference_geometry||[]){const p=m?[m[0][0]*v[0]+m[0][1]*v[1]+m[0][3],m[1][0]*v[0]+m[1][1]*v[1]+m[1][3]]:v;const [x,y]=point(p);ctx.fillRect(x-2,y-2,4,4);}}}if(state.path.length){ctx.beginPath();state.path.forEach((p,i)=>{let [x,y]=point(p);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.strokeStyle='#139c6c';ctx.lineWidth=5;ctx.stroke();let [x,y]=point(state.goal);ctx.fillStyle='#139c6c';ctx.beginPath();ctx.arc(x,y,7,0,7);ctx.fill();}if(state.pose&&state.navigation_body){const p=state.pose,c=Math.cos(p[2]),s=Math.sin(p[2]);ctx.beginPath();state.navigation_body.footprint_xy_m.forEach(([a,b],i)=>{const [x,y]=point([p[0]+c*a-s*b,p[1]+s*a+c*b]);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.closePath();ctx.fillStyle='#165cde22';ctx.fill();ctx.strokeStyle='#165cde';ctx.lineWidth=2;ctx.stroke();}if(state.pose){let [x,y]=point(state.pose);ctx.save();ctx.translate(x,y);ctx.rotate(-state.pose[2]);ctx.fillStyle='#165cde';ctx.beginPath();ctx.moveTo(15,0);ctx.lineTo(-10,-8);ctx.lineTo(-6,0);ctx.lineTo(-10,8);ctx.closePath();ctx.fill();ctx.restore();}}
for(const action of ['plan','start','resume','cancel']){const id=action==='cancel'?'stop':action;document.querySelector('#'+id).onclick=async()=>{if(busy)return;busy=true;try{await post(action,{confirmed:document.querySelector('#confirm').checked});}catch(e){document.querySelector('#error').textContent=e.message;}finally{busy=false;}}}
fetch('/map',{headers:{'Authorization':'Bearer '+token}}).then(r=>r.json()).then(d=>{grid=d;draw()});poll();
</script></html>'''


def handler_for(state,token,map_doc):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,code,data,content='application/json'):
            raw=data.encode() if isinstance(data,str) else json.dumps(data,allow_nan=False).encode()
            self.send_response(code);self.send_header('Content-Type',content);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        def authorized(self):return hmac.compare_digest(self.headers.get('Authorization','').encode(),('Bearer '+token).encode())
        def do_GET(self):
            if self.path=='/':return self.reply(200,PAGE,'text/html; charset=utf-8')
            if not self.authorized():return self.reply(403,{'error':'접속키 확인 필요'})
            if self.path=='/status':return self.reply(200,state.snapshot())
            if self.path=='/map':
                with state.lock:
                    update=state.navigation_update
                    current=dict(update['map'],data=update['fused'].ravel().tolist(),navigation_revision=update['revision']) if update else map_doc
                return self.reply(200,current)
            self.reply(404,{})
        def do_POST(self):
            if self.path!='/action' or not self.authorized():return self.reply(403,{})
            try:
                n=int(self.headers.get('Content-Length','0'))
                if not 0<n<=2048:raise ValueError('invalid request size')
                state.action(json.loads(self.rfile.read(n)));self.reply(200,state.snapshot())
            except Exception as e:self.reply(409,{'error':str(e)})
    return Handler


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('map','visibility','area','baseline','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--initial-pose',type=float,nargs=3,required=True);p.add_argument('--port',required=True);p.add_argument('--lidar-port',required=True)
    p.add_argument('--driver',type=Path,required=True);p.add_argument('--web-port',type=int,default=18768)
    p.add_argument('--turn-drive',action='store_true',help='30deg right turn then 40cm, tested teleop ratios, forward75/turn50')
    p.add_argument('--nav2',action='store_true',help='real Nav2 planner/controller with original lidar/depth and the single MCU owner; operator Start required')
    p.add_argument('--nav2-distance',type=float,choices=(.2,.3,.4,.5,.6,.9,1.2),default=.4,
                   help='bounded Nav2 preview distance in meters; does not start movement')
    p.add_argument('--nav2-goal',type=float,nargs=3,metavar=('X','Y','YAW'),
                   help='fixed map target within distance bound and 45deg final turn; operator Start still required')
    p.add_argument('--manipulation-alignment',action='store_true',help='explicit map goal: up to 90deg final turn and 3cm/0.1rad arrival verification; no speed or collision changes')
    p.add_argument('--camera-mode',choices=('wide','legacy'),default='wide',help='measured RGB profile; native depth/extrinsics preserved')
    p.add_argument('--fine-alignment',action='store_true',help='separate <=15cm/10deg final alignment; stationary verification, no automatic start')
    p.add_argument('--prepare-supported',action='store_true',help='explicit supported-empty actual-bottom preparation; no base travel')
    p.add_argument('--resume-prepared',action='store_true',help='explicit prepared torque-OFF admission after a separately verified supported release; no reset or retry')
    p.add_argument('--furniture-reference',type=Path,help='immutable original SLAM appearance/geometry reference; no manual after-location')
    p.add_argument('--furniture-memory',type=Path,help='persistent WorldStore for last verified furniture location; not free-space authority')
    p.add_argument('--furniture-navigation',type=Path,help='bound original voxel/body evidence for automatic furniture navigation updates')
    a=p.parse_args()
    if bool(a.furniture_reference)!=bool(a.furniture_memory):p.error('furniture reference and persistent memory must be supplied together')
    if a.furniture_navigation and not a.furniture_reference:p.error('furniture navigation requires registered reference and memory')
    if a.nav2 and a.turn_drive:p.error('select one navigation backend')
    if a.nav2_goal is not None and not a.nav2:p.error('map target requires Nav2')
    if a.manipulation_alignment and a.nav2_goal is None:p.error('alignment requires an explicit map goal')
    if a.fine_alignment and not a.manipulation_alignment:p.error('fine alignment requires manipulation alignment')
    if a.resume_prepared and not a.prepare_supported:p.error('resume requires explicit supported preparation')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':p.error('localhost ROS required')
    a.output.mkdir(parents=True,exist_ok=False)
    record=SessionRecord(a.output);record.lock=threading.Lock();write=record.write
    def locked_write(*args,**kwargs):
        with record.lock:write(*args,**kwargs)
    record.write=locked_write
    map_doc=json.loads(a.map.read_text());area=dict(np.load(a.area));geometry=json.loads((ROOT/'config/home_assistant/s4_base.nominal.json').read_text())
    body=validate_navigation_body(json.loads((ROOT/'config/home_assistant/navigation_body.fixed.json').read_text()),area,a.map)
    trial=make_trial(a,geometry,area,body)
    state=State(trial,record);state.async_corridor=True;state.navigation_required=bool(a.furniture_navigation);token=secrets.token_urlsafe(24)
    server=ThreadingHTTPServer(('127.0.0.1',a.web_port),handler_for(state,token,map_doc))
    a.driver_log=(a.output/'lidar.log').open('w');workers=[]
    def guarded(task):
        try:task(state,a)
        except Exception as e:
            with state.lock:
                state.error=str(e);state.controller_ready=False;state.controller='준비 중단 · 받침 유지'
            record.write('worker_error',worker=task.__name__,error=str(e))
    for task in (sensor_bridge_worker,controller_worker):
        worker=threading.Thread(target=guarded,args=(task,),daemon=True);worker.start();workers.append(worker)
    (a.output/'connection.json').write_text(json.dumps({'url':f'http://127.0.0.1:{a.web_port}/#{token}','pid':os.getpid(),'prepared_mode':a.prepare_supported})+'\n');os.chmod(a.output/'connection.json',0o600)
    def shutdown(*_):state.closed.set();threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGINT,shutdown);signal.signal(signal.SIGTERM,shutdown)
    try:server.serve_forever(poll_interval=.1)
    finally:
        state.closed.set()
        for w in workers:w.join(timeout=8)
        (a.output/'final-status.json').write_text(json.dumps(state.snapshot(),indent=2)+'\n')
        server.server_close();a.driver_log.close();record.close()
if __name__=='__main__':main()
