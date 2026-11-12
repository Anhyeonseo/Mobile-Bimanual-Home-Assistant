#!/usr/bin/env python3
"""Live localization observer for the existing manual teleop log and LDS topic.

Never opens a motor port or publishes velocity commands. Wheel stop is a labelled
stationary assumption until fresh encoder feedback arrives; never command odometry.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks.base_motion import OmniKinematics, Wheel
from home_robot_tasks.s4_recorded_mapping import transform, pose, scan_points, map_initial_pose_candidates
from check_lds01_driver import TeleopCompletion


class LiveEncoder:
    """Incremental position-count integration, with session and sequence checks."""
    def __init__(self, geometry):
        self.model=OmniKinematics(tuple(Wheel(**{k:w[k] for k in
            ('x_m','y_m','rolling_angle_rad','radius_m')}) for w in geometry['wheels']))
        self.matrix=np.eye(3);self.counts=None;self.rows=[];self.command=None
        self.active=False;self.stationary=False;self.finished=False;self.last_feedback=None
        self.session=None;self.sequence=0

    def sample(self, wheels, stamp):
        if [w['id'] for w in wheels]!=[7,8,9]:raise ValueError('wheel identity changed')
        counts=np.array([w['position_raw'] for w in wheels])
        if not np.isfinite(counts).all() or np.any(counts<0) or np.any(counts>65535):raise ValueError('invalid wheel positions')
        if self.counts is not None:
            delta=(counts-self.counts+2048)%4096-2048
            if np.any(abs(delta)>=1024):raise ValueError('ambiguous encoder wrap')
            dx,dy,da=self.model.body_twist((delta*math.tau/4096).tolist())
            a,b=(1.,da/2) if abs(da)<1e-9 else (math.sin(da)/da,(1-math.cos(da))/da)
            self.matrix=self.matrix@transform((a*dx-b*dy,b*dx+a*dy,da))
        self.counts=counts;self.last_feedback=stamp
        self.append(stamp)

    def append(self, stamp):
        if self.rows and stamp<=self.rows[-1][0]:return
        value=pose(self.matrix)
        if self.rows:value[2]=self.rows[-1][3]+(value[2]-self.rows[-1][3]+math.pi)%math.tau-math.pi
        self.rows.append([stamp,*value]);self.rows=self.rows[-5000:]

    def consume(self, row):
        event=row['event'];stamp=row['host_monotonic_ns']/1e9
        stopped=None
        if event=='stationary_baseline':stopped=row['stop']
        elif event=='hold_ready':stopped=row.get('reply',{}).get('wheels')
        elif event=='paused':stopped=row['stop']
        elif event=='finished':self.finished=True;stopped=row.get('stop')
        elif event=='armed':
            self.session=row['reply']['session'];self.sequence=0;self.active=True;self.stationary=False
            # ARM contains the freshly verified wheel positions. They bridge
            # the first D/ACK transaction without inventing a feedback grace period.
            if 'wheels' in row['reply']:self.sample(row['reply']['wheels'],stamp)
        elif event=='command':
            if self.command is not None:raise ValueError('unpaired command')
            self.command=row
        elif event=='reply':
            command=self.command;self.command=None;r=row['reply']
            if command is None:raise ValueError('unpaired feedback')
            if r.get('type')=='paused':self.active=False;self.stationary=False;return
            if (r.get('type')!='ack' or r.get('fault') is not False or
                r.get('session')!=self.session or r.get('sequence')!=self.sequence+1 or
                r.get('sequence')!=command['sequence'] or r.get('session')!=command['session']):
                raise ValueError('encoder stream session/sequence/fault mismatch')
            self.sequence=r['sequence']
            self.sample(r['wheel_feedback'],(stamp+command['host_monotonic_ns']/1e9)/2)
        if stopped is not None:
            self.active=False
            if not (stopped.get('armed') is False and stopped.get('stop_confirmed') is True and
                    stopped.get('hold_confirmed') is True and stopped.get('fault') is False and
                    all(w['goal_raw']==0 and w['torque_raw']==0 for w in stopped.get('wheels',[]))):
                raise ValueError('stationary baseline not verified')
            self.sample(stopped['wheels'],stamp);self.stationary=True

    def valid(self, now):
        return self.counts is not None and not self.finished and (self.stationary or
            self.active and self.last_feedback is not None and 0<=now-self.last_feedback<=.35)


class LogFollower(TeleopCompletion):
    def __init__(self,path,encoder):
        self.encoder=encoder
        super().__init__(path)

    def consume(self,row):
        self.encoder.consume(row)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--teleop-record',type=Path,required=True)
    p.add_argument('--map',type=Path,required=True)
    p.add_argument('--status',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=600)
    a=p.parse_args()
    if not 5<=a.seconds<=1800:p.error('duration must be 5..1800 seconds')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':p.error('LOCALHOST ROS isolation required')
    a.output.mkdir(parents=True,exist_ok=False);a.status.parent.mkdir(parents=True,exist_ok=True)
    saved=json.loads(a.map.read_text());map_hash=hashlib.sha256(a.map.read_bytes()).hexdigest()
    geometry=json.loads((ROOT/'config/home_assistant/s4_base.nominal.json').read_text())
    encoder=LiveEncoder(geometry);follower=LogFollower(a.teleop_record,encoder)
    from scipy.spatial import cKDTree
    grid=np.array(saved['data']).reshape(saved['height'],saved['width']);yy,xx=np.nonzero(grid>=65)
    tree=cKDTree(np.array(saved['origin'])+(np.column_stack((xx,yy))+.5)*saved['resolution'])
    mount_doc=json.loads((ROOT/'config/home_assistant/lidar_mount.review.json').read_text())
    mount=transform([*mount_doc['base_from_lidar_translation_m'][:2],mount_doc['yaw_rad']])
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped
    from tf2_ros import TransformBroadcaster,Buffer,TransformListener
    from lifecycle_msgs.srv import ChangeState
    from std_srvs.srv import Empty
    from ament_index_python.packages import get_package_prefix
    from home_robot_tasks.navigation_health import NavigationHealth,NavigationHealthProfile
    rclpy.init();node=rclpy.create_node('teleop_localization_observer',namespace='/s4_live_localization')
    health=NavigationHealth(NavigationHealthProfile('lidar_a'),time.monotonic)
    broadcaster=TransformBroadcaster(node);tf=Buffer();listener=TransformListener(tf,node)
    map_pub=node.create_publisher(OccupancyGrid,'map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    initial_pub=node.create_publisher(PoseWithCovarianceStamped,'initialpose',10)
    last_scan=None;last_pose=None;processed_pose=None;initialized=False;proc=None;good=0;bad=0;fit_ready=False;fit=None;display_points=[];error=None
    offset=node.get_clock().now().nanoseconds/1e9-time.monotonic()
    def ros_stamp(stamp):return Time(nanoseconds=round((stamp+offset)*1e9)).to_msg()
    def write_status(ready,message):
        now=time.monotonic()
        data=dict(schema_version=1,teleop_record=str(a.teleop_record),map_sha256=map_hash,
            host_monotonic_s=time.monotonic(),ready=ready,message=message,
            scan_age_s=None if health.scan_stamp is None else now-health.scan_stamp,
            localization_age_s=None if health.localization_stamp is None else now-health.localization_stamp,
            encoder_age_s=None if encoder.last_feedback is None else now-encoder.last_feedback,
            initialized=initialized,fit_fraction=fit,health=health.report(),encoder_ready=encoder.valid(time.monotonic()),pose_xy_yaw=None if last_pose is None else last_pose['xy_yaw'],
            scan_points=display_points,odometry_basis='verified_stop_stationary_assumption' if encoder.stationary else 'measured_encoder_positions',
            autonomous_navigation_authorized=False)
        temporary=a.status.with_suffix('.pending');temporary.write_text(json.dumps(data,allow_nan=False));temporary.replace(a.status)
        with (a.output/'health.jsonl').open('a') as stream:
            stream.write(json.dumps({k:v for k,v in data.items() if k!='scan_points'},allow_nan=False)+'\n')
    def scan_received(m):
        nonlocal last_scan
        observed=m.header.stamp.sec+m.header.stamp.nanosec/1e9-offset
        try:
            health.scan(observed_s=observed,frame=m.header.frame_id,ranges=list(m.ranges),minimum_range_m=m.range_min,maximum_range_m=m.range_max)
        except ValueError:return
        last_scan=dict(observed_s=observed,stamp=m.header.stamp,ranges=list(m.ranges),angle_min=m.angle_min,
            angle_increment=m.angle_increment,time_increment=m.time_increment)
    def pose_received(m):
        nonlocal last_pose
        if m.header.frame_id!='map':return
        q=m.pose.pose.orientation;last_pose=dict(stamp=m.header.stamp,observed_s=m.header.stamp.sec+m.header.stamp.nanosec/1e9-offset,
            xy_yaw=[m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)],covariance=list(m.pose.covariance))
        poses_file.write(json.dumps(last_pose,default=lambda x:dict(sec=x.sec,nanosec=x.nanosec))+'\n');poses_file.flush()
    node.create_subscription(LaserScan,'/lds_commissioning/scan',scan_received,qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped,'amcl_pose',pose_received,10)
    poses_file=(a.output/'poses.jsonl').open('x');log=(a.output/'amcl.log').open('x')
    lifecycle=node.create_client(ChangeState,'amcl/change_state');nomotion=node.create_client(Empty,'request_nomotion_update')
    def wait(predicate,seconds):
        deadline=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>deadline:raise TimeoutError('localization startup timeout')
            rclpy.spin_once(node,timeout_sec=.02)
    def transition(code):
        wait(lifecycle.service_is_ready,10);request=ChangeState.Request();request.transition.id=code
        future=lifecycle.call_async(request);wait(future.done,10)
        if not future.result().success:raise RuntimeError('AMCL lifecycle failed')
    try:
        write_status(False,'라이다 위치 확인 준비 중')
        exe=Path(get_package_prefix('nav2_amcl'))/'lib/nav2_amcl/amcl'
        proc=subprocess.Popen([str(exe),'--ros-args','-r','__ns:=/s4_live_localization','-r','scan:=/lds_commissioning/scan',
            '--params-file',str(ROOT/'ros2_ws/src/home_robot_tasks/config/localization.replay.yaml')],stdout=log,stderr=subprocess.STDOUT)
        transition(1);transition(3)
        msg=OccupancyGrid();msg.header.frame_id='map';msg.info.width=saved['width'];msg.info.height=saved['height'];msg.info.resolution=saved['resolution']
        msg.info.origin.position.x=saved['origin'][0];msg.info.origin.position.y=saved['origin'][1];msg.info.origin.orientation.w=1.;msg.data=saved['data'];map_pub.publish(msg)
        started=time.monotonic();next_force=next_status=0.;future=None;sent=0;last_fit_stamp=None
        while time.monotonic()-started<a.seconds:
            follower.poll();now=time.monotonic()
            if encoder.finished:break
            if proc.poll() is not None:raise RuntimeError('AMCL exited')
            if encoder.stationary:encoder.append(now)
            # Publish actual samples when driving; only stopped state uses a
            # labelled zero-motion prior. No velocity command is ever integrated.
            for row in encoder.rows:
                if row[0]<=sent:continue
                t,x,y,yaw=row;msg=TransformStamped();msg.header.stamp=ros_stamp(t);msg.header.frame_id='odom';msg.child_frame_id='base_link'
                msg.transform.translation.x=x;msg.transform.translation.y=y;msg.transform.rotation.z=math.sin(yaw/2);msg.transform.rotation.w=math.cos(yaw/2)
                broadcaster.sendTransform(msg);sent=t
            rclpy.spin_once(node,timeout_sec=.02)
            if (not initialized and encoder.stationary and last_scan is not None and health.report()['scan_ready'] and
                len(encoder.rows)>2 and last_scan['observed_s']+359*last_scan['time_increment']>=encoder.rows[0][0] and last_scan['observed_s']<=encoder.rows[-1][0]):
                write_status(False,'지도에서 현재 위치를 찾고 있어요')
                seed=map_initial_pose_candidates(last_scan,np.array(encoder.rows),mount,saved)
                (a.output/'initial-hypotheses.json').write_text(json.dumps(seed,indent=2))
                best=seed['candidates'][0];gap=seed['candidates'][1]['clipped_mean_distance_m']-best['clipped_mean_distance_m'] if len(seed['candidates'])>1 else 0
                if best['within_10cm_fraction']<.8 or gap<.02:raise RuntimeError('현재 위치 후보가 모호해요 · 지도와 로봇 위치 확인 필요')
                initial=PoseWithCovarianceStamped();initial.header.frame_id='map';initial.header.stamp=node.get_clock().now().to_msg()
                x,y,yaw=best['xy_yaw'];initial.pose.pose.position.x=x;initial.pose.pose.position.y=y;initial.pose.pose.orientation.z=math.sin(yaw/2);initial.pose.pose.orientation.w=math.cos(yaw/2)
                initial.pose.covariance[0]=initial.pose.covariance[7]=.2**2;initial.pose.covariance[35]=math.radians(15)**2
                initial_pub.publish(initial);initialized=True
            if initialized and now>=next_force and nomotion.service_is_ready() and (future is None or future.done()):
                future=nomotion.call_async(Empty.Request());next_force=now+.2
            if last_pose is not None and last_pose['observed_s']!=processed_pose and tf.can_transform('map','base_link',Time.from_msg(last_pose['stamp'])):
                covariance=np.array(last_pose['covariance']).reshape(6,6)[np.ix_([0,1,5],[0,1,5])]
                try:health.localization(observed_s=last_pose['observed_s'],frame='map',covariance_xy_yaw=covariance,transform_valid=True)
                except ValueError:pass
                processed_pose=last_pose['observed_s']
            if initialized and last_scan is not None and last_scan['observed_s']!=last_fit_stamp and encoder.valid(now):
                t=last_scan['observed_s']
                if (t+359*last_scan['time_increment']>=encoder.rows[0][0] and t<=encoder.rows[-1][0] and
                    tf.can_transform('map','base_link',Time.from_msg(last_scan['stamp']))):
                    tr=tf.lookup_transform('map','base_link',Time.from_msg(last_scan['stamp'])).transform;q=tr.rotation
                    matrix=transform((tr.translation.x,tr.translation.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)))
                    points=scan_points(last_scan,mount,np.array(encoder.rows));points=points@matrix[:2,:2].T+matrix[:2,2]
                    fit=float(np.mean(tree.query(points)[0]<=.1)) if len(points) else 0.
                    display_points=points[::4].tolist();last_fit_stamp=t
                    good=good+1 if fit>=.8 else 0;bad=bad+1 if fit<.8 else 0
                    if good>=3:fit_ready=True
                    if bad>=2:fit_ready=False
            ready=health.ready() and encoder.valid(now) and fit_ready
            if now>=next_status:
                message='위치 확인 완료 · 지도와 로봇 방향을 확인해 주세요' if ready else '위치 갱신 지연 · 지도 표시를 이동 판단에 사용하지 마세요'
                write_status(ready,message);next_status=now+.1
    except (Exception,KeyboardInterrupt) as exc:
        error=str(exc) or 'observer interrupted'
    finally:
        write_status(False,'위치 확인 종료' if not error else '위치 확인 중단 · '+error)
        if proc is not None and proc.poll() is None:
            try:transition(4);transition(2)
            except Exception:pass
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
        poses_file.close();log.close();node.destroy_node();rclpy.try_shutdown()
        (a.output/'report.json').write_text(json.dumps(dict(error=error,map_sha256=map_hash,teleop_record=str(a.teleop_record),
            initialized=initialized,motor_commands=0,finished=encoder.finished,physical_accuracy_verified=False),indent=2))
    return 1 if error else 0


if __name__=='__main__':raise SystemExit(main())
