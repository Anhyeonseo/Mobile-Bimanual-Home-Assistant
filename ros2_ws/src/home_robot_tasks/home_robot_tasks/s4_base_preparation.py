"""Review geometry from existing CAD and teleop conventions, never an enable profile."""
import hashlib
import json
import math
import statistics
from pathlib import Path

import numpy as np
from . import s2_geometry, s2_development
from .base_motion import OmniKinematics, Wheel


def prepare(root):
    root=Path(root)
    sources=('config/home_assistant/official_geometry.json',
             'config/home_assistant/s2_coordinates.development.json',
             'tools/diagnostics/s2_viewer/meshes.json',
             'tools/run/pulse_mobile_base.py')
    geometry=s2_geometry.load_geometry(root/sources[0])
    profile=s2_development.released_profile(root,geometry)
    surfaces=json.loads((root/sources[2]).read_text())
    if surfaces['geometry_sha256']!=s2_geometry.digest(geometry):
        raise ValueError('wheel surface and CAD revisions differ')
    base=s2_geometry.nominal_base_from_cad(geometry)
    joints={j['name']:j for j in geometry['mobile_reference_joints']}
    wheels=[]
    # Existing teleop: 7 left, 8 right, 9 front; +raw clockwise viewed outside.
    for motor, name, surface_index in ((7,'wheel2',1),(8,'wheel1',0),(9,'wheel3',2)):
        joint=joints[name];pose=base@s2_geometry.origin(joint)
        axis=pose[:3,:3]@np.array([float(v) for v in joint['axis']['xyz'].split()])
        axis/=np.linalg.norm(axis)
        if np.dot(axis[:2],pose[:2,3])<=0:raise ValueError('expected outward CAD wheel axis')
        rolling=-np.cross(axis,[0,0,1])
        radius=s2_geometry.origin(joint)[2,3]-surfaces['wheel_contact_z_m'][surface_index]
        wheels.append(dict(motor_id=motor,cad_joint=name,x_m=float(pose[0,3]),y_m=float(pose[1,3]),
            rolling_angle_rad=math.atan2(rolling[1],rolling[0]),radius_m=float(radius)))
    model=OmniKinematics(tuple(Wheel(**{k:w[k] for k in ('x_m','y_m','rolling_angle_rad','radius_m')}) for w in wheels))
    return dict(schema_version=1,mode='nominal_review_only',hardware_enabled=False,motion_authorized=False,
        source_sha256={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in sources},
        base_frame='base_footprint',wheel_order=[7,8,9],wheels=wheels,
        unit_twist_wheel_rates={name:model.wheel_rates(twist) for name,twist in
            [('forward',(1,0,0)),('left',(0,1,0)),('counterclockwise',(0,0,1))]},
        radius_basis='original full STL wheel contact at nominal CAD axle poses, not the measured floor-to-chassis correction',
        nominal_vs_referenced_floor_difference_m=float(-profile['ground_z_m']-wheels[0]['radius_m']),
        raw_per_rad_s=None,command_gain=None,braking_m_s2=None,
        pending=['verify actual wheel/ID direction','encoder speed-unit fit under movement',
                 'command versus measured speed gain','lidar/localization displacement versus wheel odometry',
                 'floor braking and slip','shared controller bus timing and arm/lift hold'])


def review_capture(records):
    """Fit reported speed units against encoder motion; never command/floor gain."""
    if not records or 'inventory' not in records[0]:raise ValueError('initial inventory required')
    inventory=records[0]['inventory']
    if (inventory.get('all_reads_ok') is not True or inventory.get('motor_write_commands')!=0
            or inventory.get('info',{}).get('motion_enabled') is not False
            or inventory.get('info',{}).get('wheel_samples_supported') is not True):
        raise ValueError('read-only identified wheel capture required')
    return _review_samples(records[1:])


def _review_samples(records, boundaries=()):
    previous=None;ratios={i:[] for i in (7,8,9)};travel={i:[0,0] for i in ratios}
    for index_record,record in enumerate(records):
        if index_record in boundaries:previous=None
        if type(record.get('sequence')) is not int or not 0<=record['sequence']<2**32:
            raise ValueError('invalid sequence')
        if previous and (record['sequence']-previous['sequence'])%2**32!=1:
            raise ValueError('capture restarted or lost a sample')
        if not 0<=record['host_received_s']-record['host_started_s']<=.2:
            raise ValueError('late host acquisition')
        if previous and not 0<record['host_started_s']-previous['host_started_s']<=.2:
            raise ValueError('host acquisition gap or clock reset')
        rows=record['wheels']
        if [r['id'] for r in rows]!=[7,8,9]:raise ValueError('wheel order changed')
        for index,row in enumerate(rows):
            if (row['ok'] is not True or row['hal_status']!=0
                    or any(type(row[k]) is not int for k in ('position_raw','speed_raw','started_tick_ms','ended_tick_ms'))
                    or not 0<=row['started_tick_ms']<2**32 or not 0<=row['ended_tick_ms']<2**32
                    or not 0<=row['position_raw']<4096 or not 0<=row['speed_raw']<=65535
                    or (row['ended_tick_ms']-row['started_tick_ms'])%2**32>50):
                raise ValueError('invalid wheel feedback')
            if previous:
                old=previous['wheels'][index]
                dt=((row['started_tick_ms']-old['started_tick_ms'])%2**32)/1000
                if not .001<=dt<=.2:raise ValueError('wheel sample gap or reset')
                delta=(row['position_raw']-old['position_raw']+2048)%4096-2048
                if abs(delta)>=1024:raise ValueError('ambiguous encoder step')
                signed=lambda raw: -(raw&32767) if raw&32768 else raw
                speed=(signed(row['speed_raw'])+signed(old['speed_raw']))/2
                if abs(delta)>=4 and abs(speed)>=4:
                    ratios[row['id']].append(speed*dt*4096/(delta*2*math.pi))
                    travel[row['id']][int(delta>0)]+=abs(delta)
        previous=record
    wheels=[]
    for motor,values in ratios.items():
        enough=len(values)>=8 and min(travel[motor])>=128
        center=statistics.median(values) if values else None
        relative_mad=(statistics.median(abs(v-center) for v in values)/abs(center)
                      if center else None)
        qualified=enough and relative_mad is not None and relative_mad<.15 and all(v*center>0 for v in values)
        wheels.append(dict(motor_id=motor,usable_intervals=len(values),encoder_travel_counts=travel[motor],
            candidate_available=qualified,raw_per_encoder_rad_s=abs(center) if qualified else None,
            reported_speed_sign_vs_encoder=(1 if center>0 else -1) if qualified else None,
            relative_median_deviation=relative_mad,
            reason='candidate requires physical review' if qualified else 'insufficient bidirectional movement or inconsistent speed'))
    return dict(mode='encoder_feedback_scale_review',hardware_enabled=False,motion_authorized=False,
        command_speed_gain_qualified=False,floor_travel_scale_qualified=False,
        wheels=wheels,all_candidates_available=all(w['candidate_available'] for w in wheels))


def review_teleop_capture(records):
    """Reuse encoder fitting with explicitly identified, commanded-drive evidence.

    Paused epochs are separated; never disguise powered teleop as a read-only
    motor capture or bridge a missing command/ACK with fabricated samples.
    """
    if not records or records[0].get('event')!='session':
        raise ValueError('teleop session header required')
    info=records[0].get('firmware',{})
    if info.get('firmware')!='mobile-base-hold-teleop-v7' or info.get('wheel_feedback') is not True:
        raise ValueError('identified live-feedback teleop required')
    samples=[];boundaries=[];pending=None;session=None;sequence=0;last_time=-1
    counts={k:0 for k in 'FBLRADZ'};seen=set();finished=False;pauses=0
    for record in records:
        stamp=record.get('host_monotonic_ns')
        if type(stamp) is not int or stamp<last_time:
            raise ValueError('host timestamp regression')
        last_time=stamp;event=record.get('event')
        if finished:raise ValueError('records after finished event')
        if event=='stationary_baseline':
            stopped=record.get('stop',{})
            wheels=stopped.get('wheels',[])
            if (session is not None or pending is not None or samples or not _confirmed_stop(stopped)
                    or [w.get('id') for w in wheels]!=[7,8,9]
                    or any(w.get('goal_raw')!=0 or w.get('torque_raw')!=0 for w in wheels)):
                raise ValueError('invalid stationary baseline')
        elif event=='localization_changed':
            # Advisory observations never consume an ACK or establish a stop.
            if (record.get('policy')!='manual_advisory_after_start'
                    or type(record.get('localization',{}).get('ready')) is not bool):
                raise ValueError('invalid localization annotation')
        elif event=='armed':
            reply=record['reply'];nonce=reply.get('session')
            if (session is not None or pending is not None or nonce in seen or type(nonce) is not int
                    or nonce<=0 or reply.get('sequence')!=0 or reply.get('armed') is not True
                    or reply.get('fault') is not False or reply.get('hold_confirmed') is not True):
                raise ValueError('invalid armed epoch')
            session=nonce;sequence=0;seen.add(nonce);boundaries.append(len(samples))
        elif event=='command':
            if (pending is not None or session is None or record.get('session')!=session
                    or record.get('sequence')!=sequence+1 or record.get('motion') not in counts):
                raise ValueError('unpaired command or sequence gap')
            pending=record;sequence+=1;counts[record['motion']]+=1
        elif event=='reply':
            reply=record['reply']
            if pending is None:raise ValueError('reply without command')
            if reply.get('type')=='paused':
                if not _confirmed_stop(reply):raise ValueError('unconfirmed controller pause')
                pending=None
                continue
            if (reply.get('type')!='ack' or reply.get('session')!=session or reply.get('sequence')!=sequence
                    or reply.get('armed') is not True or reply.get('fault') is not False
                    or reply.get('hold_confirmed') is not True):
                raise ValueError('rejected or mismatched drive reply')
            wheels=reply.get('wheel_feedback')
            if not isinstance(wheels,list) or len(wheels)!=3:raise ValueError('missing wheel feedback')
            samples.append(dict(sequence=sequence,
                host_started_s=pending['host_monotonic_ns']/1e9,host_received_s=stamp/1e9,
                wheels=[dict(id=w['id'],ok=True,hal_status=0,position_raw=w['position_raw'],
                    speed_raw=w['speed_raw'],started_tick_ms=w['tick_begin_ms'],ended_tick_ms=w['tick_end_ms']) for w in wheels]))
            pending=None
        elif event=='paused':
            if pending is not None or not _confirmed_stop(record.get('stop',{})):
                raise ValueError('unconfirmed pause')
            session=None;pauses+=1
        elif event=='finished':
            if pending is not None or record.get('error') is not None or not _confirmed_stop(record.get('stop') or {}):
                raise ValueError('incomplete drive or final stop')
            finished=True
        elif event not in ('session','hold_requested','hold_ready','hold_checked','arm_requested','browser_expired'):
            raise ValueError('unknown session event')
    if not finished or not samples:raise ValueError('complete drive recording required')
    result=_review_samples(samples,set(boundaries))
    result.update(source_mode='powered_manual_teleop',command_counts=counts,acknowledged_samples=len(samples),
        verified_pauses=pauses,final_stop_confirmed=True,
        missing_directions=[k for k in 'FBLRAD' if not counts[k]])
    return result


def _confirmed_stop(reply):
    return (reply.get('armed') is False and reply.get('fault') is False
            and reply.get('stop_confirmed') is True and reply.get('hold_confirmed') is True)


def teleop_speed_windows(records, poses, *, time_offset_s=0.):
    """Match acknowledged steady commands to independent, masked LiDAR poses.

    Interrupted runs contribute their complete ACK intervals. Never cross a
    pause, missing ACK, session change or scan gap. Ignore transition settling,
    then fit 2–3 s SE(2) displacements rather than noisy per-frame derivatives.
    """
    poses=np.asarray(poses,dtype=float)
    if (poses.ndim!=2 or poses.shape[1]!=4 or len(poses)<2 or
            not np.isfinite(poses).all() or np.any(np.diff(poses[:,0])<=0)):
        raise ValueError('ordered finite independent lidar poses required')
    if (not records or records[0].get('event')!='session' or
            records[0].get('firmware',{}).get('firmware')!='mobile-base-hold-teleop-v7' or
            records[0]['firmware'].get('max_raw')!=200 or not math.isfinite(time_offset_s)):
        raise ValueError('identified raw-200 teleop recording required')
    pending=None;run=[];runs=[];last=-1
    def boundary():
        nonlocal run
        if run:runs.append(run)
        run=[]
    for row in records:
        stamp=row.get('host_monotonic_ns')
        if type(stamp) is not int or stamp<last:raise ValueError('teleop clock regression')
        last=stamp
        if row['event']=='command':
            if pending is not None:boundary()
            pending=row
        elif row['event']=='reply':
            reply=row['reply'];command=pending;pending=None
            valid=(command is not None and reply.get('type')=='ack' and
                   reply.get('sequence')==command.get('sequence') and reply.get('session')==command.get('session') and
                   reply.get('armed') is True and reply.get('fault') is False and reply.get('hold_confirmed') is True)
            if not valid:boundary();continue
            at=(stamp+command['host_monotonic_ns'])/2e9+time_offset_s
            if not 0<=(stamp-command['host_monotonic_ns'])/1e9<=.2:boundary();continue
            key=(command.get('motion'),command.get('session'))
            if key[0] not in 'FBLRADZ':raise ValueError('unknown teleop motion')
            if run and (key!=run[-1][1] or not 0<at-run[-1][0]<=.2 or command['sequence']!=run[-1][2]+1):boundary()
            run.append((at,key,command['sequence']))
        elif row['event'] in ('paused','finished','armed','browser_expired'):
            boundary();pending=None
    boundary();windows=[]
    unwrapped=poses.copy();unwrapped[:,3]=np.unwrap(unwrapped[:,3])
    for run in runs:
        start,end=run[0][0]+.7,run[-1][0]-.3
        if end-start<2 or run[0][1][0]=='Z':continue
        for at in np.arange(start,end-1.99,3.):
            finish=min(at+3.,end)
            if at<poses[0,0] or finish>poses[-1,0]:continue
            lo=max(0,np.searchsorted(poses[:,0],at)-1)
            hi=min(len(poses),np.searchsorted(poses[:,0],finish)+1)
            if np.max(np.diff(poses[lo:hi,0]))>.35:continue
            a,b=np.array([[np.interp(t,unwrapped[:,0],unwrapped[:,i]) for i in range(1,4)] for t in (at,finish)])
            yaw=b[2]-a[2];c,s=math.cos(a[2]),math.sin(a[2]);dx,dy=b[:2]-a[:2]
            aa,bb=(math.sin(yaw)/yaw,(1-math.cos(yaw))/yaw) if abs(yaw)>1e-8 else (1.,0.)
            velocity=np.linalg.solve([[aa,-bb],[bb,aa]],[c*dx+s*dy,-s*dx+c*dy])/(finish-at)
            windows.append(dict(motion=run[0][1][0],start_s=float(at),end_s=float(finish),
                                twist=[*velocity,float(yaw/(finish-at))]))
    return windows


def summarize_command_speed(windows, patterns):
    expected={'F':[-200,200,0],'B':[200,-200,0],'L':[-100,-100,200],
              'R':[100,100,-200],'A':[200,200,200],'D':[-200,-200,-200]}
    if {v[0]:list(v[1]) for v in patterns.values()}!=expected:
        raise ValueError('existing three-wheel teleop pattern mismatch')
    directions={};axes=[]
    for axis,pair in enumerate(('FB','LR','AD')):
        centers=[]
        for motion,sign in zip(pair,(1,-1)):
            rows=np.asarray([w['twist'] for w in windows if w['motion']==motion])
            if rows.ndim!=2 or len(rows)<3 or not np.isfinite(rows).all():
                raise ValueError('insufficient complete steady intervals: '+motion)
            speed=rows[:,axis]*sign;center=float(np.median(speed))
            if center<=0:raise ValueError('recorded motion direction disagrees: '+motion)
            directions[motion]=dict(windows=len(rows),median_twist=np.median(rows,axis=0).tolist(),
                signed_axis_speed=center,axis_speed_q10_q90=np.quantile(speed,[.1,.9]).tolist(),
                relative_median_deviation=float(np.median(abs(speed-center))/center))
            centers.append(center)
        axes.append(float(np.mean(centers)))  # balance both directions, not sample counts
    return dict(schema_version=1,mode='recorded_teleop_command_candidate',hardware_enabled=False,
        motion_authorized=False,physical_validation_pending=True,wheel_order=[7,8,9],reference_raw=200,
        maximum_raw=200,patterns={k:expected[k] for k in ('F','L','A')},axis_speed_at_reference=axes,
        directions=directions,windows=windows,feedback_scale_changed=False,
        limitations=['Independent masked LiDAR estimate; not surveyed ground truth.',
                    'Single output amplitude; low-output deadband and combined motions require live validation.',
                    'Command model only; never replace measured feedback with command prediction.'])


def review_lidar_command_scale(campaign, patterns):
    """Reuse the completed lidar-only preparation and all original teleop epochs."""
    campaign=Path(campaign);report=json.loads((campaign/'report.json').read_text())
    if (report.get('passed') is not True or report.get('lidar_only') is not True or
            report.get('wheel_feedback_used') is not False or not report.get('visibility',{}).get('masked_ray_indices')):
        raise ValueError('independent body-shadow-masked lidar campaign required')
    hashes={}
    for name in ('report.json','trajectories.npz'):
        path=campaign/name;hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        if name in report.get('artifact_sha256',{}) and hashes[str(path)]!=report['artifact_sha256'][name]:
            raise ValueError('lidar trajectory digest mismatch')
    with np.load(campaign/'trajectories.npz',allow_pickle=False) as data:poses=data['encoder']
    windows=[]
    for epoch in report['epochs']:
        lo,hi=epoch['replay_interval_s'];selected=poses[(poses[:,0]>=lo)&(poses[:,0]<=hi)]
        for path in sorted((campaign.parent/epoch['segment']/'teleop').glob('*.jsonl')):
            hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
            rows=[json.loads(line) for line in path.read_text().splitlines() if line.strip()]
            windows.extend(dict(w,segment=epoch['segment']) for w in teleop_speed_windows(rows,selected,time_offset_s=epoch['time_offset_s']))
    result=summarize_command_speed(windows,patterns)
    result.update(source_sha256=hashes,trajectory_basis=report['trajectory_basis'],
                  visibility=report['visibility']['invalid_ray_policy'],hardware_commands=0)
    return result
