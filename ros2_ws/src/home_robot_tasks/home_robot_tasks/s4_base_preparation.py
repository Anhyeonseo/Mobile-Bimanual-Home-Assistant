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
