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
    previous=None;ratios={i:[] for i in (7,8,9)};travel={i:[0,0] for i in ratios}
    for record in records[1:]:
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
