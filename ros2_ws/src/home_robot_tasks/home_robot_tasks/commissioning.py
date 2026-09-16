"""Validate explicit board profiles and write reviewable, hashed deployment inputs.

This never flashes, connects a device, sets a mode, or promotes synthetic data to
measured data. Real limits remain operator measurements; generation is not approval.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from .fetch import InvalidTask

U=(1,0x7fffffff);I=(0,0x7fffffff);D=(-1,1)
SCHEDULE={k:U for k in ('arm_period_us','arm_reserved_us','guard_us','wheels_period_us','lift_period_us','feedback_period_us','maximum_poll_gap_us')}
AXIS=dict(velocity_direction=D,minimum_voltage_raw=(1,255),maximum_voltage_raw=(1,255),maximum_temperature=(1,100),maximum_current_raw=(1,32767))
SCHEMA=dict(devices=dict(model=[(1,65534)]*4,baud_code=(0,7),timeout_ms=U),
    mobile=dict(max_velocity_raw=[(1,32767)]*4,stopped_velocity_raw=I,command_timeout_ms=(1,1000),feedback_timeout_ms=(1,1000),lift_min_um=I,lift_max_um=U),
    feedback=dict(axes=[AXIS]*4,sample_timeout_ms=(1,1000),mode_timeout_ms=U,response_timeout_ms=U,maximum_sample_skew_ms=U),
    output=dict(schedule=SCHEDULE,wheels_budget_us=U,lift_budget_us=U,stop_budget_us=U,job_lifetime_us=U,velocity_direction=[D]*4),
    lift={**{k:U for k in ('um_per_turn','maximum_height_um','tolerance_um','maximum_speed_raw','homing_speed_raw','slowdown_distance_um','home_current_ma','maximum_current_ma','feedback_timeout_ms','homing_timeout_ms','contact_dwell_ms','maximum_encoder_step_raw')},'stopped_speed_raw':I,'encoder_up_direction':D},
    stop=dict(job_lifetime_us=U,zero_budget_us=U,hold_budget_us=U,timeout_ms=U,feedback_max_age_ms=(1,1000)),
    read_period_us=U,read_budget_us=U,quiet_us=U,current_microamps_per_raw=(1,1000000),
    arm_hold_tolerance_raw=(1,2047),arm_hold_dwell_ms=U,arm_read_period_ms=U,initialize_erased_boot_storage=bool)


def _validate(value,schema,path):
    if isinstance(schema,dict):
        if not isinstance(value,dict) or set(value)!=set(schema):raise InvalidTask('profile fields mismatch: '+path)
        for k,v in schema.items():_validate(value[k],v,path+'.'+k)
    elif isinstance(schema,list):
        if not isinstance(value,list) or len(value)!=len(schema):raise InvalidTask('profile vector mismatch: '+path)
        for i,(v,s) in enumerate(zip(value,schema)):_validate(v,s,path+f'[{i}]')
    elif schema is bool:
        if type(value) is not bool:raise InvalidTask('profile boolean required: '+path)
    elif type(value) is not int or not schema[0]<=value<=schema[1] or (schema==D and value==0):
        raise InvalidTask('profile integer out of range: '+path)


def validate_board_profile(document):
    if (not isinstance(document,dict) or set(document)!={'schema_version','mode','board','measurement_records'}
            or type(document['schema_version']) is not int or document['schema_version']!=1
            or document['mode'] not in ('simulation','measured')):
        raise InvalidTask('explicit versioned board profile required')
    _validate(document['board'],SCHEMA,'board')
    p=document['board'];s=p['output']['schedule'];l=p['lift'];m=p['mobile'];f=p['feedback'];stop=p['stop']
    if (m['lift_min_um']!=0 or m['lift_max_um']!=l['maximum_height_um']
            or l['tolerance_um']>=l['slowdown_distance_um'] or l['slowdown_distance_um']>=l['maximum_height_um']
            or l['stopped_speed_raw']>=l['homing_speed_raw'] or l['homing_speed_raw']>l['maximum_speed_raw']
            or l['maximum_speed_raw']>m['max_velocity_raw'][3] or l['home_current_ma']>=l['maximum_current_ma']
            or l['maximum_encoder_step_raw']>=2048 or l['encoder_up_direction']==0
            or l['contact_dwell_ms']>=l['homing_timeout_ms']
            or p['arm_hold_dwell_ms']>=stop['timeout_ms'] or p['arm_read_period_ms']*6>=stop['feedback_max_age_ms']
            or p['arm_read_period_ms']*1000<2*p['read_period_us']
            or stop['zero_budget_us']!=p['output']['stop_budget_us']
            or stop['job_lifetime_us']<=max(stop['zero_budget_us'],stop['hold_budget_us'])
            or s['arm_period_us']!=5000 or s['arm_reserved_us']+s['guard_us']>=s['arm_period_us']
            or any(v<=m['stopped_velocity_raw'] for v in m['max_velocity_raw'])
            or p['initialize_erased_boot_storage']):
        raise InvalidTask('inconsistent limits/timing or persistent boot reset request')
    for i,a in enumerate(f['axes']):
        if a['velocity_direction']!=p['output']['velocity_direction'][i] or a['minimum_voltage_raw']>=a['maximum_voltage_raw']:
            raise InvalidTask('RX/TX direction or voltage mismatch')
    if document['mode']=='measured':
        required={'motor_identity','joint_limits','bus_timing','lift_geometry','lift_hold','camera_extrinsics'}
        if not isinstance(document['measurement_records'],dict) or set(document['measurement_records'])!=required:
            raise InvalidTask('all commissioning measurement records required')
        for record in document['measurement_records'].values():
            if not isinstance(record,dict) or set(record)!={'path','sha256'}:raise InvalidTask('hashed measurement record required')
            path=Path(record['path'])
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=record['sha256']:
                raise InvalidTask('measurement record missing or changed')
    elif document['measurement_records']!={}:raise InvalidTask('synthetic profile cannot claim measurement records')
    return deepcopy(document)


def _initializer(value):
    if isinstance(value,dict):return '{'+','.join('.'+k+'='+_initializer(v) for k,v in value.items())+'}'
    if isinstance(value,list):return '{'+','.join(_initializer(v) for v in value)+'}'
    if type(value) is bool:return 'true' if value else 'false'
    return str(value)


def export_profile(document, destination):
    profile=validate_board_profile(document)
    destination=Path(destination)
    if destination.exists():raise InvalidTask('use a new immutable profile output directory')
    canonical=json.dumps(profile,sort_keys=True,separators=(',',':')).encode()
    sha=hashlib.sha256(canonical).hexdigest()
    header='/* Generated profile '+sha+'; not a hardware qualification. */\n'
    if profile['mode']=='simulation':
        header+='#ifndef MOBILE_PROFILE_OFFLINE_TEST\n#error "Synthetic profile: offline tests only"\n#endif\n'
    header+='static const MobileBoardProfile commissioned_mobile_profile = '+_initializer(profile['board'])+';\n'
    destination.mkdir(parents=True)
    (destination/'mobile_profile.json').write_bytes(canonical+b'\n')
    (destination/'mobile_profile.h').write_text(header)
    files={n:hashlib.sha256((destination/n).read_bytes()).hexdigest() for n in ('mobile_profile.json','mobile_profile.h')}
    manifest=dict(schema_version=1,mode=profile['mode'],profile_sha256=sha,files=files,hardware_authorized=False,
                  boot_initialization=False,automatic_homing=False)
    (destination/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest
