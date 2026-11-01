"""Compile five measured poses into a measured, immutable filming profile.

These bounds describe taught joint paths, not a collision-free workspace.
No old operational envelope or synthetic right-arm samples are reused.
"""
import hashlib
import json
from datetime import datetime,timezone

STAGES=('home','approach','grasp','closed','lift')
LABELS=('기본 자세·집게 열림','물체 위·집게 열림','집을 위치·집게 열림','팔 그대로·집게 닫힘','들어 올린 자세·닫힘')
RAW_TOLERANCE=12
POSE_TOLERANCE=20
MARGIN_RAW=256
URAD_PER_RAW=6283185/4096

def fingerprint(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def offsets(rows):
    if [r.get('id') for r in rows]!=list(range(1,7)):raise ValueError('왼팔 ID 1~6이 모두 필요합니다.')
    result=[]
    for row in rows:
        data=bytes.fromhex(row['eeprom_hex'])
        if len(data)!=40 or data[5]!=row['id'] or data[33]!=0:raise ValueError('모터 설정을 확인할 수 없습니다.')
        result.append(int.from_bytes(data[31:33],'little'))
    return result

def new_record(calibration):
    if calibration.get('status')!='centered_verified' or len(calibration.get('after',[]))!=6:
        raise ValueError('성공한 영점 보정 기록이 필요합니다.')
    return dict(schema_version=1,record_kind='left_five_pose_capture',arm='left',
        calibration_sha256=fingerprint(calibration),offsets_raw=offsets(calibration['after']),
        poses={},motion_commands_sent=False)

def validate_record(record,expected=None,*,complete=False):
    if type(record) is not dict or record.get('schema_version')!=1 or record.get('record_kind')!='left_five_pose_capture' or record.get('arm')!='left' or record.get('motion_commands_sent') is not False:
        raise ValueError('새 영점 기준의 촬영 자세 파일이 아닙니다.')
    if expected is not None:
        for key in ('calibration_sha256','offsets_raw'):
            if record.get(key)!=expected[key]:raise ValueError('영점 보정 기록이 달라졌습니다. 새 파일로 저장하세요.')
    extra=record.get('gripper_close_extra_raw',0)
    if type(extra) is not int or not 0<=extra<=32:
        raise ValueError('집게 추가 닫힘은 0~32칸이어야 합니다.')
    values=record.get('offsets_raw')
    if not isinstance(values,list) or len(values)!=6 or any(type(x)is not int or not 0<=x<=65535 for x in values):raise ValueError('잘못된 영점 설정입니다.')
    sha=record.get('calibration_sha256')
    if not isinstance(sha,str) or len(sha)!=64 or any(c not in '0123456789abcdef' for c in sha):raise ValueError('영점 식별값이 없습니다.')
    poses=record.get('poses')
    if not isinstance(poses,dict) or set(poses)-set(STAGES):raise ValueError('알 수 없는 자세 이름입니다.')
    if complete and set(poses)!=set(STAGES):raise ValueError('빠진 자세: '+', '.join(s for s in STAGES if s not in poses))
    for name,p in poses.items():
        samples=p.get('samples_raw')
        if p.get('source')!='maintenance_read_only_three_scans' or not isinstance(samples,list) or len(samples)!=3 or not isinstance(p.get('captured_at'),str):raise ValueError('실측 3회 기록이 필요합니다.')
        for row in samples:
            if not isinstance(row,list) or len(row)!=6 or any(type(x)is not int or not 0<=x<=4095 for x in row):raise ValueError('유효하지 않은 관절 측정값입니다.')
        if any(max(row[j] for row in samples)-min(row[j] for row in samples)>RAW_TOLERANCE for j in range(6)):raise ValueError(name+': 저장 중 자세가 움직였습니다.')
        if p.get('positions_raw')!=samples[-1]:raise ValueError('저장 자세와 실측 기록이 다릅니다.')
    return record

def capture(link,record,*,sleep):
    samples=[]
    for i in range(3):
        rows=link.exchange('SCAN',scan=True)
        from calibrate_left_home import validate
        validate(rows)
        if offsets(rows)!=record['offsets_raw']:raise ValueError('모터 영점이 바뀌었습니다. 저장을 중단합니다.')
        samples.append([r['position'] for r in rows])
        if i<2:sleep(.08)
    result=dict(source='maintenance_read_only_three_scans',samples_raw=samples,
                positions_raw=samples[-1],captured_at=datetime.now(timezone.utc).isoformat())
    probe=dict(record,poses={'home':result});validate_record(probe)
    return result

def compile_profile(record):
    validate_record(record,complete=True)
    q={s:list(record['poses'][s]['positions_raw']) for s in STAGES}
    if any(abs(q['grasp'][j]-q['closed'][j])>POSE_TOLERANCE for j in range(5)):
        raise ValueError('closed는 grasp의 팔 자세를 유지하고 집게만 닫아 저장하세요.')
    for a,b in (('approach','home'),('grasp','home'),('lift','closed')):
        if abs(q[a][5]-q[b][5])>POSE_TOLERANCE:raise ValueError(a+': 집게 열림/닫힘을 앞 자세와 같게 유지하세요.')
    jaw=q['closed'][5]-q['grasp'][5]
    if abs(jaw)<33 or abs(jaw)>=2048:raise ValueError('집게 열림·닫힘 값이 너무 가깝거나 회전 경계가 불명확합니다.')
    # Exact arm hold while closing; exact jaw hold while approaching/lifting.
    q['approach'][5]=q['grasp'][5]=q['home'][5]
    q['closed'][:5]=q['grasp'][:5]
    q['lift'][5]=q['closed'][5]
    jaw=q['closed'][5]-q['grasp'][5]
    if abs(jaw)<33 or abs(jaw)>=2048:raise ValueError('기본 자세의 집게 열림을 grasp와 같게 맞춰 다시 저장하세요.')
    # Keep measured scans intact; the approved contact preload is a goal offset.
    extra=record.get('gripper_close_extra_raw',0)
    q['closed'][5]+=(1 if jaw>0 else -1)*extra
    q['lift'][5]=q['closed'][5]
    if not 0<=q['closed'][5]<=4095:
        raise ValueError('집게 추가 닫힘 목표가 센서 범위를 벗어납니다.')
    directions=[1,1,-1,-1,1,-1 if jaw>0 else 1]
    bounds=[]
    for j in range(6):
        values=[q[s][j] for s in STAGES]
        lo=max(0,min(values)-MARGIN_RAW);hi=min(4095,max(values)+MARGIN_RAW)
        if max(values)-min(values)>=2048:
            raise ValueError(f'{j+1}번 관절이 회전 경계를 넘거나 이동 범위가 큽니다. 자세를 확인하세요.')
        u=sorted(round(directions[j]*(raw-2048)*URAD_PER_RAW) for raw in (lo,hi))
        bounds.append(dict(minimum_raw=lo,maximum_raw=hi,minimum_urad=u[0],maximum_urad=u[1]))
    targets=[[round(directions[j]*(q[s][j]-2048)*URAD_PER_RAW) for j in range(6)] for s in STAGES]
    profile=dict(schema_version=1,record_kind='left_pose_locked_profile',capture_sha256=fingerprint(record),
        calibration_sha256=record['calibration_sha256'],offsets_raw=record['offsets_raw'],
        stage_order=list(STAGES),targets_raw=[q[s] for s in STAGES],targets_urad=targets,
        directions=directions,bounds=bounds,range_margin_raw=MARGIN_RAW,start_policy="measured_pose_to_home_within_bounds",allowed_edges=[[0,1],[1,2],[2,3],[3,4]],
        max_joint_speed_rad_s=.20,max_joint_accel_rad_s2=.40,
        collision_checked=False,physical_replay_verified=False)
    if extra:
        profile['gripper_close_extra_raw']=extra
    profile['profile_sha256']=fingerprint(profile)
    profile['firmware_calibration_hash']=int(profile['profile_sha256'][:8],16) or 1
    return profile


def gripper_tolerance_urad(profile):
    gap=abs(profile['targets_urad'][3][5]-profile['targets_urad'][2][5])
    extra=profile.get('gripper_close_extra_raw',0)
    contact=round((extra+RAW_TOLERANCE)*URAD_PER_RAW) if extra else 0
    # Permit the contact preload plus measured jitter, but never an open jaw.
    return min(150000,gap//2,max(12000,gap//4,contact))
