"""Bottom observation -> checked lift rise -> independent measured planning start.

Camera is fixed to the base. Original pixels, object coordinates and grasp
candidates are immutable. Encoder arrival is not mechanical-stop detection.
"""
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np

from .s2_geometry import digest, JOINTS
from .pick_calibration import TOP_BOARD, RISE_BOARDS, TOP_BOARDS, TOP_MODEL_M, RISE_TOP_BASIS, TOP_BASES, rise_target_um, planning_lift_limit


def require_top_move(board, target_um):
    # 0x2492e / CRC 2696088964 controller-profile/mobile_profile.json:
    # reference_top_um=643847 is a stationary reference, NOT a move limit.
    limits={TOP_BOARD:610000, **{b:rise_target_um(b) for b in RISE_BOARDS}}
    maximum=limits.get(tuple(board))
    if maximum is None or target_um>maximum:
        raise ValueError('현재 펌웨어는 기록된 상단 30,717 카운트까지 이동할 수 없습니다. '
                         '엔코더 행정 기준 펌웨어(CRC 3399362976) 설치가 필요합니다.')


def preflight(prepared_path, output, options, session_factory, board):
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity
    from tools.diagnostics.plan_capture_carry import load_models
    from .carry_model import parameters
    from .carry_scene import apply_boxes, SceneBox
    from .unified_carry_inputs import controller_lift_height_m
    from tools.lib.pick_compute import ROOT
    data=json.loads(prepared_path.read_text())
    profile,geometry,arms,_,boxes,payload,joints=load_models(data,jaw_basis='nominal_stock_tcp_section')
    if data.get('start_revalidation') or abs(joints['s2_lift_joint'])>.002:
        raise ValueError('new bottom observation required before lift planning')
    from .pick_calibration import check_top_surface
    points=data['review']['base_points_m']
    check_top_surface(data['support_plane'],points.get('center_xyz_m',points.get('surface_xyz_m')),profile)
    target_um=rise_target_um(board)
    require_top_move(board,target_um)
    # Include both nominal encoder travel and the field-effective top model.
    heights=np.linspace(joints['s2_lift_joint'],planning_lift_limit(profile),
                        int(np.ceil(planning_lift_limit(profile)/.005))+1)
    state=RobotState();state.joint_state.name=list(joints);state.joint_state.position=list(joints.values())
    config=parameters(ROOT,profile,geometry,arms,Path(options['mobile_mesh_dir']))
    output.mkdir(parents=True,exist_ok=True)
    report=dict(status='CHECKING',calibration_id=profile['calibration_id'],capture_id=data['capture_id'],
        source_sha256=data['source_sha256'],prepared_sha256=digest(data),target_um=target_um,
        start_model_joint_positions_si=joints,checked_samples=0,maximum_sample_step_m=.005,
        continuous_collision_proof=False,environment_coverage='observed_only',mechanical_stop_detected=False)
    try:
        with session_factory(config,state,output/'moveit') as call:
            apply_boxes(call,boxes+[SceneBox('lift_target',payload.base_from_object,payload.size_m,'target',payload.mesh)])
            index=state.joint_state.name.index('s2_lift_joint')
            for height in heights:
                state.joint_state.position[index]=float(height)
                answer=call('check_state_validity',GetStateValidity.Request(robot_state=state))
                report['checked_samples']+=1
                if not answer.valid:
                    raise ValueError('lift ascent collision at '+str(round(height*1000,1))+' mm')
        report['status']='LIFT_PATH_CHECKED'
    except Exception as exc:
        report.update(status='REJECTED',reason=str(exc))
        raise
    finally:
        (output/'report.json').write_text(json.dumps(report,indent=2))
    return dict(report,report=str(output/'report.json'))


def rise_to_top(lift,observe,stop,proof,profile,board,*,check=lambda:None,clock=time.monotonic,sleep=time.sleep):
    from .device_ports import LiftPort
    from .system_stop_backend import SystemStopBackend
    from .platform_preparation import supported_lift_is_holding
    from .unified_carry_inputs import controller_lift_height_m
    target=proof.get('target_um')
    expected=rise_target_um(board)
    if (tuple(board) not in TOP_BOARDS or proof.get('status')!='LIFT_PATH_CHECKED'
            or proof.get('calibration_id')!=profile['calibration_id']
            or type(target)is not int or target!=expected):
        raise ValueError('matching calibrated lift path required')
    require_top_move(board,target)
    check();initial=observe()
    if not supported_lift_is_holding(initial) or abs(initial.height_um)>.002*1e6:
        raise ValueError('measured held bottom origin required')
    port=LiftPort(lift,clock,SystemStopBackend(stop))
    goal='pick-observed-lift-rise'
    port.start(goal,{'height_um':target})
    # Supervised travel has no arbitrary duration cutoff. Fresh feedback,
    # controller faults, target arrival and operator cancellation still apply.
    try:
        while True:
            check();sample=observe()
            if sample.boot_id!=initial.boot_id or sample.fault:
                raise ValueError('lift boot changed or fault during rise')
            result=port.poll(goal)
            if result.state=='SUCCEEDED':
                final=observe()
                if (final.boot_id!=initial.boot_id or not supported_lift_is_holding(final)
                        or final.target_um!=target or abs(final.height_um-target)>2000):
                    raise ValueError('lift arrival differs from requested top')
                return dict(initial=initial,final=final)
            if result.state in ('FAILED','CANCELLED') or result.reason:
                raise RuntimeError('lift rise failed: '+result.reason)
            sleep(.02)
    except BaseException as exc:
        # Keep the original movement failure even if STOP's ACK is also lost.
        try:stop.request()
        except Exception as error:exc.add_note('stop request failed: '+str(error))
        raise


def validate_revalidation(record,profile,capture_id,source_sha256):
    from .pick_capture import make_evidence
    from .carry_live_inputs import JointExposure
    resumed=record.get('kind')=='resumed_bottom_observation_measured_top'
    if resumed:
        prior=record.get('source_revalidation',{})
        if record.get('operator_scene_unchanged') is not True or prior.get('kind')!='observed_bottom_measured_top':
            raise ValueError('explicit unchanged scene and original bottom-to-top record required')
        validate_revalidation(prior,profile,capture_id,source_sha256)
        from .pick_calibration import RESUME_HEIGHT_BASIS
        reference=record.get('arrival_evidence',{}).get('platform',{}).get('top_reference',{})
        if reference.get('basis')==RESUME_HEIGHT_BASIS and digest(reference.get('source_platform'))!=digest(prior['arrival_evidence']['platform']):
            raise ValueError('resumed height source differs from original lift arrival')
        if record.get('observation_time_s')!=prior['observation_time_s']:
            raise ValueError('original observation timestamp changed')
    if ((not resumed and record.get('kind')!='observed_bottom_measured_top') or record.get('capture_id')!=capture_id
            or record.get('source_sha256')!=source_sha256
            or not (record.get('target_unchanged') is True or
                    record.get('target_assumption')=='fixed_bottom_observation')
            or record.get('lift_reference_basis') not in (TOP_BASES if resumed else (RISE_TOP_BASIS,))):
        raise ValueError('invalid bottom-to-top observation binding')
    evidence=record['arrival_evidence']
    rebuilt=make_evidence(evidence['camera'],[JointExposure(**j) for j in evidence['stationary_window']],
        evidence['platform'],profile,now=evidence['captured_check_s'],requested_s=evidence['requested_s'],
        session_id=evidence['session_id'],anchor=evidence['held_anchor_rad'])
    if (any(digest(v)!=digest(evidence[k]) for k,v in rebuilt.items())
            or evidence['session_id']!=record['session_id']
            or evidence['controller_boot_id']!=record['controller_boot_id']
            or evidence['camera']['color_time_s']<=record['observation_time_s']
            or evidence['model_joint_positions_si']!=record['arrival_model_joint_positions_si']
            or evidence['lift_reference_basis']!=record['lift_reference_basis']):
        raise ValueError('arrival evidence differs from measured planning start')
    return evidence


def planning_review(prepared,profile):
    original=prepared['review']
    record=prepared.get('start_revalidation')
    if record is None:return original
    evidence=validate_revalidation(record,profile,prepared['capture_id'],prepared['source_sha256'])
    source=record['source_revalidation'] if record['kind']=='resumed_bottom_observation_measured_top' else record
    if source['session_id']!=original['session_id'] or abs(original['lift_height_m'])>.002:
        raise ValueError('planning start must follow same-session bottom observation')
    result=deepcopy(original)
    result.update(lift_height_m=evidence['fixed_lift_m'],
        model_joint_positions_si=evidence['model_joint_positions_si'],lift_reference_basis=record['lift_reference_basis'])
    for side in ('left','right'):
        result[side+'_anchor_rad']={n:result['model_joint_positions_si'][side+'_model_'+n] for n in JOINTS[:5]}
    return result


def bind_arrival(prepared,snapshot,detections,arrival,proof,output):
    from tools.diagnostics.plan_capture_carry import load_models
    from .pick_capture import review_capture
    data=json.loads(prepared.read_text())
    profile,*_=load_models(data,jaw_basis='nominal_stock_tcp_section')
    if proof['status']!='LIFT_PATH_CHECKED' or proof['prepared_sha256']!=digest(data):
        raise ValueError('lift proof belongs to another observation')
    old=json.loads((snapshot/'robot_capture.json').read_text())
    new=json.loads((arrival/'robot_capture.json').read_text())
    review_capture(snapshot,old,profile);review_capture(arrival,new,profile)
    if (old['session_id']!=new['session_id'] or old['controller_boot_id']!=new['controller_boot_id']
            or new['platform']['lift']['target_um']!=proof['target_um']):
        raise ValueError('controller session changed between observation and arrival')
    # Supervised fixed-scene operation: retain the bottom RGB-D geometry.
    # Arrival frames supply measured planning-start evidence only. Do not turn
    # load-induced image shifts into target motion, or claim visual verification.
    record=dict(kind='observed_bottom_measured_top',capture_id=data['capture_id'],source_sha256=data['source_sha256'],
        session_id=old['session_id'],controller_boot_id=old['controller_boot_id'],
        observation_time_s=old['camera']['color_time_s'],target_unchanged=None,
        target_assumption='fixed_bottom_observation',target_check=None,
        lift_reference_basis=RISE_TOP_BASIS,arrival_evidence=new,
        arrival_model_joint_positions_si=new['model_joint_positions_si'])
    data['start_revalidation']=record
    planning_review(data,profile)
    output.write_text(json.dumps(data,indent=2,allow_nan=False))
    return str(output)


def bind_resume(prepared,arrival,output,*,scene_unchanged=False):
    from tools.diagnostics.plan_capture_carry import load_models
    from .pick_capture import review_capture
    if scene_unchanged is not True:raise ValueError('unchanged scene confirmation required')
    data=json.loads(Path(prepared).read_text())
    profile,*_=load_models(data,jaw_basis='nominal_stock_tcp_section')
    planning_review(data,profile)
    prior=data['start_revalidation']
    if prior['kind']!='observed_bottom_measured_top':
        raise ValueError('original bottom-to-top observation required')
    new=json.loads((Path(arrival)/'robot_capture.json').read_text())
    review_capture(Path(arrival),new,profile)
    data['start_revalidation']=dict(kind='resumed_bottom_observation_measured_top',
        capture_id=data['capture_id'],source_sha256=data['source_sha256'],
        session_id=new['session_id'],controller_boot_id=new['controller_boot_id'],
        observation_time_s=prior['observation_time_s'],target_unchanged=None,
        target_assumption='fixed_bottom_observation',operator_scene_unchanged=True,
        source_revalidation=prior,lift_reference_basis=new['lift_reference_basis'],
        arrival_evidence=new,arrival_model_joint_positions_si=new['model_joint_positions_si'])
    planning_review(data,profile)
    Path(output).write_text(json.dumps(data,indent=2,allow_nan=False))
    return str(output)
