#!/usr/bin/env python3
"""Jetson stationary pick: check is read-only; run requires an actual-height bundle."""
import argparse
from dataclasses import asdict
from datetime import datetime,timezone
import json
from pathlib import Path
import signal
import sys
import time
import uuid
import threading
from collections import deque
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks.carry_bundle import load_bundle,sha,write_json,validate_installed_binding
from home_robot_tasks.carry_execution import build_carry_session
from home_robot_tasks.carry_live_inputs import CameraWorker,FixedPlatformGuard,LiveCarryInputs,MeasuredFK,ObservationNotFresh,ResidentObservationReader,joint_exposure,match_exposure
from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.snapshot_io import hashes
from home_robot_tasks.s2_rgbd_acquisition import Capture
from home_robot_tasks.camera_evidence import near_depth_profile
from so101_arm_bridge.backend_lease import acquire_backend_lease
from so101_arm_bridge.serial_port import open_exclusive_serial
from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2
from so101_arm_bridge.stationary_pick_adapter import StationaryPickAdapter
from so101_arm_bridge.stationary_runtime import ResidentStop,verify_inventory
from so101_arm_bridge.bimanual_stream_adapter import OperationalLimits,CANONICAL_JOINT_NAMES
from so101_arm_bridge.development_ranges import read_development_ranges,calibration_hash

DEVICE='/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_005100393235511438363730-if02'
CONFIG=ROOT/'config/home_assistant'
OBSERVATION_POLICY='snapshot_then_verify'
def read(name):return json.loads((CONFIG/name).read_text())


def stop_idle_reads(transport):
    """A cleanup rejection must not replace the original observation failure."""
    original=sys.exc_info()[1]
    try:transport.stationary_platform(2)
    except Exception as cleanup:
        if original is None:raise
        diagnostics=dict(getattr(original,'diagnostics',{}))
        diagnostics['idle_reader_cleanup_error']=str(cleanup)
        try:diagnostics['firmware_runtime']=transport.stationary_runtime()
        except Exception as fault:diagnostics['firmware_runtime_error']=str(fault)
        try:diagnostics['feedback_failures']=transport.stationary_feedback_failures()
        except Exception as fault:diagnostics['feedback_failures_error']=str(fault)
        original.diagnostics=diagnostics


def identity(transport):
    hello=transport.enter_binary_mode()
    _,_,checksum=read_development_ranges(CONFIG/'arm_ranges.development.json')
    if (hello.firmware_version!=StationaryPickAdapter.firmware_version or hello.protocol_version!=2
            or hello.joint_count!=12 or hello.left_calibration_hash!=calibration_hash(checksum)
            or hello.right_calibration_hash!=calibration_hash(checksum) or hello.stop_latched):
        raise ValueError('stationary controller identity/state mismatch: '+str(hello))
    return asdict(hello)


def camera_profile():
    return near_depth_profile(read('s2_coordinates.development.json'),read('camera_device.observed.json'),read('camera_device.near.observed.json'))


def installed_bundle(folder):
    result=load_bundle(folder,for_execution=True)
    validate_installed_binding(result[1],camera_profile(),sha(CONFIG/'arm_ranges.development.json'))
    return result


def check(transport,seconds):
    report=dict(controller=identity(transport),motor_write_commands=0,motion_authorized=False)
    platform=transport.stationary_platform(0)
    report['platform_reading_active']=platform['active']
    count=0;maximum_age=0.
    with Capture(camera_profile(),read('camera_device.near.observed.json'),retain_frame=True) as camera:
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            record,images=camera.next_frame();count+=1
            maximum_age=max(maximum_age,time.monotonic()-min(record['color_time_s'],record['depth_time_s']))
        report.update(camera=camera.device,frames=count,maximum_exposure_age_s=maximum_age,
                      color_shape=list(images[0].shape),depth_shape=list(images[1].shape))
    report.update(status='READ_ONLY_CONNECTION_CHECK_PASSED',motor_response_test='NOT_RUN_POWER_OFF_CHECK')
    return report


def capture(transport,args):
    """A new RGB-D planning capture on this same resident, without torque writes."""
    if args.fixed_lift_m is None:raise ValueError('established fixed lift height required')
    from PIL import Image
    profile=camera_profile()
    if not 0<=args.fixed_lift_m<=profile['lift']['travel_m']:raise ValueError('lift outside model')
    hello=identity(transport);inventory=verify_inventory(transport,read('arm_ranges.development.json'))
    guard=FixedPlatformGuard(args.fixed_lift_m,profile['lift']['meters_per_count'])
    frames=deque(maxlen=3);history=deque(maxlen=100);lock=threading.Lock()
    def consume(record,images):
        with lock:frames.append((record,images))
    transport.stationary_platform(1)
    try:
        factory=lambda:Capture(profile,read('camera_device.near.observed.json'),retain_frame=True)
        with CameraWorker(factory,consume) as worker:
            deadline=time.monotonic()+5;chosen=None
            while time.monotonic()<deadline:
                worker.check()
                begin=time.monotonic();packet=transport.get_feedback_snapshot();end=time.monotonic()
                try:j=joint_exposure(packet,begin,end)
                except ValueError:time.sleep(.01);continue
                history.append(j)
                begin=time.monotonic();platform=transport.stationary_platform();end=time.monotonic()
                guard.update(platform,begin,end)
                with lock:available=list(frames)
                if len(history)<10 or history[-1].lower_s-history[0].upper_s<.3:continue
                positions=np.asarray([v.positions_rad for v in history])
                if np.max(np.ptp(positions,axis=0))>.01:raise ValueError('arm moved during stationary capture')
                for record,images in reversed(available):
                    try:matched=match_exposure(history,record['color_time_s'],record['depth_time_s'],time.monotonic())
                    except ValueError:continue
                    chosen=(record,images,matched);break
                if chosen:break
                time.sleep(.01)
            if chosen is None:raise ValueError('no fresh synchronized stationary capture')
        record,images,j=chosen
        folder=args.output.parent/args.output.stem/record['capture_id'];folder.mkdir(parents=True,exist_ok=False)
        Image.fromarray(images[0][:,:,::-1]).save(folder/'color.png')
        Image.fromarray(images[1]).save(folder/'depth_aligned.png');write_json(folder/'metadata.json',images[2])
        model={'s2_lift_joint':args.fixed_lift_m}
        for side,offset in (('left',0),('right',6)):
            for i,name in enumerate(geo.JOINTS):model[side+'_model_'+name]=geo.model_from_control(j.positions_rad[offset+i],profile['arms'][side]['alignment'][name])
        return dict(kind='resident_fixed_capture',schema_version=1,status='stationary_capture_verified',
            capture_id=record['capture_id'],snapshot_directory=str(folder.relative_to(args.output.parent)),
            source_sha256=hashes(folder),calibration_id=profile['calibration_id'],
            session_id=str(uuid.uuid4()),clock_id='host-monotonic-'+str(uuid.uuid4()),
            camera=record,controller=hello,initial_inventory=inventory,
            joint_exposure=asdict(j),stationary_window=[asdict(v) for v in history],
            platform=platform,fixed_lift_m=args.fixed_lift_m,
            lift_reference_basis='operator_established_height_continuous_stationary_encoder_guard',
            model_joint_positions_si=model,motor_write_commands=0,motion_authorized=False)
    finally:stop_idle_reads(transport)


def run(transport,args):
    # All admission files are checked before inventory, prepare or ARM.
    p,model,proof,reference,mask,sweep,metadata=installed_bundle(args.bundle)
    pause_before_descent=getattr(args,'pause_before_descent',False)
    if pause_before_descent and model.get('approach_mode')!='vertical':
        raise ValueError('height checkpoint requires an explicitly checked vertical descent bundle')
    if not args.execute or not args.supported_fixed_platform or args.fixed_lift_m is None:
        raise ValueError('explicit execute, supported fixed platform and established lift height required')
    if abs(args.fixed_lift_m-p.planned_lift_m)>.003:raise ValueError('actual fixed lift differs from planned height')
    identity(transport)
    inventory=verify_inventory(transport,read('arm_ranges.development.json'))
    _,entries,checksum=read_development_ranges(CONFIG/'arm_ranges.development.json')
    limits=OperationalLimits(CANONICAL_JOINT_NAMES,tuple(e['minimum_urad'] for e in entries),
        tuple(e['maximum_urad'] for e in entries),checksum,hardware_authorized=True)
    adapter=StationaryPickAdapter(transport,CONFIG/'arm_ranges.development.json',authorized_limits=limits,motion_authorized=True)
    stop=ResidentStop(transport);session=None;cancel=False;report=None;guard=None;resume_descent=False
    def interrupted(*_):
        nonlocal cancel
        cancel=True
    previous={s:signal.signal(s,interrupted) for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    if pause_before_descent:
        def confirmed(*_):
            nonlocal resume_descent
            if session is not None and session.state=='AWAITING_DESCENT_CONFIRMATION':resume_descent=True
        previous[signal.SIGUSR1]=signal.signal(signal.SIGUSR1,confirmed)
    log=[]
    try:
        adapter.prepare()
        transport.stationary_platform(1)
        guard=FixedPlatformGuard(args.fixed_lift_m,model['profile']['lift']['meters_per_count'])
        inputs=LiveCarryInputs(p,MeasuredFK(model['fk']),reference,mask,metadata['aligned_depth_intrinsics'],
            model['profile']['base_from_optical'],guard,error_budget_m=model['error_budget_m'],robot_sweep_mask=sweep,
            maximum_background_depth_m=model.get('scene_guard',{}).get('maximum_background_depth_m'),
            observation_policy=OBSERVATION_POLICY)
        camera=lambda:Capture(model['profile'],read('camera_device.near.observed.json'),retain_frame=True)
        reader=ResidentObservationReader(transport,adapter,inputs)
        with CameraWorker(camera,inputs.add_frame) as worker:
            deadline=time.monotonic()+4
            while True:
                worker.check()
                if cancel:raise ValueError('cancelled before motion')
                try:reader.read();break
                except ValueError:
                    if time.monotonic()>deadline:raise
                time.sleep(.01)
            if np.max(abs(np.asarray(inputs.feedback().positions_rad)-p.stages[0].points[0].positions_rad))>.02:
                raise ValueError('current arm anchor differs from checked plan')
            adapter.activate_stationary_hold()
            # PREPARE resets the firmware snapshot counter. Start a new measured
            # history explicitly; never relabel old camera/encoder samples.
            inputs.reset_unarmed_history()
            deadline=time.monotonic()+1.
            while True:
                worker.check()
                if cancel:raise ValueError('cancelled during stationary setup')
                try:reader.read();break
                except ValueError:
                    if time.monotonic()>deadline:raise
                time.sleep(.01)
            session=build_carry_session(p,adapter=adapter,coordinated_stop=stop,observe=reader.read,
                clock=time.monotonic,validate_current_scene=inputs.validate_stage,
                observation_policy=OBSERVATION_POLICY,ready_for_route=inputs.route_start_ready,
                pause_before_descent=pause_before_descent)
            session.start('stationary-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
            inputs.admit_snapshot()
            # A fixed-camera target will be hidden by the approaching arm. Stop
            # acquiring after admission, never make its pixels a motor interlock.
            # Joining happens in __exit__; the serial loop keeps servicing its
            # own measured feedback and heartbeat while camera shutdown finishes.
            worker.stop.set()
            print('촬영 확인 완료 → 접근 → 닫기 → 2cm 들기 → 운반 자세. 집기 성공은 마지막에 별도 확인합니다.',flush=True)
            print('유지 중 Ctrl+C는 양팔 토크를 끕니다. 팔과 물체를 받친 뒤 종료하세요.',flush=True)
            started=time.monotonic();last_state=None
            while session.state not in ('FAILED','CANCELLED'):
                name=p.stages[min(session.index,len(p.stages)-1)].name
                try:
                    inputs.set_phase(name)
                    if cancel:session.cancel()
                    elif resume_descent:
                        session.resume_descent();resume_descent=False
                    result=session.poll()
                except Exception as exc:session._stop(str(exc));result=session.poll()
                state=(session.state,session.index,session.reason)
                if state!=last_state:
                    event=dict(elapsed_s=time.monotonic()-started,state=session.state,stage=name,
                               reason=session.holding_reason if session.state=='HOLDING' else session.reason)
                    log.append(event);print(json.dumps(event,ensure_ascii=False),flush=True);last_state=state
                    if session.state=='HOLDING':print('운반 자세 유지 중 · 실제 물체를 집고 있는지는 확인이 필요합니다.',flush=True)
                    if session.state=='AWAITING_DESCENT_CONFIRMATION':
                        print('목표 위 준비 위치 유지 중 · 손끝과 탁자의 실제 간격 확인 전에는 하강하지 않습니다.',flush=True)
                if session.state=='STOPPING':
                    until=time.monotonic()+3
                    while session.state=='STOPPING' and time.monotonic()<until:session.poll();time.sleep(.02)
                    break
                time.sleep(.005)
            report=dict(status=session.state,reason=session.reason,history=session.history,events=log,
                physical_task_completed=False,holding_is_not_task_completion=True,stop_status=stop.status,
                observation_policy=OBSERVATION_POLICY,
                grasp_verification=('PENDING_POST_PICK_CHECK' if any(s['stage']=='hold' for s in session.history) else 'NOT_REACHED'),
                scene_admission=inputs.validation,scene_admitted_s=inputs.scene_time,
                runtime_diagnostics=adapter.last_runtime_diagnostics,
                adapter_fault=adapter.fault_diagnostic,
                start_timing=getattr(adapter,'last_start_timing',None),
                firmware_runtime=adapter.last_firmware_runtime,
                motion_entries=session.motion.entries,
                deferred_observations=reader.deferred,last_platform_query=guard.last_query,
                initial_inventory=inventory,capture_id=p.capture_id,candidate_id=p.candidate_id)
            report['height_checkpoint']=dict(requested=pause_before_descent,
                confirmed=session.descent_released,hold_cycles=session.pregrasp_hold_cycle)
            return report
    except Exception as exc:
        # Setup failures also need the first cause and the result of cleanup.
        # The finally block completes this same report after verifying STOP.
        report=dict(status='REJECTED',reason=str(exc),failure_phase='preparation' if session is None else 'execution',
                    physical_task_completed=False,history=[] if session is None else session.history,
                    last_platform_query=None if guard is None else guard.last_query,
                    adapter_fault=adapter.fault_diagnostic,capture_id=p.capture_id,candidate_id=p.candidate_id)
        return report
    finally:
        # A crash/exception after ARM still requests STOP; no automatic rearm or opening.
        if adapter.heartbeat_required or adapter.fault_diagnostic or (session and session.state not in ('IDLE','FAILED','CANCELLED')):
            stop.request('cleanup',True,time.monotonic())
            until=time.monotonic()+3
            while time.monotonic()<until and stop.poll('cleanup',time.monotonic()).status!='STOPPED':time.sleep(.05)
            if stop.status!='STOPPED':print('정지 미확인: 양팔·리프트를 받치고 모터 전원을 꺼주세요.',flush=True)
        else:
            try:transport.stationary_platform(2)
            except Exception:pass
        if report is not None:
            report['start_timing']=getattr(adapter,'last_start_timing',None)
            report['stop_status']=stop.status if stop.requested else 'NOT_REQUESTED'
            report['stop_diagnostics']=stop.diagnostics()
            report['serial_write_diagnostics']=transport.write_diagnostics()
            try:report['firmware_runtime']=transport.stationary_runtime()
            except Exception as exc:report['firmware_runtime_error']=str(exc)
            try:report['feedback_failures']=transport.stationary_feedback_failures()
            except Exception as exc:report['feedback_failures_error']=str(exc)
        for s,handler in previous.items():signal.signal(s,handler)


def observe_bundle(transport,args):
    """Exercise live admission/tracking without preparing or enabling motors."""
    p,model,_,reference,mask,sweep,metadata=installed_bundle(args.bundle)
    if args.fixed_lift_m is None or abs(args.fixed_lift_m-p.planned_lift_m)>.003:
        raise ValueError('established actual lift height must match bundle')
    identity(transport)
    verify_inventory(transport,read('arm_ranges.development.json'))
    guard=FixedPlatformGuard(args.fixed_lift_m,model['profile']['lift']['meters_per_count'])
    inputs=LiveCarryInputs(p,MeasuredFK(model['fk']),reference,mask,metadata['aligned_depth_intrinsics'],
        model['profile']['base_from_optical'],guard,error_budget_m=model['error_budget_m'],robot_sweep_mask=sweep,
        maximum_background_depth_m=model.get('scene_guard',{}).get('maximum_background_depth_m'),
        observation_policy=OBSERVATION_POLICY)
    transport.stationary_platform(1)
    try:
        camera=lambda:Capture(model['profile'],read('camera_device.near.observed.json'),retain_frame=True)
        with CameraWorker(camera,inputs.add_frame) as worker:
            deadline=time.monotonic()+4;started=None;count=0;maximum_delta=0.;last_platform=-1.
            while started is None or time.monotonic()-started<args.seconds:
                worker.check()
                begin=time.monotonic();packet=transport.get_feedback_snapshot();end=time.monotonic()
                try:inputs.add_joints(packet,begin,end)
                except ObservationNotFresh:
                    # Continue only while the previous accepted exposure is
                    # still fresh; feedback() below never renews its timestamp.
                    pass
                except ValueError:
                    if started is not None or time.monotonic()>deadline:raise
                    time.sleep(.01);continue
                if end-last_platform>=.03:
                    begin=time.monotonic();packet=transport.stationary_platform();end=time.monotonic()
                    try:inputs.add_platform(packet,begin,end)
                    except ObservationNotFresh:pass
                    last_platform=end
                if inputs.failure:raise inputs.failure
                try:feedback=inputs.feedback()
                except ValueError:
                    if started is not None or time.monotonic()>deadline:raise
                    time.sleep(.01);continue
                delta=float(np.max(abs(np.asarray(feedback.positions_rad)-p.stages[0].points[0].positions_rad)))
                if delta>.02:raise ValueError('current arm anchor differs from checked plan')
                maximum_delta=max(maximum_delta,delta);count+=1
                if started is None:started=time.monotonic()
                time.sleep(.005)
        return dict(status='LIVE_ADMISSION_PASSED',motor_write_commands=0,motion_authorized=False,
                    duration_s=time.monotonic()-started,feedback_samples=count,camera_frames=worker.frames,
                    maximum_start_joint_delta_rad=maximum_delta,scene_validation=inputs.validation,
                    scene_guard=model.get('scene_guard'),capture_id=p.capture_id,candidate_id=p.candidate_id)
    finally:stop_idle_reads(transport)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('check','inspect-bundle','capture','observe-bundle','run'))
    parser.add_argument('--device',default=DEVICE);parser.add_argument('--bundle',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=2.)
    parser.add_argument('--fixed-lift-m',type=float)
    parser.add_argument('--supported-fixed-platform',action='store_true')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--pause-before-descent',action='store_true',
                        help='hold checked pregrasp until SIGUSR1 after operator height check; cancel still stops')
    args=parser.parse_args()
    if not .1<=args.seconds<=10:parser.error('check duration must be 0.1..10 seconds')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    if args.output.exists():parser.error('output already exists')
    result=None
    try:
        if args.command=='inspect-bundle':
            if not args.bundle:raise ValueError('bundle required')
            p,*_=load_bundle(args.bundle)
            result=dict(status='BUNDLE_VERIFIED',hypothetical_lift=p.hypothetical_lift,hardware_commands=0)
        else:
            if args.command=='run':
                if not args.bundle or not args.execute:raise ValueError('bundle and explicit --execute required')
                installed_bundle(args.bundle)  # Reject stale plans/previews before opening serial.
            elif args.command=='observe-bundle':
                if not args.bundle:raise ValueError('bundle required')
                installed_bundle(args.bundle)
            import serial
            with acquire_backend_lease('stm32',87),open_exclusive_serial(serial,args.device,921600,.02) as port:
                transport=StreamValidationTransportV2(port,response_timeout_s=.12)
                result=(check(transport,args.seconds) if args.command=='check' else
                        capture(transport,args) if args.command=='capture' else
                        observe_bundle(transport,args) if args.command=='observe-bundle' else run(transport,args))
        print(json.dumps(result,ensure_ascii=False,default=str))
    except Exception as exc:
        result=dict(status='REJECTED',reason=str(exc))
        if hasattr(exc,'diagnostics'):result['diagnostics']=exc.diagnostics
        print(json.dumps(result,ensure_ascii=False))
    finally:
        if result is not None:write_json(args.output,result)
    return 0 if result['status'] in ('BUNDLE_VERIFIED','READ_ONLY_CONNECTION_CHECK_PASSED','stationary_capture_verified','LIVE_ADMISSION_PASSED','CANCELLED') else 2


if __name__=='__main__':raise SystemExit(main())
