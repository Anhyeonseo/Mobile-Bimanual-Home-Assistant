#!/usr/bin/env python3
"""Supported stationary grasp on the integrated controller; no base commands.

Prepare/reference/rise once, retain the same owner while native geometry is
checked, then accept one checked bundle. STOP retains load; no reset or release.
The original captured pixels are revalidated immediately before arm motion.
"""
import argparse
from collections import deque
from contextlib import ExitStack
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import multiprocessing
import os
from copy import deepcopy
from dataclasses import asdict, replace
import json
from pathlib import Path
import signal
import sys
import threading
import time
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'tools/run'),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
import numpy as np
import cv2
import traceback
import serial
from PIL import Image
import prepare_unified_controller as prep
from home_robot_tasks.platform_preparation import (
    measured_hold_source, supported_lift_is_holding, move_observed_lift,
    reference_supported_bottom,
)
from home_robot_tasks.carry_bundle import load_bundle,sha,write_json,validate_installed_binding
from home_robot_tasks.carry_execution import build_carry_session
from home_robot_tasks.carry_live_inputs import CameraWorker,LiveCarryInputs,MeasuredFK
from home_robot_tasks.camera_evidence import load_capture_profile
from home_robot_tasks.s2_rgbd_acquisition import Capture
from home_robot_tasks.stationary_rgbd import HEIGHT_DEPTH_EXPOSURE_US
from home_robot_tasks.system_stop_backend import SystemStopBackend
from home_robot_tasks.serviced_work import await_serviced
from home_robot_tasks.unified_carry_inputs import UnifiedPlatformGuard,UnifiedObservationReader
from home_robot_tasks.empty_return import load_empty_return,run_empty_return
from so101_arm_bridge.unified_arm_adapter import UnifiedArmAdapter
from so101_arm_bridge.unified_commissioning import CommissionClient
from so101_arm_bridge.backend_lease import acquire_backend_lease
from so101_arm_bridge.serial_port import open_exclusive_serial
from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2,MobileV2Exchange
from so101_arm_bridge.mobile_client import MobileClient
from so101_arm_bridge.lift_wire import LiftClient
from so101_arm_bridge.robot_evidence import RobotEvidenceClient
from so101_arm_bridge.system_stop import SystemStopClient

CONFIG=ROOT/'config/home_assistant'
OWNER='stationary-pick'

def run(args):
    # Workers must close even if startup, bundle decoding, or port acquisition
    # fails before the controller's own STOP/finally scope has been entered.
    with ExitStack() as workers:
        return _run(args, workers)


def _run(args, workers):
    if not args.execute or not args.supported_empty_bottom or not 0<=args.lift_target_um<=10000:
        raise ValueError('supported empty bottom and explicit bounded lift execution required')
    if args.empty_return and args.lift_target_um!=0:raise ValueError('empty return requires fixed bottom lift')
    cv2.setNumThreads(1)
    out=args.output;out.mkdir(parents=True,exist_ok=False)
    report=dict(status='PREPARING',motion_scope='stationary lift and checked left grasp only',
                base_motion_commands=0,physical_grasp_verified=False,events=[])
    files=workers.enter_context(ThreadPoolExecutor(max_workers=2,thread_name_prefix='pick-artifacts'))
    decoding=workers.enter_context(ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn')))
    decoding.submit(os.getpid).result()  # Start before serial ownership or torque; never fork the port.
    saving=None
    def write_state(value):
        write_json(out/'state.tmp',value);(out/'state.tmp').replace(out/'state.json')
    def save():
        nonlocal saving
        if saving is not None and not saving.done():return
        if saving is not None:saving.result()
        saving=files.submit(write_state,deepcopy(report))
    def event(name,**values):
        report.update(status=name,**values);report['events'].append(dict(state=name,host_s=time.monotonic()))
        save();print(json.dumps(dict(status=name,**values),ensure_ascii=False),flush=True)
    cancel=threading.Event();latest=deque(maxlen=1);window=deque(maxlen=12);inputs=None;lock=threading.Lock();capture_device=None
    for sig in (signal.SIGTERM,signal.SIGINT,signal.SIGHUP):signal.signal(sig,lambda *_:cancel.set())
    profile,reference=load_capture_profile(ROOT,'wide')
    # Reject obsolete geometry BEFORE acquiring hardware or enabling hold.
    # Decode once while unarmed; the later request binds this exact bundle.
    checked_folder=args.bundle.resolve()
    try:
        if args.empty_return:
            checked_sha=sha(checked_folder)
            checked_return,return_points=decoding.submit(load_empty_return,checked_folder).result()
            validate_installed_binding(checked_return['model'],profile,sha(CONFIG/'arm_ranges.development.json'))
        else:
            checked_sha=sha(checked_folder/'bundle.json')
            checked_bundle=decoding.submit(load_bundle,checked_folder,for_execution=True).result()
            validate_installed_binding(checked_bundle[1],profile,sha(CONFIG/'arm_ranges.development.json'))
    except Exception as error:
        report.update(status='REJECTED_BEFORE_CONTROL',error=str(error))
        write_state(report)
        print(json.dumps({'status':report['status'],'error':str(error)}),flush=True)
        return 2
    def consume(record,images):
        with lock:
            latest.append((record,images))
            if not window or record['color_time_s']-window[-1][0]['color_time_s']>=.5:
                window.append((record,images,capture_device.native_depth_images[record['capture_id']]))
        if inputs is not None:inputs.add_frame(record,images)
    def factory():
        nonlocal capture_device
        capture_device=Capture(profile,reference,retain_frame=True,retain_native_depth=True,depth_exposure_us=HEIGHT_DEPTH_EXPOSURE_US)
        return capture_device
    # Camera warmup is before any activation/heartbeat lease.
    with acquire_backend_lease('stm32',189), ExitStack() as resources:
      with open_exclusive_serial(serial,args.port,921600,.02) as port:
        t=StreamValidationTransportV2(port,response_timeout_s=3)
        stop=None;session=None;adapter=None;guard=None
        try:
            setup={};report['commission']=setup
            t.enter_binary_mode();initial=CommissionClient(t).query()
            if initial.flags not in (0,17) or initial.reason:raise ValueError('unpowered fresh or already-prepared board required')
            prep.inspect(t,baseline=json.loads(args.baseline.read_text()),prepare=initial.flags==0,supported=True,empty=True,report=setup)
            camera=resources.enter_context(CameraWorker(factory,consume))
            boot=setup['commission_final']['boot_id'];report['commission']=setup
            preview=UnifiedArmAdapter(t,CONFIG/'arm_ranges.development.json')
            adapter=UnifiedArmAdapter(t,CONFIG/'arm_ranges.development.json',
                authorized_limits=replace(preview.motion_limits,hardware_authorized=True),motion_authorized=True)
            anchor=adapter.prepare()
            mobile=MobileClient(MobileV2Exchange(t),lambda:int(time.monotonic()*1000));mobile.synchronize()
            if mobile.boot_id!=boot:raise ValueError('boot changed')
            lift=LiftClient(mobile);evidence=RobotEvidenceClient(t,mobile,time.monotonic,maximum_age_ms=250)
            stop=SystemStopClient(t,boot,time.monotonic)
            coordinated=SystemStopBackend(stop,SimpleNamespace(owner=OWNER))
            report['activation']=asdict(CommissionClient(t).activate_zero(supported=True,empty_grippers=True))
            adapter.start_current_pose_hold(OWNER);anchor=adapter.prepared_state
            deadline=time.monotonic()+4
            while adapter.state.value!='ready':
                if cancel.is_set() or time.monotonic()>deadline:raise RuntimeError('hold cancelled/timed out')
                adapter.poll(OWNER);time.sleep(.01)
            report['commanded_hold_anchor']=asdict(anchor)
            def observe():
                if cancel.is_set():raise RuntimeError('operator cancelled')
                camera.check();adapter.keepalive()
                sent=time.monotonic();snapshot=adapter.feedback_snapshot();received=time.monotonic()
                measured_hold_source(snapshot,anchor.positions_urad,adapter.motion_limits,sent,received)
                evidence.publish_mcu_snapshot(snapshot,sent,received,arms_safe=True,lift_hold=False,payload_safe=True)
                ls=lift.status()
                report['latest']=dict(host_s=received,joints=asdict(snapshot),lift=asdict(ls),boot_id=boot)
                if ls.fault:raise RuntimeError('lift feedback fault')
                return ls
            reference_supported_bottom(observe,mobile,lift,clock=time.monotonic,sleep=time.sleep)
            if args.lift_target_um:
                event('LIFT_RISING',lift_target_um=args.lift_target_um)
                report['lift_rise']=move_observed_lift(lift,observe,stop,args.lift_target_um)
            else:
                report['lift_rise']={'executed':False,'basis':'operator_confirmed_bottom_and_fresh_reference'}
            actual=observe().height_um/1e6
            waiting_state='WAITING_CHECKED_BUNDLE'
            if args.empty_return:
                if actual!=checked_return['planned_lift_m']:raise ValueError('return height mismatch')
                guard=UnifiedPlatformGuard(boot,actual)
                def service_return():
                    if cancel.is_set():raise RuntimeError('operator cancelled')
                    adapter.keepalive()
                    sent=time.monotonic();snapshot=adapter.feedback_snapshot();received=time.monotonic()
                    if any(not lo<=q<=hi for q,lo,hi in zip(snapshot.positions_urad,
                        adapter.motion_limits.minimum_urad,adapter.motion_limits.maximum_urad,strict=True)):
                        raise ValueError('return measured joints outside installed envelope')
                    evidence.publish_mcu_snapshot(snapshot,sent,received,arms_safe=True,lift_hold=False,payload_safe=True)
                    sent=time.monotonic();ms=mobile.status();ls=lift.status();received=time.monotonic()
                    guard.update(ms,ls,sent,received);guard.require_fresh(received)
                    report['latest']=dict(host_s=received,joints=asdict(snapshot),lift=asdict(ls),boot_id=boot)
                event('EMPTY_RETURN_RUNNING',return_sha256=checked_sha,actual_lift_m=actual,
                      motion_scope='open empty jaw, rise and return left arm; base/right arm/lift fixed')
                run_empty_return(adapter,return_points,owner=OWNER,service=service_return,
                    clock=time.monotonic,sleep=time.sleep,cancelled=cancel.is_set)
                anchor=adapter.prepared_state
                report['returned_anchor']=asdict(anchor)
                report['return_completed']=True
                waiting_state='EMPTY_RETURNED'
            event(waiting_state,actual_lift_m=actual)
            # Hold and observe while the local native planner validates actual height.
            last_save=0.;captured=False;capture_after=time.monotonic()+.5
            while not (out/'execute.json').exists():
                ls=observe()
                if not supported_lift_is_holding(ls):raise RuntimeError('lift hold lost while planning')
                if not captured:
                    with lock:
                        selected=[v for v in window if v[0]['color_time_s']>=capture_after]
                        frame=latest[-1] if len(selected)>=8 else None
                    if frame and 0<=time.monotonic()-min(frame[0]['color_time_s'],frame[0]['depth_time_s'])<.2:
                        record,images=frame;color,depth,md=images
                        captured_measurement=deepcopy(report['latest'])
                        def store_capture():
                            snap=out/'snapshot';snap.mkdir()
                            Image.fromarray(color[:,:,::-1]).save(snap/'color.png')
                            Image.fromarray(depth).save(snap/'depth_aligned.png')
                            write_json(snap/'metadata.json',md)
                            write_json(snap/'capture.json',dict(camera=record,measured=captured_measurement))
                            archive=out/'height-window';archive.mkdir()
                            records=[]
                            for i,(rec,ims,native) in enumerate(selected):
                                path=archive/(str(i).zfill(3)+'.npz')
                                np.savez_compressed(path,color_bgr=ims[0],depth_aligned=ims[1],native_depth=native)
                                records.append(dict(camera=rec,file=path.name,sha256=sha(path)))
                            write_json(archive/'metadata.json',md)
                            write_json(archive/'report.json',dict(status='HEIGHT_WINDOW_RECORDED',records=records,camera_mode='wide',
                                motor_commands=0,controller_access=False,depth_exposure_us=HEIGHT_DEPTH_EXPOSURE_US,
                                physical_stillness_verified=False,stationary_basis='held arms and lift; no base commands; operator stationary confirmation',measured_platform_evidence='../snapshot/capture.json'))
                        await_serviced(files.submit(store_capture),observe)
                        captured=True;event(waiting_state,capture_saved=True)
                if time.monotonic()-last_save>1:save();last_save=time.monotonic()
                time.sleep(.01)
            if args.empty_return:raise ValueError('empty return completed; a separate checked pick is required')
            observe();event('CHECKING_EXECUTION_REQUEST')
            request=json.loads((out/'execute.json').read_text());folder=Path(request['bundle'])
            if (folder.resolve()!=checked_folder or request['bundle_sha256']!=checked_sha or
                    sha(folder/'bundle.json')!=checked_sha):raise ValueError('bundle request mismatch')
            observe();event('LOADING_CHECKED_BUNDLE')
            p,model,proof,ref,mask,sweep,metadata=checked_bundle
            observe();event('CHECKING_CALIBRATION_BINDING')
            validate_installed_binding(model,profile,sha(CONFIG/'arm_ranges.development.json'))
            if abs(p.planned_lift_m-actual)>.000001:raise ValueError('bundle does not use measured arrival height')
            if np.max(abs(np.array(anchor.positions_urad)/1e6-p.stages[0].points[0].positions_rad))>.02:
                raise ValueError('checked anchor differs from current held posture')
            observe();event('CREATING_SCENE_ADMISSION')
            guard=UnifiedPlatformGuard(boot,actual)
            inputs=await_serviced(files.submit(LiveCarryInputs,p,MeasuredFK(model['fk']),ref,mask,
                metadata['aligned_depth_intrinsics'],model['profile']['base_from_optical'],guard,
                observation_policy='snapshot_then_verify',stationary_hold_admission=True),observe)
            observe();event('WAITING_LIVE_SCENE_ADMISSION')
            reader=UnifiedObservationReader(t,adapter,mobile,lift,evidence,inputs,time.monotonic)
            deadline=time.monotonic()+5
            while True:
                camera.check()
                if cancel.is_set():raise RuntimeError('cancelled before grasp')
                try:reader.read();break
                except ValueError:
                    if inputs.failure or time.monotonic()>deadline:raise
                time.sleep(.01)
            session=build_carry_session(p,adapter=adapter,coordinated_stop=coordinated,observe=reader.read,
                clock=time.monotonic,validate_current_scene=inputs.validate_stage,
                observation_policy='snapshot_then_verify',ready_for_route=inputs.route_start_ready,
                recoverable_handoffs=True)
            session.start(OWNER);inputs.admit_snapshot();camera.stop.set()
            event('GRASP_RUNNING',program_sha256=proof['program_sha256'],scene_admission=inputs.validation)
            previous=None;last_save=0.
            while session.state not in ('FAILED','CANCELLED'):
                if cancel.is_set():session.cancel()
                inputs.set_phase(p.stages[min(session.index,len(p.stages)-1)].name)
                session.poll()
                state=(session.state,session.index,session.reason)
                if state!=previous:
                    event(session.state,stage=inputs.phase,reason=session.reason,history=session.history,
                          handoff_diagnostics=session.handoff_diagnostics,
                          platform_observation=guard.last_observation)
                    previous=state
                if session.state=='STOPPING':break
                if time.monotonic()-last_save>1:
                    report['last_feedback']=asdict(session.last_feedback);save();last_save=time.monotonic()
                time.sleep(.005)
            if session.state not in ('FAILED','CANCELLED'):raise RuntimeError(session.reason or 'stop pending')
        except BaseException as exc:
            if guard is not None:report['platform_observation']=guard.last_observation
            report.update(status='FAILED',error=str(exc),traceback=traceback.format_exc(),diagnostics=getattr(exc,'diagnostics',None))
            print(json.dumps({'status':'FAILED','error':str(exc)},ensure_ascii=False),flush=True)
        finally:
            if stop is not None:
                try:
                    report['stop_ack']=asdict(stop.request());deadline=time.monotonic()+4
                    while time.monotonic()<deadline:
                        status,_=stop.status();report['stop']=asdict(status)
                        if status.confirmed:break
                        time.sleep(.03)
                    report['stop_confirmed']=status.confirmed
                except Exception as exc:report['stop_error']=str(exc)
            if saving is not None:saving.result()
            write_state(report)
    return 0 if report.get('stop_confirmed') else 2

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',default='/dev/ttyACM0',help='controller serial port for this installation')
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--baseline',type=Path,required=True)
    parser.add_argument('--bundle',type=Path,required=True,help='checked contact bundle, preloaded before controller access')
    parser.add_argument('--empty-return',action='store_true',help='execute only a checked empty return JSON and keep holding')
    parser.add_argument('--lift-target-um',type=int,required=True)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--supported-empty-bottom',action='store_true')
    raise SystemExit(run(parser.parse_args()))
