"""One controller owner for fresh capture, checked pick, and retained hold.

Uses supported endpoint commissioning and a separately checked bottom-to-top rise,
native bundle validation and snapshot admission. No base motion, automatic
release, controller rearm, or assumption that trajectory completion is success.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from copy import deepcopy
from dataclasses import asdict, replace
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import uuid

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
CONFIG = ROOT/'config/home_assistant'
OWNER = 'pick-station'


def read_while_held(read, check):
    """Reacquire timing-only failures during a stationary hold using new I/O."""
    from .unified_carry_inputs import UnifiedPlatformGuard, read_stationary_sample
    return read_stationary_sample(read, check, time.perf_counter, UnifiedPlatformGuard.maximum_age_s)


def prepare_execution_inputs(checked, profile, guard, options):
    """Pure bundle admission/image setup; no controller or camera handles.

    Run in the analysis worker while the serial owner continues acquiring real
    observations. Recheck the measured anchor/height on the owner afterwards.
    """
    from .carry_bundle import validate_installed_binding, sha
    from .carry_live_inputs import LiveCarryInputs, MeasuredFK
    from .pick_calibration import TOP_BASES, REVISION, audit_top_program
    program,model,proof,reference,mask,sweep,metadata=checked
    validate_installed_binding(model,profile,sha(CONFIG/'arm_ranges.development.json'))
    review=None
    if guard.supported_top:
        if (model.get('lift_reference_basis') not in TOP_BASES or model.get('calibration_revision') != REVISION
                or model.get('approach_mode') != 'vertical'):
            raise ValueError('상단 기준·수직 접근 경로가 현재 장치와 다릅니다.')
        review=audit_top_program(program,profile,expected_lift_m=guard.height)
    inputs=LiveCarryInputs(program,MeasuredFK(model['fk']),reference,mask,
        metadata['aligned_depth_intrinsics'],model['profile']['base_from_optical'],guard,
        observation_policy='snapshot_then_verify',stationary_hold_admission=True,
        fixed_planning_snapshot=options.get('single_capture_test',False),
        fixed_bottom_observation=options.get('rise_after_observation',False) or options.get('fixed_bottom_observation',False))
    return inputs,review


def check_anchor_inventory(setup, ranges):
    """Explain range mismatch before PREPARE_SHADOW latches firmware STOP.

    Never clamp the measured position or enlarge a commissioned command range.
    """
    observations=setup.get('unpowered_observations',[])
    if not observations:raise ValueError('앵커 등록 전 모터 관측이 없습니다.')
    names=('base','shoulder','elbow','wrist_flex','wrist_roll','gripper')
    labels=('회전','어깨','팔꿈치','손목 굽힘','손목 회전','그리퍼')
    for device in observations[-1]['devices']:
        motor=device['requested_id']
        if motor>6:continue
        side=device['bus'];name=names[motor-1]
        raw=(device.get('feedback') or {}).get('position_raw')
        if raw is None:raise ValueError('앵커 등록 전 관절값 누락: '+side+' '+name)
        limit=ranges['arms'][side][name]
        lo,hi=limit['minimum_unwrapped_raw'],limit['maximum_unwrapped_raw']
        if not any(lo<=raw+4096*k<=hi for k in (-1,0,1)):
            arm='왼팔' if side=='left' else '오른팔'
            raise ValueError(f'{arm} {labels[motor-1]} 관절이 등록 범위 밖입니다: '
                f'현재 raw {raw}, 허용 {lo}–{hi}. 받침을 유지하고 해당 관절을 범위 안으로 옮겨 주세요. '
                '앵커 등록 명령은 보내지 않았습니다.')


def inspect_controller_preparation(transport, ctx, **kwargs):
    """Keep the observed motor state even when commissioning is rejected."""
    from tools.run import prepare_unified_controller as prep
    setup = {}
    try:
        prep.inspect(transport, report=setup, **kwargs)
    except Exception as error:
        setup['error'] = str(error)
        raise
    finally:
        path = ctx.folder/'controller-preparation.json'
        path.write_text(json.dumps(setup, indent=2, ensure_ascii=False))
        ctx.artifact(path, '컨트롤러 준비 관측·오류 기록')
    return setup


class UnifiedPickHardware:
    def __init__(self, options):
        self.options=options
        self.resources=ExitStack()
        self.jobs=ThreadPoolExecutor(max_workers=1,thread_name_prefix='pick-camera-analysis')
        self.camera=self.adapter=self.stop=self.guard=self.inputs=self.reader=self.session=None
        self.prepared=False;self.stopped=False;self.stop_confirmed=False
        self.rising=False
        self.history=deque(maxlen=80)
        self.lock=threading.RLock();self.latest=None
        self.session_id=uuid.uuid4().hex
        self.contact_tcp=None
        self.verification_anchor=None
        self.reset_supported_empty=False
        self.execution_phase='preparation'
        self.service_trace=deque(maxlen=160)
        self.last_service_end=None

    def prepare(self, ctx):
        self.diagnostic_context=ctx
        import serial
        from .camera_evidence import load_capture_profile
        # Load execution-only modules before holding torque starts. A cold
        # import must not consume the short measured-evidence lease later.
        from . import carry_bundle, device_ports
        from .platform_preparation import measured_hold_source, reference_supported_bottom, reference_supported_top, move_observed_lift
        from .unified_carry_inputs import UnifiedPlatformGuard, controller_lift_height_m
        from .system_stop_backend import SystemStopBackend
        from so101_arm_bridge.backend_lease import acquire_backend_lease
        from so101_arm_bridge.serial_port import open_exclusive_serial
        from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2, MobileV2Exchange
        from so101_arm_bridge.unified_arm_adapter import UnifiedArmAdapter
        from so101_arm_bridge.unified_commissioning import CommissionClient, query_pick_controller
        from so101_arm_bridge.mobile_client import MobileClient
        from so101_arm_bridge.lift_wire import LiftClient
        from so101_arm_bridge.robot_evidence import RobotEvidenceClient
        from so101_arm_bridge.system_stop import SystemStopClient
        from .pick_calibration import (TOP_BOARD, TOP_BOARDS, top_reference_um, TOP_RAW_OBSERVED, check_top_sample, check_top_inventory,
                                       make_top_reference, top_model_height, verify_installed)
        top = self.options.get('supported_empty_top', False) is True
        bottom = self.options.get('supported_empty_bottom', False) is True
        if self.options.get('execute') is not True or top == bottom:
            raise ValueError('one physically confirmed, supported empty endpoint required')
        verify_installed(ROOT)
        height=self.options.get('lift_target_um',0)
        if type(height)is not int or not 0<=height<=10000 or (top and height):
            raise ValueError('supported lift startup remains bounded to 0..10mm')
        self.profile,self.reference=load_capture_profile(ROOT,self.options.get('camera_mode','wide'))
        self.resources.enter_context(acquire_backend_lease('stm32',189))
        port=self.resources.enter_context(open_exclusive_serial(serial,self.options['serial_port'],921600,.02))
        self.transport=StreamValidationTransportV2(port,response_timeout_s=3)
        board, initial = query_pick_controller(self.transport, self.profile)
        if (top or self.options.get("rise_after_observation")) and board not in TOP_BOARDS:
            raise ValueError('상단 기준 등록을 지원하는 보드와 프로필이 필요합니다.')
        if self.options.get('rise_after_observation'):
            from .pick_lift_transition import require_top_move
            from .pick_calibration import rise_target_um
            require_top_move(board,rise_target_um(board))
        ctx.note(f'보정 {self.profile["calibration_id"]} · STM32 {board[0]:#x} · 프로필 CRC32 {board[1]}')
        if initial.flags not in (0,17) or initial.reason:
            raise ValueError('fresh unpowered or prepared controller required; never steal another live owner')
        setup=inspect_controller_preparation(self.transport,ctx,
                     baseline=json.loads(Path(self.options['baseline']).read_text()),
                     prepare=initial.flags==0,supported=True,empty=True,pick_board=board,
                     release_retained=initial.flags==0 and self.options.get('supported_startup_release',False))
        check_anchor_inventory(setup, json.loads((CONFIG/'arm_ranges.development.json').read_text()))
        if setup.get('startup_release'):
            ctx.note('받침 확인에 따라 남아 있던 모터 힘 해제 완료 · 전체 토크 OFF 실측 확인')
        if top and self.options.get('map_navigation'):
            self.travel_inventory = setup['unpowered_observations'][-1]
        if top:
            # inspect() has already proven this inventory window stationary with
            # all drives unpowered. Raw position is modulo one revolution, so
            # physical endpoint confirmation comes from the operator, not raw.
            pos = check_top_inventory(setup['unpowered_observations'][-1])
            ctx.note(f'토크 OFF 현재 리프트 엔코더 {pos}')
            if not self.options.get('resume_height_source') and not TOP_RAW_OBSERVED[0] <= pos <= TOP_RAW_OBSERVED[1]:
                ctx.note(f'과거 상단 관측값 {TOP_RAW_OBSERVED[0]}–{TOP_RAW_OBSERVED[1]}과 다름 · '
                         '사용자가 확인한 기계적 최상단에서 새 기준을 등록합니다.')
        if not self.options.get('map_navigation'):
            self.start_camera(ctx)
        ctx.check()
        boot=setup['commission_final']['boot_id']
        preview=UnifiedArmAdapter(self.transport,CONFIG/'arm_ranges.development.json',pick_board=board)
        self.adapter=UnifiedArmAdapter(self.transport,CONFIG/'arm_ranges.development.json',pick_board=board,
            authorized_limits=replace(preview.motion_limits,hardware_authorized=True),motion_authorized=True)
        self.anchor=self.adapter.prepare()
        self.mobile=MobileClient(MobileV2Exchange(self.transport),lambda:int(time.monotonic()*1000))
        self.mobile.synchronize()
        if self.mobile.boot_id!=boot:
            raise ValueError('controller boot changed at preparation')
        self.lift=LiftClient(self.mobile)
        self.evidence=RobotEvidenceClient(self.transport,self.mobile,time.monotonic,maximum_age_ms=250)
        self.stop=SystemStopClient(self.transport,boot,time.monotonic)
        self.coordinated=SystemStopBackend(self.stop,SimpleNamespace(owner=OWNER))
        ctx.check()
        CommissionClient(self.transport).activate_zero(supported=True,empty_grippers=True)
        self.adapter.start_current_pose_hold(OWNER)
        self.anchor=self.adapter.prepared_state
        deadline=time.monotonic()+4
        while self.adapter.state.value!='ready':
            ctx.check()
            if time.monotonic()>deadline:raise TimeoutError('initial hold timed out')
            self.adapter.poll(OWNER);time.sleep(.01)
        def read_preparation():
            ctx.check()
            if self.camera is not None:self.camera.check()
            self.adapter.keepalive()
            sent=time.monotonic();packet=self.adapter.feedback_snapshot();received=time.monotonic()
            measured_hold_source(packet,self.anchor.positions_urad,self.adapter.motion_limits,sent,received)
            self.evidence.publish_mcu_snapshot(packet,sent,received,arms_safe=True,lift_hold=False,payload_safe=True)
            status=self.lift.status()
            if status.fault:
                error=ValueError(f'lift fault={status.fault}, state={status.state}, '
                                 f'height_um={status.height_um}, current_ma={status.current_ma}')
                error.lift_status=asdict(status)
                raise error
            return status
        def observe():
            if self.rising or getattr(self, 'base_moving', False):
                return read_preparation()
            return read_while_held(read_preparation, ctx.check)
        top_reference = None
        if top:
            def observe_top():
                status = observe()
                check_top_sample(status, anchor_raw=pos, board=board)
                return status
            registered = reference_supported_top(
                observe_top, self.mobile, self.lift, top_reference_um(self.profile,board),
                operator_confirmed=top)
            if self.options.get('resume_height_source'):
                from .pick_calibration import make_resumed_height_reference
                top_reference = make_resumed_height_reference(registered, self.profile, board,
                    self.options['resume_height_source'], operator_height_unchanged=True,height_model=self.options.get('lift_height_model','field_effective'))
                current_height = top_model_height(registered, self.profile, board, top_reference)
                ctx.note(f'이전 실측 높이 + 현재 엔코더 차이 적용 · 계획 높이 {current_height*1000:.1f} mm · 리프트 이동 없음')
            else:
                top_reference = make_top_reference(registered, self.profile, board, operator_confirmed=top,
                    height_model=self.options.get("lift_height_model","field_effective"),
                    fixed_base_station=self.options.get("fixed_base_station",False),
                    stationary_mobile=self.mobile.status() if self.options.get("fixed_base_station") else None)
            ctx.note(f'이번 세션 리프트 기준 연결 · 엔코더 {registered.position_raw} · 부트 {registered.boot_id}',
                     top_reference=top_reference)
        else:
            reference_supported_bottom(observe,self.mobile,self.lift)
        if height:
            ctx.note('검증된 범위 내 리프트 준비',target_um=height)
            move_observed_lift(self.lift,observe,self.stop,height)
        status = observe()
        actual = top_model_height(status, self.profile, board, top_reference,
            stationary_mobile=self.mobile.status() if self.options.get("fixed_base_station") else None) if top else controller_lift_height_m(status.height_um, self.profile, board=board)
        self.observe_preparation=observe
        self.guard=UnifiedPlatformGuard(boot,actual,profile=self.profile,board=board,supported_top=top,
                                        top_reference=top_reference)
        self.prepared=True
        while len(self.history)<3:
            ctx.pause(.05)
        endpoint = '상단' if top else '하단'
        ctx.note(f'컨트롤러 {endpoint} 유지 준비 완료' if self.camera is None else '카메라·컨트롤러 유지 준비 완료',actual_lift_m=actual)

    def start_camera(self, ctx):
        from .s2_rgbd_acquisition import Capture
        from .stationary_rgbd import pick_depth_exposure_us
        from functools import partial
        from .pick_camera_process import PickCameraProcess
        camera = PickCameraProcess(
            partial(Capture, self.profile, self.reference, retain_frame=True,
                    depth_exposure_us=pick_depth_exposure_us(self.options.get('camera_mode', 'wide'))),
            self._consume)
        if self.prepared:
            # The controller owner keeps renewing its hold during D415 startup.
            pending = self.jobs.submit(camera.__enter__)
            try:
                ctx.wait(pending)
            except BaseException:
                if not pending.cancel():
                    pending.add_done_callback(lambda _: camera.__exit__(None, None, None))
                raise
            self.resources.push(camera)
            self.camera = camera
        else:
            self.camera = self.resources.enter_context(camera)
        ctx.note('카메라 별도 프로세스 준비 완료', camera_runtime=camera.runtime_info)

    def _consume(self, record, images):
        with self.lock:
            self.latest=(record,images)
            inputs=self.inputs
        if inputs is not None:
            inputs.add_frame(record,images)

    def tick(self):
        if not self.prepared or self.stopped or self.rising or getattr(self,'base_moving',False):
            return
        # In-memory only: file/UI I/O belongs after STOP. Record both idle gaps
        # and transaction time so host stalls differ from slow serial replies.
        started=time.perf_counter()
        previous=getattr(self,'last_service_end',None)
        record=dict(phase=getattr(self,'execution_phase','holding'),started_s=started,
                    gap_ms=None if previous is None else (started-previous)*1000)
        try:
            return self._tick()
        except Exception as error:
            record['error']=str(error)
            raise
        finally:
            ended=time.perf_counter()
            record['duration_ms']=(ended-started)*1000
            self.last_service_end=ended
            if not hasattr(self,'service_trace'):self.service_trace=deque(maxlen=160)
            self.service_trace.append(record)

    def _tick(self):
        if self.camera is not None:self.camera.check()
        if self.session is not None:
            index=min(self.session.index,len(self.session.program.stages)-1)
            phase=self.session.program.stages[index].name
            if phase=='close' and self.contact_tcp is None and self.inputs.joints:
                self.contact_tcp=self.inputs.fk(self.inputs.joints[-1].positions_rad)
            self.inputs.set_phase(phase)
            self.session.poll()
            if self.session.state in ('FAILED','CANCELLED','STOPPING'):
                raise RuntimeError('execution stopped: '+str(self.session.reason))
            with self.inputs.lock:
                self.history=deque(self.inputs.joints,maxlen=80)
            if self.verification_anchor is not None and self.history:
                if np.max(abs(np.asarray(self.history[-1].positions_rad)-self.verification_anchor))>4*2*np.pi/4096:
                    raise ValueError('post-pick pose changed while verification was running')
            return
        if self.inputs is not None:
            self.reader.refresh()
            with self.inputs.lock:
                self.history=deque(self.inputs.joints,maxlen=80)
            return
        context = getattr(self, 'diagnostic_context', None)
        return read_while_held(self._read_idle_hold, context.check if context else lambda: None)

    def _read_idle_hold(self):
        from .carry_live_inputs import joint_exposure, ObservationNotFresh
        from .platform_preparation import measured_hold_source
        self.adapter.keepalive()
        sent=time.monotonic();packet=self.adapter.feedback_snapshot();received=time.monotonic()
        measured_hold_source(packet,self.anchor.positions_urad,self.adapter.motion_limits,sent,received)
        try:
            sample=joint_exposure(packet,sent,received)
        except ObservationNotFresh:
            # Holding only: discard a delayed observation without renewing any
            # timestamp. The measured pose envelope above still applies, and a
            # continuing outage must fail once the previous sample expires.
            if not self.history or not 0 <= received-self.history[-1].lower_s <= .2:
                raise
        else:
            if not self.history or sample.sequence!=self.history[-1].sequence:
                self.history.append(sample)
        self.evidence.publish_mcu_snapshot(packet,sent,received,arms_safe=True,lift_hold=False,payload_safe=True)
        sent=time.monotonic();mobile=self.mobile.status();lift=self.lift.status();received=time.monotonic()
        self.guard.update(mobile,lift,sent,received)

    def capture(self, ctx, *, subfolder=None):
        self.diagnostic_context=ctx
        from .pick_capture import make_evidence
        requested=time.monotonic()
        deadline=requested+5
        while True:
            ctx.service()
            with self.lock:
                pair=self.latest
            if pair is not None and self.history:
                record,images=pair
                try:
                    evidence=make_evidence(deepcopy(record),list(self.history),deepcopy(self.guard.last_observation),
                        self.profile,now=time.monotonic(),requested_s=requested,session_id=self.session_id,
                        anchor=self.history[-1].positions_rad)
                    folder=(ctx.folder/subfolder if subfolder else ctx.folder)/record['capture_id']
                    return folder,images,evidence
                except ValueError:
                    pass  # Bounded wait for a fresh stable frame; never relabel it.
            if time.monotonic()>deadline:
                raise TimeoutError('fresh held RGB-D/controller capture unavailable')
            ctx.pause(.02)

    def rise_to_pick_height(self, ctx, proof):
        from .pick_lift_transition import rise_to_top
        from .pick_calibration import RISE_TOP_BASIS, top_model_height
        from .unified_carry_inputs import UnifiedPlatformGuard
        if (not self.options.get('rise_after_observation') or self.guard.supported_top
                or self.session is not None or self.inputs is not None):
            raise ValueError('new bottom observation session required for lift rise')
        ctx.service()
        self.guard.require_fresh(time.monotonic())
        from .pick_capture import model_positions
        current=model_positions(self.history[-1],self.profile,self.guard.current_height)
        if any(abs(current[k]-v)>(.003 if k=='s2_lift_joint' else .02)
               for k,v in proof['start_model_joint_positions_si'].items()):
            raise ValueError('lift path start differs from current measured hold')
        initial = self.lift.status()
        if abs(initial.height_um)>2000:
            raise ValueError('lift must still be at observed bottom')
        self.rising=True
        progress_path=ctx.folder/'lift-progress.jsonl'
        next_progress=0.
        last_sample=None
        try:
            def observe():
                nonlocal next_progress,last_sample
                status=self.observe_preparation()
                mobile=self.mobile.status()
                if (mobile.boot_id!=self.guard.boot_id or mobile.state!=0 or mobile.session!=0
                        or mobile.reason or mobile.flags&32 or not mobile.feedback_is_fresh(200)):
                    raise ValueError('base changed during lift rise')
                now=time.monotonic();last_sample=asdict(status)
                if now>=next_progress:
                    next_progress=now+5
                    fraction=max(0.,min(1.,status.height_um/proof['target_um']))
                    ctx.progress(f'리프트 상승 중 · 엔코더 목표의 {fraction:.0%}')
                    try:
                        with progress_path.open('a') as log:
                            log.write(json.dumps(dict(observed_host_s=now,lift=last_sample))+'\n')
                    except OSError as error:
                        ctx.note('상승 관측 기록 저장 실패: '+str(error))
                return status
            arrival=rise_to_top(self.lift,observe,self.stop,proof,self.profile,self.guard.board,
                                check=ctx.check)
            final=arrival['final']
            reference=dict(basis=RISE_TOP_BASIS,height_model=self.options.get("lift_height_model","field_effective"),operator_confirmed=False,arrival_verified=True,
                bottom_referenced=True,boot_id=final.boot_id,lift_session=final.session,
                calibration_id=self.profile['calibration_id'],controller_board=list(self.guard.board),
                position_raw=final.position_raw,height_um=final.height_um,target_um=final.target_um)
            height=top_model_height(final,self.profile,self.guard.board,reference)
            self.guard=UnifiedPlatformGuard(final.boot_id,height,profile=self.profile,board=self.guard.board,
                                           supported_top=True,top_reference=reference)
            self.history.clear()
        except BaseException as exc:
            failure=dict(reason=str(exc),last_lift=getattr(exc,'lift_status',last_sample),stop_error=None)
            ctx.note('리프트 상승 중단: '+str(exc))
            try:self.abort()
            except Exception as error:
                failure['stop_error']=str(error)
                exc.add_note('stop confirmation failed: '+str(error))
            # Diagnostic failures must not replace the motion failure. Cache
            # queries run only AFTER the stop attempt and never renew motion.
            for name,method in (('controller_runtime','stationary_runtime'),
                                ('motor_read_failures','stationary_feedback_failures')):
                try:failure[name]=getattr(self.transport,method)()
                except Exception as error:failure[name]={'error':str(error)}
            path=ctx.folder/'lift-failure.json'
            try:
                path.write_text(json.dumps(failure,indent=2,ensure_ascii=False))
                ctx.artifact(path,'상승 최초 오류·마지막 관측·정지 확인')
            except Exception as error:ctx.note('상승 오류 기록 표시 실패: '+str(error))
            raise
        finally:
            self.rising=False
            if progress_path.is_file():
                try:ctx.artifact(progress_path,'리프트 상승 관측 기록')
                except Exception as error:ctx.note('상승 관측 기록 표시 실패: '+str(error))
        while len(self.history)<3:ctx.pause(.05)
        ctx.note('리프트 상승·정지 확인 완료 · 하단 관측 좌표 유지',actual_lift_m=height)

    def reposition_for_pick(self, ctx, plan):
        """Keep arm feedback and lift hold live on the same port throughout the base move."""
        from .base_alignment import run_lateral_shift
        from .platform_preparation import arm_observed_stationary, supported_lift_is_holding
        if (not self.prepared or self.stopped or self.session is not None or self.inputs is not None
                or not self.guard.supported_top or self.options.get('fixed_base_station') or self.evidence.transport_only):
            raise ValueError('an empty top hold with measured wheel feedback is required')
        ctx.service()
        self.guard.require_fresh(time.monotonic())
        mobile = self.mobile
        status = mobile.status()
        if not status.capabilities & 8:
            raise ValueError('현재 STM32 펌웨어는 통합 주행 후 주차 확인을 지원하지 않습니다.')
        def observe():
            ctx.check()
            status = self.observe_preparation()
            if not supported_lift_is_holding(status):
                raise RuntimeError('재배치 중 리프트 유지 관측 실패')
            return status
        path = ctx.folder/'base-motion.jsonl'
        with path.open('x') as stream:
            def log(event, **data):
                if 'reply' in data:
                    data['reply'] = asdict(data['reply'])
                stream.write(json.dumps(dict(event=event,host_s=time.monotonic(),**data))+'\n')
            log('plan',plan=plan)
            self.base_moving = True
            try:
                motion = run_lateral_shift(plan, mobile=mobile, observe=observe, enter_transport=None,
                    admit=lambda: arm_observed_stationary(observe, mobile, session=mobile.sequence+1),
                    check=ctx.check, log=log, settle_ms=900)
                observe()
                log('parked',reply=mobile.park())
            except BaseException:
                self.abort()
                raise
            finally:
                self.base_moving = False
                ctx.artifact(path,'통합 제어기 주행·주차 기록')
        self.history.clear()
        with self.lock:
            self.latest = None
        self.execution_phase = 'holding'
        ctx.service()
        self.guard.require_fresh(time.monotonic())
        return motion

    def lateral_shift(self, ctx, plan):
        """Empty-hand timed strafe, then whole-system STOP; this owner then retires.

        Same order as the supervised S4 navigation trial. Transport mode makes
        the session wheel-only, so arm use needs a fresh controller reset.
        """
        from .base_alignment import run_lateral_shift
        from .platform_preparation import arm_observed_stationary, enter_base_transport
        if (not self.prepared or self.stopped or self.session is not None or self.inputs is not None
                or not self.guard.supported_top):
            raise ValueError('empty supported top hold required before base alignment')
        ctx.service()
        self.guard.require_fresh(time.monotonic())
        path=ctx.folder/'base-motion.jsonl'
        def log(event,**data):
            reply=data.get('reply')
            if reply is not None:data['reply']=asdict(reply)
            with path.open('a') as stream:
                stream.write(json.dumps(dict(event=event,host_s=time.monotonic(),**data))+'\n')
        def observe(allow_stop=False):
            state=self.transport.heartbeat()
            if state.stop_latched and not allow_stop:
                raise RuntimeError('controller STOP latched during base alignment')
            if self.evidence.transport_only:
                return None  # firmware keeps the resident hold; arm evidence is wheel-only now
            return self.observe_preparation()
        self.base_moving=True
        self.execution_phase='base_alignment'
        try:
            log('plan',plan=plan)
            result=run_lateral_shift(plan,mobile=self.mobile,observe=observe,
                enter_transport=lambda:enter_base_transport(observe,self.evidence),
                admit=lambda:arm_observed_stationary(observe,self.mobile,transport_only=True),
                check=ctx.check,log=log)
            log('settled',**result)
        finally:
            # Always end with STOP, also after success.
            try:
                if self.evidence.transport_only:
                    self._stop_after_transport(observe,log)
                else:
                    self.abort()
            finally:
                self.base_moving=False
                if path.is_file():ctx.artifact(path,'베이스 정렬 명령·응답 기록')
        ctx.note('바퀴 영속도 유지와 정지 확인 완료',base_motion=result)
        return result

    def _stop_after_transport(self, observe, log):
        """Transport-scope STOP proof is ``base_confirmed`` (scope 1), as in the navigation trial."""
        if not hasattr(self,'transport_before_stop'):
            try:self.transport_before_stop=self.transport.timing_diagnostics()
            except Exception as error:self.transport_before_stop={'error':str(error)}
        self.stopped=True
        error=None
        try:self.stop.request()
        except Exception as exc:error=exc
        deadline=time.monotonic()+4
        last=None
        try:
            while time.monotonic()<deadline:
                try:observe(allow_stop=True)
                except Exception as exc:last=str(exc)
                try:status,_=self.stop.status()
                except Exception as exc:
                    last=str(exc);time.sleep(.03);continue
                if status.base_confirmed:
                    self.stop_confirmed=True
                    log('stop_confirmed',scope=status.scope,state=status.state,flags=status.flags)
                    return
                time.sleep(.03)
            raise RuntimeError('base stop not confirmed'+(': '+str(error) if error else '')
                               +(' · last observation error: '+last if last else ''))
        finally:
            self._record_controller_diagnostics()

    def execute(self, ctx, checked, *, pregrasp_check=False):
        from .unified_carry_inputs import UnifiedObservationReader
        from .carry_execution import build_carry_session
        program,model,proof,reference,mask,sweep,metadata=checked
        if bool(pregrasp_check) != (model.get('inspection_only') is True):
            raise ValueError('손끝 점검과 실물 파지의 경로 종류가 다릅니다.')
        if self.guard.supported_top and not pregrasp_check and model.get('close_at_pregrasp') is not True:
            raise ValueError('최상단 파지는 추가 하강 없이 닫는 경로로 다시 계획해야 합니다.')
        self.execution_phase='execution_input_setup'
        gaps = []
        for tick in reversed(self.service_trace):
            if tick['phase'] not in ('plan_to_execution', 'compute:load_checked'):
                break
            if tick['gap_ms'] is not None:
                gaps.append(tick['gap_ms'])
        if gaps:
            ctx.note(f'계획→실행 전환 · 상태 갱신 {len(gaps)}회 · 최대 갱신 공백 {max(gaps):.1f} ms')
        ctx.note('실행 자료 확인·관측 준비 중 · 제어기 상태 갱신 유지')
        ctx.service()
        inputs,calibration_review=ctx.wait(self.jobs.submit(
            prepare_execution_inputs,checked,self.profile,self.guard,self.options))
        if calibration_review and not calibration_review['within_recorded_ranges']:
            ctx.note('과거 보정 시험 자세 범위 밖 · 관절 한계·충돌 검사 통과, 실제 위치 오차는 검증되지 않음',
                     calibration_review=calibration_review)
        if pregrasp_check or (self.guard.supported_top and calibration_review
                              and not calibration_review['within_recorded_ranges']):
            from .pick_table_clearance import require_table_clearance
            require_table_clearance(proof, pregrasp_check=pregrasp_check)
        # These checks use the observations acquired during the worker job,
        # never the pre-job cached pose. No dispatch on a changed anchor.
        if abs(program.planned_lift_m-self.guard.current_height)>.003:
            raise ValueError('checked lift height changed')
        if np.max(abs(np.asarray(self.history[-1].positions_rad)-program.stages[0].points[0].positions_rad))>.02:
            raise ValueError('checked plan anchor differs from current measured hold')
        self.execution_phase='live_scene_admission'
        ctx.note('실행 직전 관절·리프트·장면 관측 확인 중')
        self.reader=UnifiedObservationReader(self.transport,self.adapter,self.mobile,self.lift,self.evidence,inputs,time.monotonic)
        with self.lock:self.inputs=inputs
        deadline=time.monotonic()+5
        while True:
            ctx.service()
            try:
                inputs.feedback()
                break
            except ValueError:
                if inputs.failure or time.monotonic()>deadline:raise
            ctx.pause(.01)
        ctx.check()
        self.execution_phase='session_start'
        self.session=build_carry_session(program,adapter=self.adapter,coordinated_stop=self.coordinated,transport_owner=OWNER,
            observe=self.reader.read,clock=time.monotonic,validate_current_scene=inputs.validate_stage,
            observation_policy='snapshot_then_verify',ready_for_route=inputs.route_start_ready,recoverable_handoffs=True,
            pause_before_descent=pregrasp_check)
        self.session.start(OWNER)
        inputs.admit_snapshot()
        previous=None
        while self.session.state!='HOLDING':
            self.execution_phase='route:'+self.session.program.stages[min(self.session.index,len(program.stages)-1)].name
            ctx.service()
            current=(self.session.state,self.session.index)
            if current!=previous:
                ctx.note('팔 동작: '+inputs.phase,controller_state=self.session.state)
                previous=current
            if pregrasp_check and self.session.state=='AWAITING_DESCENT_CONFIRMATION':
                ctx.progress('열린 집게 점검 자세 · 하강 전 RGB-D 재촬영')
                return self.capture(ctx, subfolder='pregrasp-check')
            if self.session.state=='AWAITING_ROUTE_REPLAN':
                raise RuntimeError('handoff requires replan; automatic loaded recovery is not allowed')
            ctx.pause(.005)
        if pregrasp_check:
            raise RuntimeError('하강 전 점검 자세에 도달하지 못했습니다.')
        if self.contact_tcp is None:
            raise RuntimeError('measured grasp reference pose missing')

    def hold_pose(self):
        if self.session is None or self.session.state!='HOLDING' or not self.history:
            raise ValueError('stationary post-pick hold required')
        def measured_pose():
            self.guard.require_fresh(time.monotonic())
            sample = self.history[-1]
            if not 0 <= time.monotonic()-sample.lower_s <= .2:
                from .carry_live_inputs import ObservationNotFresh
                raise ObservationNotFresh('post-pick joints stale')
            return np.asarray(sample.positions_rad), self.inputs.fk(sample.positions_rad)

        from .carry_live_inputs import ObservationNotFresh
        try:
            return measured_pose()
        except ObservationNotFresh:
            def refresh():
                self.reader.refresh()
                with self.inputs.lock:
                    self.history = deque(self.inputs.joints, maxlen=80)
                return measured_pose()
            return read_while_held(refresh, self.diagnostic_context.check)

    def confirm_verification_frame(self, ctx, snapshot, detection_path, prompt):
        """Recheck the verified object's depth/pixels in a fresh held frame.

        Slow model inference is never relabelled as a fresh sensor timestamp.
        A final quick comparison must fit the existing 200ms camera bound.
        """
        def load_checker():
            from PIL import Image
            from .carry_rgbd import StationarySceneCheck
            from tools.diagnostics.replay_s3_grasp import verified_mask
            report=json.loads(Path(detection_path).read_text())
            targets=[d for d in report['detections'] if d['label']==prompt and d['score']>=.5]
            if len(targets)!=1:raise ValueError('verification target became ambiguous')
            md=json.loads((Path(snapshot)/'metadata.json').read_text())
            color=np.asarray(Image.open(Path(snapshot)/'color.png').convert('RGB'))[:,:,::-1].copy()
            depth=np.asarray(Image.open(Path(snapshot)/'depth_aligned.png'),dtype=float)*md['depth_scale_m']
            spec=targets[0]['segmentation']
            mask=verified_mask(Path(detection_path).parent/spec['mask_file'],spec['sha256'],depth.shape)
            return StationarySceneCheck(color,depth,mask)
        checker=ctx.wait(self.jobs.submit(load_checker))
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            ctx.service()
            with self.lock:record,images=self.latest
            current=ctx.wait(self.jobs.submit(checker.compare,images[0],
                images[1].astype(float)*images[2]['depth_scale_m'],target_only=True))
            if not current['unchanged']:
                raise ValueError('verified target changed before reporting success')
            if 0<=time.monotonic()-min(record['color_time_s'],record['depth_time_s'])<=.2:
                return record['color_time_s']
        raise ValueError('verification refresh expired; cannot report physical success')

    def _refresh_empty_stop_evidence(self):
        # Before arm execution only: the supported empty arms still have the
        # measured preparation anchor. STOP's MCU reader supplies fresh poses.
        # Do not renew motion leases, infer a held payload, or reuse old times.
        if not self.prepared or self.session is not None or self.inputs is not None:
            return
        from .platform_preparation import measured_hold_source
        sent=time.monotonic();packet=self.transport.get_feedback_snapshot();received=time.monotonic()
        measured_hold_source(packet,self.anchor.positions_urad,self.adapter.motion_limits,sent,received)
        self.evidence.publish_mcu_snapshot(packet,sent,received,arms_safe=True,lift_hold=False,payload_safe=True)

    def prepare_next_test(self, ctx):
        """Retire a completed hold; keep torque, lift reference and serial owner."""
        if (self.stopped or not self.prepared or not self.options.get('single_capture_test')
                or (self.session is not None and self.session.state != 'HOLDING')
                or (self.session is None and self.inputs is not None)):
            raise ValueError('completed healthy top-pick hold required')
        session=self.session
        # Finish only the already running finite hold. Do not enqueue another
        # hold, open the gripper, release torque, or reset the controller.
        deadline=time.monotonic()+(session.program.stages[-1].timing['duration_s']+4 if session else 0)
        while session is not None and session.active:
            ctx.check()
            session.motion.maintain()
            self.reader.refresh()
            child=session.goal+'/hold/'+str(session.hold_cycle)
            result=session.motion.poll(child)
            pending=session.motion.pending_stop_reason()
            if pending or result.state in ('FAILED','CANCELLED'):
                raise RuntimeError(pending or result.reason or 'hold retirement failed')
            if result.state=='SUCCEEDED':
                session.motion.retire_completed(child);session.active=False
                break
            if time.monotonic()>deadline:raise RuntimeError('hold retirement timed out')
            time.sleep(.005)
        if session is not None:
            if not session.motion.ready():raise RuntimeError('arm route still active')
            self.reader.refresh()
        else:
            self.tick()  # Perception/plan rejection retained the original empty hold.
        self.guard.require_fresh(time.monotonic())
        from .platform_preparation import measured_hold_source
        from .carry_live_inputs import joint_exposure
        def read_next_anchor():
            self.adapter.keepalive()
            sent=time.monotonic();packet=self.adapter.feedback_snapshot();received=time.monotonic()
            measured_hold_source(packet,packet.positions_urad,self.adapter.motion_limits,sent,received)
            return packet, joint_exposure(packet,sent,received)
        packet,sample=read_while_held(read_next_anchor,ctx.check)
        with self.lock:
            self.session=self.inputs=self.reader=None
            self.latest=None
            self.anchor=packet
            self.history=deque([sample],maxlen=80)
            self.session_id=uuid.uuid4().hex
            self.contact_tcp=self.verification_anchor=None
        self.diagnostics_recorded=False
        self.reset_supported_empty=False
        ctx.note('다음 시험 준비 완료 · 현재 자세·리프트 높이 유지 · 새 요청에서 한 번 촬영')

    def _refresh_stop_evidence(self):
        if not self.prepared:return
        if self.session is None and self.inputs is None:
            return self._refresh_empty_stop_evidence()
        # STOP's own MCU observer continues actual joint reads. The previous
        # capture anchor is obsolete after motion; the MCU independently checks
        # both arms against its stop anchor. Never poll/resume a motion session.
        from .platform_preparation import measured_hold_source
        sent=time.monotonic();packet=self.transport.get_feedback_snapshot();received=time.monotonic()
        measured_hold_source(packet,packet.positions_urad,self.adapter.motion_limits,sent,received)
        payload_safe=self.reset_supported_empty
        if not payload_safe and self.session is not None:
            stage=self.session.program.stages[min(self.session.index,len(self.session.program.stages)-1)]
            if not stage.attached:
                payload_safe=True
            elif self.session.observation_policy=='snapshot_then_verify':
                lo,hi=self.session.program.contact_control_interval
                feedback=self.session.last_feedback
                payload_safe=(lo-1e-6<=packet.positions_urad[5]/1e6<=hi+1e-6
                              and feedback is not None and feedback.payload_relation!='empty')
        # Same supervised contact envelope as execution, not a claim of visual
        # grasp success. Unsupported/unknown payloads cannot confirm retention.
        self.evidence.publish_mcu_snapshot(packet,sent,received,
            arms_safe=True,lift_hold=False,payload_safe=bool(payload_safe))

    def _record_controller_diagnostics(self):
        ctx=getattr(self,'diagnostic_context',None)
        transport=getattr(self,'transport',None)
        if ctx is None or transport is None or getattr(self,'diagnostics_recorded',False):return
        self.diagnostics_recorded=True
        values={'last_platform_observation':getattr(self.guard,'last_observation',None),
                'execution_phase':getattr(self,'execution_phase',None),
                'service_trace':list(getattr(self,'service_trace',())),
                'transport_before_stop':getattr(self,'transport_before_stop',None),
                'camera_runtime':getattr(getattr(self,'camera',None),'runtime_info',None)}
        for name,method in (('controller_runtime','stationary_runtime'),
                            ('motor_read_failures','stationary_feedback_failures')):
            try:values[name]=getattr(transport,method)()
            except Exception as error:values[name]={'error':str(error)}
        try:
            path=ctx.folder/'controller-stop-diagnostics.json'
            path.write_text(json.dumps(values,indent=2,ensure_ascii=False))
            ctx.artifact(path,'제어기 최초 정지 원인·통신 기록')
        except Exception as error:
            try:ctx.note('제어기 정지 기록 저장 실패: '+str(error))
            except Exception:pass

    def abort(self):
        # Freeze evidence before STOP confirmation traffic can displace the
        # offending request from the bounded transport ring. Never mask STOP.
        if not hasattr(self,'transport_before_stop'):
            try:self.transport_before_stop=self.transport.timing_diagnostics()
            except Exception as error:self.transport_before_stop={'error':str(error)}
        try:self._abort()
        finally:self._record_controller_diagnostics()

    def _abort(self):
        if self.stop is None or self.stop_confirmed:
            self.stopped=True
            return
        self.stopped=True
        error=None
        try:self.stop.request()
        except Exception as exc:error=exc
        deadline=time.monotonic()+4
        last_status=None;last_evidence_error=None
        while time.monotonic()<deadline:
            try:self._refresh_stop_evidence()
            except Exception as exc:last_evidence_error=str(exc)
            status,_=self.stop.status()
            last_status={name:getattr(status,name,None) for name in
                         ("state","flags","scope","boot_id","age_ms","max_age_ms","observed_ms","actions_completed_ms")}
            if status.confirmed:
                self.stop_confirmed=True
                return
            time.sleep(.03)
        raise RuntimeError('integrated stop not confirmed'+(': '+str(error) if error else '')
                           +' · last_status='+str(last_status)
                           +(' · evidence_error='+last_evidence_error if last_evidence_error else ''))

    def close(self):
        try:
            self.abort()
        finally:
            try:self.resources.close()
            finally:self.jobs.shutdown(wait=True,cancel_futures=True)
