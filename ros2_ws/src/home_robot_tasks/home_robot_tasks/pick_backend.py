"""Real stage adapter shared by replay, hardware plan-only, and live execution."""
from pathlib import Path
import json
import shutil
import time

from .pick_service import Rejected, PlanningRejected
from .pick_worker import ComputeWorker


class PipelineBackend:
    def __init__(self, options, *, worker_factory=None, hardware_factory=None):
        self.options=options
        self.mode=options['mode']
        self.mode_detail=('계획 확인: 초기 자세 유지는 실제 수행하며 파지 경로는 전송하지 않습니다.'
                          if self.mode=='live' and options.get('plan_only') else '')
        self.worker_factory=worker_factory or ComputeWorker
        self.hardware_factory=hardware_factory
        self.worker=self.hardware=None
        self.prepared=False
        self.blocked='프로그램 준비 중'
        self.after_motion=False

    def prepare(self, ctx):
        self._prepare_worker(ctx)
        if self.mode == 'live':
            self._prepare_hardware(ctx, self.options)
        self.prepared=True
        self.blocked=''

    def _prepare_worker(self, ctx):
        from tools.lib.pick_compute import make_runtime
        self.worker = self.worker_factory(make_runtime, self.options)
        ctx.note('탐지·분할·파지 모델 준비 중')
        runtime=ctx.wait(self.worker.submit('prepare'))
        if isinstance(runtime,dict) and runtime.get('compute_scheduling') is not None:
            ctx.note('추론 CPU 사용 제한 적용',compute_scheduling=runtime['compute_scheduling'],
                     torch_threads=runtime.get('torch_threads'))

    def _prepare_hardware(self, ctx, options):
        from .pick_hardware import UnifiedPickHardware
        from .pick_capture import model_positions
        self.hardware = (self.hardware_factory or UnifiedPickHardware)(options)
        try:
            self.hardware.prepare(ctx)
        except Exception as error:
            if getattr(error,'diagnostics',None):
                path=ctx.folder/'preparation-failure.json'
                path.write_text(json.dumps(dict(reason=str(error),diagnostics=error.diagnostics),indent=2,ensure_ascii=False))
                ctx.artifact(path,'장치 준비 실패 명령·응답')
            raise
        ctx.note('상주 MoveIt 준비 중')
        ctx.wait(self.worker.submit('warm_planner', profile=self.hardware.profile,
            joints=model_positions(self.hardware.history[-1], self.hardware.profile, self.hardware.guard.height),
            output=str(ctx.station.output/'planner-startup')))

    def ready(self):
        if self.blocked:return False,self.blocked
        if not self.prepared:return False,'프로그램 준비 중'
        if self.hardware is not None and self.hardware.stopped:
            return False,'컨트롤러가 정지됐습니다. 원인 확인과 별도 초기화가 필요합니다.'
        if self.after_motion:
            return False,'팔이 파지 후 자세를 유지하고 있습니다. 물체 처리·빈손 복귀 확인 전에는 새 집기를 시작하지 않습니다.'
        return True,''

    def tick(self):
        if self.worker is not None and not self.blocked and getattr(self.worker,'closed',False):
            raise RuntimeError('compute worker was closed unexpectedly')
        if self.hardware is not None:self.hardware.tick()

    def _compute(self, ctx, method, **values):
        # Runtime failures propagate as faults. Only expected perception/route
        # rejections below are eligible to return to the unchanged idle hold.
        if self.hardware is not None:
            self.hardware.execution_phase='compute:'+method
        return ctx.wait(self.worker.submit(method,**values))

    def capture(self, ctx, request):
        if self.mode=='replay':
            source=Path(self.options['snapshot'])
            folder=ctx.folder/source.name
            folder.mkdir()
            for name in ('color.png','depth_aligned.png','metadata.json'):
                shutil.copyfile(source/name,folder/name)
            shutil.copyfile(self.options['robot_report'],folder/'robot_capture.json')
            ctx.note('저장된 동일 노출의 RGB-D/관절 기록 재생 · 새 촬영 아님')
        else:
            folder,images,evidence=self.hardware.capture(ctx)
            self._compute(ctx,'save',folder=str(folder),images=images,evidence=evidence)
        ctx.artifact(folder/'color.png','저장 영상 재생' if self.mode=='replay' else '이번 요청의 새 촬영')
        ctx.artifact(folder/'robot_capture.json','촬영 시 관절·리프트 관측')
        return dict(request,snapshot=str(folder))

    def detect(self, ctx, value):
        try:
            detections=self._compute(ctx,'detect',snapshot=value['snapshot'],prompt=value['prompt'])
        except RuntimeError as exc:
            if str(exc).startswith('target detection failed:'):
                raise Rejected(str(exc).split('\n')[0]) from exc
            raise
        ctx.artifact(Path(detections).with_name('detections.jpg'),'이번 요청의 탐지 결과')
        ctx.artifact(detections,'탐지·분할 결과')
        return dict(value,detections=detections)

    def candidates(self, ctx, value):
        output=str(ctx.folder/'prepared.json')
        try:
            prepared=self._compute(ctx,'candidates',snapshot=value['snapshot'],detections=value['detections'],
                prompt=value['prompt'],output=output,camera_mode=value.get('camera_mode', self.options.get('camera_mode')),
                location=value.get('location'))
        except RuntimeError as exc:
            if str(exc).startswith('capture/candidate rejected:'):
                if Path(output).is_file():ctx.artifact(output,'파지 후보 거절 근거')
                raise Rejected(str(exc).split('\n')[0]) from exc
            raise
        ctx.artifact(prepared,'현재 촬영에 연결된 파지 후보')
        return dict(value,prepared=prepared)

    def plan(self, ctx, value):
        try:
            result=self._compute(ctx,'plan',prepared=value['prepared'],detections=value['detections'],
                snapshot=value['snapshot'],output=str(ctx.folder/'checked'))
        except RuntimeError as exc:
            if str(exc).startswith('no executable route:'):
                path=ctx.folder/'checked/planning/report.json'
                if path.is_file():
                    ctx.artifact(path,'경로 계획 거절 근거')
                    report=self._compute(ctx,'planning_failure',report=str(path))
                    # A later native-validation failure must not be mistaken for
                    # a geometry search failure from an earlier successful plan.
                    if report.get('status')=='NO_COMPLETE_ROUTE':
                        recovery=report['recovery']
                        angle = self.options.get('table_edge_angle_deg')
                        if (angle is not None and value.get('operation') == 'top-pick'
                                and recovery.get('base_reposition_candidate')):
                            ctx.progress('같은 높이에서 탁상 좌우 정차 후보 비교 중')
                            output = ctx.folder / 'tabletop-review'
                            comparison = self._compute(ctx, 'tabletop_review', prepared=value['prepared'],
                                origin_report=str(path), output=str(output), angle=angle)
                            ctx.artifact(output/'positions.png', '탁상 좌우 정차 후보')
                            ctx.artifact(output/'summary.json', '좌우 정렬 경로 비교')
                            offset = comparison['preferred_tested_offset_mm']
                            recovery['preferred_tested_offset_mm'] = offset
                            if offset is not None:
                                recovery['reason'] = f'탁상 방향 {offset:+g} mm에서 파지 경로 확인 · 베이스 이동 전'
                        ctx.note(recovery['reason'])
                        raise PlanningRejected(str(exc).split('\n')[0],recovery) from exc
                raise Rejected(str(exc).split('\n')[0]) from exc
            raise
        ctx.artifact(result['report'],'경로 계획·충돌 검사 결과')
        height=result.get('contact_height_review')
        if height is not None:
            observed=height.get('object_observed_height_m')
            if observed is not None:
                ctx.note(f"물체 관측 높이 {observed*1000:.1f} mm · 계획 TCP 높이 "
                         f"{height['planned_tcp_height_above_support_m']*1000:.1f} mm (관측 상판 기준)",
                         contact_height_review=height)
            if (height.get('calibration_posture_review') or {}).get('within_recorded_ranges') is False:
                ctx.note('계획 접촉 자세가 과거 보정 시험 범위 밖입니다. 실제 손가락 높이는 검증되지 않았습니다.')
        ctx.note('실행 전 검증 완료',moveit_starts=result['moveit_starts'])
        return dict(value,**result)

    def execute(self, ctx, value):
        if self.mode not in ('live','station') or self.options.get('plan_only'):
            ctx.note('계획 확인 모드: 팔 경로를 전송하지 않습니다.')
            return value
        self.hardware.execution_phase='load_checked_bundle'
        checked=self._compute(ctx,'load_checked',folder=value['bundle'],binding=dict(
            prepared=value['prepared'],snapshot=value['snapshot'],profile=self.hardware.profile,
            session_id=self.hardware.session_id,prompt=value.get('prompt'),
            candidate_source=self.options.get('candidate_source')))
        # File decoding and binding validation run in the compute process while
        # Context.wait keeps the single serial owner servicing measured state.
        ctx.check()
        self.after_motion=True  # Failed dispatch is ambiguous, never a free hand.
        self.hardware.execute(ctx,checked)
        return value

    def verify(self, ctx, value):
        if self.mode not in ('live','station') or self.options.get('plan_only'):
            return dict(status='PLANNED_ONLY',physical_success=False,
                reason='새 경로와 실행 전 검증을 완료했습니다. 팔 경로는 실행하지 않았습니다.',bundle=value['bundle'])
        import numpy as np
        from .pick_rgbd_verification import StableVerification
        stable=StableVerification()
        q,_=self.hardware.hold_pose()
        self.hardware.verification_anchor=q.copy()
        last=None;state='UNKNOWN'
        try:
            for index in range(5):
                folder,images,evidence=self.hardware.capture(ctx,subfolder='verification')
                self._compute(ctx,'save',folder=str(folder),images=images,evidence=evidence)
                measured=np.asarray(evidence['joint_exposure']['positions_rad'])
                tcp=self.hardware.inputs.fk(measured)
                program=self.hardware.session.program
                lo,hi=program.contact_control_interval
                contact=bool(lo-1e-6<=measured[5]<=hi+1e-6)
                # A confirmed missing target is still not empty-hand recovery
                # authority. This flag only classifies a measured close limit.
                empty=bool(not contact and abs(measured[5]-program.close_control_rad)<=2*2*np.pi/4096)
                last=self._compute(ctx,'verification',snapshot=str(folder),bundle=value['bundle'],
                    reference_tcp=self.hardware.contact_tcp.tolist(),current_tcp=tcp.tolist(),
                    contact=contact,empty=empty,prompt=value['prompt'])
                last.update(capture_id=folder.name,observed_s=evidence['camera']['color_time_s'])
                ctx.artifact(Path(last['detections']).with_name('detections.jpg'),'파지 후 확인 '+str(index+1))
                ctx.note('파지 확인: '+last['reason'])
                state=stable.add(last)
                if state!='UNKNOWN':break
                ctx.pause(.15)
            if state=='SUCCESS':
                refreshed=self.hardware.confirm_verification_frame(ctx,folder,last['detections'],value['prompt'])
                result=dict(status='SUCCEEDED',physical_success=True,reason='새 RGB-D 3회와 그리퍼 측정에서 물체 따라오기를 확인했습니다.',
                            verification_basis='discrete_depth_and_measured_hold',refreshed_s=refreshed)
            elif state=='FAILURE':
                result=dict(status='FAILED',physical_success=False,reason='물체가 원래 위치에 남은 것으로 확인됐습니다. 자동 복귀·재시도는 시작하지 않습니다.')
            else:
                result=dict(status='UNVERIFIED',physical_success=False,reason='파지 후 증거가 충분하지 않습니다: '+str(last and last['reason']))
            result['verification_samples']=stable.samples
            return result
        finally:
            self.hardware.verification_anchor=None

    def on_failure(self, exc):
        if isinstance(exc,Rejected) and not self.after_motion:
            self.tick()  # Only a healthy unchanged hold may accept a new request.
            return
        self.abort()

    def abort(self):
        self.blocked='작업이 중단됐습니다. 장치 상태 확인 후 실행기를 다시 시작하세요.'
        try:
            if self.hardware is not None:self.hardware.abort()
        finally:
            if self.worker is not None:self.worker.close()

    def close(self):
        try:
            if self.hardware is not None:self.hardware.close()
        finally:
            if self.worker is not None:self.worker.close()
