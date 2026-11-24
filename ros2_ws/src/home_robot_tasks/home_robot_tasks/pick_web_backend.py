"""Interactive station. One device owner, with a camera-only path at the lift top."""
import json
from pathlib import Path
from hashlib import sha256

from .pick_backend import PipelineBackend
from .pick_service import Rejected


class WebPickBackend(PipelineBackend):
    def __init__(self, options, **kwargs):
        from .pick_calibration import STATION_HEIGHT_MODEL
        options = {'candidate_source': 'graspnet', 'lift_height_model': STATION_HEIGHT_MODEL, **options}
        super().__init__(options, **kwargs)
        if options['candidate_source'] == 'graspnet':
            self.mode_detail = 'Grounding DINO · SAM · GraspNet · MoveIt'
        from tools.lib.pick_compute import ROOT
        manifest = ROOT/'SHA256SUMS'
        self.kit_id = sha256(manifest.read_bytes()).hexdigest() if manifest.is_file() else None

    def prepare(self, ctx):
        from .pick_calibration import verify_installed
        from tools.lib.pick_compute import ROOT
        ctx.note('현장 보정 확인 · '+verify_installed(ROOT)[:7])
        ctx.note('집기 진입 여유 · 고정 손가락 6 mm · 움직이는 손가락 4 mm · 닫히며 작은 물체 이동 허용')
        ctx.note('리프트 높이: '+('엔코더 환산 · 630mm 처짐 보상 없음' if self.options.get('lift_height_model')=='encoder_no_sag' else '기존 630mm 실효 기준'))
        super().prepare(ctx)

    def can_prepare_next_test(self):
        h=self.hardware
        return bool(not self.blocked and h is not None
                    and getattr(h,'prepared',False) and not h.stopped and h.options.get('single_capture_test')
                    and ((h.session is None and getattr(h,'inputs',None) is None)
                         or (h.session is not None and h.session.state=='HOLDING')))

    def ready(self):
        if self.after_motion and self.can_prepare_next_test():
            return False,'동작 완료 · 물체를 치운 뒤 «다음 시험 준비»를 눌러 주세요. 정지는 필요 없습니다.'
        if getattr(self,'controller_reset_required',False):
            return False,('베이스 정렬 후 제어기가 정지 상태입니다. 받침을 유지한 채 제어기(SWD)를 리셋하고 '
                          '«픽 스테이션 초기화»를 누른 뒤 «최상단 · 카메라 검사»로 위치를 확인하세요.')
        return super().ready()

    def reset(self, ctx):
        if getattr(self,'controller_reset_required',False) and self.hardware is None:
            # The alignment owner already retired after a confirmed STOP. The
            # operator reset the controller; models and the worker stay loaded.
            self.controller_reset_required=False
            ctx.note('베이스 정렬 후 제어기 리셋 확인 · 다음 요청은 새 제어기 준비부터 시작합니다.')
            return
        if self.hardware is not None and not getattr(ctx,'supported_empty',False):
            raise Rejected('물체 제거와 양팔·리프트 받침 확인이 필요합니다.')
        if self.can_prepare_next_test():
            self.hardware.prepare_next_test(ctx)
            self.after_motion=False
            return
        had_hardware=self.hardware is not None
        self.blocked='초기화 중'
        if had_hardware:
            # Do not abandon a device owner unless its stop has been confirmed.
            self.hardware.reset_supported_empty=True
            self.hardware.abort()
            self.hardware.close()
            self.hardware=None
        if self.worker is not None:
            self.worker.close();self.worker=None
        self.prepared=False;self.after_motion=False
        self.prepare(ctx)
        if had_hardware:
            ctx.note('장치 연결 정리 완료 · 받침을 유지하고 제어기를 한 번 리셋한 뒤 다음 요청을 시작하세요. 리프트 위치는 직접 확인하세요.')
        self.blocked=''

    def check_operation(self, operation, supported_empty_bottom, supported_empty_top):
        confirmations = (supported_empty_bottom, supported_empty_top)
        if any(type(flag) is not bool for flag in confirmations):
            raise Rejected('리프트 위치 확인 값이 잘못됐습니다.')
        if operation == 'top-inspect':
            if any(confirmations) or self.hardware is not None:
                raise Rejected('장치가 자세를 유지하는 중에는 독립 촬영으로 바꿀 수 없습니다.')
        elif operation == 'top-align':
            if confirmations != (False, True):
                raise Rejected('리프트 최상단, 빈 양팔·리프트 받침과 베이스 좌우 이동 공간을 확인해 주세요.')
            if self.hardware is not None:
                raise Rejected('팔 자세 유지 중에는 베이스를 움직이지 않습니다. 다음 시험 준비나 초기화 후 다시 요청하세요.')
            if not self.options.get('top_baseline') or not Path(self.options['top_baseline']).is_file():
                raise Rejected('이 로봇의 제어기 보정 기준 파일을 열 수 없습니다.')
        elif operation in ('top-pick', 'bottom-pick', 'observe-rise-pick'):
            top = operation == 'top-pick'
            if confirmations != (not top, top):
                raise Rejected('선택한 리프트 위치와 빈 양팔의 지지 상태를 확인해 주세요.')
            if self.hardware is not None and self.hardware.guard.supported_top != top:
                raise Rejected('현재 자세 유지 세션과 다른 리프트 기준을 선택했습니다.')
            baseline = self.options.get('top_baseline' if top else 'baseline')
            if not baseline or not Path(baseline).is_file():
                raise Rejected('이 로봇의 제어기 보정 기준 파일을 열 수 없습니다.')
        else:
            raise Rejected('리프트 위치와 작업을 선택해 주세요.')
        return operation

    def _camera_capture(self, ctx, mode):
        from PIL import Image
        from .camera_evidence import load_capture_profile
        from .s2_rgbd_acquisition import Capture
        from .stationary_rgbd import pick_depth_exposure_us
        from tools.lib.pick_compute import ROOT

        profile, reference = load_capture_profile(ROOT, mode)
        ctx.progress('광각 RGB-D 촬영 중' if mode == 'wide' else '근거리 RGB-D 촬영 중')
        ctx.check()
        try:
            with Capture(profile, reference, retain_frame=True,
                         depth_exposure_us=pick_depth_exposure_us(mode)) as camera:
                ctx.check()
                record, (color, depth, metadata) = camera.next_frame()
        except (ValueError, RuntimeError, OSError) as exc:
            raise Rejected('D415 촬영을 완료하지 못했습니다. 카메라 연결과 다른 뷰어의 사용 여부를 확인해 주세요. '+str(exc)) from exc
        ctx.check()
        folder = ctx.folder/mode/record['capture_id']
        folder.mkdir(parents=True)
        Image.fromarray(color[:, :, ::-1]).save(folder/'color.png')
        Image.fromarray(depth).save(folder/'depth_aligned.png')
        (folder/'metadata.json').write_text(json.dumps(metadata, allow_nan=False))
        ctx.artifact(folder/'color.png', '새 광각 촬영' if mode == 'wide' else '새 근거리 촬영')
        return dict(snapshot=str(folder), camera_mode=mode)

    def capture(self, ctx, request):
        if self.worker is None:
            self._prepare_worker(ctx)
        operation = request['operation']
        if operation in ('top-inspect', 'top-align'):
            return dict(request, **self._camera_capture(ctx, 'wide'))
        if operation not in ('top-pick', 'bottom-pick', 'observe-rise-pick'):
            raise Rejected('지원하지 않는 동작입니다.')
        top = operation == 'top-pick'
        self.check_operation(operation, request.get('supported_empty_bottom', False),
                             request.get('supported_empty_top', False))
        if self.hardware is None and top:
            ctx.progress('상단 팔 시험 준비 · 리프트 이동 없음')
            options = dict(self.options, execute=True, supported_empty_top=True,
                supported_empty_bottom=False, camera_mode='near',
                baseline=self.options['top_baseline'], rise_after_observation=False,
                fixed_base_station=True, single_capture_test=True,
                supported_startup_release=request.get('supported_startup_release',False))
            self._prepare_hardware(ctx, options)
            ctx.note('고정 위치 팔 시험 · 바퀴 검사 없음 · 근거리 촬영 한 번 사용')
        if self.hardware is None:
            return dict(request, **self._camera_capture(ctx, 'wide'), preview_only=True)
        return super().capture(ctx, dict(request, camera_mode=self.hardware.options['camera_mode']))

    def detect(self, ctx, value):
        if value['operation'] in ('top-inspect', 'top-align') or value.get('preview_only'):
            return self._detect_camera(ctx, value)
        return super().detect(ctx, value)

    def _detect_camera(self, ctx, value):
        ctx.progress('광각 물체 탐지·분할 중' if value['camera_mode'] == 'wide' else '근거리 물체 탐지·분할 중')
        path = Path(self._compute(ctx, 'locate', snapshot=value['snapshot'], prompt=value['prompt']))
        ctx.artifact(path.with_name('detections.jpg'), '광각 탐지 결과' if value['camera_mode']=='wide' else '근거리 탐지 결과')
        ctx.artifact(path, '탐지·분할 측정값')
        report = json.loads(path.read_text())
        if not any(d['label'] == value['prompt'] for d in report['detections']):
            raise Rejected('요청한 물체를 찾지 못했습니다. 화면에 물체 전체가 보이는지 확인하고 다시 요청해 주세요.')
        return dict(value, detections=str(path))

    def _camera_result(self, ctx, value):
        model = 'GraspNet' if self.options['candidate_source'] == 'graspnet' else self.options['candidate_source']
        ctx.progress(model+' 파지 후보 계산·깊이 검사 중')
        output = ctx.folder/value['camera_mode']/'candidates.json'
        result = self._compute(ctx, 'camera_candidates', snapshot=value['snapshot'],
            detections=value['detections'], prompt=value['prompt'], camera_mode=value['camera_mode'], output=str(output),
            location=value.get('location'))
        ctx.artifact(output, '광각 파지 후보·깊이' if value['camera_mode']=='wide' else '근거리 파지 후보·깊이')
        if result['depth_fraction'] is not None:
            ctx.note(f"요청 물체의 유효 깊이 {result['depth_fraction']:.0%}")
        height = result.get('surface_height')
        if height is not None:
            ctx.note(f"상판 높이 약 {height['height_above_model_floor_m']*100:.1f} cm · "
                     '보정 파일의 바닥 기준', surface_height=height)
        return result

    def _alignment(self, ctx, result):
        from .base_alignment import describe, load_alignment_profile, plan_lateral_alignment
        if result.get('object_base_xyz_m') is None:
            raise Rejected('요청한 물체 하나의 3차원 위치를 확인하지 못해 정렬 이동량을 계산할 수 없습니다.')
        plan = plan_lateral_alignment(result['object_base_xyz_m'], load_alignment_profile())
        ctx.note('왼팔 기준 정렬: '+describe(plan), alignment=plan)
        return plan

    def candidates(self, ctx, value):
        if value['operation'] not in ('top-inspect', 'top-align') and not value.get('preview_only'):
            return super().candidates(ctx, value)
        mode, result = self._inspect(ctx, value)
        if value['operation'] == 'top-align':
            try:
                plan = self._alignment(ctx, result)
            except ValueError as exc:
                raise Rejected(str(exc)) from exc
            return dict(value, camera_mode=mode, alignment=plan)
        if value['operation'] == 'top-inspect':
            alignment = None
            if result.get('object_base_xyz_m') is not None:
                try:
                    alignment = self._alignment(ctx, result)
                except ValueError as exc:
                    ctx.note('왼팔 기준 정렬 계산 불가: '+str(exc))
            return dict(status='OBSERVED_ONLY', physical_success=False, camera_mode=mode,
                        candidates=result['count'], alignment=alignment,
                        reason=f"파지 후보 {result['count']}개를 찾았습니다.")
        if result['surface_error']:
            raise Rejected(result['surface_error'])
        return dict(value, camera_mode=mode)

    def align(self, ctx, value):
        """Camera-planned lateral shift; one controller owner, ended by STOP and retired."""
        from .base_alignment import describe
        if value.get('operation') != 'top-align':
            raise Rejected('베이스 정렬 작업이 아닙니다.')
        plan = value['alignment']
        if plan['within_tolerance']:
            return dict(status='UNVERIFIED', physical_success=False, alignment=plan,
                        reason='물체가 이미 왼팔 정렬 범위 안입니다. 베이스는 움직이지 않았습니다. '
                               '바로 «최상단 · 실물 파지»를 요청하세요.')
        if self.hardware is not None:
            raise Rejected('팔 자세 유지 중에는 베이스를 움직이지 않습니다.')
        ctx.progress('베이스 정렬용 제어기 준비 · 리프트 이동 없음')
        options = dict(self.options, execute=True, supported_empty_top=True, supported_empty_bottom=False,
                       camera_mode='near', baseline=self.options['top_baseline'], rise_after_observation=False,
                       fixed_base_station=True, single_capture_test=False,
                       supported_startup_release=value.get('supported_startup_release', False))
        from .pick_hardware import UnifiedPickHardware
        self.hardware = (self.hardware_factory or UnifiedPickHardware)(options)
        try:
            self.hardware.prepare(ctx)
        except Exception as error:
            if getattr(error, 'diagnostics', None):
                path = ctx.folder/'preparation-failure.json'
                path.write_text(json.dumps(dict(reason=str(error), diagnostics=error.diagnostics), indent=2, ensure_ascii=False))
                ctx.artifact(path, '장치 준비 실패 명령·응답')
            raise
        ctx.check()
        ctx.progress(describe(plan))
        self.after_motion = True  # A failed strafe leaves an unknown base pose.
        result = self.hardware.lateral_shift(ctx, plan)
        hardware, self.hardware = self.hardware, None
        hardware.close()
        self.after_motion = False
        self.controller_reset_required = True
        side = '왼쪽' if plan['direction'] == 'left' else '오른쪽'
        return dict(status='UNVERIFIED', physical_success=False, alignment=plan, base_motion=result,
                    reason=f'베이스를 {side}으로 약 {abs(plan["shift_m"])*100:.1f} cm 이동하고 정지를 확인했습니다. '
                           '실제 이동량은 다음 카메라 검사에서 확인합니다. 제어기 리셋 → «픽 스테이션 초기화» → '
                           '«최상단 · 카메라 검사» → «최상단 · 실물 파지» 순서로 진행하세요.')

    def prepare_pick(self, ctx, value):
        if not value.get('preview_only'):
            return value  # An existing measured hold already supplied this capture.
        top = value['operation'] == 'top-pick'
        ctx.progress('장치 기준 등록과 현재 자세 유지 · '+('최상단' if top else '최하단'))
        options = dict(self.options, execute=True, supported_empty_top=top, supported_empty_bottom=not top,
                       camera_mode=value['camera_mode'], baseline=self.options['top_baseline' if top else 'baseline'],
                       rise_after_observation=value['operation'] == 'observe-rise-pick',
                       supported_startup_release=value.get('supported_startup_release',False))
        self._prepare_hardware(ctx, options)
        # Preserve the existing fresh hold/capture binding. Preview pixels never
        # become an executable capture by changing their timestamps or metadata.
        ctx.progress('자세 유지 후 계획용 재촬영')
        value = super().capture(ctx, dict(value, preview_only=False))
        ctx.progress('재관측 물체 확인 중')
        value = super().detect(ctx, value)
        ctx.progress('실측 팔 자세에 집기 후보 연결 중')
        return super().candidates(ctx, value)

    def _inspect(self, ctx, value):
        result = self._camera_result(ctx, value)
        if result['reason'] in ('partial_mask_depth', 'insufficient_mask_depth', 'fragmented_mask_cloud', 'no_coherent_cluster'):
            ctx.note('광각 깊이가 부족해 근거리 모드로 한 번 다시 촬영합니다.')
            value = self._detect_camera(ctx, dict(value, **self._camera_capture(ctx, 'near')))
            result = self._camera_result(ctx, value)
        if result['status'] != 'CAMERA_CANDIDATES_ONLY' or not result['count']:
            raise Rejected('파지 후보를 확정하지 못했습니다. 물체가 가려졌거나 깊이가 비는 곳을 확인해 주세요. '
                           +str(result['reason'] or result['status']))
        return value['camera_mode'], result

    def rise(self, ctx, value):
        if value['operation'] != 'observe-rise-pick':
            raise Rejected('리프트 상승 작업이 아닙니다.')
        ctx.progress('하단 관측 기준으로 리프트 상승 경로 검사 중')
        proof=self._compute(ctx,'lift_preflight',prepared=value['prepared'],
                            output=str(ctx.folder/'lift-check'),board=self.hardware.guard.board)
        ctx.artifact(Path(proof['report']),'리프트 상승 경로 검사')
        ctx.check()
        # A failed request after movement cannot retry as an unchanged bottom hold.
        self.after_motion=True
        ctx.progress('리프트 최상단으로 상승 중')
        self.hardware.rise_to_pick_height(ctx,proof)
        folder,images,evidence=self.hardware.capture(ctx,subfolder='arrival')
        self._compute(ctx,'save',folder=str(folder),images=images,evidence=evidence)
        ctx.artifact(folder/'robot_capture.json','상승 후 계획 시작 자세')
        prepared=self._compute(ctx,'bind_arrival',prepared=value['prepared'],snapshot=value['snapshot'],
            detections=value['detections'],arrival=str(folder),proof=proof,
            output=str(ctx.folder/'prepared-at-top.json'))
        ctx.note('하단 사진·좌표·후보 고정 · 상승 후 물체 비교 생략 · 현재 팔 자세로 계획')
        return dict(value,prepared=prepared)

    def verify(self, ctx, value):
        if value.get('operation') != 'top-pick' or self.options.get('plan_only'):
            return super().verify(ctx, value)
        if self.hardware.session is None or self.hardware.session.state != 'HOLDING':
            raise ValueError('팔 동작 완료 상태를 확인하지 못했습니다.')
        ctx.note('팔 동작 완료 · 추가 촬영 없음 · 집기 성공 여부는 사용자 확인')
        return dict(status='UNVERIFIED', physical_success=False,
                    reason='팔 동작을 완료했습니다. 물체를 잡았는지는 직접 확인해 주세요.',
                    verification_basis='operator_observation', verification_samples=[],
                    bundle=value['bundle'])

    def on_failure(self, exc):
        if self.hardware is not None:
            return super().on_failure(exc)
        if not isinstance(exc, Rejected) and self.worker is not None:
            # A cancelled model job must finish or be retired before another request.
            self.worker.close()
            self.worker = None
