"""Explicitly resume a failed pre-execution pick from its immutable bottom observation."""
from pathlib import Path
import json
import shutil

from .pick_service import Rejected
from .pick_web_backend import WebPickBackend


def saved_request(folder):
    folder=Path(folder)
    request=json.loads((folder/'result.json').read_text())
    stages={s['id']:s['status'] for s in request['stages']}
    if (request.get('operation')!='observe-rise-pick' or stages.get('LIFTING')!='done'
            or stages.get('PLANNING')!='failed' or stages.get('EXECUTING')!='skipped'):
        raise ValueError('상승 완료 후 계획에서 중단되고 팔 실행은 시작하지 않은 요청만 이어갈 수 있습니다.')
    prepared=folder/'prepared-at-top.json'
    data=json.loads(prepared.read_text())
    snapshot=folder/data['capture_id']
    from .snapshot_io import hashes
    if hashes(snapshot)!=data['source_sha256']:
        raise ValueError('이전 관측의 원본 RGB-D 파일이 변경됐습니다.')
    detections=[p for p in snapshot.glob('*/detections.json')
                if json.loads(p.read_text()).get('source_sha256')==data['source_sha256']]
    if len(detections)!=1 or request['prompt']!=data['label']:
        raise ValueError('이전 요청의 물체와 관측 결과가 일치하지 않습니다.')
    from .camera_evidence import load_capture_profile
    from tools.lib.pick_compute import ROOT
    modes=[mode for mode in ('wide','near','legacy')
           if load_capture_profile(ROOT,mode)[0]['calibration_id']==data['review']['calibration_id']]
    if len(modes)!=1:raise ValueError('이전 촬영과 일치하는 카메라 보정이 없습니다.')
    return dict(request=request,prepared=prepared,snapshot=snapshot,detections=detections[0],camera_mode=modes[0])


class ResumePickBackend(WebPickBackend):
    def __init__(self, options, **kwargs):
        super().__init__(options,**kwargs)
        self.resume_source=saved_request(options['resume_from'])
        request=self.resume_source['request']
        self.resume_info=dict(request_id=request['id'],text=request['text'],prompt=request['prompt'])

    def check_operation(self, operation, supported_empty_bottom, supported_empty_top):
        if operation!='resume-top-pick':
            return super().check_operation(operation,supported_empty_bottom,supported_empty_top)
        super().check_operation('top-pick',supported_empty_bottom,supported_empty_top)
        if self.hardware is not None:
            raise Rejected('이어서 집기는 초기화한 실행기에서 시작해 주세요.')
        return operation

    def prepare_resume(self, ctx, value):
        # Selecting this operation confirms an unchanged base/object scene and
        # the supported, unchanged arrival height. It does not reuse an old controller anchor.
        if value.get('scene_unchanged') is not True:raise Rejected('베이스·물체가 이전 관측 그대로인지 확인해 주세요.')
        source=saved_request(self.options['resume_from'])
        if value['prompt']!=source['request']['prompt']:
            raise Rejected('이어가기는 이전에 관측한 '+source['request']['prompt']+'에만 사용할 수 있습니다.')
        snapshot=ctx.folder/source['snapshot'].name
        shutil.copytree(source['snapshot'],snapshot)
        prepared=ctx.folder/'prepared-original-at-top.json'
        shutil.copyfile(source['prepared'],prepared)
        detections=snapshot/source['detections'].relative_to(source['snapshot'])
        ctx.artifact(snapshot/'color.png','이전 하단 사진 · 좌표·후보 그대로 사용')
        ctx.note('이전 하단 관측으로 이어가기 · 물체 재탐지·리프트 상승 생략',
                 source_request=source['request']['id'])
        from .camera_evidence import load_capture_profile
        from .pick_lift_transition import planning_review
        from tools.lib.pick_compute import ROOT
        original=json.loads(prepared.read_text())
        profile,_=load_capture_profile(ROOT,source['camera_mode'])
        planning_review(original,profile)
        height_source=original['start_revalidation']['arrival_evidence']['platform']
        options=dict(self.options,resume_height_source=height_source,execute=True,supported_empty_top=True,supported_empty_bottom=False,
                     camera_mode=source['camera_mode'],baseline=self.options['top_baseline'],
                     fixed_bottom_observation=True,rise_after_observation=False,
                     supported_startup_release=value.get('supported_startup_release',False))
        self._prepare_hardware(ctx,options)
        self.after_motion=True
        # This frame timestamps fresh joint/registered-top evidence. Its pixels
        # never replace the bottom target geometry or enter grasp inference.
        folder,images,evidence=self.hardware.capture(ctx,subfolder='resume-start')
        self._compute(ctx,'save',folder=str(folder),images=images,evidence=evidence)
        resumed=self._compute(ctx,'bind_resume',prepared=str(prepared),arrival=str(folder),
                              output=str(ctx.folder/'prepared-resumed.json'),scene_unchanged=value['scene_unchanged'])
        ctx.artifact(folder/'robot_capture.json','이어가기 시점의 현재 팔·리프트 기준')
        return dict(value,snapshot=str(snapshot),detections=str(detections),prepared=resumed,
                    camera_mode=source['camera_mode'])
