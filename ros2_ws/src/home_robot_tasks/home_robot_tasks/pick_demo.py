"""Synthetic stage adapter for the local monitor. Never opens any device."""
import shutil


class DemoBackend:
    mode = 'demo'

    def __init__(self, *, step_seconds=.8, image=None, fail_stage=None):
        self.step_seconds = step_seconds
        self.image = image
        self.fail_stage = fail_stage
        self.prepared = False

    def prepare(self, ctx):
        ctx.note('모의 실행기 준비: 실제 모델을 로드하거나 장치에 연결하지 않습니다.')
        ctx.pause(self.step_seconds)
        self.prepared = True

    def ready(self):
        return self.prepared, '' if self.prepared else '모의 실행기 준비 중'

    def tick(self):
        pass

    def _step(self, name, ctx, value):
        ctx.pause(self.step_seconds)
        if self.fail_stage == name:
            raise ValueError('화면 확인용 모의 실패: '+name)
        return value

    def capture(self, ctx, value):
        result = self._step('capture', ctx, value)
        if self.image:
            path = ctx.folder/('demo-reference'+self.image.suffix.lower())
            shutil.copyfile(self.image, path)
            ctx.artifact(path, '저장된 참고 이미지 · 새 촬영 아님')
        return result

    def detect(self, ctx, value):
        return self._step('detect', ctx, value)

    def candidates(self, ctx, value):
        return self._step('candidates', ctx, value)

    def plan(self, ctx, value):
        return self._step('plan', ctx, value)

    def execute(self, ctx, value):
        return self._step('execute', ctx, value)

    def verify(self, ctx, value):
        self._step('verify', ctx, value)
        return dict(status='SUCCEEDED', physical_success=False,
                    reason='모의 단계 전환 완료. 실제 파지 결과가 아닙니다.')

    def abort(self):
        pass

    def close(self):
        self.prepared = False
