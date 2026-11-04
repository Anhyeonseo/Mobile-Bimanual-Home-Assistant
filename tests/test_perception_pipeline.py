import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from tools.run import run_perception_pipeline as pipeline
from tools.lib.run_manifest import sha256


@pytest.fixture
def inputs(tmp_path, monkeypatch):
    sample, model = tmp_path / 'sample', tmp_path / 'model'
    sample.mkdir()
    model.mkdir()
    for name in ('color.png', 'depth.png', 'workspace_mask.png', 'meta.mat'):
        (sample / name).write_bytes(name.encode())
    (model / 'model.safetensors').write_bytes(b'model')
    checkpoint = tmp_path / 'checkpoint.tar'
    checkpoint.write_bytes(b'checkpoint')
    monkeypatch.setattr(pipeline, 'provenance', lambda: {})
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(cuda=SimpleNamespace(
        is_available=lambda: True, get_device_name=lambda _: 'test-gpu')))
    output = tmp_path / 'run'
    return sample, output, ['--sample', str(sample), '--model', str(model),
                            '--checkpoint', str(checkpoint), '--prompt', '리모컨', '--output', str(output)]


@pytest.mark.parametrize('outcome,expected,calls', [
    ('empty', 'NO_DETECTIONS', 1), ('mask_rejected', 'MASK_REJECTED', 2),
    ('no_grasp', 'NO_GRASP_CANDIDATES', 3), ('accepted', 'CANDIDATES_READY', 3),
    ('process_failed', 'FAILED', 1), ('timeout', 'FAILED', 1),
    ('interrupt', 'CANCELLED', 1),
])
def test_pipeline_stops_at_actual_stage_outcome_and_preserves_report(inputs, monkeypatch, outcome, expected, calls):
    sample, output, args = inputs
    observed = []

    def run(command, **kwargs):
        observed.append(command)
        assert kwargs['env']['HF_HUB_OFFLINE'] == '1'
        if outcome == 'timeout':
            raise subprocess.TimeoutExpired(command, kwargs['timeout'])
        if outcome == 'interrupt':
            raise KeyboardInterrupt
        if outcome == 'process_failed':
            return SimpleNamespace(returncode=1)
        folder = Path(command[command.index('--output') + 1])
        folder.mkdir()
        stage = len(observed)
        if stage == 1:
            payload, name = {'detections': [] if outcome == 'empty' else [{'label': 'remote'}]}, 'detections.json'
        elif stage == 2:
            payload, name = {'objects': [{'index': 0, 'status': 'REJECTED' if outcome == 'mask_rejected' else 'OK'}],
                             'input_sha256': {n: sha256(sample / n) for n in ('color.png', 'depth.png', 'workspace_mask.png', 'meta.mat')}}, 'objects.json'
        else:
            payload, name = {'generated': 10, 'target_associated': 3, 'collision_and_empty_filtered': 2,
                             'saved': 0 if outcome == 'no_grasp' else 1}, 'grasps.json'
        (folder / name).write_text(json.dumps(payload))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(pipeline.subprocess, 'run', run)
    assert pipeline.main(args) == (130 if expected == 'CANCELLED' else 1 if expected == 'FAILED' else 0)
    report = json.loads((output / 'report.json').read_text())
    assert report['status'] == expected and len(observed) == calls
    assert not report['robot_feasibility_checked'] and not report['physical_task_completed']
    assert report['hardware_commands'] == 0
    assert len(report['stages']) == calls


def test_plan_only_needs_no_gpu_and_writes_nothing(inputs, monkeypatch):
    _, output, args = inputs
    monkeypatch.setitem(sys.modules, 'torch', None)
    assert pipeline.main([*args, '--plan-only']) == 0
    assert not output.exists()


def test_gpu_unavailable_is_an_explicit_failed_run(inputs, monkeypatch):
    _, output, args = inputs
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: False)))
    assert pipeline.main(args) == 1
    report = json.loads((output / 'report.json').read_text())
    assert report['status'] == 'FAILED' and 'outside the sandbox' in report['error']
    assert not report['stages']
