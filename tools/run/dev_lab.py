#!/usr/bin/env python3
"""Hardware-free studio lab: doctor, composed FSM scenarios, and local client API."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import secrets
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'ros2_ws/src/home_robot_tasks'),
                str(ROOT / 'ros2_ws/src/so101_arm_bridge')]

from tools.lib.run_manifest import provenance, sha256, write_json


def doctor(gpu=False):
    available = {}
    for name in ('numpy', 'pytest', 'yaml', 'serial'):
        available[name] = importlib.util.find_spec(name) is not None
    checks = {'offline_runtime': available['numpy']}
    environment = provenance()
    sources = environment.pop('source_sha256')
    environment['source_file_count'] = len(sources)
    result = {'schema_version': 1, 'hardware_commands': 0, 'checks': checks,
              'available_modules': available, 'provenance': environment}
    if gpu:
        try:
            import torch
            from transformers import Qwen3VLForConditionalGeneration, Sam2Model, Sam2Processor
            import pointnet2._ext
            import grasp_nms
            result['cuda_available'] = torch.cuda.is_available()
            if result['cuda_available']:
                result['gpu'] = torch.cuda.get_device_name(0)
                result['cuda_calculation'] = (torch.ones(1, device='cuda') + 1).item()
            checks['gpu_runtime'] = result['cuda_available'] and result.get('cuda_calculation') == 2
        except (ImportError, RuntimeError, OSError) as error:
            checks['gpu_runtime'] = False
            result['gpu_error'] = str(error)
        for name, relative in {
            'qwen_weights': 'artifacts/models/qwen3-vl-4b/model.safetensors.index.json',
            'graspnet_weights': 'artifacts/models/checkpoint-rs.tar',
            'graspnet_source': 'artifacts/graspnet-baseline/models/graspnet.py',
        }.items():
            checks[name] = (ROOT / relative).is_file()
    result['passed'] = all(checks.values())
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=ROOT / 'config/studio.simulation.json')
    sub = parser.add_subparsers(dest='command', required=True)
    check = sub.add_parser('doctor', help='Inspect this interpreter; --gpu also checks CUDA/extension imports')
    check.add_argument('--gpu', action='store_true')
    for name in ('run', 'suite'):
        command = sub.add_parser(name)
        command.add_argument('--output', type=Path, required=True, help='New directory; never overwrites an existing run')
        if name == 'run':
            command.add_argument('--scenario', default='success')
    serve = sub.add_parser('serve', help='Loopback API over KinematicRobot, separate from composed scenario runs')
    serve.add_argument('--state', type=Path, default=ROOT / 'output/dev-lab-api')
    serve.add_argument('--port', type=int, default=8765)
    args = parser.parse_args(argv)
    try:
        if args.command == 'doctor':
            result = doctor(args.gpu)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0 if result['passed'] else 1

        from home_robot_tasks.offline_lab import load_profile, run_scenario, SCENARIOS
        profile = load_profile(args.profile)
        if args.command == 'serve':
            from home_robot_tasks.gateway import main as gateway_main
            state = args.state.resolve()
            state.mkdir(parents=True, exist_ok=True)
            # Avoid changing a persisted request's world on restart.
            for name in ('world', 'map'):
                path = state / (name + '.json')
                if path.exists() and json.loads(path.read_text()) != profile[name]:
                    raise ValueError('API profile changed; select a new --state directory')
                write_json(path, profile[name])
            token = state / 'token'
            if not token.exists():
                with os.fdopen(os.open(token, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), 'w') as stream:
                    stream.write(secrets.token_urlsafe(32) + '\n')
            write_json(state / 'request.json', profile['request'])
            print(f'Local simulation API: http://127.0.0.1:{args.port}\nToken file: {token}\nRequest file: {state / "request.json"}', flush=True)
            gateway_main(['--world', str(state / 'world.json'), '--map', str(state / 'map.json'),
                          '--token-file', str(token), '--database', str(state / 'requests.db'),
                          '--port', str(args.port)])
            return 0

        if args.command == 'run' and args.scenario not in SCENARIOS:
            raise ValueError('scenario must be one of: ' + ', '.join(SCENARIOS))
        output = args.output.resolve()
        output.mkdir(parents=True, exist_ok=False)
        write_json(output / 'manifest.json', {'schema_version': 1, 'mode': 'simulation',
                   'provenance': provenance(), 'profile_sha256': sha256(args.profile),
                   'stack_template_sha256': sha256(profile['template_path'])})
        write_json(output / 'profile.resolved.json', profile)
        results = []
        for scenario in SCENARIOS if args.command == 'suite' else (args.scenario,):
            folder = output / scenario
            folder.mkdir()
            with (folder / 'events.jsonl').open('w', encoding='utf-8') as stream:
                def record(event):
                    stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
                    stream.flush()
                try:
                    report = run_scenario(profile, scenario, record)
                except Exception as error:
                    report = {'scenario': scenario, 'passed': False, 'error': f'{type(error).__name__}: {error}'}
            write_json(folder / 'result.json', report)
            results.append({'scenario': scenario, 'passed': report['passed'],
                            'status': report.get('result', {}).get('status'),
                            'report': f'{scenario}/result.json'})
            print(f'{scenario}: {"PASS" if report["passed"] else "FAIL"} {results[-1]["status"]}', flush=True)
        summary = {'schema_version': 1, 'mode': 'simulation', 'evidence_source': 'synthetic',
                   'physical_task_completed': False, 'hardware_commands': 0,
                   'passed': all(r['passed'] for r in results), 'scenarios': results}
        write_json(output / 'report.json', summary)
        print(f'Report: {output / "report.json"}')
        return 0 if summary['passed'] else 1
    except (OSError, ValueError, KeyError, ImportError) as error:
        parser.exit(2, f'Lab error: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
