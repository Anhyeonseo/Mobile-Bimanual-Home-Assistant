#!/usr/bin/env python3
"""Run the integrated home-assistant UI or an offline end-to-end regression.

No subcommand connects a robot, downloads a model or flashes firmware.
"""
from pathlib import Path
import argparse
import json
import os
import secrets
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'ros2_ws/src/home_robot_tasks'))
sys.path.insert(0, str(ROOT/'ros2_ws/src/so101_arm_bridge'))


def demo(fail_skill=None, language_url=None):
    from home_robot_tasks.application import RobotApplication
    from home_robot_tasks.fake_robot import FakeRobot
    from home_robot_tasks.gateway import Gateway, GatewayStore
    from home_robot_tasks.navigation import NavigationMap
    from home_robot_tasks.assistant.contracts import Context
    from home_robot_tasks.assistant.service import AssistantService
    from home_robot_tasks.assistant.world import WorldStore
    read = lambda name: json.loads((ROOT/'config'/name).read_text())
    clock = [0.0]
    robot = FakeRobot(); robot.fail_skill = fail_skill
    app = RobotApplication(read('home.example.json'), NavigationMap(read('navigation_map.simulation.json')), robot)
    world = WorldStore()
    context = Context(app.map.map_id, app.map.revision, 'simulation-calibration', 'offline-run')
    assistant = AssistantService(app, world, context, lambda: clock[0])
    assistant.refresh_demo()
    gateway = Gateway(app, 'offline-test-key-'*3, GatewayStore(':memory:'), assistant=assistant)
    gateway.now = lambda: clock[0]
    auth = 'Bearer '+gateway.token
    try:
        utterance = '거실 탁자에 있는 검은 마커펜을 침실 협탁에 놓아줘'
        inference = None
        if language_url:
            from home_perception.language.local_model import LocalLanguageModel
            from home_robot_tasks.assistant.local_language import meaning_from_slots
            inference = LocalLanguageModel(language_url).extract(utterance, 'offline-delivery')
            assistant.prepare(utterance, 'offline-delivery',
                meaning=meaning_from_slots(inference['extraction']), parser='local_language_v1')
        else:
            assistant.prepare(utterance, 'offline-delivery')
        preview = assistant.preview('offline-delivery')
        permit = gateway.dispatch('GET', '/v1/permit', auth, None)[1]['permit']
        code, _ = gateway.dispatch('POST', '/v1/assistant/requests/offline-delivery/execute', auth,
                                  {'preview_id': preview['preview_id']}, permit)
        if code != 202:
            raise RuntimeError('offline execution admission failed')
        for _ in range(1200):
            gateway.tick(); clock[0] += .05
            if app.lease.owner is None:
                break
        result = assistant.get('offline-delivery')
        expected = 'FAILED' if fail_skill else 'SUCCEEDED'
        if result['status'] != expected:
            raise RuntimeError(f'expected {expected}, got {result["status"]}')
        return dict(mode='simulation', physical_task_completed=False, hardware_commands=0,
                    result=result, object=world.get('marker-1'), inference=inference,
                    confirmation_source='automated_simulation_test',
                    candidate_count=preview['placement']['evaluated'],
                    feasible_candidates=len(preview['placement']['candidates']))
    finally:
        gateway.store.db.close(); world.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    serve = sub.add_parser('serve', help='authenticated loopback development UI')
    serve.add_argument('--state-dir', type=Path, default=ROOT/'output/home-assistant')
    serve.add_argument('--port', type=int, default=18766)
    serve.add_argument('--household-source-map', type=Path)
    serve.add_argument('--household-profile', type=Path, help='directory containing reviewed household-labels.json/source-map.json')
    serve.add_argument('--local-language-url', help='local llama.cpp endpoint, e.g. http://127.0.0.1:18767')
    model_server = sub.add_parser('language-server', help='run the installed local language model on GPU')
    model_server.add_argument('--model', type=Path, default=ROOT/'artifacts/s5-language/Qwen3-4B-Q4_K_M.gguf')
    model_server.add_argument('--binary', type=Path, default=ROOT/'artifacts/s5-language/llama-cuda-build/bin/llama-server')
    model_server.add_argument('--port', type=int, default=18767)
    offline = sub.add_parser('demo', help='run the real task state machine with synthetic skill feedback')
    offline.add_argument('--fail-skill', choices=['pick', 'navigate', 'place', 'verify_delivery'])
    offline.add_argument('--output', type=Path)
    offline.add_argument('--local-language-url')
    from tools.lib.integration_rehearsal import SCENARIOS
    rehearsal = sub.add_parser('rehearse', help='text request through composed runtime; synthetic devices only')
    rehearsal.add_argument('--scenario', choices=SCENARIOS, default='success')
    rehearsal.add_argument('--output', required=True, type=Path)
    rehearsal.add_argument('--local-language-url')
    verify = sub.add_parser('verify', help='local S4b/S5 regression, fixtures and reports; never connects a robot')
    verify.add_argument('--stage', choices=['all','s4b','s5','language','ros'], default='all')
    verify.add_argument('--output', required=True, type=Path, help='new results directory; existing data is never overwritten')
    verify.add_argument('--local-language-url', help='optional running loopback language server; no automatic start/download')
    verify.add_argument('--with-ros', action='store_true', help='isolated local Nav2 action/MoveIt mesh checks; no hardware launch')
    sub.add_parser('readiness', help='show unresolved physical integration inputs')
    args = parser.parse_args(argv)
    if args.command == 'rehearse':
        from tools.lib.integration_rehearsal import rehearse
        report = rehearse(args.scenario, language_url=args.local_language_url)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        print(json.dumps(dict(passed=report['passed'], status=report['result']['status'],
            hardware_commands=0, output=str(args.output)), ensure_ascii=False))
        return 0 if report['passed'] else 1
    if args.command == 'verify':
        from tools.lib.stage_test_suite import run_suite
        report = run_suite(args.output, stage=args.stage, language_url=args.local_language_url, with_ros=args.with_ros)
        print(json.dumps(dict(passed=report['passed'], physical_test_ready=False,
            language_model=report['language_model'], output=str(args.output/'report.json')), ensure_ascii=False))
        return 0 if report['passed'] else 1
    if args.command == 'language-server':
        if not args.model.is_file() or not args.binary.is_file():
            parser.error('install the pinned model/runtime first; see docs/S5_LANGUAGE_REUSE_REVIEW.md')
        if not 1 <= args.port <= 65535:
            parser.error('port must be within 1..65535')
        cuda_lib = ROOT/'artifacts/cuda-12.8/lib'
        if cuda_lib.is_dir():
            os.environ['LD_LIBRARY_PATH'] = str(cuda_lib) + (':'+os.environ['LD_LIBRARY_PATH'] if os.environ.get('LD_LIBRARY_PATH') else '')
        os.execv(str(args.binary.resolve()), [str(args.binary.resolve()), '-m', str(args.model.resolve()),
                 '--host', '127.0.0.1', '--port', str(args.port), '-ngl', '99',
                 '--ctx-size', '4096', '--parallel', '1', '--threads', '8',
                 '--alias', 'local-qwen', '--jinja', '--no-webui'])
    if args.command == 'demo':
        report = demo(args.fail_skill, args.local_language_url)
        encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True); args.output.write_text(encoded)
        print(json.dumps({'status': report['result']['status'], 'mode': 'simulation',
                          'hardware_commands': 0, 'candidate_count': report['candidate_count'],
                          'output': str(args.output) if args.output else None}, ensure_ascii=False))
        return 0
    if args.command == 'readiness':
        print((ROOT/'config/home_assistant/readiness.json').read_text())
        return 0
    if not 1 <= args.port <= 65535:
        parser.error('port must be within 1..65535')
    if args.household_source_map and not args.household_profile:
        parser.error('--household-source-map requires --household-profile')
    args.state_dir.mkdir(parents=True, exist_ok=True)
    key_file = args.state_dir/'access-key.txt'
    if not key_file.exists():
        fd = os.open(key_file, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            stream.write(secrets.token_urlsafe(36)+'\n')
    print(f'홈 어시스턴트: http://127.0.0.1:{args.port}/assistant', flush=True)
    print(f'접속 키 파일: {key_file}', flush=True)
    print('실제 집 기록 검토 모드: 실물 실행은 비활성입니다.' if args.household_profile else '소프트웨어 개발 모드: 실제 장치는 연결하지 않습니다.', flush=True)
    from home_robot_tasks.gateway import main as gateway_main
    gateway_main(['--world', str(ROOT/'config/home.example.json'),
                  '--map', str(ROOT/'config/navigation_map.simulation.json'),
                  '--token-file', str(key_file), '--database', str(args.state_dir/'requests.db'),
                  '--assistant-database', str(args.state_dir/'world.db'),
                  '--port', str(args.port)] +
                 (['--household-profile', str(args.household_profile)] if args.household_profile else ['--assistant-demo']) +
                 (['--household-source-map', str(args.household_source_map)] if args.household_source_map else []) +
                 (['--local-language-url', args.local_language_url] if args.local_language_url else []))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
