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
sys.path.insert(0, str(ROOT/'ros2_ws/src/home_robot_tasks'))


def demo(fail_skill=None):
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
        assistant.prepare('거실 탁자에 있는 검은 마커펜을 침실 협탁에 놓아줘', 'offline-delivery')
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
                    result=result, object=world.get('marker-1'),
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
    offline = sub.add_parser('demo', help='run the real task state machine with synthetic skill feedback')
    offline.add_argument('--fail-skill', choices=['pick', 'navigate', 'place', 'verify_delivery'])
    offline.add_argument('--output', type=Path)
    sub.add_parser('readiness', help='show unresolved physical integration inputs')
    args = parser.parse_args(argv)
    if args.command == 'demo':
        report = demo(args.fail_skill)
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
    args.state_dir.mkdir(parents=True, exist_ok=True)
    key_file = args.state_dir/'access-key.txt'
    if not key_file.exists():
        fd = os.open(key_file, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            stream.write(secrets.token_urlsafe(36)+'\n')
    print(f'홈 어시스턴트: http://127.0.0.1:{args.port}/assistant', flush=True)
    print(f'접속 키 파일: {key_file}', flush=True)
    print('소프트웨어 개발 모드: 실제 장치는 연결하지 않습니다.', flush=True)
    from home_robot_tasks.gateway import main as gateway_main
    gateway_main(['--world', str(ROOT/'config/home.example.json'),
                  '--map', str(ROOT/'config/navigation_map.simulation.json'),
                  '--token-file', str(key_file), '--database', str(args.state_dir/'requests.db'),
                  '--assistant-database', str(args.state_dir/'world.db'), '--assistant-demo',
                  '--port', str(args.port)])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
