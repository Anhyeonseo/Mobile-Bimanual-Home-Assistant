#!/usr/bin/env python3
"""노트북 터미널에서 Jetson의 GraspNet 파지 실행기를 조작한다.

start는 서버만 시작한다. pick에는 실제 받침·빈 집게·리프트 끝 위치 확인이 필요하다.
재시도, 원점 보정, 오류 초기화, 기준 범위 변경은 수행하지 않는다.
"""
import argparse
import json
import os
from pathlib import PurePosixPath
import re
import shlex
import socket
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import ProxyHandler, Request, build_opener

DEFAULT_ROOT = '/home/hyper/robot/releases/pick-graspnet-cli-20261118'


class Client:
    def __init__(self, port):
        self.base = f'http://127.0.0.1:{port}'
        self.opener = build_opener(ProxyHandler({}))

    def status(self):
        with self.opener.open(self.base+'/api/status', timeout=5) as response:
            return json.load(response)

    def post(self, path, payload):
        with self.opener.open(self.base+'/', timeout=5) as response:
            match = re.search(r"const token='([^']+)'", response.read().decode())
        if match is None:
            raise ValueError('파지 실행기의 요청 토큰을 찾지 못했습니다.')
        request = Request(self.base+path, data=json.dumps(payload, ensure_ascii=False).encode(),
                          headers={'Content-Type': 'application/json', 'X-Pick-Token': match[1],
                                   'Origin': self.base})
        # POST is deliberately sent once, including after an ambiguous connection failure.
        with self.opener.open(request, timeout=5) as response:
            return json.load(response)


def require_ready(status):
    if status.get('mode') != 'station' or status.get('candidate_source') != 'graspnet':
        raise ValueError('GraspNet station이 아닙니다. 이 명령의 start로 실행기를 시작하세요.')
    if not status.get('accepting'):
        raise ValueError(status.get('blocked_reason') or '실행기가 준비 중이거나 작업 중입니다. status로 확인하세요.')


def payload_for(args):
    if args.command == 'inspect':
        return dict(text=args.text, operation='top-inspect')
    top, bottom = args.supported_empty_top, args.supported_empty_bottom
    if top == bottom:
        raise ValueError('실제 위치와 받침·빈 집게 상태를 확인한 옵션 하나가 필요합니다.')
    return dict(text=args.text, operation='top-pick' if top else 'bottom-pick',
                supported_empty_top=top, supported_empty_bottom=bottom)


def wait_result(client, identifier, *, timeout=2100):
    deadline = time.monotonic()+timeout
    seen = set()
    last_phase = None
    while time.monotonic() < deadline:
        state = client.status()
        for event in state.get('events', []):
            key = (event['at'], event['message'])
            if key not in seen:
                seen.add(key)
                print(event['message'], flush=True)
        for request in state.get('history', []):
            if request.get('id') == identifier:
                result = request['result']
                print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
                if result.get('status') == 'SUCCEEDED' and result.get('physical_success') is True:
                    return 0
                if result.get('status') == 'OBSERVED_ONLY':
                    print('카메라 검사 완료. 실제 파지는 실행하지 않았습니다.', flush=True)
                    return 0
                return 2
        if state.get('phase') != last_phase:
            last_phase = state.get('phase')
            print(state.get('phase_label', last_phase), flush=True)
        if state.get('phase') in ('FAULT', 'BLOCKED'):
            raise ValueError(state.get('blocked_reason') or '실행기가 중단됐습니다.')
        time.sleep(.5)
    raise TimeoutError('결과 대기 시간이 초과됐습니다. 성공으로 처리하지 않습니다.')


def start_argv(args):
    root = PurePosixPath(args.remote_root)
    if not root.is_absolute():
        raise ValueError('--remote-root는 Jetson의 절대경로여야 합니다.')
    command = shlex.join([
        '/home/hyper/realsense-grounding/.venv/bin/python', 'tools/run/pick_service.py',
        '--mode', 'station', '--port', str(args.port), '--candidate-source', 'graspnet',
        '--device', 'cpu', '--camera-mode', 'wide', '--artifacts-dir', str(root/'artifacts'),
        '--mobile-mesh-dir', str(root/'assets/mobile_meshes'),
        '--top-baseline', '/home/hyper/robot/diagnostics/tape-top-reference-20261117-preinstall-baseline.json',
        '--baseline', '/home/hyper/robot/shared/diagnostics/s4-trial-20261114/baseline.json',
        '--serial-port', args.serial_port])
    script = '\n'.join([
        'set -eo pipefail', 'cd '+shlex.quote(str(root)),
        'source tools/lib/pick_ros_env.sh', 'pick_source_ros',
        'export PYTHONPATH="/home/hyper/realsense-grounding/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}:/home/hyper/robot/shared/venvs/s4-jazzy/lib/python3.12/site-packages"',
        'export HF_HUB_OFFLINE=1 ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1',
        'export ROS_LOG_DIR="$PWD/output/ros"', 'mkdir -p "$ROS_LOG_DIR"', 'exec '+command])
    return ['ssh', '-tt', '-o', 'ExitOnForwardFailure=yes', '-o', 'ConnectTimeout=10',
            '-o', 'ServerAliveInterval=10', '-o', 'ServerAliveCountMax=2',
            '-L', f'127.0.0.1:{args.port}:127.0.0.1:{args.port}', args.jetson_host,
            'bash -lc '+shlex.quote(script)]


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--serial-port', default=os.environ.get('PICK_SERIAL', '/dev/ttyACM0'),
                   help='Jetson controller serial device')
    p.add_argument('--port', type=int, default=18769, help='로컬/Jetson 포트 (기본 18769)')
    commands = p.add_subparsers(dest='command', required=True)
    start = commands.add_parser('start', help='Jetson 실행기와 SSH 연결 시작; 이 터미널 유지')
    start.add_argument('--jetson-host', default='hyper@192.168.35.2')
    start.add_argument('--remote-root', default=DEFAULT_ROOT)
    pick = commands.add_parser('pick', help='새 촬영 → GraspNet → 경로 검사 → 실제 파지')
    pick.add_argument('text', help='예: 리모컨 또는 리모컨을 집어줘')
    confirmation = pick.add_mutually_exclusive_group(required=True)
    confirmation.add_argument('--supported-empty-top', action='store_true',
                              help='실제 최상단, 양팔 받침, 빈 집게를 현장에서 확인')
    confirmation.add_argument('--supported-empty-bottom', action='store_true',
                              help='실제 최하단, 양팔 받침, 빈 집게를 현장에서 확인')
    inspect = commands.add_parser('inspect', help='최상단 카메라·GraspNet 후보 검사; 모터 명령 없음')
    inspect.add_argument('text')
    commands.add_parser('status', help='실행기 상태 조회; 제어기 상태 직접 조회는 아님')
    commands.add_parser('cancel', help='현재 작업/자세 유지 중단 요청; STOP 완료와 구분')
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if not 1 <= args.port <= 65535:
        p.error('--port must be within 1..65535')
    client = Client(args.port)
    attempted = False
    try:
        if args.command == 'start':
            command = start_argv(args)
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', args.port))
            print(f'GraspNet 실행기 시작 · http://127.0.0.1:{args.port}\n'
                  '이 터미널을 유지하세요. 다른 터미널에서 status/inspect/pick을 실행합니다.\n'
                  '이 명령만으로 모터를 켜거나 파지하지 않습니다.', flush=True)
            os.execvp(command[0], command)
        if args.command == 'status':
            print(json.dumps(client.status(), ensure_ascii=False, indent=2))
            return 0
        if args.command == 'cancel':
            print(json.dumps(client.post('/api/cancel', {}), ensure_ascii=False))
            print('중단 요청 결과입니다. STOP 완료를 뜻하지 않습니다. status와 실물 상태를 확인하세요.')
            return 0
        require_ready(client.status())
        payload = payload_for(args)
        attempted = True
        identifier = client.post('/api/requests', payload)['id']
        print('요청 ID: '+identifier, flush=True)
        code = wait_result(client, identifier)
        attempted = False
        return code
    except (OSError, ValueError, TimeoutError, KeyError, KeyboardInterrupt) as exc:
        detail = '사용자 중단' if isinstance(exc, KeyboardInterrupt) else str(exc)
        if isinstance(exc, HTTPError):
            detail = exc.read().decode(errors='replace')
            if 400 <= exc.code < 500:
                attempted = False  # Explicit rejection is not an accepted request.
        print('완료하지 못했습니다: '+detail, file=sys.stderr)
        if attempted:
            try:
                client.post('/api/cancel', {})
                print('중단을 요청했습니다. STOP 완료는 별도 확인이 필요합니다.', file=sys.stderr)
            except (OSError, ValueError):
                print('중단 요청 전달도 확인하지 못했습니다. 실물 정지 상태를 확인하세요.', file=sys.stderr)
        print('자동 재전송·재시도는 하지 않습니다. start 터미널과 status를 확인하세요.', file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 2


if __name__ == '__main__':
    raise SystemExit(main())
