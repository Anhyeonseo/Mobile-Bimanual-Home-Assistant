#!/usr/bin/env python3
"""Presentation console and read-only recording screen. Never opens a robot."""
import argparse
import json
from pathlib import Path
import shlex
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'ros2_ws/src/home_robot_tasks')]
from tools.lib.presentation_session import FilmSession, STAGES, render


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    console = sub.add_parser('console', help='요청 입력과 촬영 단계 표시; 로봇 명령 없음')
    console.add_argument('--output', type=Path, required=True)
    console.add_argument('--clip', choices=('full', *STAGES), default='full')
    console.add_argument('--take', type=int, default=1)
    watch = sub.add_parser('watch', help='화면 녹화용 읽기 전용 화면')
    watch.add_argument('folder', type=Path)
    args = parser.parse_args(argv)
    session = None
    try:
        if args.command == 'watch':
            previous = None
            while True:
                try:
                    state = json.loads((args.folder / 'state.json').read_text())
                    content = render(state)
                    if content != previous:
                        if sys.stdout.isatty():
                            print('\033[2J\033[H', end='')
                        print(content, flush=True)
                        previous = content
                    if state['status'].startswith('FILM_MARKERS_'):
                        break
                except FileNotFoundError:
                    if previous is None:
                        print('명령 입력 대기 중…', flush=True)
                        previous = 'waiting'
                time.sleep(.2)
            return 0
        session = FilmSession(args.output, args.clip, args.take)
        print('촬영 콘솔 · 로봇 구동 명령은 보내지 않습니다.')
        while session.state['status'] != 'READY':
            event = session.request(input('한국어 요청 > '))
            print(event['message'])
        print(render(session.state))
        print('\nstart: 다음 장면 시작 | done: 실제 완료 확인 | fail 이유 | mark 메모')
        print('result "결과.json" "원본 촬영 폴더": 실제 분석 연결 | quit: 기록 종료')
        while True:
            try:
                words = shlex.split(input('촬영 > '))
                if not words:
                    continue
                if words == ['quit']:
                    break
                if words == ['start']:
                    event = session.start()
                elif words == ['done']:
                    event = session.finish()
                elif words[0] == 'fail' and len(words) > 1:
                    event = session.finish(' '.join(words[1:]))
                elif words[0] == 'mark' and len(words) > 1:
                    event = session.event('marker', ' '.join(words[1:]), 'operator')
                elif words[0] == 'result' and len(words) == 3:
                    event = session.attach(words[1], words[2])
                else:
                    raise ValueError('start / done / fail / mark / result / quit 명령을 사용하세요.')
                print(event['message'], flush=True)
            except (ValueError, OSError, KeyError, TypeError) as exc:
                print('확인 필요:', exc)
        return 0
    except (EOFError, KeyboardInterrupt):
        print('\n화면 종료 · 로봇 정지 명령이 아닙니다.')
        return 0
    except (ValueError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    finally:
        if session:
            session.close()
            print('저장:', session.output)


if __name__ == '__main__':
    raise SystemExit(main())
