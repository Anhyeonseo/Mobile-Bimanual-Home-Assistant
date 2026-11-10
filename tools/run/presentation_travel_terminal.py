#!/usr/bin/env python3
"""Operator-staged travel log screen. No navigation, sensors, or motor commands."""
import argparse
import atexit
import json
from pathlib import Path
import sys
import termios
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'ros2_ws/src/home_robot_tasks')]
from tools.lib.presentation_session import write_json
from tools.run.presentation_terminal_sequence import STEPS


def screen(destination, phase):
    desk = destination == 'desk'
    place = '책상' if desk else '침대'
    stage = 2 if desk else 4
    active = stage if phase < 3 else stage + 1
    rows = ['\033]11;#300a24\007\033]10;#eeeeec\007\033[2J\033[H',
        '\033[1mALOHA Mini1  |  HOME ASSISTANT\033[0m',
        '\033[90m시연 진행 표시 · 운영자 촬영 큐\033[0m', '',
        '\033[32mrequest >\033[0m 책상에 있는 리모컨 침대로 가져와줘', '',
        '\033[1mTASK PROGRESS\033[0m', '']
    for index, label in enumerate(STEPS[:active], 1):
        color, pointer = ('\033[1;32m', '>>') if index == active else ('\033[37m', '  ')
        rows.append(f'{color}{pointer}  {index:02d}  {label}\033[0m')
    rows.extend(['', '\033[1mTRAVEL LOG\033[0m', '',
        f'\033[36m[목적지]\033[0m {place}',
        '\033[36m[작업]  \033[0m ' + ('리모컨을 집을 위치로 이동' if desk else '리모컨 운반')])
    if phase >= 1:
        rows.append('\033[33m[상태]  \033[0m 이동 중...')
    if phase >= 2:
        rows.append(f'\033[32m[확인]  \033[0m {place} 도착')
    if phase >= 3:
        rows.append('\033[36m[다음]  \033[0m ' + ('카메라 촬영 및 리모컨 인식' if desk else '내려놓기 준비'))
    rows.extend(['', '\033[90m' + ['목적지 확인', '이동 단계 진행', '도착 확인', '다음 작업 준비'][phase] + '\033[0m', ''])
    return '\n'.join(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    args = parser.parse_args()
    args.session.mkdir(parents=True, exist_ok=False)
    if sys.stdin.isatty():
        old = termios.tcgetattr(sys.stdin)
        quiet = termios.tcgetattr(sys.stdin)
        quiet[3] &= ~termios.ECHO
        termios.tcsetattr(sys.stdin, termios.TCSANOW, quiet)
        atexit.register(termios.tcsetattr, sys.stdin, termios.TCSANOW, old)
    previous = None
    print('\033[2J\033[H촬영 준비', flush=True)
    while True:
        file = args.session / 'cue.json'
        if file.exists():
            cue = json.loads(file.read_text())
            destination, phase = cue['destination'], cue['phase']
            if destination not in ('desk', 'bed') or phase not in range(4):
                raise ValueError('Invalid filming cue')
            if (destination, phase) != previous:
                print(screen(destination, phase), end='', flush=True)
                write_json(args.session / 'displayed.json', dict(destination=destination,
                    phase=phase, source='operator_filming_cue', robot_commands=0,
                    physical_completion_verified=False))
                previous = destination, phase
        time.sleep(.04)


if __name__ == '__main__':
    main()
