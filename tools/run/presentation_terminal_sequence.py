#!/usr/bin/env python3
"""Cumulative operator filming cues, displayed in a real terminal, without motion."""
import argparse
import json
from pathlib import Path
import sys
import time
import atexit
import termios

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'ros2_ws/src/home_robot_tasks')]
from tools.lib.presentation_session import interpret, write_json

STEPS = ['명령 입력 완료', '책상으로 이동', '리모컨 집기', '침대로 이동', '침대에 놓기', '다음 명령 대기']


def screen(stage, intent, request_visible=True):
    lines = ['\033]11;#300a24\007\033]10;#eeeeec\007\033[2J\033[H',
             '\033[1mALOHA Mini1  |  HOME ASSISTANT\033[0m',
             '\033[90m시연 진행 표시 · 운영자 촬영 큐\033[0m', '',
             '\033[32mrequest >\033[0m ' + (intent['utterance'] if request_visible else ''), '',
             (f"  대상: 리모컨    출발: {intent['source_phrase']}    목적지: {intent['destination_phrase']}" if request_visible else ''),
             '', '\033[1mTASK PROGRESS\033[0m', '']
    for index, label in enumerate(STEPS[:stage], 1):
        current = index == stage
        color = '\033[1;32m' if current else '\033[37m'
        pointer = '>>' if current else '  '
        lines.extend([f'{color}{pointer}  {index:02d}  {label}\033[0m', ''])
    if stage == 6:
        lines.extend(['\033[32mrequest >\033[0m _', ''])
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--blank-start', action='store_true')
    parser.add_argument('--utterance', default='책상에 있는 리모컨 침대로 가져다줘')
    args = parser.parse_args()
    # This display is driven by cue files; keyboard echo would contaminate takes.
    if sys.stdin.isatty():
        previous_tty = termios.tcgetattr(sys.stdin)
        quiet_tty = termios.tcgetattr(sys.stdin)
        quiet_tty[3] &= ~termios.ECHO
        termios.tcsetattr(sys.stdin, termios.TCSANOW, quiet_tty)
        atexit.register(termios.tcsetattr, sys.stdin, termios.TCSANOW, previous_tty)
    args.session.mkdir(parents=True, exist_ok=False)
    # The filming adapter normalizes this equivalent delivery wording for the
    # existing bounded parser, while retaining the original input in the record.
    normalized = args.utterance.replace('가져와줘', '가져다줘')
    parsed = interpret(normalized, 'presentation-sequence')
    parsed['intent']['utterance'] = args.utterance
    assert not parsed['intent']['issues']
    write_json(args.session / 'intent.json', parsed)
    previous = None
    print(screen(0, parsed['intent'], not args.blank_start), end='', flush=True)
    while True:
        path = args.session / 'cue.json'
        if path.exists():
            cue = json.loads(path.read_text())
            stage = cue['stage']
            if not isinstance(stage, int) or not 0 <= stage <= len(STEPS):
                raise ValueError('Invalid filming stage')
            visible = cue.get('request_visible', True)
            if (stage, visible) != previous:
                print(screen(stage, parsed['intent'], visible), end='', flush=True)
                write_json(args.session / 'displayed.json', dict(stage=stage,
                    visible_steps=STEPS[:stage], source='operator_filming_cue',
                    request_visible=visible, physical_completion_verified=False, robot_commands=0))
                previous = (stage, visible)
        time.sleep(.05)


if __name__ == '__main__':
    main()
