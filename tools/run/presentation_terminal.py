#!/usr/bin/env python3
"""Actual terminal request parser for filming. Never connects to robot hardware."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import select
import sys
import time
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'ros2_ws/src/home_robot_tasks')]
from tools.lib.presentation_session import FilmSession


def log(component, message, color='36'):
    stamp = datetime.now(ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S')
    print(f'\033[90m{stamp}\033[0m  \033[{color}m[{component:8}]\033[0m {message}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    args = parser.parse_args()
    session = FilmSession(args.session)
    # Per-window terminal colors, not desktop profile changes.
    print('\033]11;#300a24\007\033]10;#eeeeec\007\033[2J\033[H', end='')
    print('\033[1mALOHA Mini1 | Home Assistant\033[0m\n')
    log('SYSTEM', '요청 처리 콘솔 준비 완료', '32')
    log('LANGUAGE', '한국어 요청 파서 연결 · bounded_ko_en_v1')
    log('SESSION', '화면 촬영 세션 · 로봇 실행 연결 없음', '90')
    print('\n요청을 입력하세요.\n', flush=True)
    external = args.session / 'request.txt'
    try:
        while True:
            print('\033[1;32mrequest >\033[0m ', end='', flush=True)
            while True:
                if external.exists():
                    raw = external.read_text().strip()
                    external.rename(args.session / 'submitted-request.txt')
                    print(raw, flush=True)
                    break
                if select.select([sys.stdin], [], [], .1)[0]:
                    raw = sys.stdin.readline()
                    if not raw:
                        return
                    raw = raw.strip()
                    break
            event = session.request(raw)
            if event['type'] == 'request_rejected':
                log('LANGUAGE', event['message'], '33')
                continue
            intent = session.state['request']['intent']
            print()
            log('INPUT', intent['utterance'])
            log('LANGUAGE', '요청 해석 완료', '32')
            log('INTENT', f"action = {intent['operation']}")
            log('TARGET', f"object = {intent['category']}  |  대상: 리모컨")
            log('LOCATION', f"source = {intent['source_phrase']}  →  destination = {intent['destination_phrase']}")
            print('\n\033[1mTASK SEQUENCE\033[0m')
            for number, step in enumerate(['책상으로 이동', '리모컨 인식 및 집기', '침대로 운반', '침대에 내려놓기'], 1):
                print(f'  {number:02d}  [대기]  {step}', flush=True)
            print()
            log('TASK', '요청 등록 완료 · 다음 단계: 책상으로 이동', '32')
            log('STATE', 'WAITING_FOR_EXECUTION · 실행 시작 대기', '33')
            print('\nCtrl+C: 촬영 콘솔 종료', flush=True)
            while True:
                time.sleep(.2)
    except KeyboardInterrupt:
        print()
    finally:
        session.close()


if __name__ == '__main__':
    main()
