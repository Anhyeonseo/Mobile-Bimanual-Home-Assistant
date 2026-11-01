#!/usr/bin/env python3
"""Check SSH terminal key repeat without importing or opening robot interfaces."""
import os
import select
import sys
import termios
import time
import tty


def main():
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        raise SystemExit("대화형 SSH 터미널에서 실행하세요.")
    old = termios.tcgetattr(fd)
    events = []
    deadline = time.monotonic() + 30
    print("모터 연결 없는 키보드 검사입니다. S를 누른 채 결과가 나올 때까지 유지하세요(약 4초). X: 종료", flush=True)
    try:
        tty.setcbreak(fd)
        while time.monotonic() < deadline:
            if not select.select([fd], [], [], min(.05, max(0, deadline-time.monotonic())))[0]:
                continue
            keys = os.read(fd, 4096)
            if not keys or any(key in keys for key in (b'x', b'X', b'\x03', b'\x1b')):
                print("검사 취소")
                return 2
            now = time.monotonic()
            for key in keys:
                if key in (ord('s'), ord('S')):
                    if not events:
                        deadline = now + 4
                    events.append(now)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    # Exclude the desktop's initial repeat delay, but require sustained input
    # through the end of the test within the controller's 250 ms input lease.
    steady = [t for t in events if t >= events[0]+1] if events else []
    gaps = [b-a for a, b in zip(steady, steady[1:])]
    passed = (len(steady) >= 10 and steady[-1]-steady[0] >= 2
              and max(gaps, default=999) < .20 and deadline-steady[-1] < .20)
    print(f"S 입력 {len(events)}회 · " + ("연속 입력 정상" if passed else "연속 입력 미확인 — 아직 전체 하강을 실행하지 마세요."))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
