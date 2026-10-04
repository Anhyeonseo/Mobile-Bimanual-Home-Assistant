#!/usr/bin/env python3
"""Pi/SSH keyboard base teleop. Default INFO only; --run explicitly enables wheels.

W/S forward/back, A/D strafe, Q/E rotate; Space stop, X/Esc/Ctrl-C exit.
Terminal key repeat supplies a 300 ms input lease. Firmware link lease is 400 ms.
"""
import argparse
import binascii
import json
import os
from pathlib import Path
import secrets
import select
import sys
import termios
import time
import tty
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/so101_arm_bridge"))
from so101_arm_bridge.backend_lease import acquire_backend_lease
from so101_arm_bridge.serial_port import open_exclusive_serial

KEYS = {"w": "F", "s": "B", "a": "L", "d": "R", "q": "A", "e": "D"}
INPUT_LEASE = .3
PERIOD = .05


def frame(prefix):
    data = prefix.encode("ascii")
    return data + f" {binascii.crc_hqx(data, 0xffff):04X}\n".encode("ascii")


def receive(port, seconds):
    deadline = time.monotonic() + seconds
    data = bytearray()
    while time.monotonic() < deadline:
        part = port.read(1)
        if part == b"\n":
            value = json.loads(data.decode("ascii"))
            if not isinstance(value, dict): raise RuntimeError("invalid response")
            return value
        data.extend(part)
        if len(data) > 2048: raise RuntimeError("oversized response")
    raise RuntimeError("firmware reply timeout")


def send(port, data):
    if port.write(data) != len(data): raise RuntimeError("partial command; no retry")


def inspect(port):
    port.reset_input_buffer(); send(port, b"INFO\n")
    info = receive(port, 2)
    expected = dict(firmware="mobile-base-teleop-v4", host_baud=921600, servo_baud=1000000,
                    ids=[7, 8, 9, 10], max_raw=200, watchdog_ms=400)
    if info != expected: raise RuntimeError("unexpected firmware; wheels not enabled")
    return info


def validate(reply, kind, nonce, seq):
    if (reply.get("type") != kind or reply.get("session") != nonce or
            reply.get("sequence") != seq or reply.get("armed") is not True or
            reply.get("fault") is not False):
        raise RuntimeError("controller rejected command: " + json.dumps(reply))


def stop(port):
    send(port, b"STOP\n")
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        reply = receive(port, max(.01, deadline - time.monotonic()))
        if reply.get("type") == "stopped":
            if reply.get("armed") is not False or reply.get("stop_confirmed") is not True:
                raise RuntimeError("stop unconfirmed: " + json.dumps(reply))
            return reply
    raise RuntimeError("stop unconfirmed")


def command_for(keys, now, current, last_key):
    """Drain terminal events once per tick; never retain commands after release timeout."""
    if len(keys) > 64: raise RuntimeError("input burst rejected")
    for key in keys.lower():
        if key in ("x", "\x1b", "\x03"): return "Z", now, True
        if key == " ": current, last_key = "Z", now
        elif key in KEYS: current, last_key = KEYS[key], now
    if now - last_key >= INPUT_LEASE: current = "Z"
    return current, last_key, False


def run(port, input_fd):
    if not os.isatty(input_fd): raise RuntimeError("--run needs an interactive terminal")
    info = inspect(port)
    print(json.dumps(info))
    print("W/S 전후 · A/D 좌우 · Q/E 회전 | Space 정지 | X/Esc 종료", flush=True)
    print("이동 키를 누르고 유지하세요. 입력 반복이 0.3초 끊기면 정지합니다.", flush=True)
    old = termios.tcgetattr(input_fd)
    arm_sent = False
    original_error = None
    stopped = None
    try:
        tty.setcbreak(input_fd)
        nonce = secrets.randbelow(0xffffffff) + 1
        arm_sent = True  # even a partial/uncertain ARM needs cleanup
        send(port, frame(f"ARM {nonce:08X}"))
        validate(receive(port, 6), "armed", nonce, 0)
        seq, current, last_key = 0, "Z", float("-inf")
        while True:
            tick = time.monotonic()
            keys = ""
            if select.select([input_fd], [], [], 0)[0]:
                raw = os.read(input_fd, 4096)
                if not raw: break
                keys = raw.decode("ascii", errors="strict")
            current, last_key, exiting = command_for(keys, tick, current, last_key)
            if exiting: break
            seq += 1
            if seq > 0xffffffff: raise RuntimeError("sequence exhausted")
            send(port, frame(f"D {nonce:08X} {seq:08X} {current}"))
            validate(receive(port, .3), "ack", nonce, seq)
            remaining = PERIOD - (time.monotonic() - tick)
            if remaining > 0: time.sleep(remaining)
    except BaseException as error:
        original_error = error
    finally:
        try:
            if arm_sent: stopped = stop(port)
        except Exception as error:
            original_error = RuntimeError(f"{original_error or 'exit'}; STOP failed: {error}. Cut motor 12 V power.")
        finally:
            termios.tcsetattr(input_fd, termios.TCSADRAIN, old)
    if stopped: print(json.dumps(stopped, indent=2))
    if original_error is not None and not isinstance(original_error, KeyboardInterrupt): raise original_error
    return stopped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--web", action="store_true", help="Browser key-down/key-up control; enables wheels")
    parser.add_argument("--bind", default="127.0.0.1", help="Browser listener IP, e.g. Pi LAN address")
    parser.add_argument("--web-port", type=int, default=8765)
    parser.add_argument("--run", action="store_true", help="Enable wheels and start keyboard teleop")
    args = parser.parse_args()
    if args.web and args.run: parser.error("choose --web or --run")
    try:
        import serial
        with acquire_backend_lease("stm32", 0):
            with open_exclusive_serial(serial, args.port, 921600, .01) as port:
                if args.web:
                    from base_teleop_web import run_web
                    run_web(port, args.bind, args.web_port)
                elif args.run: run(port, sys.stdin.fileno())
                else: print(json.dumps(inspect(port), indent=2))
        return 0
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        print(json.dumps({"error": str(error), "next_action": "If wheels do not stop, cut motor 12 V power. A latched firmware fault requires motor power off and MCU reset."}), file=sys.stderr)
        return 2


if __name__ == "__main__": raise SystemExit(main())
