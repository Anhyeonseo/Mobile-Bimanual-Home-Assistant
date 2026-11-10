#!/usr/bin/env python3
"""Pi/SSH keyboard base teleop. Default INFO only; --run explicitly enables wheels.

W/S forward/back, A/D strafe, Q/E rotate; Space stop, X/Esc/Ctrl-C exit.
Terminal key repeat supplies a 300 ms input lease. Firmware link lease is 400 ms.
"""
import argparse
import binascii
from contextlib import contextmanager
import json
import os
from pathlib import Path
import secrets
import select
import signal
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
PRESERVE_HOLD = False


def response_timeout():
    # Allow an already-started bounded wheel STOP + hold readback to finish.
    # No new motion/lease refresh is sent while waiting for this response.
    return 3.0 if PRESERVE_HOLD else .3


def frame(prefix):
    data = prefix.encode("ascii")
    return data + f" {binascii.crc_hqx(data, 0xffff):04X}\n".encode("ascii")


def receive(port, seconds):
    deadline = time.monotonic() + seconds
    data = getattr(port, '_base_reply_buffer', None)
    if data is None:
        data = bytearray()
        port._base_reply_buffer = data
    while time.monotonic() < deadline:
        part = port.read(1)
        if part == b"\n":
            raw = bytes(data); data.clear()
            value = json.loads(raw.decode("ascii"))
            if not isinstance(value, dict): raise RuntimeError("invalid response")
            return value
        data.extend(part)
        if len(data) > 3072: raise RuntimeError("oversized response")
    raise RuntimeError("firmware reply timeout")


def send(port, data):
    if port.write(data) != len(data): raise RuntimeError("partial command; no retry")


def inspect(port):
    port.reset_input_buffer(); port._base_reply_buffer = bytearray(); send(port, b"INFO\n")
    info = receive(port, 2)
    expected = dict(firmware="mobile-base-teleop-v4", host_baud=921600, servo_baud=1000000,
                    ids=[7, 8, 9, 10], max_raw=200, watchdog_ms=400)
    if PRESERVE_HOLD:
        expected.update(firmware='mobile-base-hold-teleop-v7',hold_monitor=True,held_axes=13,
                        arm_lift_writes='explicit_setup_only',hold_setup=True)
    if info != expected: raise RuntimeError("unexpected firmware; wheels not enabled")
    return info


def validate(reply, kind, nonce, seq):
    if (reply.get("type") != kind or reply.get("session") != nonce or
            reply.get("sequence") != seq or reply.get("armed") is not True or
            reply.get("fault") is not False):
        raise RuntimeError("controller rejected command: " + json.dumps(reply))
    if PRESERVE_HOLD and reply.get('hold_confirmed') is not True:
        raise RuntimeError('arm/gripper/lift hold not confirmed: ' + json.dumps(reply))


def stop(port):
    send(port, b"STOP\n")
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        reply = receive(port, max(.01, deadline - time.monotonic()))
        if reply.get("type") == "stopped":
            if reply.get("armed") is not False or reply.get("stop_confirmed") is not True:
                raise RuntimeError("stop unconfirmed: " + json.dumps(reply))
            if PRESERVE_HOLD and reply.get('hold_confirmed') is not True:
                raise RuntimeError('wheels stopped, but arm/gripper/lift hold unconfirmed: ' + json.dumps(reply))
            return reply
    raise RuntimeError("stop unconfirmed")


def check_hold(port):
    """Read current hold registers on the candidate; no ARM, D, or STOP."""
    if not PRESERVE_HOLD: raise RuntimeError('hold-check requires hold-monitor firmware')
    info = inspect(port)
    send(port, b'HOLD\n')
    reply = receive(port, 3.0)
    if (reply.get('type') != 'hold' or reply.get('armed') is not False or
            reply.get('fault') is not False or reply.get('hold_confirmed') is not True):
        raise RuntimeError('hold preflight failed: ' + json.dumps(reply))
    return dict(info=info, hold=reply, motor_write_commands=0)


def prepare_hold(port, execute=False):
    """Plan only by default; explicit execute fixes OFF axes at current pose.

    This does not restore a previous grasp force or command any wheel motion.
    No retries/automatic torque release on an uncertain or partial result.
    """
    if not PRESERVE_HOLD: raise RuntimeError('hold setup requires hold firmware')
    inspect(port)
    settled=settle_wheels(port) if execute else None
    nonce=secrets.randbelow(0xffffffff)+1
    send(port,frame(f'PREP {nonce:08X}'))
    plan=receive(port,6)
    if (plan.get('type')!='setup' or plan.get('ok') is not True or
            plan.get('pending') is not True or plan.get('token')!=nonce):
        raise RuntimeError('hold setup plan rejected: '+json.dumps(plan))
    if not execute:return dict(plan=plan,executed=False)
    # Single explicit activation. Never re-send HSET after a lost reply.
    send(port,frame(f'HSET {nonce:08X}'))
    result=receive(port,6)
    if (result.get('type')!='setup' or result.get('ok') is not True or
            result.get('pending') is not False or result.get('reason')!='SETUP_HOLD_READY'):
        raise RuntimeError('hold setup incomplete; support load; do not drive: '+json.dumps(result))
    return dict(wheels=settled,plan=plan,result=result,verification=check_hold(port),executed=True)


def settle_wheels(port):
    """Explicit zero/OFF for wheels before hold setup; never certifies arm hold."""
    send(port,b'STOP\n')
    reply=receive(port,6)
    if (reply.get('type')!='stopped' or reply.get('armed') is not False or
            reply.get('stop_confirmed') is not True or reply.get('fault') is not False):
        raise RuntimeError('wheel preparation failed; do not activate hold: '+json.dumps(reply))
    return reply


def command_for(keys, now, current, last_key):
    """Drain terminal events once per tick; never retain commands after release timeout."""
    if len(keys) > 64: raise RuntimeError("input burst rejected")
    keys = keys.lower()
    if any(key in keys for key in ("x", "\x1b", "\x03")): return "Z", now, True
    # A key-repeat event buffered after Space must not undo this stop request.
    if " " in keys: return "Z", now, False
    for key in keys:
        if key in KEYS: current, last_key = KEYS[key], now
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
            validate(receive(port, response_timeout()), "ack", nonce, seq)
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


@contextmanager
def stopping_signals():
    """Route SSH hangup and process termination through the normal STOP cleanup."""
    previous = {}
    interrupted = False

    def stop_requested(_signal, _frame):
        nonlocal interrupted
        # A second hangup/TERM must not interrupt the bounded STOP transaction.
        if not interrupted:
            interrupted = True
            raise KeyboardInterrupt()

    try:
        for sig in (signal.SIGHUP, signal.SIGTERM):
            previous[sig] = signal.signal(sig, stop_requested)
        yield
    finally:
        for sig, handler in previous.items(): signal.signal(sig, handler)


def main():
    global PRESERVE_HOLD
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--web", action="store_true", help="Browser key-down/key-up control; enables wheels")
    parser.add_argument("--bind", default="127.0.0.1", help="Browser listener IP, e.g. Pi LAN address")
    parser.add_argument("--web-port", type=int, default=8765)
    parser.add_argument("--run", action="store_true", help="Enable wheels and start keyboard teleop")
    parser.add_argument('--preserve-hold', action='store_true',
        help='Require hold-monitor firmware; never write arm/gripper/lift targets or torque')
    parser.add_argument('--check-hold', action='store_true', help='Read hold registers only; never enable wheels')
    parser.add_argument('--prepare-hold',action='store_true',help='Read-only current-pose hold plan; --execute activates OFF axes')
    parser.add_argument('--execute',action='store_true',help='With --prepare-hold, explicitly enable current-pose holding; no wheel motion')
    parser.add_argument('--stop-wheels',action='store_true',help='Explicit wheel zero/OFF before hold setup; does not change arm/lift torque')
    args = parser.parse_args()
    PRESERVE_HOLD = args.preserve_hold or args.check_hold or args.prepare_hold or args.stop_wheels
    if args.web and args.run: parser.error("choose --web or --run")
    if args.check_hold and (args.web or args.run): parser.error('--check-hold cannot enable wheels')
    if args.prepare_hold and (args.web or args.run or args.check_hold):parser.error('--prepare-hold is a separate stationary step')
    if args.execute and not args.prepare_hold:parser.error('--execute requires --prepare-hold')
    if args.stop_wheels and (args.prepare_hold or args.web or args.run or args.check_hold):parser.error('--stop-wheels is a separate step')
    try:
        import serial
        with stopping_signals(), acquire_backend_lease("stm32", 0):
            with open_exclusive_serial(serial, args.port, 921600, .01) as port:
                if args.stop_wheels:
                    inspect(port);print(json.dumps(settle_wheels(port),indent=2))
                elif args.prepare_hold:print(json.dumps(prepare_hold(port,args.execute),indent=2))
                elif args.check_hold: print(json.dumps(check_hold(port), indent=2))
                elif args.web:
                    from base_teleop_web import run_web
                    run_web(port, args.bind, args.web_port)
                elif args.run: run(port, sys.stdin.fileno())
                else: print(json.dumps(inspect(port), indent=2))
        return 0
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        print(json.dumps({"error": str(error), "next_action": "No automatic retry. Check the reported state; a rejected read-only plan alone does not require MCU reset. Support the load before recovery."}), file=sys.stderr)
        return 2


if __name__ == "__main__": raise SystemExit(main())
