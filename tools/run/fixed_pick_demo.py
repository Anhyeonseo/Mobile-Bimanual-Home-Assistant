#!/usr/bin/env python3
"""고정 위치 집기: 실측 자세 저장, 오프라인 검사, 명시적 한 팔 재생."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import asdict
import json
import os
from pathlib import Path
import select
import signal
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/so101_arm_bridge"))

from so101_arm_bridge.backend_lease import acquire_backend_lease
from so101_arm_bridge.bimanual_stream_adapter import (
    CALIBRATION_HASH, F8_CAPABILITIES, F8_FIRMWARE_VERSION,
    ResidentBimanualStreamAdapter, load_operational_limits,
)
from so101_arm_bridge.device_discovery import resolve_serial_device
from so101_arm_bridge.fixed_pick_demo import (
    FixedPickAdapter, LABELS, OWNER, STAGES, capture_pose, check_recipe,
    compose_targets, execute_leg, limits_digest, load_json, measured_feedback,
    new_recipe, put_capture, require_tracking, validate_recipe,
)
from so101_arm_bridge.left_only_adapter import (
    LEFT_FIRMWARE_VERSION, LeftOnlyAdapter, compatible_left, left_limits,
)
from so101_arm_bridge.serial_port import open_exclusive_serial
from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2


def save_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=path.name + ".", delete=False) as stream:
            name = stream.name
            json.dump(document, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n"); stream.flush(); os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if name and os.path.exists(name):
            os.unlink(name)


@contextmanager
def connection(args):
    import serial
    device = resolve_serial_device(args.port)
    with acquire_backend_lease("stm32", 0):
        with open_exclusive_serial(serial, device, args.baud, .05) as port:
            yield StreamValidationTransportV2(
                port, response_timeout_s=.4,
                physical_joint_count=6 if getattr(args, "left_only", False) else 12,
            )


def compatible(hello):
    return (hello.protocol_version == 2 and hello.joint_count == 12
            and hello.firmware_version == F8_FIRMWARE_VERSION
            and hello.left_calibration_hash == hello.right_calibration_hash == CALIBRATION_HASH
            and hello.capabilities == F8_CAPABILITIES and not hello.stop_latched)


class OperatorStop(Exception):
    """An operator q command, including while a trajectory is running."""


class TerminalCommands:
    """Read bounded raw input without TextIOWrapper's hidden line buffering."""

    def __init__(self, stream):
        self.fd = stream.fileno()
        self.pending = bytearray()

    def read(self, timeout=0.):
        if not select.select([self.fd], [], [], timeout)[0]:
            return []
        data = os.read(self.fd, 1024)
        if not data:
            raise EOFError("입력 연결이 종료되어 정지합니다.")
        self.pending.extend(data)
        if len(self.pending) > 512:
            raise ValueError("입력이 너무 깁니다. 조종 입력을 중단합니다.")
        lines = self.pending.split(b"\n")
        self.pending = bytearray(lines.pop())
        return [line.decode("utf-8", errors="replace").strip().lower() for line in lines]

    def during_motion(self):
        # run/return typed during motion must not turn into a later automatic
        # action. q always cancels; a partial non-stop command is also cleared.
        if "q" in self.read():
            raise OperatorStop()
        if self.pending.strip().lower() not in (b"q", b""):
            self.pending.clear()


def inspect(args):
    with connection(args) as transport:
        hello = transport.enter_binary_mode()
        result = asdict(hello)
        is_left = getattr(args, "left_only", False)
        result.update(compatible=(compatible_left if is_left else compatible)(hello), motion_commands_sent=False,
                      required_firmware=hex(LEFT_FIRMWARE_VERSION if is_left else F8_FIRMWARE_VERSION))
        print(json.dumps(result, indent=2))
        if not result["compatible"]:
            raise ValueError("현재 펌웨어/보정/정지 상태가 집기 도구와 맞지 않습니다. 자동으로 변경하지 않습니다.")


def teach(args, limits, digest):
    if not sys.stdin.isatty():
        raise ValueError("teach는 직접 입력할 수 있는 터미널에서 실행하세요.")
    recipe = load_json(args.file) if args.file.exists() else new_recipe(args.arm, digest, left_only=getattr(args, "left_only", False))
    validate_recipe(recipe, limits, digest, complete=False)
    if recipe["arm"] != args.arm:
        raise ValueError("기존 자세 파일의 팔과 --arm이 다릅니다. 다른 파일을 사용하세요.")
    # Both arms participate in this established resident protocol, even for a
    # single-arm demo. Prepare/refresh explicitly disable torque on both arms.
    is_left = getattr(args, "left_only", False)
    print("왼팔 단독 저장: 왼팔 토크를 해제합니다. 왼팔과 물체를 받쳐주세요." if is_left else
          "수동 저장 모드: 양팔 토크를 해제합니다. 양팔과 물체를 받쳐 둔 상태여야 합니다.", flush=True)
    print("자세를 손으로 맞춘 뒤 이름을 입력하세요. 같은 이름은 그 자세를 다시 저장합니다.")
    print(" | ".join(f"{stage}: {LABELS[stage]}" for stage in STAGES))
    with connection(args) as transport:
        adapter = (LeftOnlyAdapter if is_left else ResidentBimanualStreamAdapter)(transport, limits)
        adapter.prepare()
        while True:
            try:
                command = input("저장할 자세 / show / done / q > ").strip().lower()
            except EOFError:
                break
            if command == "q":
                break
            if command == "show":
                print("저장됨: " + ", ".join(s for s in STAGES if s in recipe["poses"]))
                continue
            if command == "done":
                try:
                    print(json.dumps(check_recipe(recipe, limits, digest), ensure_ascii=False, indent=2))
                    break
                except ValueError as error:
                    print(f"다시 저장할 항목 확인: {error}", flush=True)
                    continue
            if command not in STAGES:
                print("home / approach / grasp / closed / lift 중 하나를 입력하세요.")
                continue
            try:
                record = capture_pose(adapter, limits)
                updated = put_capture(recipe, command, record, limits, digest)
                save_json(args.file, updated)
                recipe = updated
                offset = 0 if args.arm == "left" else 6
                print(f"{command} 저장: {record['positions_rad'][offset:offset+6]}", flush=True)
            except ValueError as error:
                print(f"저장하지 않음: {error}", flush=True)
    print(f"자세 파일: {args.file.resolve()}")


def wait_command(adapter, limits, hold_target, commands, *, clock=time.monotonic, sleep=time.sleep):
    # Arm-only firmware stops refreshing feedback after a finite leg finishes.
    # Feed identical finite holds so its normal tracking reader stays in use;
    # a heartbeat alone keeps torque enabled but cannot make that snapshot fresh.
    # Do not refresh_unarmed_anchor here: that would disable both arms' torque.
    pending = None

    def read_control():
        nonlocal pending
        lines = commands.read()
        if "q" in lines:
            raise OperatorStop()
        if pending is None:
            pending = next((line for line in lines if line), None)

    while True:
        adapter.keepalive()
        require_tracking(measured_feedback(adapter, limits), hold_target)
        read_control()
        if pending is not None:
            return pending
        execute_leg(adapter, hold_target, limits, commanded_start=hold_target,
                    interrupt=read_control, clock=clock, sleep=sleep)
        if pending is not None:
            return pending


def play(args, limits, digest):
    recipe = load_json(args.file)
    report = check_recipe(recipe, limits, digest)
    if not args.execute:
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print("사전 검사만 완료했습니다. 장치는 열지 않았습니다. 실제 재생은 --execute를 사용하세요.")
        return
    if not sys.stdin.isatty():
        raise ValueError("재생은 연결이 유지되는 대화형 터미널에서 실행하세요.")
    commands = TerminalCommands(sys.stdin)
    # Create the log before any device access; a full/unwritable disk must not
    # first be discovered while a loaded arm is holding an object.
    log = dict(recipe_sha256=limits_digest(args.file), status="STARTING", events=[],
               physical_pick_verified=False)
    log_path = args.log or args.file.with_name(args.file.stem + ".last-run.json")
    if log_path.resolve() == args.file.resolve():
        raise ValueError("로그 파일은 자세 파일과 달라야 합니다.")
    save_json(log_path, log)
    is_left = getattr(args, "left_only", False)
    support = "왼팔" if is_left else "양팔"
    print(f"{recipe['arm']} 팔 재생. 시작 전 {support}이 저장한 home 자세여야 합니다.", flush=True)
    print(f"초기화는 {support} 토크를 해제합니다. 재생 종료/오류 시에도 토크가 해제될 수 있습니다.")
    print(f"들기 후 자동으로 놓지 않습니다. return으로 원래 위치에 놓고, {support}을 받친 뒤 q로 종료하세요.", flush=True)
    with connection(args) as transport:
        adapter = (LeftOnlyAdapter if is_left else FixedPickAdapter)(transport, limits, motion_authorized=True,
            home_reference=recipe["poses"]["home"]["positions_rad"])
        motion_attempted = False
        try:
            adapter.prepare()
            anchor = adapter.refresh_unarmed_anchor()
            home = tuple(q / 1e6 for q in anchor.positions_urad)
            targets = compose_targets(recipe, home, limits, digest)
            # Check every outward and reverse segment before ARM, not midway.
            check_recipe(recipe, limits, digest)

            last_target = None

            def leg(stage):
                nonlocal motion_attempted, last_target
                print(f"이동: {stage} — {LABELS[stage]}", flush=True)
                motion_attempted = True
                execute_leg(adapter, targets[stage], limits,
                            commanded_start=last_target, interrupt=commands.during_motion)
                last_target = targets[stage]
                log["events"].append(dict(stage=stage, reached=True, at=time.time()))

            leg("home")
            sequence, location = ("approach", "grasp", "closed", "lift"), "home"
            while True:
                for stage in sequence:
                    leg(stage)
                location = "lift" if sequence[-1] == "lift" else "home"
                log["status"] = "HOLDING_" + location.upper()
                log["last_pose"] = location
                commands.during_motion()
                # Keep disk I/O out of the watchdog-critical held session.
                print("촬영 자세 유지 중. return + Enter: 원래 위치에 내려놓기 / q: 종료" if location == "lift"
                      else "기본 자세 유지 중. run + Enter: 다시 집기 / q: 종료", flush=True)
                while True:
                    command = wait_command(adapter, limits, targets[location], commands)
                    if command == "q":
                        log["status"] = "STOPPED"
                        return
                    if command == "return" and location == "lift":
                        sequence = ("closed", "grasp", "approach", "home")
                        break
                    if command == "run" and location == "home":
                        sequence = ("approach", "grasp", "closed", "lift")
                        break
                    print("현재 자세에서 가능한 명령을 입력하세요.", flush=True)
        except OperatorStop:
            log["status"] = "STOPPED"
        except BaseException as error:
            log.update(status="INTERRUPTED" if isinstance(error, KeyboardInterrupt) else "FAILED",
                       error=str(error) or type(error).__name__)
            raise
        finally:
            if motion_attempted:
                try:
                    adapter.stop(OWNER)
                    log["stop_latched"] = True
                    print("팔 정지·토크 해제 명령 확인. 새 실행 전 STM32 정지 잠금 상태를 확인하세요.", flush=True)
                except Exception as error:
                    log.update(stop_latched=False, stop_error=str(error))
                    save_json(log_path, log)
                    raise RuntimeError("정지 응답을 확인하지 못했습니다. 로봇을 지지하고 모터 전원을 차단하세요.") from error
            save_json(log_path, log)


def main(argv=None):
    left_mode = "--left-only" in (sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description=("왼팔 단독 집기: 관절 6개 자세 저장·검사·재생." if left_mode else __doc__))
    parser.add_argument("--left-only", action="store_true", help="전용 펌웨어로 왼팔 6개만 사용")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", type=int, choices=(921600,), default=921600)
    parser.add_argument("--limits", type=Path, default=ROOT / "config/bimanual_operational_limits.json")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("inspect", help="식별 정보만 조회; 토크 활성화/해제 없음")
    teaching = sub.add_parser("teach", help=("왼팔 토크를 해제하고 자세 저장" if left_mode else "양팔 토크를 해제하고 손으로 맞춘 자세 저장"))
    teaching.add_argument("--arm", choices=(("left",) if left_mode else ("left", "right")), default=None)
    teaching.add_argument("--file", type=Path, required=True)
    checking = sub.add_parser("check", help="파일과 관절 경로만 검사; 장치 연결 없음")
    checking.add_argument("--file", type=Path, required=True)
    playing = sub.add_parser("play", help="기본은 사전 검사; --execute일 때만 구동")
    playing.add_argument("--file", type=Path, required=True)
    playing.add_argument("--execute", action="store_true")
    playing.add_argument("--log", type=Path)
    args = parser.parse_args(argv)
    if args.command == "teach":
        args.arm = args.arm or ("left" if args.left_only else "right")
    try:
        if args.left_only and args.command == "teach" and args.arm != "left":
            raise ValueError("왼팔 단독 모드에서는 --arm left만 지원합니다.")
        if args.command == "inspect":
            inspect(args)
            return 0
        limits = load_operational_limits(args.limits)
        if args.left_only:
            limits = left_limits(limits)
        digest = limits_digest(args.limits)
        if args.command == "teach":
            teach(args, limits, digest)
        elif args.command == "check":
            print(json.dumps(check_recipe(load_json(args.file), limits, digest), ensure_ascii=False, indent=2))
        else:
            play(args, limits, digest)
        return 0
    except KeyboardInterrupt:
        print("사용자 중단", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"실행 중단: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    def interrupted(_signal, _frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    raise SystemExit(main())
