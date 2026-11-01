#!/usr/bin/env python3
"""ID10 lift-only commissioning. INFO by default; no unattended movement.

--prepare backs up EEPROM, switches mode 0 -> 1, verifies zero/torque OFF.
--check-direction: S initiates one MCU-bounded 150 ms positive-speed pulse.
--run --direction-confirmed: hold S to descend; Space stop; B finish at bottom.
STOP retains torque; this tool never restores position mode automatically.
"""
import argparse
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import secrets
import select
import sys
import termios
import time
import tty

from teleop_mobile_base import (acquire_backend_lease, open_exclusive_serial,
                               frame, send, validate, stopping_signals)

INFO = dict(firmware="lift-teleop-v4", host_baud=921600, id=10, max_raw=100,
            watchdog_ms=400, sample_gap_ms=200, down_only=True, stop_keeps_torque=True)
INPUT_LEASE = .25
PERIOD = .05


def receive(port, seconds):
    """Keep partial lines across timeouts so STOP can consume a late reply."""
    if not hasattr(port, "_lift_reply_buffer"): port._lift_reply_buffer = bytearray()
    data = port._lift_reply_buffer
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        part = port.read(1)
        if part == b"\n":
            raw = bytes(data); data.clear()
            value = json.loads(raw.decode("ascii"))
            if not isinstance(value, dict): raise RuntimeError("invalid response")
            return value
        data.extend(part)
        if len(data) > 2048: raise RuntimeError("oversized response")
    raise RuntimeError(f"firmware reply timeout; partial_bytes={len(data)}; partial={bytes(data[:160])!r}")


def inspect(port):
    port.reset_input_buffer()
    port._lift_reply_buffer = bytearray()
    send(port, b"INFO\n")
    info = receive(port, 2)
    if info != INFO: raise RuntimeError("리프트 전용 펌웨어가 아닙니다. 이동하지 않았습니다.")
    return info


def link_check(port, path):
    """Motor-free check: overlapping host commands while MCU deliberately waits."""
    inspect(port)
    send(port, b"STATUS\n"); before = receive(port, 2)
    if before.get("armed") is not False or before.get("fault") is not False:
        raise RuntimeError("통신 검사는 오류 없는 대기 상태에서만 가능합니다.")
    count = 0
    for batch in range(20):
        nonces = [batch*10+i+1 for i in range(10)]
        send(port, b"".join(frame(f"ECHO {n:08X}") for n in nonces))
        for n in nonces:
            row = receive(port, 1)
            if row != dict(type="echo", nonce=n, motor_write_count=before["motor_write_count"]):
                raise RuntimeError("통신 검사 응답 불일치: "+json.dumps(row))
            count += 1
    send(port, b"STATUS\n"); after = receive(port, 2)
    if after.get("fault") is not False or after.get("armed") is not False or after.get("motor_write_count") != before["motor_write_count"]:
        raise RuntimeError("통신 검사 후 상태 오류")
    report = dict(status="link_check_passed", responses=count, blocking_delay_per_reply_ms=25,
                  motor_writes_during_check=0, before=before, after=after)
    save_new(path, report)
    print(f"통신 응답 {count}개 정상 · 모터 쓰기 0회 · {path}", flush=True)
    return report


def idle_check(port, path):
    """Explicit powered preflight: hold zero speed; no nonzero commands or input."""
    inspect(port)
    before = snapshot(port)
    if before["eeprom"][33] != 1 or before["goal"] != 0: raise RuntimeError("속도 모드/정지 상태가 아닙니다.")
    if Path(path).exists(): raise RuntimeError("기존 기록을 덮어쓰지 않습니다.")
    rows=[]; error=None; stopped=None; nonce=secrets.randbelow(0xffffffff)+1
    try:
        send(port, frame(f"TEST {nonce:08X}")); row=receive(port, 2)
        rows.append(row); validate(row, "armed", nonce, 0)
        initial_nonzero=row["nonzero_speed_writes"]
        capture=Capture()
        for seq in range(1,101):
            tick=time.monotonic()
            send(port, frame(f"D {nonce:08X} {seq:08X} Z")); row=receive(port, .3)
            rows.append(row); validate(row, "ack", nonce, seq); capture.add(row)
            if row["nonzero_speed_writes"]!=initial_nonzero or row["goal_raw"]!=0 or abs(row["relative_counts"])>8:
                raise RuntimeError("정지 유지 검사 중 위치/명령 변화")
            time.sleep(max(0, PERIOD-(time.monotonic()-tick)))
        if capture.stationary() is None: raise RuntimeError("정지 상태가 안정적이지 않습니다.")
    except BaseException as exc: error=exc
    finally:
        try:
            stopped=stop(port)
            if stopped.get("fault") is not False: raise RuntimeError("제어기 오류")
        except Exception as exc: error=RuntimeError(f"{error or '종료'}; 정지 확인 실패: {exc}")
    report=dict(status="idle_check_passed" if error is None else "failed", before=before,
                replies=rows, stop=stopped, error=str(error) if error else None,
                nonzero_commands_attempted=0, calibration_valid=False)
    save_new(path,report)
    if error is not None: raise error
    print(f"정지 상태 응답 100개 정상 · 하강 명령 0회 · 정지 확인 완료 · {path}", flush=True)
    return report


def save_new(path, data):
    """Never overwrite another measurement or the pre-write EEPROM backup."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n"); f.flush(); os.fsync(f.fileno())


def snapshot(port):
    send(port, b"SNAP\n")
    value = receive(port, 2)
    eeprom = value.get("eeprom")
    if (value.get("type") != "snapshot" or value.get("id") != 10 or
        not isinstance(eeprom, list) or len(eeprom) != 40 or
        any(type(v) is not int or not 0 <= v <= 255 for v in eeprom) or
        eeprom[3:7] != [9, 3, 10, 0] or eeprom[33] not in (0, 1) or
        value.get("lock") != 1 or value.get("torque") not in (0, 1)):
        raise RuntimeError("모터 설정 조회 실패: " + json.dumps(value))
    return value


def prepare(port, path):
    inspect(port)
    before = snapshot(port)
    if before["torque"] != 0: raise RuntimeError("설정 전환은 토크 OFF에서만 가능합니다.")
    save_new(path, dict(kind="lift_preparation_backup", created_utc=datetime.now(timezone.utc).isoformat(), before=before))
    send(port, frame(f"PREP {secrets.randbelow(0xffffffff)+1:08X}"))
    reply = receive(port, 3)
    if reply.get("type") != "prepared" or reply.get("fault") is not False or reply.get("torque_raw") != 0 or reply.get("goal_raw") != 0:
        raise RuntimeError("설정 전환 확인 실패: " + json.dumps(reply))
    after = snapshot(port)
    expected = list(before["eeprom"]); expected[33] = 1
    if after["eeprom"] != expected or after["torque"] != 0 or after["goal"] != 0:
        raise RuntimeError("모드 외 설정 변경 또는 정지 상태 확인 실패")
    save_new(str(path)+".verified.json", dict(before=before, after=after, reply=reply))
    print("리프트 속도 모드 준비 완료 · 토크 OFF · 자동 이동 없음", flush=True)


def stop(port):
    send(port, b"STOP\n")
    deadline = time.monotonic()+2
    while time.monotonic() < deadline:
        reply = receive(port, max(.01, deadline-time.monotonic()))
        if reply.get("type") == "stopped":
            if reply.get("armed") is not False or reply.get("stop_confirmed") is not True:
                raise RuntimeError("정지 확인 실패: " + json.dumps(reply))
            return reply
    raise RuntimeError("정지 응답 없음")


def command_for(keys, now, current, last_key):
    if len(keys) > 64: raise RuntimeError("입력 과다: 붙여넣기 대신 S 키를 사용하세요.")
    keys = keys.lower()
    if any(k in keys for k in ("x", "\x1b", "\x03")): return "Z", now, "abort"
    if "b" in keys: return "Z", now, "bottom"
    if " " in keys: return "Z", now, None
    if "s" in keys: current, last_key = "D", now
    if now-last_key >= INPUT_LEASE: current = "Z"
    return current, last_key, None


class StartupNotStationary(RuntimeError):
    def __init__(self, reply):
        super().__init__("시작 자세가 안정되지 않았습니다. 모터 쓰기 없이 시작을 취소했습니다. 리프트를 받쳐 고정하세요.")
        self.reply = reply


def arm_when_stationary(port, nonce, test, record):
    """Retry only a proven pre-enable rejection; never retry uncertain ARM."""
    send(port, b"STATUS\n")
    baseline = receive(port, 2); record(baseline)
    if (baseline.get("type") != "status" or baseline.get("armed") is not False
            or baseline.get("fault") is not False
            or type(baseline.get("motor_write_count")) is not int):
        raise RuntimeError("시작 전 제어기 상태 오류: " + json.dumps(baseline))
    deadline = time.monotonic() + 3
    for attempt in range(20):
        send(port, frame(f"{'TEST' if test else 'ARM'} {nonce:08X}"))
        reply = receive(port, 2); record(reply)
        retryable = (reply.get("type") == "error"
                     and reply.get("reason") == "ARM_NOT_STATIONARY"
                     and reply.get("armed") is False and reply.get("fault") is False
                     and reply.get("torque_raw") == 0 and reply.get("goal_raw") == 0
                     and reply.get("motor_write_count") == baseline["motor_write_count"])
        if not retryable:
            validate(reply, "armed", nonce, 0)
            return reply
        if attempt == 19 or time.monotonic() >= deadline:
            raise StartupNotStationary(reply)
        if attempt == 0:
            print("시작 순간 움직임이 감지됐습니다. 받친 자세를 유지하세요. 잠시 안정된 뒤 다시 확인합니다.", flush=True)
        time.sleep(.1)


class Capture:
    def __init__(self):
        self.recent = deque(maxlen=16)
        self.last_tick = self.last_count = None
        self.top = None

    def add(self, row):
        for key in ("relative_counts", "sample_tick", "sample_count", "speed_raw", "goal_raw", "torque_raw"):
            if type(row.get(key)) is not int: raise RuntimeError("위치 기록 형식 오류")
        if row["torque_raw"] != 1 or row.get("fault") is not False: raise RuntimeError("리프트 상태 오류")
        if self.last_tick is not None:
            gap = (row["sample_tick"]-self.last_tick) & 0xffffffff
            if not 0 < gap <= 200 or row["sample_count"] <= self.last_count:
                raise RuntimeError("연속 측정 기록이 끊겼습니다. 보정값으로 사용하지 않습니다.")
        self.last_tick, self.last_count = row["sample_tick"], row["sample_count"]
        self.recent.append(row)

    def stationary(self):
        if len(self.recent) < 9: return None
        rows = list(self.recent)[-9:]
        if ((rows[-1]["sample_tick"]-rows[0]["sample_tick"]) & 0xffffffff) < 350: return None
        values = [r["relative_counts"] for r in rows]
        if (any(r["goal_raw"] != 0 or r["speed_raw"] & 0x7fff for r in rows) or max(values)-min(values) > 8): return None
        return round(sum(values)/len(values))

    def result(self, bottom, top_cm, bottom_cm):
        if self.top is None or bottom-self.top < 128: raise RuntimeError("전체 하강 기록이 충분하지 않습니다.")
        delta = bottom-self.top
        return dict(status="candidate_endpoints_recorded", calibration_valid=True,
                    hardware_motion_validated=False, repeatability_verified=False,
                    top_height_cm=top_cm, bottom_height_cm=bottom_cm,
                    top_counts=self.top, bottom_counts=bottom, travel_counts=delta,
                    mm_per_count=(top_cm-bottom_cm)*10/delta,
                    mm_per_turn=(top_cm-bottom_cm)*10*4096/delta,
                    positive_raw_direction="down", absolute_origin_persists=False,
                    note="Operator-marked endpoints; review before applying a runtime profile.")


def run(port, input_fd, path, *, pulse_check=False, check_duration=.15, top_cm=90, bottom_cm=26):
    if check_duration not in (.15, 2.0): raise ValueError("지원하지 않는 방향 검사 시간")
    visible_check = pulse_check and check_duration == 2.0
    if not os.isatty(input_fd): raise RuntimeError("리프트 조작은 대화형 터미널에서 실행하세요.")
    inspect(port)
    before = snapshot(port)
    if before["eeprom"][33] != 1 or before["goal"] != 0:
        raise RuntimeError("먼저 리프트 속도 모드 준비가 필요합니다.")
    path = Path(path); trace = Path(str(path)+".jsonl")
    if path.exists() or trace.exists(): raise RuntimeError("같은 이름의 기록이 있습니다. 새 파일 이름을 사용하세요.")
    path.parent.mkdir(parents=True, exist_ok=True)
    old = termios.tcgetattr(input_fd)
    capture = Capture(); error = stopped = outcome = None; arm_sent = False
    report = dict(status="incomplete_do_not_use", calibration_valid=False, before=before,
                  started_utc=datetime.now(timezone.utc).isoformat(), test_only=pulse_check,
                  check_duration_s=check_duration if pulse_check else None,
                  check_timer_owner="host_with_400ms_mcu_watchdog" if visible_check else "mcu" if pulse_check else None,
                  top_height_cm=top_cm, bottom_height_cm=bottom_cm)
    with trace.open("x") as log:
        def record(row):
            log.write(json.dumps(dict(host_monotonic=time.monotonic(), reply=row))+"\n");log.flush()
        try:
            tty.setcbreak(input_fd)
            nonce = secrets.randbelow(0xffffffff)+1
            arm_sent = True
            arm_when_stationary(port, nonce, pulse_check and not visible_check, record)
            seq, current, last_key = 0, "Z", float("-inf")
            finishing = False; ready = False; last_print = 0.; finish_started = None
            began = time.monotonic()
            pulse_started = None
            print("정지 상태 확인 중… S 키는 준비 표시 후 누르세요.", flush=True)
            while True:
                tick = time.monotonic()
                if tick-began > 1200: raise RuntimeError("20분 제한 도달")
                keys = ""
                if select.select([input_fd], [], [], 0)[0]:
                    raw = os.read(input_fd, 4096)
                    if not raw: break
                    keys = raw.decode("ascii", errors="strict")
                current, last_key, action = command_for(keys, tick, current, last_key)
                if action == "abort": break
                if action == "bottom" and ready and not pulse_check:
                    finishing = True; finish_started = tick
                if not ready or finishing: current, last_key = "Z", float("-inf")
                if visible_check:
                    if pulse_started is not None and " " in keys:
                        outcome = dict(status="direction_check_stopped_by_operator", calibration_valid=False,
                                       physical_direction_confirmed=False)
                        break
                    if pulse_started is not None and tick-pulse_started >= check_duration:
                        outcome = dict(status="direction_check_finished", calibration_valid=False,
                                       physical_direction_confirmed=False)
                        break  # finally sends STOP even if S remains held
                    if current == "D" and pulse_started is None: pulse_started = tick
                    # A bounded one-shot direction test must not depend on the
                    # terminal's autorepeat configuration. Full descent still
                    # uses command_for's deadman lease. Space/X interrupt here.
                    if pulse_started is not None: current = "D"
                seq += 1
                if seq > 0xffffffff: raise RuntimeError("sequence exhausted")
                send(port, frame(f"D {nonce:08X} {seq:08X} {current}"))
                row = receive(port, .3); record(row)
                if pulse_check and row.get("type") == "stopped" and row.get("reason") == "TEST_PULSE_COMPLETE" and row.get("stop_confirmed") is True and row.get("fault") is False:
                    outcome = dict(status="direction_check_finished", calibration_valid=False,
                                   observed_delta_counts=row.get("relative_counts"), physical_direction_confirmed=False)
                    break
                validate(row, "ack", nonce, seq); capture.add(row)
                stationary = capture.stationary()
                if not ready and stationary is not None:
                    capture.top = stationary; ready = True
                    if visible_check: message = "방향 확인 준비 | S 한 번 → 2초 저속 동작 후 정지·종료. 중간 정지: Space 또는 X."
                    elif pulse_check: message = "방향 확인 준비 | S 한 번 → 약 0.15초 작동 후 정지. 실제로 내려가는지 확인하세요."
                    else: message = "준비 완료 | S 누르는 동안 하강 · Space 정지 · B 최저점 저장 · X 종료"
                    print(message, flush=True)
                if ready and row["relative_counts"] < capture.top-16:
                    raise RuntimeError("예상과 반대 방향의 회전이 감지되어 중단합니다.")
                if finishing and stationary is not None:
                    outcome = capture.result(stationary, top_cm, bottom_cm); break
                if finishing and tick-finish_started > 3: raise RuntimeError("최저점 정지 확인 실패")
                if ready and tick-last_print > 1:
                    print(f"\r누적 회전값 {row['relative_counts']-capture.top:+d} | {'하강' if current=='D' else '정지'}    ", end="", flush=True); last_print=tick
                remaining = PERIOD-(time.monotonic()-tick)
                if remaining > 0: time.sleep(remaining)
        except StartupNotStationary as exc:
            # Every rejected attempt explicitly confirms no enable/write took
            # place. A STOP here cannot confirm holding torque (it is OFF).
            arm_sent = False
            report["startup_rejected_without_motor_writes"] = True
            report["startup_reply"] = exc.reply
            error = exc
        except BaseException as exc:
            error = exc
        finally:
            try:
                if arm_sent:
                    stopped = stop(port); record(stopped)
                    if stopped.get("fault") is not False:
                        # A latched motion fault invalidates the capture, but
                        # is distinct from failure to confirm a physical stop.
                        report["controller_fault_latched"] = True
                        if error is None:
                            error = RuntimeError("제어기 오류로 측정을 취소했습니다. 정지 응답은 확인됐습니다.")
            except Exception as exc:
                error = RuntimeError(f"{error or '종료'}; STOP 확인: {exc}. 리프트를 받치고 모터 전원을 꺼주세요.")
            finally:
                termios.tcsetattr(input_fd, termios.TCSADRAIN, old)
                log.flush(); os.fsync(log.fileno())
    if outcome and error is None:
        if pulse_check: outcome["observed_delta_counts"] = stopped.get("relative_counts")
        report.update(outcome)
    if error is not None: report["error"] = str(error) or type(error).__name__
    report["stop"] = stopped; report["trace_sha256"] = hashlib.sha256(trace.read_bytes()).hexdigest()
    report["finished_utc"] = datetime.now(timezone.utc).isoformat()
    save_new(path, report)
    if report.get("startup_rejected_without_motor_writes"):
        status = "시작 취소 · 모터 쓰기 없음 · 토크 OFF · 리프트를 받쳐주세요."
    else:
        status = "정지 응답 확인 완료 · 토크 유지" if stopped and stopped.get("stop_confirmed") is True else "정지·토크 상태를 확인하지 못했습니다. 리프트를 받치고 모터 전원을 꺼주세요."
    print(f"\n기록 저장: {path}\n{status}\n모터 모드는 자동 복구하지 않습니다.", flush=True)
    if error and not isinstance(error, KeyboardInterrupt): raise error
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--prepare", action="store_true")
    group.add_argument("--check-direction", action="store_true")
    group.add_argument("--run", action="store_true")
    group.add_argument("--link-check", action="store_true", help="Motor-free buffered UART check")
    group.add_argument("--idle-check", action="store_true", help="Explicit torque-on zero-speed check; no descent commands")
    parser.add_argument("--direction-confirmed", action="store_true", help="Use only after the short pulse physically moved DOWN")
    parser.add_argument("--check-duration", type=float, choices=(.15, 2.0), default=.15,
                        help="2 seconds is an S-triggered one-shot with host deadline and MCU 400 ms link watchdog; Space/X stop")
    parser.add_argument("--file", type=Path)
    parser.add_argument("--top-cm", type=float, default=90)
    parser.add_argument("--bottom-cm", type=float, default=26)
    args = parser.parse_args()
    if (args.prepare or args.check_direction or args.run or args.link_check or args.idle_check) and not args.file: parser.error("--file is required")
    if args.run and not args.direction_confirmed: parser.error("--check-direction 후 실제 하강을 확인하고 --direction-confirmed를 지정하세요.")
    if args.check_duration != .15 and not args.check_direction: parser.error("--check-duration requires --check-direction")
    if not 0 < args.bottom_cm < args.top_cm < 200: parser.error("invalid measured heights")
    try:
        import serial
        with stopping_signals(), acquire_backend_lease("stm32", 0):
            with open_exclusive_serial(serial, args.port, 921600, .01) as port:
                if args.link_check: link_check(port, args.file)
                elif args.idle_check: idle_check(port, args.file)
                elif args.prepare: prepare(port, args.file)
                elif args.run or args.check_direction: run(port, sys.stdin.fileno(), args.file, pulse_check=args.check_direction, check_duration=args.check_duration, top_cm=args.top_cm, bottom_cm=args.bottom_cm)
                else: print(json.dumps(inspect(port), indent=2))
        return 0
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f"실행 중단: {exc}", file=sys.stderr); return 2


if __name__ == "__main__": raise SystemExit(main())
