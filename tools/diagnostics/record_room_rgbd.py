#!/usr/bin/env python3
"""Record native D415 RGB-D on the camera host; no robot or inference connection."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gc
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib.metadata
import json
from pathlib import Path
import select
import shutil
import signal
import sys
import threading
import time
import uuid

if __package__:
    from .room_scan_quality import MotionHint, Readiness, image_quality, summarize_quality
else:
    from room_scan_quality import MotionHint, Readiness, image_quality, summarize_quality

WIDTH, HEIGHT, FPS = 1280, 720, 30
GIB = 1024 ** 3


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


class FrameStats:
    """Count unique frames, including repeated framesets emitted by the synchronizer."""
    def __init__(self):
        self.count = 0
        self.first_ms = self.last_ms = None
        self.last_number = None
        self.missing_numbers = 0
        self.max_gap_ms = 0.0
        self.nonmonotonic = 0

    def add(self, number, timestamp):
        if number == self.last_number:
            return
        if self.last_ms is not None:
            gap = timestamp - self.last_ms
            self.max_gap_ms = max(self.max_gap_ms, gap)
            self.nonmonotonic += int(gap <= 0 or number < self.last_number)
            self.missing_numbers += max(0, number - self.last_number - 1)
        else:
            self.first_ms = timestamp
        self.last_ms, self.last_number = timestamp, number
        self.count += 1

    def report(self):
        seconds = (self.last_ms - self.first_ms) / 1000 if self.count > 1 else 0.0
        return {"frames": self.count, "duration_seconds": seconds,
                "observed_fps": (self.count - 1) / seconds if seconds > 0 else 0.0,
                "missing_frame_numbers": self.missing_numbers,
                "max_gap_ms": self.max_gap_ms, "nonmonotonic": self.nonmonotonic}


def assess_playback(streams, duration, expected_seconds=None):
    issues = []
    for name in ("color", "depth"):
        s = streams[name]
        if s["frames"] < 2 or s["duration_seconds"] < max(0, duration - 0.5):
            issues.append(f"{name}: incomplete duration")
        if s["observed_fps"] < FPS * 0.9 or s["max_gap_ms"] > 250 or s["nonmonotonic"]:
            issues.append(f"{name}: frame timing needs review")
    if duration <= 0 or (expected_seconds is not None and duration < expected_seconds - 1):
        issues.append("recording ended earlier than expected")
    return issues


class Preview:
    """Read-only, loopback preview; camera settings and recording are terminal-only."""
    def __init__(self, port):
        self.jpeg = None
        self.phase = "카메라 준비 중"
        self.finished = False
        self.quality = None
        self.readiness = Readiness()
        self.motion = MotionHint()
        self.depth_scale = 0.001
        self.samples = []
        self.record_begin = None
        self.updated = 0.0
        self.server = None
        if port == 0:
            return
        monitor = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def do_GET(self):
                if self.path == "/":
                    data, kind = ("<!doctype html><html lang='ko'><meta charset='utf-8'>"
                                  "<title>방 스캔 촬영</title><style>body{background:#15202b;color:white;"
                                  "font:18px sans-serif;margin:3%}img{width:100%;max-width:1280px}"
                                  "</style><h1>방 스캔 촬영</h1><p id='status'></p><p id='quality'></p><img id='frame'>"
                                  "<p>렌즈를 가리지 말고 가구와 주변을 함께 촬영하세요. 완료 안내 전에는 카메라를 내려놓지 마세요.</p>"
                                  "<p>화면 속 사람·물체는 움직이지 않게 하고, 마지막에는 시작 위치와 방향으로 돌아오세요.</p>"
                                  "<p>왼쪽 RGB · 오른쪽 깊이 / 녹화 시작·종료는 SSH 터미널에서 Enter</p>"
                                  "<script>const timer=setInterval(async()=>{try{const s=await(await fetch('/status')).json();"
                                  "document.querySelector('#status').textContent=s.phase;"
                                  "document.querySelector('#quality').textContent=s.quality_text;"
                                  "if(s.finished){clearInterval(timer);return;}"
                                  "document.querySelector('#frame').src='/frame.jpg?t='+Date.now()}"
                                  "catch{clearInterval(timer);document.querySelector('#status').textContent="
                                  "'미리보기 연결 종료 · 저장/검사 결과는 터미널에서 확인하세요'}},500)</script>"
                                  "</html>").encode(), "text/html; charset=utf-8"
                elif self.path == "/status":
                    data, kind = json.dumps({"phase": monitor.phase, "finished": monitor.finished,
                                             "quality_text": monitor.quality_text()}, ensure_ascii=False).encode(), "application/json"
                elif self.path.startswith("/frame.jpg") and monitor.jpeg:
                    data, kind = monitor.jpeg, "image/jpeg"
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", kind)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def publish(self, frames, cv2, np):
        if time.monotonic() - self.updated < 0.5:
            return
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not color or not depth:
            return
        rgb = cv2.resize(np.asanyarray(color.get_data()), (640, 360))
        raw = cv2.resize(np.asanyarray(depth.get_data()), (640, 360), interpolation=cv2.INTER_NEAREST)
        now = time.monotonic()
        self.quality = image_quality(rgb, raw, self.depth_scale)
        motion = self.motion.update(rgb, now)
        self.quality["image_widths_per_second"] = motion
        if motion is not None and motion > 0.20:
            self.quality["warnings"].append("화면 이동이 빠름 · 속도를 줄이고 같은 가구가 화면에 이어지도록 촬영하세요")
            self.quality["ready"] = False
        self.readiness.update(now, self.quality["ready"])
        if self.record_begin is not None:
            self.samples.append({"seconds": now - self.record_begin, **self.quality})
        # Display only: the bag retains unaligned, unfiltered 16-bit depth.
        depth_image = cv2.applyColorMap(cv2.convertScaleAbs(raw, alpha=255 * self.depth_scale / 4), cv2.COLORMAP_TURBO)
        depth_image[raw == 0] = 0
        ok, encoded = cv2.imencode(".jpg", np.hstack((rgb, depth_image)), [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.jpeg, self.updated = encoded.tobytes(), time.monotonic()

    def quality_text(self):
        if self.quality is None:
            return "화질 확인 중"
        q = self.quality
        return f"0.5~3m 깊이 {q['useful_depth_fraction']:.0%} · " + (
            " / ".join(q["warnings"]) if q["warnings"] else "화질 양호 · 경로 연결과 촬영 범위는 복원 후 확인")

    def close(self):
        if self.server:
            self.phase = "촬영 화면 종료 · 저장/검사 결과는 터미널에서 확인하세요"
            self.finished = True
            # Give the 500ms browser poll a chance to stop before the SSH destination closes.
            time.sleep(1)
            self.server.shutdown()
            self.server.server_close()


def line_available():
    if select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline()
        if not line:
            raise RuntimeError("SSH 입력 연결이 끊어졌습니다.")
        return line.strip().lower()
    return None


def configuration(rs, serial=None, bag=None):
    cfg = rs.config()
    if serial:
        cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    if bag:
        cfg.enable_record_to_file(str(bag))
    return cfg


def profile_metadata(profile, rs):
    result = {}
    for name, stream in (("color", rs.stream.color), ("depth", rs.stream.depth)):
        p = profile.get_stream(stream).as_video_stream_profile()
        intr = p.get_intrinsics()
        result[name] = {"width": p.width(), "height": p.height(), "fps": p.fps(), "format": str(p.format()),
                        "intrinsics": {k: getattr(intr, k) for k in ("fx", "fy", "ppx", "ppy", "coeffs")},
                        "distortion_model": str(intr.model)}
        if (p.width(), p.height(), p.fps()) != (WIDTH, HEIGHT, FPS):
            raise RuntimeError(f"요청과 다른 스트림 설정: {name}")
    extr = profile.get_stream(rs.stream.depth).get_extrinsics_to(profile.get_stream(rs.stream.color))
    result["depth_to_color"] = {"rotation": extr.rotation, "translation_m": extr.translation}
    result["depth_scale_m"] = profile.get_device().first_depth_sensor().get_depth_scale()
    return result


def verify(bag, rs, expected_seconds=None):
    """Read every native stream via callbacks, without a synchronizer dropping frames."""
    context = rs.context()
    playback = context.load_device(str(bag)).as_playback()
    playback.set_real_time(False)
    duration = playback.get_duration().total_seconds()
    stats = {name: FrameStats() for name in ("color", "depth")}
    opened, started, seen = [], [], set()

    def read(frame):
        name = "color" if frame.get_profile().stream_type() == rs.stream.color else "depth"
        stats[name].add(frame.get_frame_number(), frame.get_timestamp())

    try:
        for sensor in playback.query_sensors():
            profiles = []
            for p in sensor.get_stream_profiles():
                if p.stream_type() not in (rs.stream.color, rs.stream.depth):
                    continue
                v = p.as_video_stream_profile()
                expected_format = rs.format.bgr8 if p.stream_type() == rs.stream.color else rs.format.z16
                if (v.width(), v.height(), p.fps(), p.format()) != (WIDTH, HEIGHT, FPS, expected_format):
                    raise RuntimeError("파일의 해상도·형식이 촬영 요청과 다릅니다.")
                seen.add(p.stream_type())
                profiles.append(p)
            if profiles:
                sensor.open(profiles)
                opened.append(sensor)
        if seen != {rs.stream.color, rs.stream.depth}:
            raise RuntimeError("파일에 RGB와 깊이가 모두 필요합니다.")
        for sensor in opened:
            sensor.start(read)
            started.append(sensor)
        deadline = time.monotonic() + max(60, duration * 2)
        while not (playback.current_status() == rs.playback_status.stopped and any(s.count for s in stats.values())):
            if time.monotonic() > deadline:
                raise RuntimeError("파일 전체 재생 검사가 제한 시간을 넘었습니다.")
            time.sleep(0.05)
        streams = {name: s.report() for name, s in stats.items()}
        issues = assess_playback(streams, duration, expected_seconds)
        return {"status": "pass" if not issues else "needs_review", "issues": issues,
                "bag_bytes": bag.stat().st_size, "bag_duration_seconds": duration,
                "streams": streams, "scene_quality_review_required": True}
    finally:
        for sensor in started:
            sensor.stop()
        for sensor in opened:
            sensor.close()


def verify_and_save(bag, rs, expected_seconds=None):
    """Leave an explicit state even when SDK playback fails or the process is killed."""
    destination = bag.with_name("verification.json")
    write_json(destination, {"status": "checking", "started_at_utc": datetime.now(timezone.utc).isoformat()})
    try:
        result = verify(bag, rs, expected_seconds)
    except Exception as error:
        write_json(destination, {"status": "error", "error": str(error), "scene_quality_review_required": True})
        raise
    write_json(destination, result)
    return result


def record(args, rs, cv2, np):
    if not args.no_prompt and not sys.stdin.isatty():
        raise RuntimeError("대화형 SSH(-t)로 실행하세요. 자동 검사만 --no-prompt를 사용합니다.")
    args.output.mkdir(parents=True, exist_ok=True)
    required = WIDTH * HEIGHT * 5 * FPS * args.seconds * 1.2 + 2 * GIB
    free = shutil.disk_usage(args.output).free
    if free < required:
        raise RuntimeError(f"여유 공간이 부족합니다. 최소 {required / GIB:.1f}GiB 필요")
    context = rs.context()
    devices = [d for d in context.query_devices() if "D415" in d.get_info(rs.camera_info.name)]
    if len(devices) != 1:
        raise RuntimeError(f"D415 한 대를 연결하세요. 현재 {len(devices)}대")
    device = devices[0]
    usb = device.get_info(rs.camera_info.usb_type_descriptor)
    if not usb.startswith("3"):
        raise RuntimeError(f"USB 3 연결이 필요합니다. 현재 {usb}")
    serial = device.get_info(rs.camera_info.serial_number)
    color, depth = device.first_color_sensor(), device.first_depth_sensor()
    color_keys = ("enable_auto_exposure", "exposure", "gain", "enable_auto_white_balance", "white_balance")
    saved_color = {key: color.get_option(getattr(rs.option, key)) for key in color_keys}
    saved_emitter = depth.get_option(rs.option.emitter_enabled)
    stop = threading.Event()
    old_signals = {sig: signal.signal(sig, lambda *_: stop.set()) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    preview, pipeline, recorder, started, folder = None, None, None, False, None
    metadata = {"schema_version": 1, "purpose": "handheld_room_scan", "robot_extrinsics_valid": False,
                "requested_seconds": args.seconds, "sdk_version": importlib.metadata.version("pyrealsense2"),
                "device": {"name": device.get_info(rs.camera_info.name), "usb": usb},
                "raw_depth_unaligned_unfiltered": True, "status": "preparing"}
    try:
        preview = Preview(args.preview_port)
        folder = args.output / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:6])
        folder.mkdir()
        write_json(folder / "settings.json", metadata)
        recorder = rs.recorder(str(folder / "recording.bag"), device)
        recorder.pause()
        color, depth = recorder.first_color_sensor(), recorder.first_depth_sensor()
        color.set_option(rs.option.enable_auto_exposure, 1)
        color.set_option(rs.option.enable_auto_white_balance, 1)
        depth.set_option(rs.option.emitter_enabled, 1)
        pipeline = rs.pipeline(context)
        pipeline.set_device(recorder)
        profile = pipeline.start(configuration(rs, serial))
        started = True
        preview.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        print("카메라를 방의 주요 가구 쪽으로 향해 주세요. 5초간 밝기·색을 맞춥니다.", flush=True)
        until = time.monotonic() + 5
        while time.monotonic() < until and not stop.is_set():
            preview.publish(pipeline.wait_for_frames(5000), cv2, np)
        if stop.is_set():
            metadata["status"] = "cancelled"
            return None
        fixed = {key: color.get_option(getattr(rs.option, key)) for key in ("exposure", "gain", "white_balance")}
        if args.exposure is not None:
            fixed["exposure"] = args.exposure
        if args.white_balance is not None:
            fixed["white_balance"] = args.white_balance
        color.set_option(rs.option.enable_auto_exposure, 0)
        color.set_option(rs.option.enable_auto_white_balance, 0)
        for key, value in fixed.items():
            color.set_option(getattr(rs.option, key), value)
        preview.readiness = Readiness()
        preview.phase = "밝기·색 고정 완료 · 화면 확인 후 터미널에서 Enter로 녹화 시작"
        print(f"RGB 고정값: {fixed}\n카메라를 든 상태에서 렌즈 가림 없이 방을 비춰주세요.\n"
              "사람은 화면 밖으로, 물체는 그대로 두세요. 마지막에는 시작 위치·방향으로 돌아옵니다.\n"
              "화질이 2초간 안정되면 Enter → 녹화 / q + Enter → 취소\n"
              "화질 안내가 실제 장면과 다를 때만 start + Enter로 시작할 수 있습니다.", flush=True)
        waiting_since, next_hint = time.monotonic(), 0
        while not stop.is_set():
            preview.publish(pipeline.wait_for_frames(5000), cv2, np)
            now = time.monotonic()
            ready = preview.readiness.ready(now)
            if now >= next_hint:
                print(preview.quality_text() + (" · Enter로 시작 가능" if ready else " · 안정된 화면 대기"), flush=True)
                next_hint = now + 3
            if args.no_prompt:
                if ready or args.allow_poor_start:
                    break
                if now - waiting_since > 15:
                    raise RuntimeError("화질 준비 검사 실패. 렌즈 가림·거리·밝기를 확인하세요.")
                continue
            line = line_available()
            if line == "q":
                metadata["status"] = "cancelled"
                return None
            if line is not None:
                if ready or line == "start":
                    metadata["quality_start_override"] = not ready
                    break
                print("아직 준비되지 않았습니다. " + preview.quality_text(), flush=True)
        if stop.is_set():
            metadata["status"] = "cancelled"
            return None
        (folder / "ground-truth.txt").write_text(
            "물체의 실제 위치를 적고 별도 배치 사진을 이 폴더에 복사해 주세요.\n"
            "방 이름: \n촬영자: \n물체 / 가구 / 가구 위 상세 위치\n"
            "리모컨 /  / \n컵 /  / \n계산기 /  / \n기타: \n"
            "촬영 중 물체 이동·사람 출입 여부: \n특이사항: \n")
        metadata.update(profile_metadata(profile, rs))
        metadata["color_options"] = {key: color.get_option(getattr(rs.option, key)) for key in color_keys}
        metadata["depth_options"] = {key: depth.get_option(getattr(rs.option, key)) for key in
                                     ("enable_auto_exposure", "exposure", "gain", "emitter_enabled", "laser_power") if depth.supports(getattr(rs.option, key))}
        if metadata["color_options"]["enable_auto_exposure"] or metadata["color_options"]["enable_auto_white_balance"]:
            raise RuntimeError("RGB 자동 노출·화이트밸런스가 꺼지지 않았습니다.")
        metadata.update(status="recording", started_at_utc=datetime.now(timezone.utc).isoformat())
        write_json(folder / "settings.json", metadata)
        recorder.resume()
        begin = time.monotonic()
        preview.record_begin = begin
        metadata.setdefault("quality_start_override", bool(args.no_prompt and args.allow_poor_start))
        end = begin + args.seconds
        next_report, ending, saved_start = 0, False, False
        print(f"녹화 시작: {folder}\n처음 3초는 정지. 종료할 때 Enter → 3초 정지 후 저장.", flush=True)
        while time.monotonic() < end and not stop.is_set():
            frames = pipeline.wait_for_frames(5000)
            preview.publish(frames, cv2, np)
            elapsed, remaining = time.monotonic() - begin, end - time.monotonic()
            instruction = "처음 3초 정지" if elapsed < 3 else "천천히 이동"
            if remaining <= 3:
                instruction = "마지막 3초 정지"
            preview.phase = f"녹화 {elapsed:.0f}초 / 남은 시간 {remaining:.0f}초 · {instruction}"
            if not saved_start and elapsed >= 1 and preview.quality and preview.quality["ready"] and frames.get_color_frame():
                if not cv2.imwrite(str(folder / "scene-start.jpg"), np.asanyarray(frames.get_color_frame().get_data())):
                    raise RuntimeError("시작 장면 사진 저장 실패")
                saved_start = True
            if not args.no_prompt and line_available() is not None and not ending:
                end = min(end, time.monotonic() + 3)
                ending = True
                print("그 자리에서 3초간 정지해 주세요. 저장 후 종료합니다.", flush=True)
            if elapsed >= next_report:
                print(preview.phase, flush=True)
                if preview.quality and preview.quality["warnings"]:
                    print(preview.quality_text(), flush=True)
                next_report = elapsed + 1
                if shutil.disk_usage(folder).free < 2 * GIB:
                    raise RuntimeError("여유 공간이 2GiB 아래로 내려가 녹화를 종료합니다.")
        metadata.update(status="interrupted" if stop.is_set() else "recorded", elapsed_seconds=time.monotonic() - begin)
    except Exception as error:
        metadata.update(status="error", error=str(error))
        raise
    finally:
        if started:
            try:
                recorder.pause()
                pipeline.stop()
            except Exception as error:
                metadata.update(status="error", stop_error=str(error))
        # Restore the previous preview settings; preserve the recording's fixed values in settings.json.
        restore_errors = []
        for key in ("exposure", "gain", "white_balance", "enable_auto_exposure", "enable_auto_white_balance"):
            try:
                color.set_option(getattr(rs.option, key), saved_color[key])
            except Exception as error:
                restore_errors.append(str(error))
        try:
            depth.set_option(rs.option.emitter_enabled, saved_emitter)
        except Exception as error:
            restore_errors.append(str(error))
        if folder:
            metadata["restore_errors"] = restore_errors
            write_json(folder / "settings.json", metadata)
            if preview:
                write_json(folder / "quality.json", {"summary": summarize_quality(preview.samples),
                                                    "samples": preview.samples})
        if preview:
            preview.close()
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
    # Release the SDK recorder before opening its file for playback.
    del profile, pipeline, recorder, color, depth
    gc.collect()
    print("녹화 종료. 이제 카메라를 내려놓아도 됩니다. 전체 RGB·깊이 재생 검사 중…", flush=True)
    result = verify_and_save(folder / "recording.bag", rs, metadata["elapsed_seconds"])
    print(f"재생 검사: {result['status']} · {result['bag_duration_seconds']:.1f}초 · {result['bag_bytes'] / GIB:.2f}GiB\n저장 폴더: {folder}", flush=True)
    print("화질 기록: quality.json · 파일 정상 여부와 3D 복원 가능 여부는 별도입니다.", flush=True)
    if result["issues"]:
        raise RuntimeError("재생 검사 확인 필요: " + "; ".join(result["issues"]))
    if metadata["status"] != "recorded":
        raise RuntimeError("녹화가 중단되었습니다. 저장 파일은 일부 촬영본으로 확인하세요.")
    return folder


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path.home() / "robot/shared/data/room-scans")
    parser.add_argument("--seconds", type=int, default=180)
    parser.add_argument("--preview-port", type=int, default=8765, help="Loopback HTTP port; 0 disables preview server")
    parser.add_argument("--exposure", type=float, help="Fixed RGB exposure in SDK units (100 microseconds)")
    parser.add_argument("--white-balance", type=float, help="Fixed RGB white balance in kelvin")
    parser.add_argument("--no-prompt", action="store_true", help="Start after warm-up; for an explicitly requested timed capture")
    parser.add_argument("--allow-poor-start", action="store_true", help="Explicitly bypass image-readiness checks with --no-prompt")
    parser.add_argument("--verify", type=Path, help="Only replay this bag; do not open the physical camera")
    args = parser.parse_args(argv)
    if not 6 <= args.seconds <= 600:
        parser.error("--seconds는 6~600초여야 합니다.")
    import pyrealsense2 as rs
    try:
        if args.verify:
            result = verify_and_save(args.verify, rs)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return int(result["status"] != "pass")
        import cv2
        import numpy as np
        record(args, rs, cv2, np)
        return 0
    except Exception as error:
        print(f"촬영/검사 중단: {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
