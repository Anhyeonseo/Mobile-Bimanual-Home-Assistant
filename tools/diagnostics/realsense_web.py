#!/usr/bin/env python3
"""Loopback-only RGB-D monitor and snapshot search, accessed through SSH forwarding."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import sys
import threading
import time
from urllib.parse import urlsplit

import cv2

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.diagnostics import realsense_viewer as viewer
from tools.lib.grounding_rgbd import normalize_prompt


class Monitor:
    def __init__(self, output, device="cuda", detector_factory=None):
        self.output, self.device = output, device
        self.detector_factory = detector_factory
        self.lock = threading.Lock()
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.detector = None
        self.snapshot = None
        self.jpeg = None
        self.updated = 0.0
        self.sequence = 0
        self.busy = False
        self.phase = "대기 중"
        self.query = None
        self.error = None
        self.camera_error = None
        self.report = None
        self.result_jpeg = None
        self.result_id = 0

    def publish(self, rgb, raw, native, preview, metadata, filtered):
        now = time.monotonic()
        if now - self.updated < 1 / 8:
            return
        ok, encoded = cv2.imencode(".jpg", preview[viewer.HEADER:], [cv2.IMWRITE_JPEG_QUALITY, 78])
        if not ok:
            raise RuntimeError("Could not encode camera preview")
        # SDK frame buffers are reused; retain an owned, coherent RGB-D snapshot.
        snapshot = (rgb.copy(), raw.copy(), native.copy(), preview.copy(), metadata, filtered.copy())
        with self.lock:
            self.snapshot, self.jpeg = snapshot, encoded.tobytes()
            self.updated, self.sequence = now, self.sequence + 1

    def status(self):
        with self.lock:
            age = time.monotonic() - self.updated if self.snapshot is not None else None
            metadata = self.snapshot[4] if self.snapshot is not None else {}
            return {
                "camera_ready": age is not None and age < 2 and self.camera_error is None,
                "camera_error": self.camera_error, "frame_age_seconds": age,
                "sequence": self.sequence, "captured_at_utc": metadata.get("captured_at_utc"),
                "fps": metadata.get("observed_fps"), "valid_depth_percent": metadata.get("valid_depth_percent"),
                "busy": self.busy, "phase": self.phase, "query": self.query,
                "error": self.error, "result_id": self.result_id, "report": self.report,
            }

    def search(self, prompt):
        if not isinstance(prompt, str) or len(prompt) > 200:
            raise ValueError("물체 설명은 200자 이내의 문자열이어야 합니다.")
        prompt = normalize_prompt(prompt)
        with self.lock:
            if self.busy:
                raise RuntimeError("이미 분석 중입니다.")
            if self.snapshot is None or time.monotonic() - self.updated > 2 or self.camera_error:
                raise RuntimeError("새 카메라 프레임을 기다려 주세요.")
            snapshot = self.snapshot
            self.busy, self.query, self.error = True, prompt, None
            self.report, self.result_jpeg = None, None
            self.phase = "모델 준비 중 · 첫 검색은 시간이 걸립니다" if self.detector is None else "촬영본 분석 중"
        self.executor.submit(self._detect, snapshot, prompt)

    def _detect(self, snapshot, prompt):
        try:
            folder = viewer.save_snapshot(self.output, *snapshot)
            if self.detector is None:
                if self.detector_factory is None:
                    from tools.lib.grounding_rgbd import GroundingRGBD
                    self.detector_factory = lambda device: GroundingRGBD(device, use_sam=True)
                self.detector = self.detector_factory(self.device)
            with self.lock:
                self.phase = "Grounding DINO 탐지 + SAM 마스킹 중"
            report, _, result_dir = self.detector.analyze_snapshot(folder, prompt)
            jpeg = (result_dir / "detections.jpg").read_bytes()
            with self.lock:
                self.report, self.result_jpeg = report, jpeg
                self.result_id += 1
                self.phase = "분석 완료"
        except Exception as error:
            with self.lock:
                self.error, self.phase = str(error), "분석 실패"
        finally:
            with self.lock:
                self.busy = False


def handler_for(monitor):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def respond(self, code, body, content_type="application/json; charset=utf-8"):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/":
                self.respond(200, Path(__file__).with_name("realsense_web.html").read_bytes(), "text/html; charset=utf-8")
            elif path == "/api/status":
                self.respond(200, monitor.status())
            elif path in ("/frame.jpg", "/result.jpg"):
                with monitor.lock:
                    data = monitor.jpeg if path == "/frame.jpg" else monitor.result_jpeg
                self.respond(200, data, "image/jpeg") if data else self.respond(503, {"error": "Image not ready"})
            else:
                self.respond(404, {"error": "Not found"})

        def do_POST(self):
            if self.path != "/api/detect":
                return self.respond(404, {"error": "Not found"})
            # No CORS: only our same-origin page may start GPU work.
            origin = self.headers.get("Origin")
            if (self.headers.get("X-Requested-With") != "rgbd-viewer" or
                    self.headers.get("Content-Type", "").split(";")[0] != "application/json" or
                    (origin is not None and urlsplit(origin).netloc != self.headers.get("Host"))):
                return self.respond(403, {"error": "Same-origin JSON request required"})
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 2048:
                    raise ValueError("Invalid request size")
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict):
                    raise ValueError("Expected JSON object")
                monitor.search(body.get("prompt"))
                self.respond(202, {"accepted": True})
            except (ValueError, UnicodeDecodeError) as error:
                self.respond(400, {"error": str(error)})
            except RuntimeError as error:
                self.respond(409, {"error": str(error)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--profile", choices=("near", "far"), default="near")
    parser.add_argument("--device", choices=("cuda", "cpu", "auto"), default="cuda")
    parser.add_argument("--output", type=Path, default=ROOT / "tmp/realsense-web")
    args = parser.parse_args()
    depth_args = (["--depth-width", "640", "--depth-height", "360", "--min-depth", "0.2", "--max-depth", "0.8"]
                  if args.profile == "near" else ["--min-depth", "0.3", "--max-depth", "2.0"])
    camera_args = viewer.parse_args(["--headless", "--frames", "1", *depth_args])
    camera_args.frames = 0  # Bounded by the server stop event instead.
    monitor = Monitor(args.output, args.device)
    stop = threading.Event()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_for(monitor))
    server.daemon_threads = True

    def capture():
        try:
            viewer.run(camera_args, frame_callback=monitor.publish, stop_event=stop)
        except Exception as error:
            with monitor.lock:
                monitor.camera_error = str(error)
            print(f"CAMERA ERROR: {error}", file=sys.stderr, flush=True)

    def shutdown(*_):
        stop.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGHUP, shutdown)
    camera = threading.Thread(target=capture, daemon=True)
    camera.start()
    print(f"WEB http://127.0.0.1:{args.port} profile={args.profile}", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        stop.set()
        server.server_close()
        camera.join(timeout=7)
        monitor.executor.shutdown(wait=True)


if __name__ == "__main__":
    main()
