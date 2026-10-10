"""Snapshot isolation and request handling while the live camera keeps publishing."""
import json
import threading
import time

import numpy as np
import pytest

pytest.importorskip("pyrealsense2")
cv2 = pytest.importorskip("cv2")
from tools.diagnostics.realsense_web import Monitor


def test_search_keeps_clicked_frame_while_live_frames_change(tmp_path):
    entered, release = threading.Event(), threading.Event()
    observed = []

    class Detector:
        def __init__(self, device):
            entered.set()
            assert release.wait(5)

        def analyze_snapshot(self, folder, prompt):
            depth = cv2.imread(str(folder / "depth_aligned.png"), cv2.IMREAD_UNCHANGED)
            metadata = json.loads((folder / "metadata.json").read_text())
            observed.append((int(depth[0, 0]), metadata["captured_at_utc"], prompt))
            (folder / "detections.jpg").write_bytes(b"result")
            return {"status": "OK"}, None, folder

    monitor = Monitor(tmp_path, detector_factory=Detector)
    rgb = np.zeros((8, 8, 3), np.uint8)
    depth = np.full((8, 8), 350, np.uint16)
    preview = np.zeros((142, 16, 3), np.uint8)
    try:
        monitor.publish(rgb, depth, depth, preview, {"captured_at_utc": "first"}, depth)
        depth[:] = 900  # Simulate reuse of the SDK frame buffer.
        monitor.search("black marker")
        assert entered.wait(5)
        with pytest.raises(RuntimeError, match="분석 중"):
            monitor.search("screwdriver")
        monitor.updated = 0
        monitor.publish(rgb, depth, depth, preview, {"captured_at_utc": "second"}, depth)
        release.set()
        monitor.executor.shutdown(wait=True)
        assert observed == [(350, "first", "black marker.")]
        assert monitor.status()["captured_at_utc"] == "second"
        assert monitor.status()["report"] == {"status": "OK"}
        monitor.updated = time.monotonic() - 3
        with pytest.raises(RuntimeError, match="프레임"):
            monitor.search("black marker")
        assert not monitor.status()["camera_ready"]
    finally:
        release.set()
        monitor.executor.shutdown(wait=True)
