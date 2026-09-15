from dataclasses import replace
import numpy as np
import pytest
from home_robot_tasks.capture_port import (
    CapturePort,
    ImageDetection,
    ImagePair,
    LatestRGBDSource,
    detection_samples,
)
from home_robot_tasks.workflow_ports import Phase, SequencePort
from home_robot_tasks.execution import Feedback
from home_robot_tasks.skill_ports import PortResult
from home_robot_tasks.stationary_rgbd import (
    StationaryRGBD,
    CapturePolicy,
    MotionEvidence,
    RGBDFrame,
)
from home_robot_tasks.perception import CameraIntrinsics


class Port:
    def __init__(self):
        self.state = "RUNNING"
        self.started = []
        self.cancelled = []
        self.cancel_error = False

    def ready(self):
        return True

    def start(self, g, p):
        self.started.append((g, p))

    def poll(self, g):
        return PortResult(self.state)

    def cancel(self, g):
        self.cancelled.append(g)
        if self.cancel_error:
            raise RuntimeError("lost cancel ACK")

    def result(self, g):
        return [ImageDetection("remote", 0.9, ((20, 20), (40, 20), (40, 30), (20, 30)))]


def test_sequence_requires_fresh_independent_arrival_and_waits_cancel():
    now = [0.0]
    p = Port()
    conditions = {"stopped": True, "safe": True, "arrived": False}
    s = SequencePort(
        lambda _: [Phase("navigate", p, {}, ("stopped",), ("safe",), ("arrived",), 2)],
        lambda g, n, t: Feedback(g, t, "RUNNING", dict(conditions)),
        lambda: now[0],
    )
    s.start("g", {})
    s.poll("g")
    assert p.started
    p.state = "SUCCEEDED"
    now[0] = 0.1
    assert s.poll("g").state == "RUNNING"
    conditions["arrived"] = True
    assert s.poll("g").state == "SUCCEEDED"
    p.state = "RUNNING"
    s.start("h", {})
    s.poll("h")
    conditions["safe"] = False
    assert s.poll("h").state == "RUNNING" and s.owner == "h" and p.cancelled
    p.state = "CANCELLED"
    assert s.poll("h").state == "FAILED" and s.owner is None


def test_cancel_failure_does_not_release_owner_and_timeout_is_finite():
    p = Port()
    p.cancel_error = True
    now = [0.0]
    s = SequencePort(
        lambda _: [Phase("lift", p, {}, (), (), (), 1)],
        lambda g, n, t: Feedback(g, t, "RUNNING", {}),
        lambda: now[0],
    )
    s.start("g", {})
    s.poll("g")
    now[0] = 1
    assert s.poll("g").state == "RUNNING" and s.owner == "g"
    assert "cancel_unconfirmed" in s.entries["g"]["reason"]
    p.state = "CANCELLED"
    assert s.poll("g").state == "FAILED"


@pytest.mark.parametrize("timeout", [float("nan"), float("inf"), 0, -1])
def test_bad_phase_budget(timeout):
    s = SequencePort(
        lambda _: [Phase("x", Port(), {}, (), (), (), timeout)], None, lambda: 0
    )
    with pytest.raises(ValueError):
        s.start("g", {})


def capture_fixture():
    now = [1.0]
    motion = [MotionEvidence(1.0, 0.0, "pose-1", True, True)]
    camera = StationaryRGBD(CapturePolicy(width=64, height=48))
    frame = RGBDFrame(
        "image-1",
        1.0,
        "pose-1",
        "camera_color_optical_frame",
        "simulation-only",
        64,
        48,
        CameraIntrinsics(50, 50, 32, 24),
        "base_link",
        1.0,
        ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)),
        True,
    )
    pair = ImagePair(
        frame,
        np.zeros((48, 64, 3), np.uint8),
        np.ones((48, 64), np.float32),
        1.0,
        ("seat",),
        (),
    )
    source = LatestRGBDSource()
    detector = Port()
    detector.state = "SUCCEEDED"
    c = CapturePort(source, detector, camera, lambda: motion[0], lambda: now[0])
    return c, source, pair, now, motion


def test_capture_freezes_buffers_and_samples_oriented_depth():
    c, source, pair, now, motion = capture_fixture()
    source.publish(pair)
    c.start("g", {"requested_s": 1.0})
    c.poll("g")
    pair.depth_m[:] = 2
    assert c.poll("g").state == "SUCCEEDED"
    frame, kwargs = c.result("g")
    d = kwargs["detections"][0]
    assert len(d.samples_uv_depth) == 81 and all(
        z == 1 for _, _, z in d.samples_uv_depth
    )
    target = c.camera.locate(
        frame, motion[0], d.samples_uv_depth, now_s=1.0, requested_s=1.0
    )
    assert target["pose_revision"] == "pose-1"


@pytest.mark.parametrize(
    "fault", ["moved", "stale", "depth_skew", "unregistered", "shape"]
)
def test_capture_rejects_motion_and_invalid_image_evidence(fault):
    c, source, pair, now, motion = capture_fixture()
    if fault == "depth_skew":
        pair = replace(pair, depth_observed_s=0.9)
    if fault == "unregistered":
        pair = replace(pair, frame=replace(pair.frame, registered_rectified=False))
    if fault == "shape":
        pair = replace(pair, depth_m=np.ones((2, 2)))
    source.publish(pair)
    c.start("g", {"requested_s": 1.0})
    c.poll("g")
    if fault == "moved":
        motion[0] = replace(motion[0], pose_revision="pose-2")
    if fault == "stale":
        now[0] = 2.0
        motion[0] = replace(motion[0], observed_s=2.0)
    c.poll("g")
    assert c.poll("g").state == "FAILED"
