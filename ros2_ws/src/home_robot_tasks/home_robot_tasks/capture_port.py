"""Registered RGB-D capture and detector boundary; no old workcell homography.

The source and detector implement ready/start/poll/cancel. Their results remain
asynchronous so slow inference cannot block task deadlines or stop polling.
RGB values are uint8 RGB, depth is float metres in the SAME rectified image.
"""

from dataclasses import dataclass
import math
import numpy as np
from .fetch import InvalidTask
from .navigation import number
from .search import Detection
from .skill_ports import PortResult
from .stationary_rgbd import RGBDFrame


@dataclass(frozen=True)
class ImagePair:
    frame: RGBDFrame
    rgb: object
    depth_m: object
    depth_observed_s: float
    visible_regions: tuple = ()
    occluded_regions: tuple = ()


@dataclass(frozen=True)
class ImageDetection:
    object_id: str
    confidence: float
    # Four oriented rectangle corners in rectified colour pixels.
    corners_uv: tuple


def detection_samples(item, pair):
    if (
        not isinstance(item, ImageDetection)
        or not 0 <= number(item.confidence, "confidence") <= 1
    ):
        raise InvalidTask("invalid image detection")
    corners = np.asarray(item.corners_uv, dtype=float)
    if corners.shape != (4, 2) or not np.isfinite(corners).all():
        raise InvalidTask("four finite OBB corners required")
    h, w = pair.depth_m.shape
    if np.any(corners < 0) or np.any(corners[:, 0] >= w) or np.any(corners[:, 1] >= h):
        raise InvalidTask("detection outside image")
    a, b, c, d = corners
    cross = lambda x, y: x[0] * y[1] - x[1] * y[0]
    signs = [
        cross(
            corners[(i + 1) % 4] - corners[i],
            corners[(i + 2) % 4] - corners[(i + 1) % 4],
        )
        for i in range(4)
    ]
    if not (all(v > 0 for v in signs) or all(v < 0 for v in signs)):
        raise InvalidTask("convex ordered corners required")
    # Interior bilinear grid avoids background at the bounding rectangle edges.
    samples = []
    for v in np.linspace(0.2, 0.8, 9):
        for u in np.linspace(0.2, 0.8, 9):
            xy = (1 - v) * ((1 - u) * a + u * b) + v * ((1 - u) * d + u * c)
            x, y = int(round(xy[0])), int(round(xy[1]))
            z = float(pair.depth_m[y, x])
            if math.isfinite(z) and z > 0:
                samples.append((x, y, z))
    return Detection(item.object_id, item.confidence, tuple(samples))


class CapturePort:
    def __init__(
        self,
        source,
        detector,
        camera,
        motion,
        clock,
        *,
        timeout_s=5,
        maximum_rgb_depth_skew_s=0.02
    ):
        self.source, self.detector, self.camera, self.motion, self.clock = (
            source,
            detector,
            camera,
            motion,
            clock,
        )
        self.timeout = number(timeout_s, "capture timeout")
        self.skew = number(maximum_rgb_depth_skew_s, "RGB/depth skew")
        if (
            not 0 < self.skew <= self.camera.policy.maximum_age_s
            or not 0 < self.timeout <= 30
        ):
            raise InvalidTask("invalid capture bounds")
        self.entries = {}
        self.owner = None

    def ready(self):
        return self.owner is None and self.source.ready() and self.detector.ready()

    def start(self, goal_id, parameters):
        if not self.ready() or goal_id in self.entries or len(self.entries) >= 128:
            raise InvalidTask("capture owned or duplicate")
        now = number(self.clock(), "capture start")
        requested = number(parameters["requested_s"], "requested time")
        if not 0 <= requested <= now:
            raise InvalidTask("invalid capture request time")
        self.camera.stationary_since(self.motion(), now)
        self.entries[goal_id] = {
            "state": "RUNNING",
            "port": self.source,
            "started": now,
            "requested": requested,
            "reason": "",
            "cancel": False,
            "pair": None,
        }
        self.owner = goal_id
        try:
            self.source.start(goal_id, parameters)
        except Exception as error:
            self._stop(goal_id, str(error))

    def _stop(self, goal_id, reason, cancelled=False):
        e = self.entries[goal_id]
        if e["cancel"]:
            return
        e.update(
            cancel=True,
            reason=reason,
            destination="CANCELLED" if cancelled else "FAILED",
        )
        try:
            e["port"].cancel(goal_id)
        except Exception:
            e["reason"] += ";cancel_unconfirmed"

    def cancel(self, goal_id):
        self._stop(goal_id, "cancelled", True)

    def poll(self, goal_id):
        e = self.entries[goal_id]
        if e["state"] != "RUNNING":
            return PortResult(e["state"], e["reason"])
        try:
            now = number(self.clock(), "capture clock")
            if now < e["started"] or now - e["started"] >= self.timeout:
                self._stop(goal_id, "capture_timeout_or_clock")
            if e["cancel"]:
                if e["port"].poll(goal_id).state in {
                    "SUCCEEDED",
                    "FAILED",
                    "CANCELLED",
                }:
                    e["state"] = e["destination"]
                    self.owner = None
                return PortResult(e["state"], e["reason"])
            result = e["port"].poll(goal_id)
            if result.state in {"FAILED", "CANCELLED"}:
                self._stop(goal_id, result.reason or "capture_backend_failed")
            elif result.state == "SUCCEEDED":
                if e["port"] is self.source:
                    pair = self.source.result(goal_id)
                    if not isinstance(pair, ImagePair):
                        raise InvalidTask("image pair required")
                    self.camera.validate_frame(
                        pair.frame, self.motion(), now_s=now, requested_s=e["requested"]
                    )
                    if (
                        abs(
                            number(pair.depth_observed_s, "depth time")
                            - pair.frame.observed_s
                        )
                        > self.skew
                    ):
                        raise InvalidTask("unsynchronized RGB and depth")
                    if (
                        not isinstance(pair.rgb, np.ndarray)
                        or pair.rgb.dtype != np.uint8
                        or pair.rgb.shape != (pair.frame.height, pair.frame.width, 3)
                    ):
                        raise InvalidTask("uint8 RGB required")
                    if (
                        not isinstance(pair.depth_m, np.ndarray)
                        or pair.depth_m.dtype.kind != "f"
                        or pair.depth_m.shape != pair.rgb.shape[:2]
                    ):
                        raise InvalidTask("registered depth in metres required")
                    # Freeze buffers before the camera reuses them while inference runs.
                    e["pair"] = ImagePair(
                        pair.frame,
                        pair.rgb.copy(),
                        pair.depth_m.copy(),
                        pair.depth_observed_s,
                        pair.visible_regions,
                        pair.occluded_regions,
                    )
                    e["port"] = self.detector
                    self.detector.start(goal_id, {"rgb": e["pair"].rgb})
                else:
                    pair = e["pair"]
                    self.camera.validate_frame(
                        pair.frame, self.motion(), now_s=now, requested_s=e["requested"]
                    )
                    items = self.detector.result(goal_id)
                    if not isinstance(items, (list, tuple)) or len(items) > 64:
                        raise InvalidTask("bounded detections required")
                    e["result"] = (
                        pair.frame,
                        {
                            "detections": tuple(
                                detection_samples(i, pair) for i in items
                            ),
                            "visible_regions": pair.visible_regions,
                            "occluded_regions": pair.occluded_regions,
                        },
                    )
                    e["state"] = "SUCCEEDED"
                    self.owner = None
        except Exception as error:
            self._stop(goal_id, str(error))
        return PortResult(e["state"], e["reason"])

    def result(self, goal_id):
        e = self.entries[goal_id]
        if e["state"] != "SUCCEEDED":
            raise InvalidTask("capture incomplete")
        return e["result"]


class LatestRGBDSource:
    """ROS/image replay producer writes the latest synchronized pair, never a queue."""

    def __init__(self):
        import threading

        self.lock = threading.Lock()
        self.latest = None
        self.entries = {}
        self.owner = None

    def publish(self, pair):
        if not isinstance(pair, ImagePair):
            raise InvalidTask("image pair required")
        with self.lock:
            if (
                self.latest is not None
                and pair.frame.observed_s <= self.latest.frame.observed_s
            ):
                raise InvalidTask("nonmonotonic image exposure")
            self.latest = ImagePair(
                pair.frame,
                np.array(pair.rgb, copy=True),
                np.array(pair.depth_m, copy=True),
                pair.depth_observed_s,
                pair.visible_regions,
                pair.occluded_regions,
            )

    def ready(self):
        return self.owner is None

    def start(self, goal_id, parameters):
        if not self.ready() or goal_id in self.entries or len(self.entries) >= 128:
            raise InvalidTask("image source owned or duplicate")
        self.entries[goal_id] = {
            "state": "RUNNING",
            "requested": parameters["requested_s"],
        }
        self.owner = goal_id

    def poll(self, goal_id):
        e = self.entries[goal_id]
        with self.lock:
            if (
                e["state"] == "RUNNING"
                and self.latest is not None
                and self.latest.frame.observed_s >= e["requested"]
            ):
                e["result"] = self.latest
                e["state"] = "SUCCEEDED"
                self.owner = None
        return PortResult(e["state"])

    def cancel(self, goal_id):
        if self.entries[goal_id]["state"] == "RUNNING":
            self.entries[goal_id]["state"] = "CANCELLED"
            self.owner = None

    def result(self, goal_id):
        return self.entries[goal_id]["result"]


class InferencePort:
    """One bounded worker; timeout/cancel discards late results by goal identity.

    The detector callable only receives an image and returns ImageDetection[].
    It has no robot port, so a cancelled inference cannot issue a late motion.
    """

    def __init__(self, detect):
        from concurrent.futures import ThreadPoolExecutor

        self.detect = detect
        self.worker = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="rgbd-inference"
        )
        self.entries = {}
        self.future = None

    def ready(self):
        return self.future is None or self.future.done()

    def start(self, goal_id, parameters):
        if not self.ready() or goal_id in self.entries or len(self.entries) >= 128:
            raise InvalidTask("inference busy or duplicate")
        future = self.worker.submit(self.detect, parameters["rgb"].copy())
        self.future = future
        self.entries[goal_id] = {"future": future, "cancel": False}

    def poll(self, goal_id):
        e = self.entries[goal_id]
        if e["cancel"]:
            return PortResult("CANCELLED")
        if not e["future"].done():
            return PortResult("RUNNING")
        return PortResult(
            "FAILED" if e["future"].exception() else "SUCCEEDED",
            "inference_error" if e["future"].exception() else "",
        )

    def cancel(self, goal_id):
        self.entries[goal_id]["cancel"] = True
        self.entries[goal_id]["future"].cancel()

    def result(self, goal_id):
        if self.poll(goal_id).state != "SUCCEEDED":
            raise InvalidTask("inference incomplete")
        return self.entries[goal_id]["future"].result()

    def close(self):
        self.worker.shutdown(wait=False, cancel_futures=True)
