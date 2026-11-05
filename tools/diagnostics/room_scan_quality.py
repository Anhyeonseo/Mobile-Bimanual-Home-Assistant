"""Small image-quality hints shared by capture and offline review.

These heuristics do not certify tracking, coverage, or a collision map.
Depth statistics refer to the supplied depth image, not RGB pixel correspondences.
"""
from __future__ import annotations


def image_quality(color, depth, depth_scale=0.001):
    import cv2
    import numpy as np

    gray = cv2.cvtColor(cv2.resize(color, (320, 180)), cv2.COLOR_BGR2GRAY)
    meters = cv2.resize(depth, (320, 180), interpolation=cv2.INTER_NEAREST) * depth_scale
    useful = (meters >= 0.5) & (meters <= 3.0)
    features = cv2.goodFeaturesToTrack(gray, 200, 0.01, 7)
    result = {
        "valid_depth_fraction": float(np.mean(meters > 0)),
        "useful_depth_fraction": float(np.mean(useful)),
        "near_depth_fraction": float(np.mean((meters > 0) & (meters < 0.5))),
        "sharpness": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        "dark_fraction": float(np.mean(gray < 20)),
        "bright_fraction": float(np.mean(gray > 245)),
        "features": 0 if features is None else len(features),
    }
    result["warnings"] = quality_warnings(result)
    result["ready"] = not result["warnings"]
    return result


def quality_warnings(metrics):
    warnings = []
    if metrics["useful_depth_fraction"] < 0.35:
        warnings.append("깊이 부족 · 렌즈 가림과 거리를 확인하고 가구를 0.5~3m에서 비춰주세요")
    if metrics["sharpness"] < 30 or metrics["features"] < 25:
        warnings.append("선명도·무늬 부족 · 잠시 멈추고 가구 모서리와 주변을 함께 비춰주세요")
    if metrics["dark_fraction"] > 0.5:
        warnings.append("화면이 어두움 · 조명 또는 노출을 확인하세요")
    if metrics["bright_fraction"] > 0.5:
        warnings.append("화면이 너무 밝음 · 창문 또는 노출을 확인하세요")
    return warnings


class Readiness:
    """Require a continuous interval of recent good samples before normal start."""
    def __init__(self, seconds=2.0):
        self.seconds = seconds
        self.since = None
        self.last = None

    def update(self, now, good):
        if self.last is not None and now - self.last > 1.5:
            self.since = None
        self.last = now
        if not good:
            self.since = None
        elif self.since is None:
            self.since = now
        return self.ready(now)

    def ready(self, now):
        return (self.since is not None and self.last is not None
                and now - self.last <= 1.5 and now - self.since >= self.seconds)


class MotionHint:
    """Image displacement hint, not metric camera velocity or SLAM tracking."""
    def __init__(self):
        self.previous = None
        self.when = None

    def update(self, color, now):
        import cv2
        import numpy as np
        gray = cv2.cvtColor(cv2.resize(color, (320, 180)), cv2.COLOR_BGR2GRAY)
        speed = None
        if self.previous is not None and 0.1 < now - self.when < 1.5:
            points = cv2.goodFeaturesToTrack(self.previous, 80, .01, 7)
            if points is not None and len(points) >= 12:
                tracked, status, _ = cv2.calcOpticalFlowPyrLK(self.previous, gray, points, None)
                if tracked is not None:
                    good = status[:, 0].astype(bool)
                    if np.count_nonzero(good) >= 12:
                        displacement = np.linalg.norm(tracked[good] - points[good], axis=2)
                        speed = float(np.median(displacement) / (320 * (now - self.when)))
        self.previous, self.when = gray, now
        return speed


def summarize_quality(samples):
    count = len(samples)
    good = sum(bool(s["ready"]) for s in samples)
    return {"samples": count, "good_samples": good,
            "good_sample_fraction": good / count if count else 0.0,
            "status": "needs_review" if not count or good / count < 0.9 else "image_hints_ok",
            "tracking_verified": False, "coverage_verified": False,
            "note": "화질 참고값입니다. 3D 경로 연결·집 전체 촬영 완료 여부는 별도 복원 검사가 필요합니다."}
