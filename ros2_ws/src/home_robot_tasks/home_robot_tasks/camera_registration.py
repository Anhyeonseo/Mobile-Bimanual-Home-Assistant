"""Offline L3A fixed-camera registration from independent 3-D correspondences.

Fits base_from_optical (metres), never a grasp pose or a motion permission.
Validation points are held out from fitting. No ROS, sensor or actuator I/O.
"""
from __future__ import annotations

import hashlib
import json
import math
import numpy as np


def _text(value, name):
    if not isinstance(value, str) or not value.strip() or len(value) > 160:
        raise ValueError(f"{name}: nonempty bounded text required")
    return value


def _number(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name}: finite number required")
    return float(value)


def _points(rows, *, minimum, seen_ids):
    if not isinstance(rows, list) or not minimum <= len(rows) <= 256:
        raise ValueError(f"between {minimum} and 256 correspondences required")
    camera, base, ids = [], [], []
    for row in rows:
        if not isinstance(row, dict) or set(row) != {"id", "camera_xyz_m", "base_xyz_m"}:
            raise ValueError("id, camera_xyz_m and base_xyz_m required")
        identifier = _text(row["id"], "point id")
        if identifier in seen_ids:
            raise ValueError("fit/check point IDs must be unique")
        seen_ids.add(identifier)
        ids.append(identifier)
        for key, values in (("camera_xyz_m", camera), ("base_xyz_m", base)):
            xyz = row[key]
            if not isinstance(xyz, list) or len(xyz) != 3:
                raise ValueError("3-D point required")
            point = [_number(v, key) for v in xyz]
            if max(abs(v) for v in point) > 10:
                raise ValueError("point outside local 10m workspace; check metre units")
            values.append(point)
        if camera[-1][2] <= 0:
            raise ValueError("positive optical depth required")
    return np.asarray(camera), np.asarray(base), ids


def _spread(points, name):
    # Planar targets are valid for 3-D/3-D registration; a line or tiny cluster
    # cannot constrain the full rotation reliably. These are tool defaults,
    # not a measured camera accuracy or workspace coverage guarantee.
    values = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)
    rms = values / math.sqrt(len(points))
    if rms[1] < .01 or rms[1] / rms[0] < .01:
        raise ValueError(f"{name}: points need spread in at least two directions")


def _errors(camera, base, ids, rotation, translation):
    errors = np.linalg.norm(camera @ rotation.T + translation - base, axis=1)
    return {"count": len(ids), "rms_m": float(np.sqrt(np.mean(errors**2))),
            "maximum_m": float(errors.max()),
            "per_point_m": dict(zip(ids, errors.tolist()))}


def fit_fixed_camera(record):
    """Return a reviewable candidate; never silently discard mismatched points."""
    keys = {"schema_version", "units", "source_frame", "target_frame",
            "mount_revision", "base_reference", "provenance", "maximum_error_m",
            "fit", "check"}
    if not isinstance(record, dict) or set(record) != keys:
        raise ValueError("complete fixed-camera correspondence record required")
    if type(record["schema_version"]) is not int or record["schema_version"] != 1:
        raise ValueError("unsupported correspondence schema")
    if record["units"] != "m" or record["target_frame"] != "base_link":
        raise ValueError("metre points in base_link required")
    if record["source_frame"] not in {"color_camera_optical_frame", "camera_color_optical_frame"}:
        raise ValueError("explicit colour optical frame required")
    if record["provenance"] not in {"measured", "simulation"}:
        raise ValueError("measured or simulation provenance required")
    for key in ("mount_revision", "base_reference"):
        _text(record[key], key)
    tolerance = _number(record["maximum_error_m"], "maximum_error_m")
    if not 0 < tolerance <= .1:
        raise ValueError("explicit error threshold in (0, 0.1] metres required")
    ids = set()
    camera, base, fit_ids = _points(record["fit"], minimum=6, seen_ids=ids)
    check_camera, check_base, check_ids = _points(record["check"], minimum=3, seen_ids=ids)
    for points, name in ((camera, "camera fit"), (base, "base fit"),
                         (check_camera, "camera check"), (check_base, "base check")):
        _spread(points, name)
    # Reject exact reuse even when IDs have been renamed. Actual independent
    # measurement still requires operator review; numbers cannot prove that.
    for points in (np.vstack((camera, check_camera)), np.vstack((base, check_base))):
        distances = np.linalg.norm(points[:, None] - points[None, :], axis=2)
        np.fill_diagonal(distances, np.inf)
        if np.any(distances < 1e-6):
            raise ValueError("distinct fit and held-out check positions required")
    x, y = camera.mean(axis=0), base.mean(axis=0)
    u, _, vt = np.linalg.svd((camera - x).T @ (base - y))
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    translation = y - rotation @ x
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rotation, translation
    fit = _errors(camera, base, fit_ids, rotation, translation)
    check = _errors(check_camera, check_base, check_ids, rotation, translation)
    passed = max(fit["maximum_m"], check["maximum_m"]) <= tolerance
    canonical = json.dumps(record, sort_keys=True, separators=(",", ":"), allow_nan=False)
    digest = hashlib.sha256(canonical.encode()).hexdigest()
    return {"schema_version": 1, "record_kind": "fixed_camera_registration",
            "calibration_id": "fixed-camera-" + digest[:20],
            "source_sha256": digest, "source_hash_encoding": "canonical_json",
            "source_frame": record["source_frame"], "target_frame": "base_link",
            "mount_revision": record["mount_revision"], "base_reference": record["base_reference"],
            "provenance": record["provenance"], "units": "m",
            "transform_convention": "p_base = base_from_optical @ p_optical",
            "base_from_optical": matrix.tolist(), "fit": fit, "check": check,
            "maximum_error_m": tolerance, "within_threshold": passed,
            "status": "candidate_pending_review" if passed else "rejected_error_threshold",
            "hardware_enabled": False, "motion_authorized": False,
            "physical_accuracy_verified": False,
            "limitations": ["Measured provenance is an operator assertion pending review.",
                            "Errors apply only to the submitted sampled workspace.",
                            "Independent camera/TCP validation and mount revision binding remain required."]}
