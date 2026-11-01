"""Offline calibration failures and connection to the existing observation path."""
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from home_robot_tasks.camera_registration import fit_fixed_camera
from home_robot_tasks.assistant.contracts import Context, Stamp
from home_robot_tasks.assistant.perception import TransformSample, report_entities

ROOT = Path(__file__).resolve().parents[1]


def record():
    # Optical right/down/forward -> base forward/left/up, with a tilt.
    angle = .35
    tilt = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0],
                     [-np.sin(angle), 0, np.cos(angle)]])
    rotation = tilt @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    translation = np.array([.08, -.04, .58])
    camera = np.array([[-.2, -.15, .5], [.2, -.15, .5], [-.2, .15, .5],
                       [.2, .15, .5], [0, -.1, .7], [.1, .1, .8],
                       [-.12, .08, .6], [.12, -.06, .65], [0, .16, .75]])
    base = camera @ rotation.T + translation
    pairs = [dict(id=f"p{i}", camera_xyz_m=c.tolist(), base_xyz_m=b.tolist())
             for i, (c, b) in enumerate(zip(camera, base))]
    return dict(schema_version=1, units="m", source_frame="color_camera_optical_frame",
                target_frame="base_link", mount_revision="synthetic-mount-1",
                base_reference="synthetic base origin; not a physical survey",
                provenance="simulation", maximum_error_m=.015, fit=pairs[:6], check=pairs[6:]), rotation, translation


def test_fit_recovers_optical_to_base_without_granting_motion():
    data, rotation, translation = record()
    result = fit_fixed_camera(data)
    assert result["within_threshold"]
    assert np.allclose(np.asarray(result["base_from_optical"])[:3, :3], rotation)
    assert np.allclose(np.asarray(result["base_from_optical"])[:3, 3], translation)
    assert result["check"]["maximum_m"] < 1e-12
    assert not result["motion_authorized"] and not result["physical_accuracy_verified"]
    assert result["provenance"] == "simulation"


def test_holdout_is_not_used_to_hide_wrong_coordinates():
    data, _, _ = record()
    original = fit_fixed_camera(data)
    data["check"][0]["base_xyz_m"][0] += .04
    bad = fit_fixed_camera(data)
    assert np.allclose(bad["base_from_optical"], original["base_from_optical"])
    assert bad["status"] == "rejected_error_threshold"
    assert bad["check"]["maximum_m"] == pytest.approx(.04)
    assert bad["calibration_id"] != original["calibration_id"]


@pytest.mark.parametrize("case", ["collinear", "duplicates", "reused_holdout", "wrong_units",
                                 "nan", "bool", "negative_depth", "missing_check"])
def test_unusable_correspondences_are_rejected(case):
    data, _, _ = record()
    if case == "collinear":
        for i, pair in enumerate(data["fit"]): pair["camera_xyz_m"] = [i * .05, 0, .5]
    elif case == "duplicates": data["check"][0]["id"] = data["fit"][0]["id"]
    elif case == "reused_holdout":
        data["check"][0] = {**deepcopy(data["fit"][0]), "id": "renamed"}
    elif case == "wrong_units": data["units"] = "mm"
    elif case == "nan": data["fit"][0]["base_xyz_m"][0] = float("nan")
    elif case == "bool": data["fit"][0]["base_xyz_m"][0] = True
    elif case == "negative_depth": data["fit"][0]["camera_xyz_m"][2] = -1
    else: data["check"] = []
    with pytest.raises(ValueError): fit_fixed_camera(data)


def test_planar_targets_work_but_nonrigid_scale_or_reflection_fails():
    data, _, _ = record()
    planar = deepcopy(data)
    for pair in planar["fit"] + planar["check"]:
        pair["camera_xyz_m"][2] = .6
        pair["base_xyz_m"] = [pair["camera_xyz_m"][0] + .2, pair["camera_xyz_m"][1], .9]
    assert fit_fixed_camera(planar)["within_threshold"]
    for pair in data["fit"] + data["check"]: pair["base_xyz_m"][0] *= -1
    reflected = fit_fixed_camera(data)
    assert not reflected["within_threshold"]
    assert np.linalg.det(np.asarray(reflected["base_from_optical"])[:3, :3]) == pytest.approx(1)


def test_registration_matrix_connects_to_existing_grounding_observation():
    data, rotation, translation = record()
    result = fit_fixed_camera(data)
    context = Context("local-survey", "1", result["calibration_id"], "offline", "base_link")
    stamp = Stamp(context=context, capture_id="capture-1", observed_s=10)
    transform = TransformSample(stamp, data["source_frame"], "base_link", result["base_from_optical"], "simulation")
    point = [.03, -.02, .6]
    report = {"schema_version": 1, "coordinate_frame": data["source_frame"], "detections": [
        {"id": 0, "label": "black marker", "score": .9,
         "surface_point": {"valid": True, "xyz_m": point}}]}
    entity = report_entities(report, stamp=stamp, context=context, now_s=10, transform=transform)[0]
    assert np.allclose(entity["entity"]["position_m"], rotation @ point + translation)
    assert entity["grasp_pose"] is None


def test_cli_writes_review_and_preserves_existing_output(tmp_path):
    data, _, _ = record()
    points, output = tmp_path / "points.json", tmp_path / "candidate.json"
    points.write_text(json.dumps(data))
    argv = [sys.executable, str(ROOT / "tools/diagnostics/s2_coordinates.py"), "fit-camera",
            "--points", str(points), "--output", str(output)]
    assert subprocess.run(argv, capture_output=True).returncode == 0
    original = output.read_bytes()
    assert subprocess.run(argv, capture_output=True).returncode == 2
    assert output.read_bytes() == original
