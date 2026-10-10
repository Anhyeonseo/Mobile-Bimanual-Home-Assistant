"""Geometry checks independent of the detector, network, GPU, and camera."""
import numpy as np
import pytest
import json

pytest.importorskip("pyrealsense2")
cv2 = pytest.importorskip("cv2")
from tools.lib.grounding_rgbd import GroundingRGBD, normalize_prompt, surface_point


INTRINSICS = {"width": 32, "height": 24, "fx": 100.0, "fy": 100.0,
              "ppx": 16.0, "ppy": 12.0, "coeffs": [0.0]*5, "model": "distortion.none"}


def test_metric_projection_uses_optical_axes_and_depth_scale():
    depth = np.full((24, 32), 2000, np.uint16)
    point = surface_point(depth, 0.001, INTRINSICS, [16, 12, 24, 20])
    assert point["valid"]
    assert point["pixel"] == [20, 16]
    assert point["xyz_m"] == pytest.approx([0.08, 0.08, 2.0])
    assert point["range_m"] > point["depth_z_m"]


def test_missing_depth_does_not_produce_a_coordinate():
    point = surface_point(np.zeros((24, 32), np.uint16), 0.001, INTRINSICS, [4, 4, 20, 20])
    assert not point["valid"] and point["xyz_m"] is None
    assert point["reason"] == "insufficient_depth"


def test_depth_discontinuity_rejects_background_mixture():
    depth = np.full((24, 32), 500, np.uint16)
    depth[:, 16:] = 2000
    point = surface_point(depth, 0.001, INTRINSICS, [8, 4, 24, 20])
    assert point["reason"] == "mixed_or_unstable_depth"
    assert point["xyz_m"] is None


def test_center_hole_uses_a_real_valid_pixel_and_records_which_one():
    depth = np.full((24, 32), 1000, np.uint16)
    depth[12, 16] = 0
    point = surface_point(depth, 0.001, INTRINSICS, [8, 4, 24, 20])
    assert point["valid"] and point["pixel"] != [16, 12]
    x, y = point["pixel"]
    assert depth[y, x] * 0.001 == point["depth_z_m"]
    assert point["valid_fraction"] < 1


@pytest.mark.parametrize("box", [[-10, -10, -1, -1], [5, 5, 5, 10], [0, 0, float('nan'), 3]])
def test_invalid_boxes_cannot_sample_wrapped_array_indices(box):
    point = surface_point(np.ones((24, 32), np.uint16), 0.001, INTRINSICS, box)
    assert not point["valid"] and point["xyz_m"] is None


def test_thin_box_does_not_sample_background_outside_box():
    depth = np.full((24, 32), 2500, np.uint16)
    depth[4:20, 15:18] = 500
    point = surface_point(depth, 0.001, INTRINSICS, [15, 4, 18, 20])
    assert point["valid"] and point["depth_z_m"] == 0.5
    assert point["patch_xyxy"][0::2] == [15, 18]


def test_wrong_intrinsics_resolution_rejected():
    with pytest.raises(ValueError, match="dimensions"):
        surface_point(np.zeros((12, 16), np.uint16), 0.001, INTRINSICS, [1, 1, 8, 8])


def test_mask_surface_ignores_background_and_never_leaves_mask():
    mask = np.zeros((24, 32), bool)
    mask[4:20, 5:8] = True  # Thin off-centre object; box centre is background.
    depth = np.full(mask.shape, 2000, np.uint16)
    depth[mask] = 400
    point = surface_point(depth, 0.001, INTRINSICS, [2, 2, 30, 22], mask=mask)
    assert point["valid"] and point["depth_z_m"] == 0.4
    x, y = point["pixel"]
    assert mask[y, x]
    depth[mask] = 0
    point = surface_point(depth, 0.001, INTRINSICS, [2, 2, 30, 22], mask=mask)
    assert not point["valid"] and point["xyz_m"] is None
    assert point["reason"] == "insufficient_depth"


def test_empty_or_mismatched_mask_cannot_fall_back_to_box_depth():
    depth = np.full((24, 32), 400, np.uint16)
    point = surface_point(depth, 0.001, INTRINSICS, [2, 2, 30, 22], mask=np.zeros(depth.shape, bool))
    assert point["reason"] == "empty_mask" and point["xyz_m"] is None
    with pytest.raises(ValueError, match="mask"):
        surface_point(depth, 0.001, INTRINSICS, [2, 2, 30, 22], mask=np.ones((12, 16), bool))


def test_prompt_normalization():
    assert normalize_prompt(" Black marker pen ") == "black marker pen."
    with pytest.raises(ValueError):
        normalize_prompt(" ... ")


def test_terminal_queries_reuse_model_but_capture_again_and_recover_from_failure(capsys):
    from tools.diagnostics.realsense_viewer import interactive_search, parse_args

    args = parse_args(["--interactive-prompt", "--device", "cpu"])
    assert args.headless and args.frames > 0
    queries = iter(["  ", "black marker", "screwdriver", "pliers", "q"])
    models, captures = [], []

    def load(*_):
        model = object()
        models.append(model)
        return model

    def capture(query_args, detector):
        captures.append((query_args.prompt, detector))
        if query_args.prompt == "screwdriver.":
            raise RuntimeError("Camera temporarily unavailable")
        return {"status": "NO_DETECTIONS", "inference_seconds": 0.1, "detections": []}, None, "test-result"

    interactive_search(args, input_fn=lambda _: next(queries), detector_factory=load, capture_fn=capture)
    assert len(models) == 1
    assert captures == [(p, models[0]) for p in ("black marker.", "screwdriver.", "pliers.")]
    assert "temporarily unavailable" in capsys.readouterr().err


@pytest.mark.parametrize("found", [False, True])
@pytest.mark.parametrize("sam_quality", [None, 0.9, 0.2])
def test_snapshot_result_keeps_frame_provenance_and_handles_no_detection(tmp_path, found, sam_quality):
    cv2.imwrite(str(tmp_path / "color.png"), np.zeros((24, 32, 3), np.uint8))
    cv2.imwrite(str(tmp_path / "depth_aligned.png"), np.full((24, 32), 1000, np.uint16))
    # A smoothed display file must never silently replace measured coordinates.
    cv2.imwrite(str(tmp_path / "depth_preview_filtered.png"), np.full((24, 32), 2000, np.uint16))
    metadata = {"coordinate_frame": "color_camera_optical_frame", "alignment": "depth to color via rs.align",
                "depth_scale_m": 0.001, "aligned_depth_intrinsics": INTRINSICS,
                "captured_at_utc": "2026-10-10T00:00:00+00:00", "depth_frame": {"number": 7},
                "color_frame": {"number": 8}}
    (tmp_path / "metadata.json").write_text(json.dumps(metadata))
    detector = GroundingRGBD.__new__(GroundingRGBD)
    detector.device, detector.versions = "test", {}
    detector.box_threshold, detector.text_threshold = 0.3, 0.25
    detector.use_sam = sam_quality is not None
    if detector.use_sam:
        from types import SimpleNamespace
        mask = np.zeros((24, 32), bool)
        mask[6:18, 14:19] = True
        detector.segmenter = SimpleNamespace(segment=lambda *_: ([(mask, sam_quality)], 0.02))
    boxes = [{"id": 0, "label": "marker pen", "score": 0.8, "box_xyxy": [12, 4, 20, 20]}] if found else []
    detector.detect = lambda rgb, prompt: (boxes, 0.01)
    result, image, result_dir = detector.analyze_snapshot(tmp_path, "black marker pen")
    expected = "NO_DETECTIONS" if not found else "DETECTED_NO_DEPTH" if sam_quality == 0.2 else "OK"
    assert result["status"] == expected
    assert result["depth_frame"] == {"number": 7}
    assert result["color_frame"] == {"number": 8}
    assert len(result["source_sha256"]["depth_aligned.png"]) == 64
    assert result["robot_transform"] is None
    assert json.loads((result_dir / "detections.json").read_text())["status"] == result["status"]
    if found:
        point = result["detections"][0]["surface_point"]
        if sam_quality == 0.2:
            assert point["xyz_m"] is None and point["reason"] == "low_mask_quality"
        elif sam_quality is None:
            assert point["xyz_m"] == pytest.approx([0, 0, 1])
        else:
            x, y = point["pixel"]
            assert mask[y, x] and point["xyz_m"][2] == 1
        if detector.use_sam:
            info = result["detections"][0]["segmentation"]
            saved = cv2.imread(str(result_dir / info["mask_file"]), cv2.IMREAD_UNCHANGED)
            assert np.array_equal(saved > 0, mask)
            assert len(info["sha256"]) == 64
    else:
        assert result["detections"] == []
