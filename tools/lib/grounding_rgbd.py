"""Text-guided detection and measured surface points from one RGB-D snapshot."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
MODEL_ID = "IDEA-Research/grounding-dino-tiny"
MODEL_REVISION = "a2bb814dd30d776dcf7e30523b00659f4f141c71"


def normalize_prompt(prompt):
    prompt = prompt.strip().lower()
    if not prompt or not prompt.strip(". "):
        raise ValueError("Provide an English object description, e.g. black marker pen")
    return prompt if prompt.endswith(".") else prompt + "."


def rs_intrinsics(data):
    import pyrealsense2 as rs

    intr = rs.intrinsics()
    for name in ("width", "height", "fx", "fy", "ppx", "ppy", "coeffs"):
        setattr(intr, name, data[name])
    if not all(math.isfinite(v) for v in (intr.fx, intr.fy, intr.ppx, intr.ppy, *intr.coeffs)) or min(intr.fx, intr.fy) <= 0:
        raise ValueError("Invalid camera intrinsics")
    intr.model = getattr(rs.distortion, data["model"].split(".")[-1])
    return intr


def surface_point(depth_raw, scale, intrinsics, box, radius=4,
                  min_valid_fraction=0.5, max_spread_m=0.05, mask=None):
    """Use an actual median-depth pixel near the box centre, never an invented depth.

    This is a surface-point heuristic within a 2D box, not segmentation, a 3D
    object centroid, or a grasp pose. A locally smooth background can still pass.
    """
    import pyrealsense2 as rs

    if depth_raw.dtype != np.uint16 or depth_raw.ndim != 2:
        raise ValueError("Expected uint16 aligned depth")
    h, w = depth_raw.shape
    intr = rs_intrinsics(intrinsics)
    if (intr.width, intr.height) != (w, h):
        raise ValueError("Aligned depth dimensions do not match intrinsics")
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("Invalid depth scale")
    rejected = {"valid": False, "xyz_m": None, "method": "box_center_patch_surface_pixel"}
    if mask is not None:
        if mask.shape != depth_raw.shape or mask.dtype != np.bool_:
            raise ValueError("Expected boolean mask matching aligned depth dimensions")
        rejected["method"] = "sam_mask_interior_patch_surface_pixel"
        if not mask.any():
            return rejected | {"reason": "empty_mask"}
    if len(box) != 4 or not all(math.isfinite(v) for v in box):
        return rejected | {"reason": "invalid_box"}
    x1, y1, x2, y2 = box
    x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
    if x2 <= x1 or y2 <= y1:
        return rejected | {"reason": "empty_box"}
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    margin = 0
    if mask is not None:
        interior = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8),
                             borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
        if np.count_nonzero(interior) >= 3:
            sampling_mask, margin = interior, 1
        else:
            sampling_mask = mask
        my, mx = np.nonzero(sampling_mask)
        index = np.argmin((mx - mx.mean())**2 + (my - my.mean())**2)
        cx, cy = float(mx[index]), float(my[index])
        x1, y1, x2, y2 = 0, 0, w, h
    # Pixel-centre bounds, clipped to both the detection and the image.
    left, right = max(math.ceil(x1), int(cx) - radius), min(math.ceil(x2), int(cx) + radius + 1)
    top, bottom = max(math.ceil(y1), int(cy) - radius), min(math.ceil(y2), int(cy) + radius + 1)
    patch = depth_raw[top:bottom, left:right]
    valid = patch > 0
    eligible = np.ones(patch.shape, dtype=bool) if mask is None else sampling_mask[top:bottom, left:right]
    valid &= eligible
    fraction = float(np.count_nonzero(valid) / np.count_nonzero(eligible)) if eligible.any() else 0.0
    stats = {"patch_xyxy": [left, top, right, bottom], "valid_fraction": fraction,
             "min_valid_fraction": min_valid_fraction, "max_spread_m": max_spread_m}
    if mask is not None:
        stats.update(mask_pixels=int(mask.sum()), boundary_margin_px=margin,
                     eligible_patch_pixels=int(eligible.sum()))
    if fraction < min_valid_fraction or np.count_nonzero(valid) < 3:
        return rejected | stats | {"reason": "insufficient_depth"}
    ys, xs = np.nonzero(valid)
    zs = patch[valid].astype(np.float64) * scale
    spread = float(np.percentile(zs, 90) - np.percentile(zs, 10))
    stats["depth_p90_minus_p10_m"] = spread
    if spread > max_spread_m:
        return rejected | stats | {"reason": "mixed_or_unstable_depth"}
    # Median depth first; ties use the pixel closest to the box centre.
    order = np.lexsort(((xs + left - cx)**2 + (ys + top - cy)**2, np.abs(zs - np.median(zs))))
    chosen = int(order[0])
    pixel = [int(xs[chosen] + left), int(ys[chosen] + top)]
    z = float(zs[chosen])
    xyz = rs.rs2_deproject_pixel_to_point(intr, pixel, z)
    if not all(math.isfinite(v) for v in xyz):
        return rejected | stats | {"reason": "invalid_projection"}
    return {"valid": True, "reason": None, "method": rejected["method"],
            **stats, "pixel": pixel, "depth_z_m": z, "xyz_m": xyz,
            "range_m": math.sqrt(sum(v*v for v in xyz))}


def annotate(rgb, report, masks=None):
    overlay = rgb.copy()
    palette = [(180, 210, 30), (220, 100, 240), (30, 190, 255), (220, 200, 70)]
    for i, mask in enumerate(masks or []):
        color = palette[i % len(palette)]
        overlay[mask] = (0.55 * overlay[mask] + 0.45 * np.array(color)).astype(np.uint8)
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay, contours, -1, color, 1)
    canvas = cv2.copyMakeBorder(overlay, 76, 0, 0, 0, cv2.BORDER_CONSTANT)
    cv2.putText(canvas, f"CAPTURE: {report['prompt']} | {report['status']}", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    detail = "SAM 2.1 mask | measured surface XYZ (m)" if masks else "Camera XYZ (m), surface estimate | not a grasp pose"
    cv2.putText(canvas, detail, (10, 53),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1, cv2.LINE_AA)
    for detection in report["detections"]:
        x1, y1, x2, y2 = [int(v) for v in detection["box_xyxy"]]
        point = detection["surface_point"]
        color = (0, 220, 0) if point["valid"] else (0, 180, 255)
        cv2.rectangle(canvas, (x1, y1+76), (x2, y2+76), color, 2)
        label = f"#{detection['id']} {detection['label']} {detection['score']:.2f}"
        cv2.putText(canvas, label, (max(0, x1), max(91, y1+71)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
        if point["valid"]:
            x, y = point["pixel"]
            cv2.drawMarker(canvas, (x, y+76), (255, 255, 255), cv2.MARKER_CROSS, 15, 2)
            text = "XYZ " + ", ".join(f"{v:+.3f}" for v in point["xyz_m"])
        else:
            text = "NO XYZ: " + point["reason"]
        cv2.putText(canvas, text, (max(0, x1), min(canvas.shape[0]-8, y2+96)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    return canvas


class GroundingRGBD:
    def __init__(self, device="auto", box_threshold=0.30, text_threshold=0.25, use_sam=False):
        import torch
        import transformers
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

        if not 0 < box_threshold < 1 or not 0 < text_threshold < 1:
            raise ValueError("Detection thresholds must be between 0 and 1")
        self.device = ("cuda" if torch.cuda.is_available() else "cpu") if device == "auto" else device
        self.box_threshold, self.text_threshold = box_threshold, text_threshold
        self.use_sam, self.segmenter = use_sam, None
        self.versions = {"torch": torch.__version__, "transformers": transformers.__version__}
        options = {"revision": MODEL_REVISION, "cache_dir": str(ROOT / "artifacts/models/huggingface")}
        print(f"Loading {MODEL_ID} on {self.device} ...", flush=True)
        self.processor = AutoProcessor.from_pretrained(MODEL_ID, **options)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(
            MODEL_ID, use_safetensors=True, disable_custom_kernels=True, **options
        ).to(self.device).eval()

    def detect(self, rgb, prompt):
        import torch
        from PIL import Image
        from torchvision.ops import nms

        image = Image.fromarray(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
        inputs = self.processor(images=image, text=normalize_prompt(prompt), return_tensors="pt").to(self.device)
        start = time.monotonic()
        with torch.inference_mode():
            outputs = self.model(**inputs)
        result = self.processor.post_process_grounded_object_detection(
            outputs, inputs.input_ids, threshold=self.box_threshold,
            text_threshold=self.text_threshold, target_sizes=[rgb.shape[:2]],
        )[0]
        boxes, scores = result["boxes"].cpu(), result["scores"].cpu()
        labels = result["text_labels"] if "text_labels" in result else result["labels"]
        indices = nms(boxes, scores, 0.5).tolist()
        detections = [{"id": rank, "label": str(labels[i]), "score": float(scores[i]),
                       "box_xyxy": boxes[i].tolist()} for rank, i in enumerate(indices)]
        return detections, time.monotonic() - start

    def analyze_snapshot(self, folder, prompt):
        folder = Path(folder).resolve()
        meta_path = folder / "metadata.json"
        metadata = json.loads(meta_path.read_text(encoding="utf-8"))
        rgb = cv2.imread(str(folder / "color.png"))
        depth = cv2.imread(str(folder / "depth_aligned.png"), cv2.IMREAD_UNCHANGED)
        if rgb is None or depth is None or depth.shape != rgb.shape[:2]:
            raise ValueError("Missing or mismatched RGB/aligned-depth snapshot")
        if metadata.get("coordinate_frame") != "color_camera_optical_frame" or not metadata.get("alignment", "").startswith("depth to color"):
            raise ValueError("Snapshot must contain depth aligned to the color camera")
        # Validate geometry even when the detector produces no boxes.
        surface_point(depth, metadata["depth_scale_m"], metadata["aligned_depth_intrinsics"], [0, 0, 1, 1])
        detections, elapsed = self.detect(rgb, prompt)
        sam_enabled = getattr(self, "use_sam", False)
        segmented, segmentation_seconds = [], 0.0
        segmentation_info = None
        if sam_enabled:
            from tools.lib.sam_rgbd import SAMMasks, MODEL_ID as SAM_ID, MODEL_REVISION as SAM_REVISION
            segmentation_info = {"model_id": SAM_ID, "model_revision": SAM_REVISION,
                                 "min_predicted_iou": 0.5, "mask_encoding": "uint8 PNG: 255 object, 0 background"}
            if detections:
                if self.segmenter is None:
                    self.segmenter = SAMMasks(self.device)
                segmented, segmentation_seconds = self.segmenter.segment(rgb, [d["box_xyxy"] for d in detections])
                if len(segmented) != len(detections):
                    raise ValueError("SAM must return one mask per detection")
        result_dir = folder / ("grounding_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ"))
        result_dir.mkdir()
        masks = []
        for i, detection in enumerate(detections):
            mask = None
            if sam_enabled:
                mask, quality = segmented[i]
                if mask.shape != depth.shape or mask.dtype != np.bool_ or not math.isfinite(quality):
                    raise ValueError("Invalid SAM mask or quality")
                mask_file = f"mask_{detection['id']:03d}.png"
                if not cv2.imwrite(str(result_dir / mask_file), mask.astype(np.uint8) * 255):
                    raise OSError("Could not save segmentation mask")
                detection["segmentation"] = {"mask_file": mask_file, "pixels": int(mask.sum()),
                                             "predicted_iou": float(quality),
                                             "sha256": hashlib.sha256((result_dir / mask_file).read_bytes()).hexdigest()}
                masks.append(mask)
            detection["surface_point"] = surface_point(
                depth, metadata["depth_scale_m"], metadata["aligned_depth_intrinsics"], detection["box_xyxy"], mask=mask
            )
            if sam_enabled and quality < 0.5:
                detection["surface_point"] = {"valid": False, "xyz_m": None, "reason": "low_mask_quality",
                                              "method": "sam_mask_interior_patch_surface_pixel"}
        report = {
            "schema_version": 2 if sam_enabled else 1, "prompt": normalize_prompt(prompt),
            "status": "NO_DETECTIONS" if not detections else
                      "OK" if any(d["surface_point"]["valid"] for d in detections) else "DETECTED_NO_DEPTH",
            "model_id": MODEL_ID, "model_revision": MODEL_REVISION, "device": self.device,
            "versions": self.versions, "box_threshold": self.box_threshold, "text_threshold": self.text_threshold,
            "nms_iou_threshold": 0.5, "inference_seconds": elapsed + segmentation_seconds,
            "detection_seconds": elapsed, "segmentation_seconds": segmentation_seconds,
            "segmentation": segmentation_info,
            "analyzed_at_utc": datetime.now(timezone.utc).isoformat(),
            "source_snapshot": str(folder), "captured_at_utc": metadata["captured_at_utc"],
            "source_sha256": {name: hashlib.sha256((folder / name).read_bytes()).hexdigest()
                              for name in ("color.png", "depth_aligned.png", "metadata.json")},
            "depth_frame": metadata["depth_frame"], "color_frame": metadata["color_frame"],
            "coordinate_frame": "color_camera_optical_frame", "axes": "X right, Y down, Z forward; metres",
            "robot_transform": None, "point_semantics": ("measured surface pixel inside SAM mask; not object centroid or grasp pose" if sam_enabled else "surface pixel near bounding-box centre; not object centroid or grasp pose"),
            "detections": detections,
        }
        image = annotate(rgb, report, masks)
        if not cv2.imwrite(str(result_dir / "detections.jpg"), image):
            raise OSError("Could not save detection preview")
        (result_dir / "detections.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
        print("GROUNDING " + json.dumps({"status": report["status"], "detections": detections,
                                         "output": str(result_dir)}), flush=True)
        return report, image, result_dir
