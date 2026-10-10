#!/usr/bin/env python3
"""Live aligned D415 RGB-D inspection in camera coordinates, without ROS."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

WINDOW = "D415 | click: measure | S: save | Q/ESC: quit"
HEADER = 134
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.diagnostics.realsense_depth_preview import DepthPreviewFilter


def intrinsics_dict(intrinsics):
    return {
        key: getattr(intrinsics, key)
        for key in ("width", "height", "fx", "fy", "ppx", "ppy", "coeffs")
    } | {"model": str(intrinsics.model)}


def frame_info(frame):
    return {
        "number": frame.get_frame_number(),
        "timestamp_ms": frame.get_timestamp(),
        "timestamp_domain": str(frame.get_frame_timestamp_domain()),
    }


def measure(depth, pixel):
    x, y = pixel
    z = depth.get_distance(x, y)
    result = {"pixel": [x, y], "valid": bool(math.isfinite(z) and z > 0)}
    if result["valid"]:
        intrinsics = depth.profile.as_video_stream_profile().get_intrinsics()
        xyz = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], z)
        result.update(depth_z_m=z, xyz_m=xyz, range_m=math.sqrt(sum(v*v for v in xyz)))
    return result


def draw(rgb, depth_raw, scale, pixel, measurement, fps, max_depth, grounding_status=None,
         min_depth=0.0, depth_view="RAW"):
    height, width = depth_raw.shape
    normalized = np.clip((depth_raw.astype(np.float32) * scale - min_depth) / (max_depth - min_depth), 0, 1)
    colored = cv2.applyColorMap((normalized * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    colored[depth_raw == 0] = 0
    canvas = np.zeros((height + HEADER, 2 * width, 3), dtype=np.uint8)
    canvas[HEADER:, :width] = rgb
    canvas[HEADER:, width:] = colored
    valid = np.count_nonzero(depth_raw) / depth_raw.size * 100
    lines = [
        f"RGB | {depth_view} depth, {min_depth:g}-{max_depth:g}m | {fps:.1f} fps | valid {valid:.1f}% | F: raw/smooth",
        f"Pixel {pixel}: NO DEPTH" if not measurement["valid"] else
        f"Pixel {pixel}: Z={measurement['depth_z_m']:.3f} m | range={measurement['range_m']:.3f} m | "
        f"XYZ=({measurement['xyz_m'][0]:+.3f}, {measurement['xyz_m'][1]:+.3f}, {measurement['xyz_m'][2]:+.3f}) m",
        "XYZ from RAW depth: X right, Y down, Z forward | click image | S save | Q/ESC quit",
    ]
    if grounding_status:
        lines.append(grounding_status)
    for i, line in enumerate(lines):
        cv2.putText(canvas, line, (10, 24 + i * 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.49, (230, 230, 230), 1, cv2.LINE_AA)
    for offset in (0, width):
        center = (pixel[0] + offset, pixel[1] + HEADER)
        cv2.drawMarker(canvas, center, (0, 0, 0), cv2.MARKER_CROSS, 19, 3)
        cv2.drawMarker(canvas, center, (255, 255, 255), cv2.MARKER_CROSS, 17, 1)
    return canvas


def save_snapshot(output, rgb, aligned_raw, native_raw, preview, metadata, filtered_raw=None):
    folder = output / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    folder.mkdir(parents=True, exist_ok=False)
    for name, array in (("color.png", rgb), ("depth_aligned.png", aligned_raw),
                        ("depth_native.png", native_raw), ("preview.jpg", preview)):
        if not cv2.imwrite(str(folder / name), array):
            raise RuntimeError(f"Could not write {folder / name}")
    if filtered_raw is not None and not cv2.imwrite(str(folder / "depth_preview_filtered.png"), filtered_raw):
        raise RuntimeError(f"Could not write filtered preview depth in {folder}")
    (folder / "metadata.json").write_text(
        json.dumps(metadata, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    print(f"SAVED {folder}", flush=True)
    return folder


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", help="D415 serial; required if multiple D415s are connected")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--depth-width", type=int, default=1280, help="Native depth width; D415 quality mode defaults to 1280")
    parser.add_argument("--depth-height", type=int, default=720)
    parser.add_argument("--depth-filter", choices=("off", "spatial", "temporal"), default="spatial",
                        help="Display smoothing; temporal is intended for static scenes")
    parser.add_argument("--laser-power", type=float, help="Optional supported IR power; restored on exit")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--max-depth", type=float, default=3.0, help="Visualization limit in metres")
    parser.add_argument("--min-depth", type=float, default=0.0, help="Visualization minimum in metres")
    parser.add_argument("--headless", action="store_true", help="Bounded capture without a window")
    parser.add_argument("--frames", type=int, default=0, help="Stop after this many frames; 0 = until closed")
    parser.add_argument("--pixel", nargs=2, type=int, metavar=("X", "Y"), help="Initial measurement pixel")
    parser.add_argument("--save-on-exit", action="store_true")
    parser.add_argument("--output", type=Path, default=ROOT / "tmp/realsense")
    parser.add_argument("--prompt", help="Enable D-key detection, e.g. 'black marker pen'")
    parser.add_argument("--interactive-prompt", action="store_true",
                        help="Read object descriptions from the terminal; capture and detect per line, reuse the model")
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--box-threshold", type=float, default=0.30)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument("--sam", action="store_true", help="Segment detections and use only mask depth for XYZ")
    args = parser.parse_args(argv)
    if args.interactive_prompt:
        args.headless = True
        if args.frames == 0:
            args.frames = 30
    if min(args.width, args.height, args.depth_width, args.depth_height, args.fps) <= 0 or args.frames < 0:
        parser.error("Dimensions/fps must be positive and frames must be nonnegative")
    if not all(math.isfinite(v) for v in (args.min_depth, args.max_depth)) or not 0 <= args.min_depth < args.max_depth:
        parser.error("Depth color range must satisfy 0 <= min < max")
    if args.laser_power is not None and (not math.isfinite(args.laser_power) or args.laser_power < 0):
        parser.error("--laser-power must be finite and nonnegative")
    if args.headless and args.frames == 0:
        parser.error("--headless requires --frames > 0")
    args.pixel = tuple(args.pixel or (args.width // 2, args.height // 2))
    if not (0 <= args.pixel[0] < args.width and 0 <= args.pixel[1] < args.height):
        parser.error("--pixel must be inside the image")
    if not args.headless and sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        parser.error("No DISPLAY available; use --headless --frames 90 --save-on-exit")
    if args.prompt is not None and not args.prompt.strip(". "):
        parser.error("--prompt must name an object in English")
    if not 0 < args.box_threshold < 1 or not 0 < args.text_threshold < 1:
        parser.error("Detection thresholds must be between 0 and 1")
    return args


def run(args, detector=None, frame_callback=None, stop_event=None):
    if args.prompt and detector is None:
        sys.path.insert(0, str(ROOT))
        from tools.lib.grounding_rgbd import GroundingRGBD
        detector = GroundingRGBD(args.device, args.box_threshold, args.text_threshold, use_sam=args.sam)
    context = rs.context()
    devices = [d for d in context.query_devices() if "D415" in d.get_info(rs.camera_info.name)]
    if args.serial:
        devices = [d for d in devices if d.get_info(rs.camera_info.serial_number) == args.serial]
    if len(devices) != 1:
        raise RuntimeError(f"Expected one D415, found {len(devices)}. Check USB, permissions, or --serial.")
    device = devices[0]
    device_info = {
        name: device.get_info(key) for name, key in (
            ("name", rs.camera_info.name), ("serial", rs.camera_info.serial_number),
            ("firmware", rs.camera_info.firmware_version), ("usb", rs.camera_info.usb_type_descriptor),
        ) if device.supports(key)
    }
    config = rs.config()
    config.enable_device(device_info["serial"])
    config.enable_stream(rs.stream.depth, args.depth_width, args.depth_height, rs.format.z16, args.fps)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    pipeline = rs.pipeline(context)
    started = False
    window_open = False
    selected = [args.pixel]
    count = 0
    previous = None
    gaps = {"depth": 0, "color": 0}
    repeats = {"depth": 0, "color": 0}
    last_snapshot = None
    worker = ThreadPoolExecutor(max_workers=1) if detector and not args.headless else None
    pending = None
    sensor = None
    original_laser = None
    preview_filter = DepthPreviewFilter(args.depth_filter)
    show_filtered = args.depth_filter != "off"
    grounding_status = f"D: capture + detect '{args.prompt}'" if detector else None
    grounding_result = None

    def on_mouse(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN and HEADER <= y < HEADER + args.height and 0 <= x < 2 * args.width:
            selected[0] = (x % args.width, y - HEADER)

    try:
        profile = pipeline.start(config)
        started = True
        sensor = profile.get_device().first_depth_sensor()
        if args.laser_power is not None:
            power_range = sensor.get_option_range(rs.option.laser_power)
            if not power_range.min <= args.laser_power <= power_range.max:
                raise ValueError(f"IR power must be between {power_range.min} and {power_range.max}")
            original_laser = sensor.get_option(rs.option.laser_power)
            sensor.set_option(rs.option.laser_power, args.laser_power)
        scale = sensor.get_depth_scale()
        native_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        extrinsics = profile.get_stream(rs.stream.depth).get_extrinsics_to(color_profile)
        calibration = {
            "depth_scale_m": scale,
            "native_depth_intrinsics": intrinsics_dict(native_intrinsics),
            "color_intrinsics": intrinsics_dict(color_profile.get_intrinsics()),
            "depth_to_color": {"rotation_column_major": extrinsics.rotation, "translation_m": extrinsics.translation},
            "coordinate_frame": "color_camera_optical_frame",
            "axes": "X right, Y down, Z forward; metres",
            "robot_transform": None,
            "depth_capture": {"width": args.depth_width, "height": args.depth_height, "fps": args.fps},
            "depth_sensor_options": {name: sensor.get_option(getattr(rs.option, name)) for name in
                                     ("visual_preset", "emitter_enabled", "laser_power", "enable_auto_exposure")},
            "depth_preview_filter": preview_filter.describe(),
            "depth_color_range_m": [args.min_depth, args.max_depth],
        }
        align = rs.align(rs.stream.color)
        for _ in range(30):
            pipeline.wait_for_frames(5000)
        if not args.headless:
            # Fixed image size makes mouse coordinates identical to image pixels.
            cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
            window_open = True
            cv2.setMouseCallback(WINDOW, on_mouse)
        print("READY " + json.dumps(device_info), flush=True)
        start = time.monotonic()
        rate_start = start
        rate_count = 0
        fps = 0.0
        while True:
            if stop_event is not None and stop_event.is_set():
                break
            if pending is not None and pending.done():
                try:
                    report, result_image, result_dir = pending.result()
                    cv2.imshow("Grounding DINO | captured frame result", result_image)
                    grounding_status = f"{report['status']}: {len(report['detections'])} detection(s) | D: new capture"
                except Exception as error:
                    print(f"GROUNDING ERROR: {error}", file=sys.stderr, flush=True)
                    grounding_status = "Detection failed; see terminal | D: retry"
                pending = None
            frames = pipeline.wait_for_frames(5000)
            native_depth = frames.get_depth_frame()
            aligned = align.process(frames)
            depth, color = aligned.get_depth_frame(), aligned.get_color_frame()
            if not depth or not color or not native_depth:
                raise RuntimeError("Incomplete RGB-D frameset")
            count += 1
            now = time.monotonic()
            rate_count += 1
            if now - rate_start >= 0.5:
                fps = rate_count / (now - rate_start)
                rate_count, rate_start = 0, now
            current = {"depth": native_depth.get_frame_number(), "color": color.get_frame_number()}
            if previous is not None:
                for name in current:
                    gaps[name] += max(0, current[name] - previous[name] - 1)
                    repeats[name] += int(current[name] == previous[name])
            previous = current
            rgb = np.asanyarray(color.get_data())
            raw = np.asanyarray(depth.get_data())
            native_raw = np.asanyarray(native_depth.get_data())
            filtered_frame = preview_filter.process(frames) if args.depth_filter != "off" else depth
            filtered_raw = np.asanyarray(filtered_frame.get_data()).copy()
            # Alignment can change projected footprint by a pixel: never invent
            # displayed depth at a pixel without an original valid measurement.
            filtered_raw[raw == 0] = 0
            sample = measure(depth, selected[0])
            preview = draw(rgb, filtered_raw if show_filtered else raw, scale, selected[0], sample, fps,
                           args.max_depth, grounding_status, args.min_depth,
                           "SMOOTH" if show_filtered else "RAW")
            metadata = {
                "schema_version": 1,
                "captured_at_utc": datetime.now(timezone.utc).isoformat(),
                "device": device_info, **calibration,
                "aligned_depth_intrinsics": intrinsics_dict(depth.profile.as_video_stream_profile().get_intrinsics()),
                "depth_frame": frame_info(native_depth), "color_frame": frame_info(color),
                "depth_storage": "uint16 PNG; multiply by depth_scale_m for metres; zero is invalid",
                "color_storage": "PNG encoded by OpenCV from BGR8",
                "alignment": "depth to color via rs.align; no hole filling or temporal filtering",
                "measurement_depth_file": "depth_aligned.png",
                "preview_depth_file": "depth_preview_filtered.png",
                "preview_showing_filtered": show_filtered,
                "sample": sample, "valid_depth_percent": float(np.count_nonzero(raw) / raw.size * 100),
                "framesets": count, "observed_fps": count / (now - start),
                "frame_number_gaps": dict(gaps), "repeated_frames": dict(repeats),
            }
            if frame_callback is not None:
                frame_callback(rgb, raw, native_raw, preview, metadata, filtered_raw)
            key = -1
            closed = False
            if not args.headless:
                cv2.imshow(WINDOW, preview)
                key = cv2.waitKey(1) & 0xFF
                closed = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1
            if key in (ord("s"), ord("S")):
                last_snapshot = save_snapshot(args.output, rgb, raw, native_raw, preview, metadata, filtered_raw)
            if key in (ord("f"), ord("F")):
                show_filtered = not show_filtered if args.depth_filter != "off" else False
            if key in (ord("d"), ord("D")) and worker is not None and pending is None:
                last_snapshot = save_snapshot(args.output, rgb, raw, native_raw, preview, metadata, filtered_raw)
                pending = worker.submit(detector.analyze_snapshot, last_snapshot, args.prompt)
                grounding_status = "Analyzing captured RGB-D ... (live view continues)"
            if closed or key in (27, ord("q"), ord("Q")) or (args.frames and count >= args.frames):
                if args.save_on_exit or (args.headless and detector):
                    last_snapshot = save_snapshot(args.output, rgb, raw, native_raw, preview, metadata, filtered_raw)
                break
        if count == 0:
            return None
        print("RESULT " + json.dumps({
            "framesets": count, "observed_fps": metadata["observed_fps"],
            "frame_number_gaps": gaps, "repeated_frames": repeats,
            "valid_depth_percent": metadata["valid_depth_percent"], "sample": sample,
            "snapshot": str(last_snapshot) if last_snapshot else None,
        }), flush=True)
        if args.headless and detector:
            pipeline.stop()
            started = False
            grounding_result = detector.analyze_snapshot(last_snapshot, args.prompt)
    finally:
        if original_laser is not None:
            try:
                sensor.set_option(rs.option.laser_power, original_laser)
            except RuntimeError as error:
                print(f"Could not restore IR power: {error}", file=sys.stderr)
        if started:
            pipeline.stop()
        if window_open:
            cv2.destroyAllWindows()
        if worker is not None:
            worker.shutdown(wait=True)
    return grounding_result


def interactive_search(args, input_fn=input, detector_factory=None, capture_fn=run):
    """One fresh RGB-D capture per text query; keep model weights loaded between queries."""
    if detector_factory is None:
        from tools.lib.grounding_rgbd import GroundingRGBD
        detector_factory = lambda *options: GroundingRGBD(*options, use_sam=args.sam)
    from tools.lib.grounding_rgbd import normalize_prompt

    detector = None
    first_prompt = args.prompt
    print("Type an English object description (e.g. black marker). Enter q to quit.", flush=True)
    while True:
        try:
            prompt = first_prompt if first_prompt is not None else input_fn("Find object> ")
        except EOFError:
            break
        first_prompt = None
        if prompt.strip().lower() in ("q", "quit", "exit"):
            break
        try:
            args.prompt = normalize_prompt(prompt)
        except ValueError as error:
            print(error, flush=True)
            continue
        if detector is None:
            detector = detector_factory(args.device, args.box_threshold, args.text_threshold)
        try:
            report, _, result_dir = capture_fn(args, detector=detector)
        except (RuntimeError, ValueError, OSError, cv2.error) as error:
            print(f"Search failed: {error}", file=sys.stderr, flush=True)
            continue
        print(f"{report['status']} | {report['inference_seconds']:.2f}s inference | {result_dir}", flush=True)
        for detection in report["detections"]:
            point = detection["surface_point"]
            position = ("camera XYZ (m): " + ", ".join(f"{v:+.3f}" for v in point["xyz_m"])) if point["valid"] else ("NO XYZ: " + point["reason"])
            print(f"  #{detection['id']} {detection['label']} ({detection['score']:.2f}) | {position}", flush=True)


def main():
    args = parse_args()
    try:
        if args.interactive_prompt:
            interactive_search(args)
        else:
            run(args)
    except KeyboardInterrupt:
        return 130
    except (RuntimeError, ValueError, OSError, cv2.error) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
