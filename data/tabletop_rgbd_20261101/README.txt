RGB-D TABLETOP DATASET — 2026-11-01

10 real captures from a RealSense D415 connected to Jetson.
Captured through realsense_web.py using realsense_viewer.save_snapshot.
Each numbered folder preserves all six snapshot files without modification.
No robot was moved. No robot-coordinate transform is provided.

Scenes:
01–02: cup, remote control, calculator in different arrangements
03–04: remote control, marker pen, utility knife in different arrangements
05–06: remote control only
07–08: two black cups (08 includes partial occlusion)
09: cup, remote control, marker pen, utility knife
10: cup, remote control, calculator, marker pen, utility knife;
    marker pen slightly overlaps the calculator edge

Per-capture files:
color.png: original color image, 640x480. OpenCV loads it as BGR;
           convert BGR to RGB if the receiving model expects RGB.
depth_aligned.png: unfiltered depth aligned to color, uint16 PNG, 640x480.
depth_native.png: native sensor depth, uint16 PNG, 640x360.
depth_preview_filtered.png: filtered depth for display only.
preview.jpg: RGB and colored depth visualization, not metric depth.
metadata.json: original camera intrinsics, distortion, depth-to-color
               extrinsics, depth scale, timestamps and frame information.
capture_record.json: scene label and source folder added for this delivery.

Point-cloud input:
Use color.png + depth_aligned.png + metadata.aligned_depth_intrinsics.
Read depth with cv2.IMREAD_UNCHANGED to preserve uint16.
Z in metres = raw pixel value * metadata.depth_scale_m.
A raw value of zero is invalid; exclude it, do not treat it as a surface.
These captures have zero distortion coefficients in aligned intrinsics:
X = (u - ppx) * Z / fx; Y = (v - ppy) * Z / fy.
Coordinates: camera optical frame, X right, Y down, Z forward; metres.
Do not pair native depth pixels directly with color pixels.
Use each capture's metadata rather than assumed calibration constants.

Known limitations:
Black cups and other dark objects contain missing depth pixels, visible
in the previews. Original measurements are preserved without filling holes.
Some captures contain background furniture outside the tabletop.
Scene labels are manual descriptions, not segmentation annotations.
Qwen and GraspNet were not run as part of this delivery; model-specific
input preprocessing and inference still need to be performed separately.

Verification:
All 10 RGB images and capture timestamps are distinct. All images decode;
raw depths retain uint16; aligned dimensions match their intrinsics.
Projection of valid aligned pixels produces finite camera-frame XYZ.
ZIP integrity and each archived file's SHA-256 were checked against source.
manifest.json lists scene categories and valid depth counts.
SHA256SUMS.txt lists checksums for all other files in this dataset folder.
