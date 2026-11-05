# S2 robot shape viewer

`viewer.js` evaluates the same pinned SO101 joint chain as S2 and draws labels and
coordinate axes. `mesh-renderer.js` renders actual part surfaces using embedded
WebGL buffers; it has no network or robot connection. `meshes.json` is a reduced,
quantized display-only derivative of upstream binary STLs, not a collision model.
It includes the source SHA256 and Git blob ID for each asset.

Upstream sources, both Apache-2.0:

- TheRobotStudio/SO-ARM100 at `a758567c3978dfeefe282ede0500085a48fe8f78`,
  `Simulation/SO101/assets/`. Every used local arm STL matched the blob ID in that
  pinned official Git tree. Each visual's origin is copied from the official
  `so101_new_calib.urdf`, separate from its joint origin.
- liyiteng/AlohaMini at `17c6a98d79881a45ab869c1f392ed89c0723a298`,
  `AlohaMini1/simulation/src/Aloha/meshes/`: `base_link.STL`, `vertical_link.STL`,
  `wheel1.STL`. These downloaded files also matched that tree's blob IDs.
  The shared wheel1 shape is repeated at the three official wheel joint positions
  and orientations, including the 42.2 mm axle Z. These are nominal CAD placements.

Upstream license text and source URDFs are retained under
`config/home_assistant/official_sources/`. Attribution belongs to those projects.

Mesh reduction averages vertices within a small spatial grid, removes collapsed
and duplicate triangles, and quantizes the remaining coordinates to unsigned
16-bit values within each mesh's bounds. The compact bundle is embedded in the
HTML so no STL loader or external resource request is needed. The D415 housing now uses the official RealSense peripheral STL; its sensor
frames use the pinned official Xacro nominal chain, not device-calibrated extrinsics.

The Mini 1 body uses a display reference rotation (CAD X -> viewer Y, CAD Y ->
viewer -X). The no-profile carriage now follows the official `vertical_move` origin and axis,
starting at the calculated lower contact. It is independent of illustrative arm
roots. Mini1 arm mounting locations come from its fixed joints; the SO101 adapter
orientation remains illustrative. Profile-backed CAD body placement remains a
reference view, never a replacement for commissioned transforms. Official arm meshes
follow the exact FK and visual origins. Camera/arm positions still come from the
explicit illustration or validated S2 profile. Moving a slider never moves hardware.

Wheel placement correction (2026-11-05): the earlier illustrative 46 mm clearance
was wrong and is removed. Wheel centers retain the official joint Z of 42.2 mm;
the full source wheel extends down to -49.49552 mm locally. Its assembled contact
plane is therefore -7.29552 mm in the CAD base frame. The ground is placed there;
the robot's joints are not moved. Wheel display bounds preserve the original
98.99105 mm diameter so mesh reduction cannot change the apparent contact height.
This is a **nominal CAD ground plane, not a measured installed base transform**.
Neither arm/camera FK nor a commissioned base origin is translated. The default camera is on the front
antenna's front-facing sloped face (CAD negative Y, display positive X), with
the housing rear against that face and its rear top edge aligned to the face's
upper edge. The face normal supplies the illustrative 45-degree pitch and CAD
yaw; camera housing dimensions and optical frame placement follow the official D415
model, with nominal extrinsics explicitly distinguished from device calibration. There is no
bracket above the tip. A supplied validated profile always overrides that camera
pose. The planned mounting site is recorded in
`installation.development.json`; actual camera extrinsics remain uncalibrated.

To rebuild after supplying the pinned Mini 1 mesh directory:

```bash
python3 tools/diagnostics/s2_viewer/build_meshes.py \
  --mini1-meshes <official-mini1-mesh-directory> \
  --output tools/diagnostics/s2_viewer/meshes.json
```

Then generate a fresh visual with `tools/diagnostics/s2_visualize.py`. Existing
outputs are not overwritten. Use the returned HTML fragment in the inline viewer.

`tools/diagnostics/s2_lift_contacts.py` computes first contact with two named
stop surfaces from the original source triangles and the official slide axis:
the complete connected base deck and antenna foot solids, including their full
thickness and sidewalls. The earlier Z=880..920 mm crop omitted the thick foot
underside and incorrectly allowed 22.75 mm extra ascent; that result is superseded.
The corrected upper contact is the vertical slider upper end at CAD Z=865.093 mm.
The lower contact remains at the deck upper face, CAD Z=64 mm.
It intersects projected triangle footprints and optimizes the affine height gap;
this preserves the small slide-axis tilt and avoids whole-mesh bounding-box limits.
Each result records the mating triangles, contact point, source hashes, and scope.
The reference result is stored in `lift-contacts.json`. It is not a global collision
check: gears, rail interfaces, wiring, load deflection and actual hardware tolerances
are outside that selected-surface calculation. No motion limits are changed.

`tools/diagnostics/s2_lift_visualize.py` consumes that report to show the two contact
poses, with the carriage yellow and the contact point/triangles pink. Historical
26/90 cm heights are no longer used as positioning inputs. Nominal results are
23.49/87.87 cm (left SO-ARM plate) and 23.56/87.94 cm (right plate), with
64.38 cm travel. Installed validation is pending and measured records are preserved.
The user clarified the datum as the upper plate where SO-ARM attaches, superseding
the central T-connector face (22.85/87.24 cm). Source triangles 19718/17518 seed
coplanar mounting surface fits, evaluated at the corresponding official arm mount
XY; they agree with official mount Z within 3 micrometres. The report preserves
both datums and both endpoint heights. Legacy single-height fields use the left
plate explicitly; the renderer points to that same surface. CAD left/right height
difference is approximately 0.70 mm. Residuals versus historical 26/90 cm remain
about -24 to -25 mm at the bottom and -21 mm at the top. Travel differs by +3.85 mm.
The official assembly guide steps 13–19 show the pinion and motor inside the
front carriage, engaging the main-post rack. The old arm-centered display
translation was wrong and is removed for the nominal model. No profile is changed.

`build_lift_inspection_meshes.py` builds `lift-inspection-meshes.json` for this
comparison. It retains every original triangle of both stop solids and the entire
carriage, with only 16-bit coordinate quantization (under 0.01 mm per axis); other
fixed body parts remain reduced. Collision calculations always use original STL
coordinates, not either display bundle. Rebuild using `--mesh-dir` and `--output`.

Nominal base reference update (2026-11-05): profile-free viewers now apply
`s2_commissioning.nominal_base_from_cad()` to all body geometry, ground, example
arm mounts and the camera example together. The origin is the mean of the
three wheel joint origins at axle height. Floor clearance is therefore 49.49552 mm
below this new origin; the physical wheel/body relationship is unchanged.
Accepted profiles retain their own base definition, without silent migration.


D415 correction (2026-11-05): replaced the earlier 100×26×23 mm schematic box
and three invented lens locations with `realsenseai/realsense-ros` official
`realsense2_description/meshes/d415.stl` and `_d415.urdf.xacro`, pinned at
`7a87097c499e3b0b69518d54a8e7a9ac1924e87c`. Source hashes are in
`config/home_assistant/d415_official_geometry.json`; Xacro and Apache-2.0 license
are retained in `official_sources/`. The 21 MB original STL is a local ignored
input; `d415-mesh.json` is the reduced indexed display derivative, preserving
source bounds. The original model specifies 99×23×20.05 mm (width×height×depth).

`home_robot_tasks.d415_geometry` evaluates only numeric property expressions
and the official fixed joint chain. It separates the bottom screw frame,
color optical frame, depth optical frame, and mesh frame. The rear top center
of the case is aligned to the antenna face's upper edge. In the nominal model,
that point is `[35, -11.5, -10.18] mm` in the RGB optical frame, so the RGB
origin is no longer treated as the housing center. The source itself labels
inter-sensor offsets approximate; deployed code must use SDK/ROS device-calibrated
extrinsics. No measured profile, physical calibration or motion setting is changed.

Confirmed underside clearance (2026-11-05): the operator measured approximately
28 mm from the floor to the chassis bottom-plate UNDERSIDE and confirmed this
exact datum. `installation.development.json` records the measurement separately
from nominal wheel geometry. Profile-free viewers use that floor reference;
passing `installation={}` to `bundle()` retains a pure nominal CAD view.
The wheel and motor poses, camera pose, lift contacts and travel are unchanged.
The renderer identifies wheels as nominal CAD reference geometry, which remains
20.70 mm above the measured floor until installed wheel mounting is verified.
Accepted coordinate profiles are never changed by this installation record.
`lift-floor-reference.json` is derived using `endpoint_bundle()['floor_reference']`;
left heights are 25.56/89.94 cm and right 25.63/90.01 cm. These combine CAD contacts
with an approximate measured floor reference, not precise measured endpoints.
