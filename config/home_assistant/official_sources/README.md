# Pinned upstream geometry

These are **unmodified reference files**, retained to make geometry reproducible
without fetching a moving upstream branch. Source URL, commit, path and SHA256 are
in [`../official_geometry.json`](../official_geometry.json). That manifest also
contains exact joint records extracted from the two URDF files; the S2 loader
checks them against the source bytes.

| Local reference | Upstream project | Intended use |
|---|---|---|
| `alohamini1.urdf` | liyiteng/AlohaMini, **AlohaMini1/** subtree | First-generation CAD reference frames; zero exported joint limits are not operational limits |
| `so101_new_calib.urdf` | TheRobotStudio/SO-ARM100, **Simulation/SO101/** | SO-ARM101 follower link transforms and joint axes, new calibration zero convention |
| `alohamini_config.py.txt` | liyiteng/lerobot_alohamini | Document Mini 1 profile: SO arm, STS3215 lift, 84 mm/rev. **Not executable configuration:** upstream class default selects Mini 2 |

The upstream projects distribute these under Apache-2.0; the license text is
retained in `LICENSE`. Attribution belongs to the corresponding upstream authors.
No mesh binaries are included in this reference bundle. The source URDF mesh
paths refer to upstream assets; use the generated S2 **kinematic-only** URDF for
coordinate publication, not these source files as a complete installed model.

The Mini 1 CAD arm-root frame and the standalone SO101 `base_link` are different
datums. The software does not connect them by name or transplant the old
repository's Home offsets. Installed arm roots are fitted from independent TCP
observations. `gripper_frame_link` is a fixed tool reference, not an opening-
dependent contact center. Installed operating limits remain separate.
