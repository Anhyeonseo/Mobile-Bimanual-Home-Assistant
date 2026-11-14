"""Camera-only RGB-D shot for calibration (floor, table, checkerboard). No controller access.

Run on the Jetson from inside the installed release folder:
    python cam_shot.py <new output folder>
"""
import sys, json, time
from pathlib import Path
ROOT = Path.cwd()
sys.path[:0] = [str(ROOT), str(ROOT/'ros2_ws/src/home_robot_tasks'), str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from PIL import Image
from home_robot_tasks.s2_rgbd_acquisition import Capture
from home_robot_tasks.camera_evidence import near_depth_profile
CONFIG = ROOT/'config/home_assistant'
read = lambda name: json.loads((CONFIG/name).read_text())
out = Path(sys.argv[1])
if out.exists():
    sys.exit('already exists: ' + str(out))
profile = near_depth_profile(read('s2_coordinates.development.json'), read('camera_device.observed.json'), read('camera_device.near.observed.json'))
with Capture(profile, read('camera_device.near.observed.json'), retain_frame=True) as camera:
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        record, images = camera.next_frame()
out.mkdir(parents=True)
Image.fromarray(images[0][:, :, ::-1]).save(out/'color.png')
Image.fromarray(images[1]).save(out/'depth_aligned.png')
(out/'metadata.json').write_text(json.dumps(images[2], indent=2))
(out/'record.json').write_text(json.dumps(record, default=str))
print('saved', out, images[0].shape)
