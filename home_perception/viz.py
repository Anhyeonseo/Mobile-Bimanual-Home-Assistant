"""Write one perception run to a Rerun recording for offline viewing."""
import cv2
import numpy as np


def write_recording(path, entries):
    """entries: (snapshot, [(observation, mask, points)]) in capture order."""
    import rerun as rr
    from scipy.spatial.transform import Rotation
    recording = rr.RecordingStream('home_perception_tabletop')
    recording.save(str(path))
    for index, (snapshot, items) in enumerate(entries):
        recording.set_time('capture', sequence=index)
        recording.log('image/rgb', rr.Image(cv2.cvtColor(snapshot.bgr, cv2.COLOR_BGR2RGB)))
        recording.log('image/depth', rr.DepthImage(snapshot.depth_raw, meter=1 / snapshot.depth_scale_m))
        recording.log('capture', rr.TextLog(snapshot.capture_id))
        for number, (observation, mask, points) in enumerate(items):
            name = f'{number}_{observation.label}'.replace(' ', '_').replace('/', '_')
            caption = f'{observation.label} [{observation.status}]'
            x1, y1, x2, y2 = observation.box_xyxy
            recording.log(f'image/rgb/box/{name}', rr.Boxes2D(mins=[x1, y1], sizes=[x2 - x1, y2 - y1], labels=caption))
            if mask is not None:
                recording.log(f'image/rgb/mask/{name}', rr.SegmentationImage(mask.astype(np.uint8)))
            if points is not None:
                recording.log(f'world/points/{name}', rr.Points3D(points, radii=.001))
            if observation.obb:
                box = observation.obb
                recording.log(f'world/obb/{name}', rr.Boxes3D(
                    centers=[box['center_m']], half_sizes=[np.asarray(box['extent_m']) / 2],
                    quaternions=[rr.Quaternion(xyzw=Rotation.from_matrix(np.asarray(box['rotation']).copy()).as_quat())],
                    labels=caption))
