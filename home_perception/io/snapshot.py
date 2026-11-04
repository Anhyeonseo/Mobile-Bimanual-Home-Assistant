"""Read realsense_viewer.save_snapshot folders. Files only, never devices."""
from dataclasses import dataclass
import json
from pathlib import Path

import cv2
import numpy as np

from tools.lib.run_manifest import sha256

SOURCES = ('color.png', 'depth_aligned.png', 'metadata.json')


@dataclass
class Snapshot:
    capture_id: str
    folder: Path
    bgr: np.ndarray
    depth_raw: np.ndarray
    depth_scale_m: float
    intrinsics: dict
    captured_at_utc: str
    frame_id: str
    metadata: dict
    source_sha256: dict
    objects: list | None = None

    def depth_m(self):
        return self.depth_raw.astype(np.float64) * self.depth_scale_m


def load_snapshot(folder):
    folder = Path(folder).resolve()
    metadata = json.loads((folder / 'metadata.json').read_text(encoding='utf-8'))
    bgr = cv2.imread(str(folder / 'color.png'), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(folder / 'depth_aligned.png'), cv2.IMREAD_UNCHANGED)
    if bgr is None or depth is None or depth.shape != bgr.shape[:2]:
        raise ValueError('Missing or mismatched RGB/aligned-depth snapshot')
    if depth.dtype != np.uint16:
        raise ValueError('Expected uint16 aligned depth')
    if (metadata.get('coordinate_frame') != 'color_camera_optical_frame'
            or not metadata.get('alignment', '').startswith('depth to color')):
        raise ValueError('Snapshot must contain depth aligned to the color camera')
    intrinsics = metadata['aligned_depth_intrinsics']
    if (intrinsics['width'], intrinsics['height']) != depth.shape[::-1]:
        raise ValueError('Aligned depth dimensions do not match intrinsics')
    if min(intrinsics['fx'], intrinsics['fy']) <= 0 or any(intrinsics['coeffs']):
        raise ValueError('Expected rectified aligned intrinsics')
    scale = metadata['depth_scale_m']
    if not 0 < scale <= .1:
        raise ValueError('Invalid depth scale in meters')
    record = folder / 'capture_record.json'
    objects = json.loads(record.read_text(encoding='utf-8'))['objects'] if record.exists() else None
    return Snapshot(folder.name, folder, bgr, depth, scale, intrinsics, metadata['captured_at_utc'],
                    metadata['coordinate_frame'], metadata,
                    {name: sha256(folder / name) for name in SOURCES}, objects)


def snapshot_folders(root):
    return sorted(path.parent for path in Path(root).glob('*/metadata.json'))
