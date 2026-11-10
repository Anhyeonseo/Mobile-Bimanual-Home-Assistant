"""Store one validated matched exposure for offline consumers; no device I/O."""
import hashlib
import json

FILES = ('color.png', 'depth_aligned.png', 'metadata.json')


def hashes(folder):
    return {name: hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in FILES}


def save_frame(root, images, row):
    """Called after the acquisition loop. Store no image payload in the serial report."""
    from PIL import Image
    color, depth, metadata = images
    camera = row['camera']
    if (metadata['color_frame'] != camera['color_frame'] or metadata['depth_frame'] != camera['depth_frame']
            or camera['capture_id'] != row['coordinates']['capture_id']):
        raise ValueError('RGB-D exposure and coordinates differ')
    folder = root/camera['capture_id']
    folder.mkdir(parents=True, exist_ok=False)
    Image.fromarray(color[:, :, ::-1]).save(folder/FILES[0])
    Image.fromarray(depth).save(folder/FILES[1])
    (folder/FILES[2]).write_text(json.dumps(metadata, indent=2, allow_nan=False))
    return {'capture_id': folder.name, 'sequence': row['sequence'], 'source_sha256': hashes(folder)}
