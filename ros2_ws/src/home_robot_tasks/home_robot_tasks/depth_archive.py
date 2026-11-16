"""Content-addressed scan volume chunks in the existing WorldStore transaction.

Archived depth is geometry, never fresh clearance. Loading requires the original
map/calibration identity; a live DepthVolume.reset clears it before integration.
"""
from dataclasses import asdict
import hashlib
import math

from .assistant.contracts import document
from .fetch import InvalidTask
from .navigation import identifier, number
from .navigation_depth import DepthProfile, DepthVolume

CHUNK_CELLS = 4096


def digest(value):
    return hashlib.sha256(document(value).encode()).hexdigest()


def encode(volume, *, clock_id):
    identifier(clock_id, 'depth clock')
    identifier(volume.epoch, 'depth epoch')
    if volume.last_stamp is None or volume.error or getattr(volume, 'historical', False):
        raise InvalidTask('valid current volume required for archive')
    stamp = number(volume.last_stamp, 'depth stamp')
    if not 0 <= stamp or len(volume.cells) > volume.p.maximum_voxels:
        raise InvalidTask('invalid depth archive size or time')
    cells = []
    low = math.floor(volume.p.minimum_height_m / volume.p.resolution_m)
    high = math.ceil(volume.p.maximum_height_m / volume.p.resolution_m)
    for key, (occupied, observed) in sorted(volume.cells.items()):
        if (len(key) != 3 or any(type(v) is not int or abs(v) > 1000000 for v in key)
                or not low <= key[2] < high or type(occupied) is not bool
                or not 0 <= number(observed, 'voxel time') <= stamp):
            raise InvalidTask('invalid archived voxel')
        cells.append([*key, occupied, observed])
    chunks = [cells[i:i+CHUNK_CELLS] for i in range(0, len(cells), CHUNK_CELLS)]
    records = {digest(chunk): chunk for chunk in chunks}
    manifest = dict(schema_version=1, profile=asdict(volume.p), epoch=volume.epoch,
                    clock_id=clock_id, last_stamp=stamp, cell_count=len(cells),
                    chunks=[digest(c) for c in chunks], historical_only=True)
    return manifest, records


def persist(db, records):
    """Caller owns the WorldStore lock and transaction, including scan checkpoint."""
    for key, value in records.items():
        old=db.execute('SELECT body FROM assistant_records WHERE kind=? AND id=?',('depth_chunk',key)).fetchone()
        if old and old[0]!=document(value):raise InvalidTask('existing depth chunk is corrupt')
        db.execute('INSERT OR IGNORE INTO assistant_records VALUES (?,?,?)',
                   ('depth_chunk', key, document(value)))


def restore(store, manifest, *, expected_epoch, calibration_id):
    fields = {'schema_version', 'profile', 'epoch', 'clock_id', 'last_stamp',
              'cell_count', 'chunks', 'historical_only'}
    if (not isinstance(manifest, dict) or set(manifest) != fields
            or type(manifest['schema_version']) is not int or manifest['schema_version'] != 1
            or manifest['historical_only'] is not True or manifest['epoch'] != expected_epoch):
        raise InvalidTask('depth archive identity mismatch')
    identifier(manifest['clock_id'], 'archive clock')
    identifier(expected_epoch, 'archive epoch')
    try:
        profile = DepthProfile(**manifest['profile'])
    except (TypeError, KeyError) as error:
        raise InvalidTask('invalid archived depth profile') from error
    if profile.calibration != calibration_id:
        raise InvalidTask('depth archive calibration mismatch')
    count, chunks = manifest['cell_count'], manifest['chunks']
    if (type(count) is not int or not 0 <= count <= profile.maximum_voxels
            or not isinstance(chunks, list) or len(chunks) != math.ceil(count/CHUNK_CELLS)):
        raise InvalidTask('depth archive work budget exceeded')
    volume = DepthVolume(profile)
    volume.reset(expected_epoch)
    volume.last_stamp = number(manifest['last_stamp'], 'archive time')
    volume.error = None
    for index, key in enumerate(chunks):
        if not isinstance(key, str) or len(key) != 64:
            raise InvalidTask('invalid depth chunk digest')
        value = store.record('depth_chunk', key)
        expected = min(CHUNK_CELLS, count - index*CHUNK_CELLS)
        if not isinstance(value, list) or len(value) != expected or digest(value) != key:
            raise InvalidTask('depth archive missing or corrupt chunk')
        for cell in value:
            if not isinstance(cell, list) or len(cell) != 5:
                raise InvalidTask('invalid archived voxel')
            xyz = tuple(cell[:3])
            if any(type(v) is not int for v in xyz) or xyz in volume.cells:
                raise InvalidTask('duplicate or invalid archived voxel')
            volume.cells[xyz] = (cell[3], cell[4])
    # Reuse the full encoder validation after reconstructing the bounded volume.
    checked, _ = encode(volume, clock_id=manifest['clock_id'])
    if checked != manifest:
        raise InvalidTask('noncanonical depth archive')
    volume.historical = True
    volume.error = 'archived geometry requires new live depth epoch'
    return volume
