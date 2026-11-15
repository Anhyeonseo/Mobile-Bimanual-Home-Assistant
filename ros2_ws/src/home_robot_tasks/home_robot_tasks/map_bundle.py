"""Portable saved maps and ROS OccupancyGrid conversion, without motion access."""
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile

from .assistant.contracts import document
from .fetch import InvalidTask
from .navigation import NavigationMap, identifier, number


def from_occupancy_grid(message, *, map_id, revision, floor_id, envelope,
                        clearance_m, occupied_threshold=65):
    """Preserve ROS increasing-Y rows. Intermediate probabilities stay unknown.

    The conservative map planner accepts axis-aligned maps only. An upstream
    rotated map needs an explicit resampling adapter, never a discarded yaw.
    """
    info, pose = message.info, message.info.origin
    if (type(info.width) is not int or type(info.height) is not int
            or not 1 <= info.width <= 256 or not 1 <= info.height <= 256
            or len(message.data) != info.width*info.height
            or type(occupied_threshold) is not int or not 1 <= occupied_threshold <= 100):
        raise InvalidTask('bounded occupancy grid required')
    quaternion = tuple(number(getattr(pose.orientation, k), 'map rotation') for k in ('x','y','z','w'))
    if any(abs(v) > 1e-7 for v in quaternion[:3]) or abs(abs(quaternion[3])-1) > 1e-7:
        raise InvalidTask('rotated occupancy map requires explicit resampling')
    cells = []
    for value in message.data:
        if type(value) is not int or not -1 <= value <= 100:
            raise InvalidTask('invalid occupancy probability')
        cells.append('#' if value >= occupied_threshold else '.' if value == 0 else '?')
    result = dict(schema_version=1, map_id=map_id, revision=revision, floor_id=floor_id,
        frame_id=message.header.frame_id, floor_z_m=number(pose.position.z,'floor Z'),
        resolution_m=number(info.resolution,'map resolution'),
        origin=dict(x=number(pose.position.x,'map origin'), y=number(pose.position.y,'map origin')),
        rows=[''.join(cells[i:i+info.width]) for i in range(0,len(cells),info.width)],
        robot_radius_m=envelope.radius_m, clearance_m=clearance_m, surface_tolerance_m=.03)
    NavigationMap(result)
    return result


def align_to_depth_lattice(map_document, *, resolution_m, revision):
    """Conservative area-overlap resampling of an axis-aligned SLAM map.

    slam_toolbox origins need not lie on the depth lattice. Copying its rows to
    a rounded origin would move obstacles. Every overlapping occupied cell wins;
    free requires the whole target cell to be covered by observed free input.
    """
    nav=NavigationMap(map_document);r=number(resolution_m,'target lattice')
    identifier(revision,'resampled map revision')
    if revision==nav.revision or not .01<=r<=.25 or not .25<=r/nav.resolution<=4:
        raise InvalidTask('bounded explicit new map lattice required')
    ox,oy=math.floor(nav.ox/r)*r,math.floor(nav.oy/r)*r
    width=math.ceil((nav.ox+nav.width*nav.resolution-ox)/r-1e-9)
    height=math.ceil((nav.oy+nav.height*nav.resolution-oy)/r-1e-9)
    if not 1<=width<=256 or not 1<=height<=256:raise InvalidTask('resampled map exceeds cell budget')
    rows=[]
    for y in range(height):
        row=[]
        for x in range(width):
            left,bottom=ox+x*r,oy+y*r;right,top=left+r,bottom+r
            sx0=math.floor((left-nav.ox)/nav.resolution+1e-9)
            sx1=math.ceil((right-nav.ox)/nav.resolution-1e-9)
            sy0=math.floor((bottom-nav.oy)/nav.resolution+1e-9)
            sy1=math.ceil((top-nav.oy)/nav.resolution-1e-9)
            values=[nav.rows[j][i] if 0<=i<nav.width and 0<=j<nav.height else '?'
                    for j in range(sy0,sy1) for i in range(sx0,sx1)]
            row.append('#' if '#' in values else '.' if values and all(v=='.' for v in values) else '?')
        rows.append(''.join(row))
    result={**deepcopy(map_document),'revision':revision,'resolution_m':r,'origin':dict(x=ox,y=oy),'rows':rows}
    NavigationMap(result)
    return result


def occupancy_values(map_document):
    nav = NavigationMap(map_document)
    return [dict(zip('.#?', (0,100,-1)))[c] for row in nav.rows for c in row]


def nav2_image(map_document):
    """PGM has top-left origin, opposite ROS map rows; unknown is gray 205."""
    nav = NavigationMap(map_document)
    pixels = bytes({'.':254, '#':0, '?':205}[c] for row in reversed(nav.rows) for c in row)
    return f'P5\n{nav.width} {nav.height}\n255\n'.encode()+pixels


def write_nav2_map(destination, name, map_document):
    identifier(name, 'map filename')
    nav = NavigationMap(map_document)
    path = Path(destination)
    (path/(name+'.pgm')).write_bytes(nav2_image(map_document))
    # JSON is a YAML subset accepted by Nav2's YAML loader.
    (path/(name+'.yaml')).write_text(json.dumps(dict(image=name+'.pgm', mode='trinary',
        resolution=nav.resolution, origin=[nav.ox, nav.oy, 0.], negate=0,
        occupied_thresh=.65, free_thresh=.196), indent=2)+'\n')
    (path/(name+'.json')).write_text(document(map_document)+'\n')


def export_scan(scan, session_id, destination):
    """Atomic portable artifact: maps, historical depth chunks and semantic records.

    Nothing here changes active localization or grants motion permission. Export
    only SAVED scans; users get an immutable checkpoint, never a half-built map.
    """
    destination = Path(destination)
    if destination.exists():
        raise InvalidTask('map bundle destination already exists')
    value = scan.get(session_id)
    if value['status'] != 'SAVED':
        raise InvalidTask('saved scan required for export')
    nav = scan.navigation_document(session_id)
    scan.archived_volume(session_id)  # Verify every chunk before publication.
    destination.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.map-bundle-', dir=destination.parent))
    try:
        write_nav2_map(stage, 'localization', value['map_document'])
        write_nav2_map(stage, 'navigation', nav)
        (stage/'scan.json').write_text(document(value)+'\n')
        chunks = value['navigation_build']['depth_archive']['chunks']
        (stage/'depth').mkdir()
        for key in chunks:
            (stage/'depth'/(key+'.json')).write_text(document(scan.store.record('depth_chunk',key))+'\n')
        rooms = [scan.store.get(k) for k in value.get('room_entity_ids', [])]
        revisions={value['map_document']['revision'],nav['revision']}
        entities=[e for e in scan.store.entities() if e['stamp']['context']['map_id']==nav['map_id']
                  and e['stamp']['context']['map_revision'] in revisions
                  and e['stamp']['context']['calibration_id']==scan.context.calibration_id]
        furniture=[f for f in scan.store.records('furniture_instance') if f['context']['map_id']==nav['map_id']
                   and f['context']['map_revision'] in revisions
                   and f['context']['calibration_id']==scan.context.calibration_id]
        captures=[c for c in scan.store.records('semantic_capture')
                  if c['stamp']['context']['map_id']==nav['map_id']
                  and c['stamp']['context']['map_revision'] in revisions
                  and c['stamp']['context']['calibration_id']==scan.context.calibration_id]
        (stage/'semantics.json').write_text(document(dict(entities=entities,furniture=furniture,
                                                        rgbd_captures=captures))+'\n')
        if value.get('source_slam_map'):
            write_nav2_map(stage,'source_slam',value['source_slam_map'])
        (stage/'rooms.json').write_text(document(rooms)+'\n')
        hashes = {str(p.relative_to(stage)):hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in sorted(stage.rglob('*')) if p.is_file()}
        manifest = dict(schema_version=1, kind='s4_saved_map', files_sha256=hashes,
            map_id=nav['map_id'], revision=nav['revision'],
            calibration_id=scan.context.calibration_id, localization_required=True,
            live_travel_guard_required=True, physical_map_verified=False,
            depth_is_historical=True, hardware_commands=0)
        (stage/'manifest.json').write_text(document(manifest)+'\n')
        os.rename(stage, destination)
        return manifest
    finally:
        if stage.exists(): shutil.rmtree(stage)


def read_bundle(path):
    """Verify before giving either map to ROS. Do not follow external symlinks."""
    path = Path(path).resolve()
    manifest_path = path/'manifest.json'
    if not manifest_path.is_file() or manifest_path.stat().st_size > 2_000_000:
        raise InvalidTask('bounded map bundle manifest required')
    manifest = json.loads(manifest_path.read_text())
    hashes = manifest.get('files_sha256')
    if (manifest.get('schema_version') != 1 or manifest.get('kind') != 's4_saved_map'
            or not isinstance(hashes,dict) or not 1 <= len(hashes) <= 150
            or not {'localization.json','navigation.json','scan.json','rooms.json'} <= hashes.keys()):
        raise InvalidTask('invalid map bundle manifest')
    for name, expected in hashes.items():
        candidate = path/name
        if (not isinstance(name,str) or Path(name).is_absolute() or '..' in Path(name).parts
                or not candidate.resolve().is_relative_to(path) or not candidate.is_file()
                or candidate.stat().st_size > 2_000_000
                or hashlib.sha256(candidate.read_bytes()).hexdigest() != expected):
            raise InvalidTask('map bundle missing or changed file')
    maps = {name:json.loads((path/(name+'.json')).read_text()) for name in ('localization','navigation')}
    for value in maps.values(): NavigationMap(value)
    nav = maps['navigation']
    if (manifest.get('map_id'),manifest.get('revision')) != (nav['map_id'],nav['revision']):
        raise InvalidTask('map bundle identity mismatch')
    return deepcopy(manifest), maps
