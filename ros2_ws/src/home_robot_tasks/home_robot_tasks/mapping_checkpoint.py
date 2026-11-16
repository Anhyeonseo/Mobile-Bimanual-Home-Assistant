"""Atomic, hash-verified mapping checkpoints in the existing SQLite store.

Graph bytes, scan state, semantic records and archived depth commit together.
Only complete generations are discoverable; two older generations remain for
fallback. Loading never renews observation timestamps or authorizes movement.
"""
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import tempfile

from .assistant.contracts import document, finite
from .fetch import InvalidTask
from .navigation import NavigationMap, identifier


def checksum(raw):
    return hashlib.sha256(raw).hexdigest()


class MappingCheckpoints:
    MAX_BYTES = 256 * 1024 * 1024

    def __init__(self, store):
        self.store = store
        with store.lock, store.db:
            store.db.execute('PRAGMA synchronous=FULL')
            store.db.execute('CREATE TABLE IF NOT EXISTS mapping_checkpoint_assets '
                             '(sha256 TEXT PRIMARY KEY, body BLOB NOT NULL)')

    def save(self, session_id, *, stopped, graph=None):
        """Call after freezing the mapping worker and confirming whole-robot stop.

        graph is the frozen slam_toolbox serializer output (.data/.posegraph),
        supplied by the trusted worker, not by an HTTP request. Simulation has
        no pose graph and is labelled accordingly; it cannot masquerade as live.
        """
        if stopped is not True:
            raise InvalidTask('confirmed_stop_required_for_checkpoint')
        identifier(session_id, 'session')
        with self.store.lock:
            scan = self.store.record('scan', session_id)
            if not scan or not scan.get('map_document'):
                raise InvalidTask('observed_map_required_for_checkpoint')
            NavigationMap(scan['map_document'])
            graph = {} if graph is None else graph
            if (not isinstance(graph, dict) or set(graph) not in (set(), {'slam.data', 'slam.posegraph'}) or
                    any(not isinstance(v, bytes) or not v for v in graph.values())):
                raise InvalidTask('complete_serialized_pose_graph_required')
            if scan['mode'] != 'simulation' and not graph:
                raise InvalidTask('replay_checkpoint_requires_pose_graph')
            snapshot = deepcopy(scan)
            # Shutdown permissions and execution intent never survive a restore.
            snapshot.update(status='PAUSED', motion_authorized=False,
                            power_disconnect_allowed=False, simulated_shutdown_ready=False, exit_requested=False)
            assets = {'scan.json': document(snapshot).encode(), **graph}
            context = scan['context']
            for kind in ('furniture_instance', 'furniture_observation', 'semantic_capture'):
                for record in self.store.records(kind):
                    c = record.get('context', record.get('stamp', {}).get('context'))
                    if kind == 'furniture_observation':
                        c = record['observation']['stamp']['context']
                    if c and (c['map_id'], c['calibration_id']) == (context['map_id'], context['calibration_id']):
                        key = record.get('instance_id') or record.get('observation', {}).get('observation_id') or record['stamp']['capture_id']
                        assets[f'{kind}/{key}.json'] = document(record).encode()
            for entity in self.store.entities():
                c = entity['stamp']['context']
                if (c['map_id'], c['calibration_id']) == (context['map_id'], context['calibration_id']):
                    assets['entity/'+entity['entity_id']+'.json'] = document(entity).encode()
            build = scan.get('navigation_build') or {}
            for digest in build.get('depth_archive', {}).get('chunks', []):
                chunk = self.store.record('depth_chunk', digest)
                if chunk is None or checksum(document(chunk).encode()) != digest:
                    raise InvalidTask('checkpoint_depth_chunk_missing_or_corrupt')
                assets['depth/'+digest+'.json'] = document(chunk).encode()
            if len(assets) > 4096 or sum(map(len, assets.values())) > self.MAX_BYTES:
                raise InvalidTask('mapping_checkpoint_budget_exceeded')
            manifest = dict(schema_version=1, session_id=session_id, context=context,
                map_revision=scan['map_document']['revision'], mode=scan['mode'],
                graph_available=bool(graph), historical_only=True,
                assets={name: checksum(raw) for name, raw in sorted(assets.items())})
            encoded = document(manifest)
            key = checksum(encoded.encode())
            head = self.store.record('mapping_checkpoint_head', session_id) or {'generations': []}
            generations = [key] + [k for k in head['generations'] if k != key][:2]
            # SQLite FULL synchronization commits blobs and pointer as one unit.
            with self.store.db:
                for name, raw in assets.items():
                    digest = manifest['assets'][name]
                    old = self.store.db.execute('SELECT body FROM mapping_checkpoint_assets WHERE sha256=?', (digest,)).fetchone()
                    if old and bytes(old[0]) != raw:
                        raise InvalidTask('checkpoint_asset_corrupt')
                    self.store.db.execute('INSERT OR IGNORE INTO mapping_checkpoint_assets VALUES (?,?)', (digest, raw))
                self.store.db.execute('INSERT OR REPLACE INTO assistant_records VALUES (?,?,?)',
                    ('mapping_checkpoint', key, encoded))
                self.store.db.execute('INSERT OR REPLACE INTO assistant_records VALUES (?,?,?)',
                    ('mapping_checkpoint_head', session_id, document(dict(generations=generations))))
                # Bound retained history; shared content is retained by digest.
                heads = self.store.db.execute("SELECT body FROM assistant_records WHERE kind='mapping_checkpoint_head'").fetchall()
                keep = {k for h in heads for k in json.loads(h[0])['generations']}
                for old_key, in self.store.db.execute("SELECT id FROM assistant_records WHERE kind='mapping_checkpoint'").fetchall():
                    if old_key not in keep:
                        self.store.db.execute("DELETE FROM assistant_records WHERE kind='mapping_checkpoint' AND id=?", (old_key,))
                referenced = {v for m, in self.store.db.execute("SELECT body FROM assistant_records WHERE kind='mapping_checkpoint'")
                              for v in json.loads(m)['assets'].values()}
                for digest, in self.store.db.execute('SELECT sha256 FROM mapping_checkpoint_assets').fetchall():
                    if digest not in referenced:
                        self.store.db.execute('DELETE FROM mapping_checkpoint_assets WHERE sha256=?', (digest,))
            persistent = bool(self.store.db.execute('PRAGMA database_list').fetchone()[2])
            return dict(checkpoint_id=key, graph_available=bool(graph), files=len(assets), durable=persistent)

    def load(self, session_id):
        with self.store.lock:
            head = self.store.record('mapping_checkpoint_head', session_id)
            if not head:
                raise InvalidTask('mapping_checkpoint_missing')
            for index, key in enumerate(head['generations']):
                try:
                    manifest = self.store.record('mapping_checkpoint', key)
                    if (not manifest or checksum(document(manifest).encode()) != key or
                            manifest['session_id'] != session_id):
                        continue
                    assets = {}
                    for name, digest in manifest['assets'].items():
                        row = self.store.db.execute('SELECT body FROM mapping_checkpoint_assets WHERE sha256=?', (digest,)).fetchone()
                        if row is None or checksum(bytes(row[0])) != digest:
                            raise InvalidTask('checkpoint_asset_corrupt')
                        assets[name] = bytes(row[0])
                    scan = json.loads(assets['scan.json'])
                    NavigationMap(scan['map_document'])
                    return dict(checkpoint_id=key, manifest=manifest, scan=scan,
                                assets=assets, fallback_used=index > 0)
                except (InvalidTask, ValueError, KeyError, TypeError):
                    continue
            raise InvalidTask('no_valid_mapping_checkpoint')

    def restore_scan(self, session_id, context):
        result = self.load(session_id)
        source = result['scan']['context']
        if any(source[k] != getattr(context, k) for k in ('map_id', 'frame_id', 'calibration_id')):
            raise InvalidTask('checkpoint_map_or_calibration_changed')
        scan = deepcopy(result['scan'])
        scan.update(status='PAUSED', survey_state='RECOVERY_REQUIRED', reason='restart_requires_fresh_observation',
            checkpoint_id=result['checkpoint_id'], checkpoint_fallback_used=result['fallback_used'],
            power_disconnect_allowed=False, simulated_shutdown_ready=False, resume_checks_required=True,
            hints=['이전 지도를 불러왔습니다. 이어하기를 누르면 현재 위치와 센서 상태를 확인합니다.'])
        # Scan metadata only. Original depth and semantic bytes remain historical
        # in the checkpoint; current live volumes must be reset after this boot.
        with self.store.lock, self.store.db:
            for name, raw in result['assets'].items():
                if name.startswith('depth/'):
                    self.store.db.execute('INSERT OR REPLACE INTO assistant_records VALUES (?,?,?)',
                        ('depth_chunk', name.split('/')[1][:-5], raw.decode()))
            self.store.db.execute('INSERT OR REPLACE INTO assistant_records VALUES (?,?,?)',
                ('scan', session_id, document(scan)))
        return result | {'scan': scan}

    def materialize_graph(self, session_id, destination):
        result = self.load(session_id)
        if not result['manifest']['graph_available']:
            raise InvalidTask('checkpoint_has_no_pose_graph')
        destination = Path(destination)
        if destination.exists():
            raise InvalidTask('new_graph_destination_required')
        destination.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='.restore-', dir=destination.parent) as temporary:
            stage = Path(temporary)/'graph';stage.mkdir()
            for name in ('slam.data', 'slam.posegraph'):
                with (stage/name).open('wb') as stream:
                    stream.write(result['assets'][name]);stream.flush();os.fsync(stream.fileno())
            fd = os.open(stage, os.O_RDONLY | os.O_DIRECTORY)
            try:os.fsync(fd)
            finally:os.close(fd)
            os.rename(stage, destination)
            fd = os.open(destination.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:os.fsync(fd)
            finally:os.close(fd)
        return destination/'slam'


@dataclass(frozen=True)
class ResumeEvidence:
    boot_id: str
    observed_s: float
    map_revision: str
    calibration_id: str
    xy: tuple
    localized: bool
    sensors_ready: bool
    arms_ready: bool
    lift_ready: bool
    stopped: bool
    source: str

    def check(self, *, boot_id, map_revision, calibration_id, requested_s, now_s, mode):
        stamp = finite(self.observed_s, 'resume observation', 0)
        if (self.boot_id != boot_id or self.map_revision != map_revision or
                self.calibration_id != calibration_id or stamp < requested_s or
                not 0 <= finite(now_s, 'resume now', 0)-stamp <= .5 or
                self.source != ('simulation' if mode == 'simulation' else 'measured') or
                any(v is not True for v in (self.localized, self.sensors_ready, self.arms_ready, self.lift_ready, self.stopped))):
            raise InvalidTask('fresh_restart_readiness_required')
        if not isinstance(self.xy, tuple) or len(self.xy) != 2:
            raise InvalidTask('fresh_localized_position_required')
        return tuple(finite(v, 'localized XY') for v in self.xy)
