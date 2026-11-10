from copy import deepcopy
import numpy as np
import pytest
from test_scan_map_generation import make_service, begin
from home_robot_tasks.fetch import InvalidTask


def archive(path):
    service, recorded, envelope = make_service(path)
    volume = begin(service, recorded)
    service.scan.build_navigation('home-scan', volume, envelope, now_s=1.1)
    return service, volume


def test_restarted_volume_preserves_geometry_without_reusing_live_clearance(tmp_path):
    path = tmp_path/'home.db'
    service, volume = archive(path)
    cells = deepcopy(volume.cells)
    service.store.close()
    service, _, _ = make_service(path, clock_id='restarted-clock')
    try:
        loaded = service.scan.archived_volume('home-scan')
        assert loaded.cells == cells and loaded.historical
        assert loaded.projected_obstacles() == volume.projected_obstacles()
        assert not loaded.fresh(loaded.last_stamp)
        assert not any(loaded.free(k, stamp) for k, (_, stamp) in cells.items())
        with pytest.raises(InvalidTask, match='archived depth'):
            loaded.integrate(np.ones((1,1)), (1,1,0,0), np.eye(4), observed_s=1.2,
                now_s=1.2, frame=loaded.p.frame, calibration=loaded.p.calibration,
                epoch=loaded.epoch, motion_qualified=True)
        loaded.reset('new-travel')
        assert not loaded.cells and not loaded.historical and not loaded.fresh(1.2)
    finally:
        service.store.close()


@pytest.mark.parametrize('change', ['missing_chunk', 'changed_chunk', 'manifest', 'calibration'])
def test_archive_corruption_fails_closed(change, tmp_path):
    service, _ = archive(tmp_path/'home.db')
    try:
        scan = service.scan.get('home-scan')
        manifest = scan['navigation_build']['depth_archive']
        key = manifest['chunks'][0]
        if change == 'missing_chunk':
            service.store.db.execute('DELETE FROM assistant_records WHERE kind=? AND id=?', ('depth_chunk',key))
        elif change == 'changed_chunk':
            chunk = service.store.record('depth_chunk',key)
            chunk[0][3] = not chunk[0][3]
            service.store.save_record('depth_chunk',key,chunk)
        else:
            if change == 'manifest': manifest['cell_count'] += 1
            else: manifest['profile']['calibration'] = 'changed'
            service.store.save_record('scan','home-scan',scan)
        with pytest.raises(InvalidTask): service.scan.archived_volume('home-scan')
    finally:
        service.store.close()


def test_archive_and_generated_map_roll_back_together(tmp_path, monkeypatch):
    from home_robot_tasks import depth_archive
    service, recorded, envelope = make_service(tmp_path/'home.db')
    volume = begin(service, recorded)
    def failed(db, records):
        key, value = next(iter(records.items()))
        original(db, {key:value})
        raise OSError('disk full')
    original = depth_archive.persist
    monkeypatch.setattr(depth_archive, 'persist', failed)
    try:
        with pytest.raises(OSError): service.scan.build_navigation('home-scan',volume,envelope,now_s=1.1)
        assert not service.store.records('depth_chunk')
        assert not service.scan.get('home-scan')['navigation_ready']
    finally:
        service.store.close()
