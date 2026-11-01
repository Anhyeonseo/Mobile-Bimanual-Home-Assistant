"""Height obstacles are part of the generated, saved and reloaded home map."""
from copy import deepcopy
from dataclasses import replace

import pytest

from test_home_assistant import read
from test_semantic_home_mapping import bed_scene
from home_robot_tasks.application import RobotApplication
from home_robot_tasks.fake_robot import FakeRobot
from home_robot_tasks.fetch import InvalidTask
from home_robot_tasks.navigation import NavigationMap, NavigateRequest
from home_robot_tasks.navigation_depth import DepthVolume
from home_robot_tasks.assistant.contracts import Context
from home_robot_tasks.assistant.service import AssistantService
from home_robot_tasks.assistant.world import WorldStore


def make_service(path, *, clock_id='clock', envelope=None):
    doc, recorded, default_envelope = bed_scene()
    app = RobotApplication(read('home.example.json'), NavigationMap(doc), FakeRobot(), initial_xy=(.15,.55))
    store = WorldStore(path)
    context = Context(doc['map_id'], doc['revision'], 'cal', clock_id)
    service = AssistantService(app, store, context, lambda:1.1,
                               travel_envelope=envelope or default_envelope)
    return service, recorded, default_envelope


def begin(service, recorded):
    scan = service.scan
    scan.start('home-scan','처음 방문한 집',mode='replay')
    scan.ingest_map('home-scan',service.app.map._document)
    volume = DepthVolume(recorded.p)
    volume.reset(scan.depth_epoch('home-scan'))
    # Replay a synthetic surveyed volume. Production uses native depth frames.
    volume.cells, volume.last_stamp, volume.error = deepcopy(recorded.cells),recorded.last_stamp,None
    return volume


def test_bed_is_blocked_in_saved_map_after_restart_before_new_depth_arrives(tmp_path):
    path=tmp_path/'home.db'
    service,recorded,envelope=make_service(path)
    volume=begin(service,recorded)
    generated=service.scan.build_navigation('home-scan',volume,envelope,now_s=1.1)
    assert generated['map_document']['rows'][5][5] == '.'
    assert generated['navigation_map_document']['rows'][5][5] == '#'
    service.scan.review('home-scan')
    saved=service.scan.save('home-scan')
    assert saved['saved_artifact'] == 'full_body_map_bundle' and saved['navigation_ready']
    raw_revision=saved['map_document']['revision']
    body_revision=saved['navigation_map_document']['revision']
    assert raw_revision != body_revision
    assert service.store.get_map('bedroom',raw_revision)['rows'][5][5] == '.'
    assert service.store.get_map('bedroom',body_revision)['rows'][5][5] == '#'
    service.store.close()

    restarted,_,_=make_service(path,clock_id='new-process-clock')
    try:
        assert restarted.activate_scan('home-scan',0)['status'] == 'LOCALIZATION_REQUIRED'
        assert restarted.app.map.revision == body_revision
        assert not restarted.app.pose_known and not restarted.app.tasks
        with pytest.raises(InvalidTask,match='blocked'):
            restarted.app.map._cell(.55,.55)
        goal=NavigateRequest('route','bedroom',body_revision,'floor-1','map',1.05,.55,0.,0.)
        path_xy=restarted.app.map.plan(goal,(.15,.55))['path_xy']
        assert any(abs(p['y']-.55) > .1 for p in path_xy)
    finally:
        restarted.store.close()


def test_raw_lidar_scan_can_be_archived_but_not_activated_for_navigation(tmp_path):
    service,recorded,_=make_service(tmp_path/'raw.db')
    try:
        begin(service,recorded)
        service.scan.review('home-scan')
        saved=service.scan.save('home-scan')
        assert saved['saved_artifact'] == 'scan_record_only' and not saved['navigation_ready']
        with pytest.raises(InvalidTask,match='height_geometry_required'):
            service.activate_scan('home-scan',0)
    finally:
        service.store.close()


def test_slam_revision_change_invalidates_generated_grid_and_old_depth_epoch(tmp_path):
    service,recorded,envelope=make_service(tmp_path/'revised.db')
    try:
        volume=begin(service,recorded)
        service.scan.build_navigation('home-scan',volume,envelope,now_s=1.1)
        revised={**service.app.map._document,'revision':'loop-closure-2'}
        value=service.scan.ingest_map('home-scan',revised)
        assert not value['navigation_ready'] and value['navigation_map_document'] is None
        with pytest.raises(InvalidTask,match='epoch'):
            service.scan.build_navigation('home-scan',volume,envelope,now_s=1.1)
    finally:
        service.store.close()


@pytest.mark.parametrize('change',['height','radius','calibration','map_contents','missing_profile'])
def test_activation_rejects_changed_robot_or_map_provenance(change,tmp_path):
    service,recorded,envelope=make_service(tmp_path/'changed.db')
    try:
        volume=begin(service,recorded)
        service.scan.build_navigation('home-scan',volume,envelope,now_s=1.1)
        service.scan.review('home-scan'); service.scan.save('home-scan')
        if change=='height':service.travel_envelope=replace(envelope,height_m=1.3)
        if change=='radius':service.travel_envelope=replace(envelope,radius_m=.3)
        if change=='missing_profile':service.travel_envelope=None
        if change=='calibration':service.scan.context=replace(service.context,calibration_id='moved-camera')
        if change=='map_contents':
            value=service.scan.get('home-scan')
            value['navigation_map_document']['rows'][5]='.'*12
            service.store.save_record('scan','home-scan',value)
        with pytest.raises(InvalidTask):service.activate_scan('home-scan',0)
        assert not service.app.tasks and service.app.map.revision=='laser-v1'
    finally:
        service.store.close()


def test_build_and_save_do_not_trigger_motion_and_saved_map_is_immutable(tmp_path):
    service,recorded,envelope=make_service(tmp_path/'immutable.db')
    try:
        volume=begin(service,recorded)
        first=service.scan.build_navigation('home-scan',volume,envelope,now_s=1.1)
        second=service.scan.build_navigation('home-scan',volume,envelope,now_s=1.2)
        assert first['navigation_map_document']==second['navigation_map_document']
        service.scan.review('home-scan')
        saved=service.scan.save('home-scan')
        assert service.scan.save('home-scan')==saved
        with pytest.raises(InvalidTask):service.scan.build_navigation('home-scan',volume,envelope,now_s=1.2)
        assert not service.app.tasks and service.app.lease.owner is None
    finally:
        service.store.close()


def test_finished_projection_cannot_restore_a_scan_discarded_during_computation(tmp_path,monkeypatch):
    import home_robot_tasks.assistant.scanning as scanning
    service,recorded,envelope=make_service(tmp_path/'concurrent.db')
    try:
        volume=begin(service,recorded)
        fuse=scanning.fuse_navigation_map
        def finish_after_discard(*args,**kwargs):
            result=fuse(*args,**kwargs)
            service.scan.discard('home-scan')
            return result
        monkeypatch.setattr(scanning,'fuse_navigation_map',finish_after_discard)
        with pytest.raises(InvalidTask,match='scan_changed'):
            service.scan.build_navigation('home-scan',volume,envelope,now_s=1.1)
        assert service.scan.get('home-scan')['status']=='DISCARDED'
        assert service.scan.get('home-scan')['navigation_map_document'] is None
    finally:
        service.store.close()
