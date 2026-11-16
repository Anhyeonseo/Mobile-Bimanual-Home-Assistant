from dataclasses import replace
import sqlite3
import pytest
from test_home_assistant import environment, AUTH, read
from home_robot_tasks.mapping_checkpoint import MappingCheckpoints
from home_robot_tasks.assistant.service import AssistantService
from home_robot_tasks.assistant.world import WorldStore
from home_robot_tasks.application import RobotApplication
from home_robot_tasks.navigation import NavigationMap
from home_robot_tasks.fake_robot import FakeRobot
from home_robot_tasks.fetch import InvalidTask


def start(env):
    now,s,g=env
    s.survey.start('restart-test','우리 집',g.now())
    now[0]+=.1;g.tick()
    return now,s,g


def stop_and_save(env):
    now,s,g=start(env)
    response=g.dispatch('POST','/v1/assistant/scans/restart-test/save_exit',AUTH,{})[1]
    assert response['survey_state']=='STOPPING'
    assert not response.get('simulated_shutdown_ready',False)
    for _ in range(20):now[0]+=.1;g.tick()
    return now,s,g


def test_save_exit_waits_for_stop_is_idempotent_and_does_not_power_off(environment):
    now,s,g=stop_and_save(environment)
    scan=s.scan.get('restart-test')
    assert scan['survey_state']=='SAVED_FOR_EXIT' and scan['simulated_shutdown_ready']
    assert not scan['power_disconnect_allowed'] and s.app.lease.owner is None
    assert s.survey.checkpoints.load('restart-test')['scan']['map_document']==scan['map_document']
    assert g.dispatch('POST','/v1/assistant/scans/restart-test/save_exit',AUTH,{})[1]==scan
    count=len(s.app.tasks)
    for _ in range(10):now[0]+=.1;g.tick()
    assert len(s.app.tasks)==count


def test_unconfirmed_stop_never_saves_exit(environment):
    now,s,g=start(environment)
    s.app.adapter.never_stop=True
    s.survey.action('restart-test','save_exit',g.now())
    for _ in range(150):now[0]+=.1;g.tick()
    scan=s.scan.get('restart-test')
    assert not scan.get('simulated_shutdown_ready',False)
    assert s.app.lease.owner is not None


def test_failed_write_keeps_previous_generation_and_no_ready_indicator(environment,monkeypatch):
    now,s,g=start(environment)
    original=s.survey.checkpoints.load('restart-test')['checkpoint_id']
    def fail(*a,**kw):raise OSError('disk full')
    monkeypatch.setattr(s.survey.checkpoints,'save',fail)
    s.survey.action('restart-test','save_exit',g.now())
    for _ in range(20):now[0]+=.1;g.tick()
    scan=s.scan.get('restart-test')
    assert scan['survey_state']=='SAVE_FAILED' and not scan['simulated_shutdown_ready']
    assert s.survey.checkpoints.load('restart-test')['checkpoint_id']==original


def test_new_process_clock_and_moved_robot_require_fresh_position(environment):
    now,s,g=stop_and_save(environment)
    old=s.scan.get('restart-test')
    app=RobotApplication(read('home.example.json'),NavigationMap(read('navigation_map.simulation.json')),FakeRobot())
    new_context=replace(s.context,clock_id='new-boot-clock')
    resumed=AssistantService(app,s.store,new_context,lambda:200.)
    assert resumed.scan.get('restart-test')['survey_state']=='RECOVERY_REQUIRED'
    assert not resumed.scan.get('restart-test')['simulated_shutdown_ready']
    assert app.lease.owner is None
    # Current synthetic pose differs from checkpoint; never copy saved XY.
    app.xy=(1.75,1.75)
    resumed.survey.action('restart-test','resume',0.)
    assert app.xy==(1.75,1.75)
    scan=resumed.scan.get('restart-test')
    assert scan['context']['clock_id']=='new-boot-clock'
    assert scan['restored_from_context']==old['context']
    assert not scan['navigation_ready'] and scan['captures']==[]


@pytest.mark.parametrize('field,value',[
    ('boot_id','previous-boot'),('observed_s',-1.),('observed_s',0.),('observed_s',100.),('map_revision','old-map'),
    ('calibration_id','old-calibration'),('localized',False),('sensors_ready',False),
    ('arms_ready',False),('lift_ready',False),('stopped',False),('source','measured'),
    ('xy',(999.,999.))])
def test_invalid_resume_evidence_cannot_start_motion(environment,field,value):
    now,s,g=stop_and_save(environment)
    original=s.survey.readiness_provider
    s.survey.readiness_provider=lambda t,r:replace(original(t,r),**{field:value})
    count=len(s.app.tasks)
    s.survey.action('restart-test','resume',g.now())
    for _ in range(65):now[0]+=.1;g.tick()
    assert s.scan.get('restart-test')['status']=='PAUSED'
    assert len(s.app.tasks)==count and s.app.lease.owner is None


def test_delayed_valid_evidence_resumes_with_no_new_browser_permission(environment):
    now,s,g=stop_and_save(environment)
    s.app.adapter.false_conditions.add('localized')
    permit=g.dispatch('GET','/v1/permit',AUTH,None)[1]['permit']
    g.dispatch('POST','/v1/assistant/scans/restart-test/resume',AUTH,{},permit)
    assert s.scan.get('restart-test')['status']=='PAUSED'
    now[0]+=.1;s.app.adapter.false_conditions.clear();g.tick()
    assert s.scan.get('restart-test')['status']=='SCANNING'


def test_corrupt_latest_generation_falls_back_and_all_corrupt_refuses(environment):
    now,s,g=stop_and_save(environment)
    cp=s.survey.checkpoints
    head=s.store.record('mapping_checkpoint_head','restart-test')
    assert len(head['generations'])>=2
    latest=head['generations'][0]
    with s.store.db:
        s.store.db.execute("UPDATE assistant_records SET body='{}' WHERE kind='mapping_checkpoint' AND id=?",(latest,))
    recovered=cp.load('restart-test')
    assert recovered['fallback_used'] and recovered['checkpoint_id']!=latest
    with s.store.db:s.store.db.execute("UPDATE assistant_records SET body='{}' WHERE kind='mapping_checkpoint'")
    with pytest.raises(InvalidTask,match='no_valid'):cp.load('restart-test')


def test_transaction_abort_does_not_publish_partial_generation(environment):
    now,s,g=start(environment)
    cp=s.survey.checkpoints;before=cp.load('restart-test')['checkpoint_id']
    with s.store.db:
        s.store.db.execute("CREATE TRIGGER fail_checkpoint BEFORE INSERT ON assistant_records WHEN NEW.kind='mapping_checkpoint_head' BEGIN SELECT RAISE(ABORT,'write interrupted'); END")
    s.survey._save(sensor_sequence=999)
    with pytest.raises(sqlite3.IntegrityError):cp.save('restart-test',stopped=True)
    assert cp.load('restart-test')['checkpoint_id']==before


def test_graph_pair_hashes_and_historical_records_survive_database_reopen(environment,tmp_path):
    now,s,g=stop_and_save(environment)
    cp=s.survey.checkpoints
    graph={'slam.data':b'synthetic data','slam.posegraph':b'synthetic graph'}
    saved=cp.save('restart-test',stopped=True,graph=graph)
    database=s.store.db.execute('PRAGMA database_list').fetchone()[2]
    reopened=WorldStore(database)
    try:
        restored=MappingCheckpoints(reopened)
        result=restored.load('restart-test')
        assert result['checkpoint_id']==saved['checkpoint_id']
        assert result['assets']['slam.data']==graph['slam.data']
        assert any(name.startswith('entity/') for name in result['assets'])
        path=restored.materialize_graph('restart-test',tmp_path/'restored')
        assert path.with_suffix('.posegraph').read_bytes()==graph['slam.posegraph']
        assert result['manifest']['historical_only']
    finally:reopened.close()


def test_changed_calibration_refuses_restore(environment):
    now,s,g=stop_and_save(environment)
    with pytest.raises(InvalidTask,match='calibration_changed'):
        s.survey.checkpoints.restore_scan('restart-test',replace(s.context,calibration_id='different'))


def test_save_before_first_observation_and_resume_afterward(environment):
    now,s,g=environment
    s.survey.start('restart-test','우리 집',0.)
    scan=s.survey.action('restart-test','save_exit',0.)
    assert scan['survey_state']=='SAVED_FOR_EXIT'
    assert scan['map_document'] is not None
    s.survey.action('restart-test','resume',0.)
    assert s.scan.get('restart-test')['status']=='SCANNING'


def test_process_death_during_uncommitted_write_recovers_previous_checkpoint(environment):
    import subprocess, sys
    now,s,g=stop_and_save(environment)
    before=s.survey.checkpoints.load('restart-test')['checkpoint_id']
    database=s.store.db.execute('PRAGMA database_list').fetchone()[2]
    # Kill the writer before committing its pointer update; this exercises the
    # actual SQLite journal on disk rather than an in-memory exception mock.
    code='''import os,sqlite3,sys
c=sqlite3.connect(sys.argv[1]);c.execute('PRAGMA synchronous=FULL')
c.execute('BEGIN IMMEDIATE')
c.execute("UPDATE assistant_records SET body='{}' WHERE kind='mapping_checkpoint_head'")
c.execute("INSERT INTO mapping_checkpoint_assets VALUES ('unfinished',?)",(b'x'*1000000,))
os._exit(31)
'''
    result=subprocess.run([sys.executable,'-c',code,database])
    assert result.returncode==31
    reopened=WorldStore(database)
    try:
        cp=MappingCheckpoints(reopened)
        assert cp.load('restart-test')['checkpoint_id']==before
        assert reopened.db.execute("SELECT count(*) FROM mapping_checkpoint_assets WHERE sha256='unfinished'").fetchone()[0]==0
    finally:reopened.close()


def test_retains_three_complete_generations_and_no_orphan_blobs(environment):
    now,s,g=stop_and_save(environment)
    for i in range(8):
        s.survey._save(sensor_sequence=i)
        s.survey.checkpoints.save('restart-test',stopped=True,
            graph={'slam.data':bytes([i+1]),'slam.posegraph':b'graph'})
    head=s.store.record('mapping_checkpoint_head','restart-test')
    assert len(head['generations'])==3
    manifests=s.store.records('mapping_checkpoint')
    assert len(manifests)==3
    expected={v for m in manifests for v in m['assets'].values()}
    actual={r[0] for r in s.store.db.execute('SELECT sha256 FROM mapping_checkpoint_assets')}
    assert expected==actual


def test_replay_save_requires_complete_graph_and_preserves_previous(environment):
    now,s,g=stop_and_save(environment)
    cp=s.survey.checkpoints;previous=cp.load('restart-test')['checkpoint_id']
    s.survey._save(mode='replay')
    with pytest.raises(InvalidTask,match='requires_pose_graph'):
        cp.save('restart-test',stopped=True)
    with pytest.raises(InvalidTask,match='complete_serialized'):
        cp.save('restart-test',stopped=True,graph={'slam.data':b'incomplete'})
    with pytest.raises(InvalidTask,match='confirmed_stop'):
        cp.save('restart-test',stopped=False)
    assert cp.load('restart-test')['checkpoint_id']==previous
