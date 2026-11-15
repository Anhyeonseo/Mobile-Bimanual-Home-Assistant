"""Recorded home + dialogue + observation boundaries, without devices."""
from copy import deepcopy
from dataclasses import replace
import hashlib
import json

import pytest

from test_home_assistant import environment, AUTH
from test_context_labels import build
from home_robot_tasks.assistant.context_labels import load_household_profile, HouseholdMemory
from home_robot_tasks.assistant.world import WorldStore
from home_robot_tasks.assistant.perception import ingest_object_capture
from home_robot_tasks.assistant.contracts import Stamp
from home_robot_tasks.fetch import InvalidTask


def profile(tmp_path):
    folder=tmp_path/'house';folder.mkdir()
    occupancy=dict(width=20,height=20,resolution=.1,origin=[0,0],data=[0]*400)
    raw=json.dumps(occupancy).encode(); (folder/'source-map.json').write_bytes(raw)
    labels=build();labels['source_map_sha256']=hashlib.sha256(raw).hexdigest()
    (folder/'household-labels.json').write_text(json.dumps(labels))
    return load_household_profile(folder),folder


@pytest.fixture
def home(environment,tmp_path):
    now,service,gateway=environment
    p,_=profile(tmp_path);service.attach_household(p)
    for value in service.household.status()['names']:
        service.household.name(value['entity_id'],value['label'],value['aliases'],value['revision'])
    yield now,service,gateway


def meaning(operation='fetch_object',source='화장대',destination='침대 옆 탁자'):
    return dict(operation=operation,category='marker',attributes={'color':'black'},
                source_phrase=source,destination_phrase=destination,issues=[])


def prepare(service,rid='observe',operation='fetch_object',source='화장대',destination='침대 옆 탁자'):
    return service.prepare('화장대에 있는 검은 마커펜을 침대 옆 탁자에 놓아줘',rid,
                           meaning=meaning(operation,source,destination))


def packet(service,rid='observe',mode='recorded',count=1):
    context=replace(service.context,clock_id='recorded-test') if mode=='recorded' else service.context
    stamp=Stamp(service.clock(),context,'view-one').to_dict()
    return dict(request_id=rid,mode=mode,stamp=stamp,
        transform=dict(stamp=deepcopy(stamp),matrix=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
                       source_frame='color_camera_optical_frame',target_frame=context.frame_id,
                       source='recorded' if mode=='recorded' else 'measured'),
        report=dict(schema_version=2,coordinate_frame='color_camera_optical_frame',detections=[
            dict(id=i,label='black marker pen',score=.8,
                 verification=dict(accepted=True,category='marker'),
                 surface_point=dict(valid=True,xyz_m=[.2+i*.1,.3,.6])) for i in range(count)]),
        evidence=dict(rgb_sha256='a'*64,depth_sha256='b'*64),support_id='table-0',support_verified=False)


def test_load_checks_original_map_bytes_and_copy_content(tmp_path):
    p,folder=profile(tmp_path)
    raw=(folder/'source-map.json').read_bytes();source=tmp_path/'original.json';source.write_bytes(raw)
    (folder/'source-map.json').write_text(json.dumps(json.loads(raw),indent=2))
    assert load_household_profile(folder,source_map=source)['source_map_sha256']==p['source_map_sha256']
    changed=json.loads(raw);changed['data'][0]=100;(folder/'source-map.json').write_text(json.dumps(changed))
    with pytest.raises(InvalidTask,match='hash_mismatch'):load_household_profile(folder,source_map=source)


def test_names_require_review_without_promoting_anchor_or_clock(environment,tmp_path):
    _,service,_=environment;p,_=profile(tmp_path);service.attach_household(p)
    before=service.store.get('table-0');result=prepare(service)
    assert 'source_label_unconfirmed' in result['grounding']['issues']
    after=service.household.name('table-0','화장대',['화장 테이블'],1)
    for field in ('position_m','geometry_kind','stamp','source'):
        assert after[field]==before[field]
    assert after['source']=='inferred' and after['geometry_kind']=='bbox_center'
    with pytest.raises(InvalidTask,match='revision_conflict'):service.household.name('table-0','다른 이름',[],1)


def test_names_persist_and_duplicate_alias_is_not_first_match(home):
    _,service,_=home
    value=prepare(service,destination='탁자')
    assert 'ambiguous_destination' in value['grounding']['issues']
    with pytest.raises(InvalidTask,match='ambiguous'):service.answer('observe','탁자')
    result=service.answer('observe','침대 옆 탁자에')
    assert result['grounding']['destination_candidates'][0]['entity_id']=='table-1'
    reopened=HouseholdMemory(service.store,service.household.profile,clock_id='restarted')
    assert reopened.store.get('table-0')['state']=='observed'
    assert reopened.store.get('table-0')['stamp']['context']['clock_id']=='past'


def test_personal_alias_updates_only_names(home):
    _,service,gateway=home
    old=service.store.get('sleep')
    code,value=gateway.dispatch('POST','/v1/assistant/household/names',AUTH,
        dict(entity_id='sleep',label='안방',aliases=['침실'],expected_revision=old['revision']))
    assert code==200 and value['stamp']==old['stamp']
    result=prepare(service,destination='안방 화장대')
    assert result['grounding']['destination_candidates'][0]['entity_id']=='table-0'


def test_recorded_detection_never_becomes_current_or_verified_support(home):
    _,service,_=home;prepare(service)
    p=packet(service);saved=ingest_object_capture(service,p)
    item=service.store.get(saved['entity_ids'][0])
    assert item['source']=='inferred' and item['state']=='stale' and item['support_id'] is None
    result=service.get('observe')
    assert result['grounding']['object_candidates'][0]['fresh'] is False
    assert 'object_support_unconfirmed' in result['grounding']['issues']
    assert not service.household_review('observe')['executable']
    assert ingest_object_capture(service,p)==saved
    p['evidence']['rgb_sha256']='c'*64
    with pytest.raises(InvalidTask,match='identity_conflict'):ingest_object_capture(service,p)


def test_live_matching_clock_can_refresh_find_but_cannot_execute(home):
    _,service,gateway=home;prepare(service,operation='find_object',source='',destination='')
    saved=ingest_object_capture(service,packet(service,mode='live'))
    result=service.get('observe')
    assert result['status']=='READY_TO_PLAN' and result['grounding']['object_candidates'][0]['fresh']
    with pytest.raises(InvalidTask,match='physical_execution'):service.envelope('observe','anything')
    assert not service.app.tasks and gateway.app.lease.owner is None


def test_empty_or_unconfirmed_latest_view_does_not_reuse_old_positive(home):
    now,service,_=home;prepare(service,operation='find_object',source='',destination='')
    ingest_object_capture(service,packet(service,mode='live'))
    now[0]+=1
    p=packet(service,mode='live');p['stamp']['capture_id']='view-two';p['transform']['stamp']=deepcopy(p['stamp'])
    p['report']['detections'][0]['verification']['accepted']=False
    result=ingest_object_capture(service,p)
    assert result['result']=='NO_CONFIRMED_OBJECT'
    assert service.get('observe')['status']=='SEARCH_REQUIRED'
    assert not service.get('observe')['grounding']['object_candidates']
    assert len([v for v in service.store.entities() if v['entity_id'].startswith('object-')])==1


def test_duplicate_candidates_need_selection_and_new_capture_invalidates_it(home):
    now,service,_=home;prepare(service,operation='find_object',source='',destination='')
    record=ingest_object_capture(service,packet(service,mode='live',count=2))
    assert 'ambiguous_object' in service.get('observe')['grounding']['issues']
    service.clarify('observe',{'object_id':record['entity_ids'][0]})
    assert service.refresh_request('observe')['status']=='READY_TO_PLAN'
    now[0]+=1;p=packet(service,mode='live');p['stamp']['capture_id']='different';p['transform']['stamp']=deepcopy(p['stamp'])
    ingest_object_capture(service,p)
    assert 'object_id' not in service.get('observe')['selection']


@pytest.mark.parametrize('mutation', [
    lambda p:p['stamp']['context'].update(map_revision='wrong'),
    lambda p:p['transform']['stamp'].update(observed_s=20),
    lambda p:p['transform']['matrix'][0].__setitem__(0,2),
    lambda p:p['evidence'].update(rgb_sha256='missing'),
    lambda p:p.update(support_verified=True),
    lambda p:p['report']['detections'].append(dict(id=3,label='marker',score=.8,surface_point={'valid':True,'xyz_m':[0,0,-1]}))])
def test_bad_capture_is_atomic(home,mutation):
    _,service,_=home;prepare(service)
    before=service.store.entities();p=packet(service);mutation(p)
    # The malformed second item must pass semantic verification to reach depth validation.
    if len(p['report']['detections'])>1:p['report']['detections'][1]['verification']=dict(accepted=True,category='marker')
    with pytest.raises(InvalidTask):ingest_object_capture(service,p)
    assert service.store.entities()==before and not service.store.records('object_capture')


def test_wrong_clock_and_old_live_observation_are_refused(home):
    now,service,_=home;prepare(service)
    p=packet(service);p['stamp']['context']['clock_id']=service.context.clock_id
    with pytest.raises(InvalidTask,match='live_clock'):ingest_object_capture(service,p)
    p=packet(service,mode='live');now[0]+=10
    with pytest.raises(InvalidTask,match='stale'):ingest_object_capture(service,p)


def test_cancel_discards_late_observation_and_raw_motion_admission_is_closed(home):
    _,service,gateway=home;prepare(service)
    gateway.dispatch('POST','/v1/assistant/requests/observe/cancel',AUTH,{})
    with pytest.raises(InvalidTask,match='finalized'):ingest_object_capture(service,packet(service))
    assert service.app.admission() is False
    for route in ('demo/observe','localize-simulation','surveys'):
        with pytest.raises(InvalidTask,match='review_only'):
            gateway.dispatch('POST','/v1/assistant/'+route,AUTH,{})
    assert not service.app.tasks


def test_evidence_api_requires_authentication(home):
    _,_,gateway=home
    assert gateway.dispatch('GET','/v1/assistant/household/evidence/0',None,None)[0]==401
    with pytest.raises(InvalidTask,match='evidence_unavailable'):
        gateway.dispatch('GET','/v1/assistant/household/evidence/0',AUTH,None)


def test_concrete_destination_can_be_answered_and_cancel_works_when_faulted(home):
    _,service,gateway=home
    m=meaning(destination='여기')
    result=service.prepare('검은 마커펜을 여기로 옮겨줘','follow-up',meaning=m)
    assert 'concrete_destination_required' in result['grounding']['issues']
    result=service.answer('follow-up','침대 옆 탁자에')
    assert 'concrete_destination_required' not in result['grounding']['issues']
    gateway.faulted=True
    code,result=gateway.dispatch('POST','/v1/assistant/requests/follow-up/answer',AUTH,{'utterance':'취소'})
    assert code==200 and result['status']=='CANCELLED'
    assert not service.app.tasks


def test_recorded_map_api_never_returns_demo_map_or_demo_route(home):
    _,service,gateway=home
    code,result=gateway.dispatch('GET','/v1/map',AUTH,None)
    assert code==200 and result['map_id']==service.context.map_id
    assert result['map_id']!=service.app.map.map_id
    with pytest.raises(InvalidTask,match='review_only'):
        gateway.dispatch('POST','/v1/navigation/preview',AUTH,{})
    assert gateway.dispatch('GET','/v1/health',AUTH,None)[1]['mode']=='recorded_home_review'


def test_text_cancel_invalidates_prepared_requests_without_running_task(home):
    _,service,gateway=home
    prepare(service,'pending-one');prepare(service,'pending-two')
    code,value=gateway.dispatch('POST','/v1/assistant/language-requests',AUTH,
                               {'request_id':'cancel-all','utterance':'취소'})
    assert code==200 and value['status']=='CANCELLED'
    assert set(value['cancelled_pending_request_ids'])=={'pending-one','pending-two'}
    assert service.get('pending-one')['status']=='CANCELLED' and not service.app.tasks


def test_request_refresh_failure_rolls_back_entire_capture(home,monkeypatch):
    _,service,_=home;prepare(service)
    before=service.store.entities()
    def fail(*args,**kwargs):raise RuntimeError('interrupted')
    monkeypatch.setattr(service,'refresh_request',fail)
    with pytest.raises(RuntimeError):ingest_object_capture(service,packet(service))
    assert service.store.entities()==before and not service.store.records('object_capture')
