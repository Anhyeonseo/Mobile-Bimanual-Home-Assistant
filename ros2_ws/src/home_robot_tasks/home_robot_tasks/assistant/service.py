"""L4 assistant orchestration over the existing RobotApplication/TaskExecutor.

Parsing/preview/scan never execute. Explicit execution is admitted through the
existing gateway's token, short-lived permit, journal and single task lease.
"""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import math

from ..fetch import InvalidTask, FetchRequest, plan_fetch
from ..navigation import NavigationMap, identifier
from ..navigation_fusion import TravelEnvelope
from ..stationary_lift import preset_step
from .contracts import Context, Stamp, document, finite
from .language import parse_request, ground_intent
from .meaning import intent_from_meaning, apply_interaction_policy, INTERACTION_POLICY
from .furniture import FurnitureMemory
from .placement import PlacementPlanner, PlacementProfile
from .scanning import ScanManager
from .simulation import SyntheticFeasibility, seed_demo
from ..home_survey import SimulatedHomeSurvey


class AssistantService:
    def __init__(self, app, store, context, clock, *, travel_envelope=None):
        if travel_envelope is not None and not isinstance(travel_envelope, TravelEnvelope):
            raise InvalidTask('typed travel envelope required')
        self.app, self.store, self.context, self.clock = app, store, context, clock
        self.household = None
        self.travel_envelope = travel_envelope
        self.furniture = FurnitureMemory(store, context)
        self.store.add_map(app.map._document)
        self.scan = ScanManager(store, context, busy=lambda: app.lease.owner is not None)
        self.survey = SimulatedHomeSurvey(self)
        old_admission = app.admission
        app.admission = lambda: (not self.scan.active() or self.survey.submitting) and (old_admission is None or old_admission())
        for record in store.records('intent'):
            if record.get('task_request_id') and record['status'] not in {
                    'SUCCEEDED', 'FAILED', 'CANCELLED', 'INTERRUPTED'}:
                record.update(status='INTERRUPTED', reason='server_restarted', executable=False)
                store.save_record('intent', record['request_id'], record)

    def status(self):
        values = self.store.entities(self.context)
        return dict(mode='recorded_home_review' if self.household else 'simulation', hardware_enabled=False, context=self.context.to_dict(),
                    household=self.household.status() if self.household else None,
                    interaction_policy=deepcopy(INTERACTION_POLICY),
                    active_request=self.app.lease.owner, entities=values, scans=self.scan.sessions(),
                    requests=self.store.records('intent'), furniture=[
                        {k: v for k, v in item.items() if k != 'observed_voxels'}
                        for item in self.furniture.instances()], capabilities={
                        'text_input': True, 'voice_input': False, 'scan_replay': self.household is None,
                        'recorded_home_review': self.household is not None, 'object_capture_input':True,
                        'external_meaning_input': True, 'semantic_api_connected': False,
                        'furniture_replay_mapping': True, 'live_furniture_mapping': False,
                        'autonomous_survey_simulation': True, 'autonomous_survey_hardware': False,
                        'mapping_save_exit': True, 'mapping_restart_resume': True,
                        'physical_poweroff': False,
                        'live_slam': False, 'camera_extrinsics_verified': False,
                        'physical_manipulation': False, 'synthetic_workflow': self.household is None},
                    pending=(['실제 새 촬영과 로봇 위치 연결', 'S4 정차·집기·운반 통합 검증'] if self.household else
                             ['실제 설치·센서 사양 확인', '공통 펌웨어 실물 검증', '카메라·관절·리프트 보정', '하중·파지 결과 센서 확인']))

    def attach_household(self, profile):
        from .context_labels import HouseholdMemory
        if self.app.lease.owner or self.scan.active():
            raise InvalidTask('robot_or_scan_busy')
        self.household = HouseholdMemory(self.store, profile, clock_id=self.context.clock_id)
        self.context = self.household.context
        self.app.admission = lambda: False

    def _ground(self, intent, capture_key=None):
        result = ground_intent(intent, self.store, self.context, self.clock())
        if not capture_key or intent['issues']:
            return result
        capture = self.store.record('object_capture', capture_key)
        if not capture or capture['request_id'] != intent['request_id']:
            raise InvalidTask('capture_request_mismatch')
        candidates = [c for c in self.store.candidates(context=self.context,
            category=intent['category'],attributes=intent['attributes'],now_s=self.clock())
            if c['entity']['entity_id'] in capture['entity_ids']]
        result['capture'] = capture
        result['issues'] = [x for x in result['issues'] if x != 'ambiguous_object']
        sources = result['source_candidates']
        if len(sources)==1:
            src = sources[0]
            support_hint = self.store.get(capture['candidate_support_id']) if capture['candidate_support_id'] else None
            candidates = [c for c in candidates if
                (c['entity']['support_id']==src['entity_id'] if src['kind']=='surface' else c['entity']['room_id']==src['entity_id'])
                or (support_hint and (support_hint['entity_id']==src['entity_id'] or support_hint['room_id']==src['entity_id']))]
        result['object_candidates'] = candidates
        if len(candidates)>1:
            result['issues'].append('ambiguous_object')
        if candidates and intent['operation']=='fetch_object' and any(not c['entity']['support_id'] for c in candidates):
            result['issues'].append('object_support_unconfirmed')
        result['status'] = ('NEEDS_CLARIFICATION' if result['issues'] else 'SEARCH_REQUIRED' if not candidates
            else 'REOBSERVE_REQUIRED' if not candidates[0]['fresh'] else 'READY_TO_PLAN')
        return result

    def refresh_request(self, request_id, *, capture_key=None):
        value = self.get(request_id)
        if value.get('task_request_id') or value['status'] in {'CANCELLED','INTERRUPTED','SUCCEEDED','FAILED'}:
            raise InvalidTask('request_already_finalized')
        if capture_key:
            value['capture_key'] = capture_key
            value['selection'].pop('object_id', None)
        result = self._ground(value['intent'], value.get('capture_key'))
        selected = value['selection'].get('object_id')
        if selected:
            matches = [c for c in result['object_candidates'] if c['entity']['entity_id']==selected]
            if len(matches)==1:
                result['object_candidates']=matches
                result['issues']=[x for x in result['issues'] if x!='ambiguous_object']
                result['status']='NEEDS_CLARIFICATION' if result['issues'] else 'READY_TO_PLAN' if matches[0]['fresh'] else 'REOBSERVE_REQUIRED'
            else:
                value['selection'].pop('object_id',None)
        value.update(grounding=result,status=result['status'],revision=value['revision']+1)
        self.store.save_record('intent',request_id,value)
        return value

    def answer(self, request_id, utterance):
        """Resolve a short place/object reply only within this pending question."""
        import re
        from .language import resolve_place, parse_request
        from .contracts import text
        value = self.get(request_id)
        answer = text(utterance, 'answer', 160)
        if parse_request(answer,'answer-check')['operation']=='cancel':
            raise InvalidTask('use_cancel_endpoint')
        if value['status'] != 'NEEDS_CLARIFICATION':
            raise InvalidTask('no_pending_question')
        issues = set(value['grounding']['issues'])
        if 'ambiguous_object' in issues:
            matches = [c['entity'] for c in value['grounding']['object_candidates']
                if answer in [c['entity']['label'],*c['entity']['aliases'],c['entity']['entity_id']]]
            if len(matches)==1:
                return self.clarify(request_id,{'object_id':matches[0]['entity_id']})
        slot = None
        if issues & {'destination_required','concrete_destination_required','unknown_destination','ambiguous_destination','placement_surface_required'}:
            slot='destination_id'
        elif issues & {'unknown_source','ambiguous_source'}:
            slot='source_id'
        if not slot:
            raise InvalidTask('answer_requires_explicit_selection')
        phrase = re.sub(r'(?:에\s*놓아줘|으로\s*옮겨줘|로\s*옮겨줘|으로|로|에)$','',answer).strip()
        matches = resolve_place(phrase,self.store.entities(self.context))
        if len(matches)!=1:
            raise InvalidTask('answer_place_ambiguous' if matches else 'answer_place_unknown')
        return self.clarify(request_id,{slot:matches[0]['entity_id']})

    def household_review(self, request_id):
        value = self.refresh_request(request_id)
        ground = value['grounding']
        places = ground['destination_candidates']
        sources = ground['source_candidates']
        from .language import NOUNS
        target = NOUNS.get(value['intent']['category'], ('물체',))[0]
        summary = f"{value['utterance']} — 현재 기록과 관측을 확인합니다."
        if len(places)==1:
            summary = f"{self._place_label(sources[0])+'에서 ' if len(sources)==1 else ''}{target}을 {self._place_label(places[0])}에 옮기는 요청입니다."
        return dict(request_id=request_id,mode='recorded_home_review',preview_id=None,
            confirmation={'required':True,'summary':summary,'scope':'one_complete_object_transfer'},
            placement={'status':'PHYSICAL_VALIDATION_PENDING','candidates':[],'evaluated':0},
            grounding=ground, executable=False, hardware_commands=0,physical_task_completed=False,
            note='실제 집의 저장 지도와 관측 후보를 연결한 검토입니다. 정차·집기 경로와 실물 실행 승인은 아직 만들지 않았습니다.')

    def refresh_demo(self):
        if self.app.lease.owner or self.scan.active():
            raise InvalidTask('robot_or_scan_busy')
        return {'updated_entities': seed_demo(self.store, self.context, self.clock()), 'mode': 'simulation'}

    def observe_furniture(self, observation):
        if self.app.lease.owner:
            raise InvalidTask('robot_busy')
        return self.furniture.observe(observation, now_s=self.clock())

    def prepare(self, utterance, request_id, *, meaning=None, parser='external_semantic_api_v1'):
        identifier(request_id, 'request_id')
        # External inference happens outside the gateway. Both paths converge
        # on the same grounding, preview, lease and execution validation.
        intent = (parse_request(utterance, request_id) if meaning is None else
                  intent_from_meaning(utterance, request_id, meaning, parser=parser))
        intent = apply_interaction_policy(intent)
        previous = self.store.record('intent', request_id)
        if previous:
            if (previous['utterance'] != utterance or
                    previous.get('input_meaning') != meaning or
                    (meaning is not None and previous['intent']['parser'] != parser)):
                raise InvalidTask('request_id_conflict')
            return previous
        if len(self.store.records('intent')) >= 128 and intent['operation'] != 'cancel':
            raise InvalidTask('intent_history_full')
        result = self._ground(intent)
        value = dict(request_id=request_id, utterance=utterance, intent=intent, grounding=result,
                     input_meaning=deepcopy(meaning),
                     status=result['status'], selection={}, revision=1, executable=False,
                     task_request_id=None, physical_task_completed=False)
        self.store.save_record('intent', request_id, value)
        return value

    def get(self, request_id):
        value = self.store.record('intent', identifier(request_id, 'request_id'))
        if value is None:
            raise InvalidTask('unknown_assistant_request')
        return value

    def clarify(self, request_id, selection):
        value = self.get(request_id)
        if value['task_request_id'] is not None or value['status'] in {'INTERRUPTED', 'CANCELLED'}:
            raise InvalidTask('request_already_finalized')
        if not isinstance(selection, dict) or not selection or set(selection) - {'object_id', 'destination_id', 'source_id'}:
            raise InvalidTask('invalid clarification')
        for key in selection.values(): identifier(key, 'selected entity')
        merged = {**value['selection'], **selection}
        intent = deepcopy(value['intent'])
        for field, phrase in (('destination_id', 'destination_phrase'), ('source_id', 'source_phrase')):
            if field in merged:
                selected = self.store.get(merged[field])
                if not selected or selected['kind'] not in {'room', 'surface'} or selected not in self.store.entities(self.context):
                    raise InvalidTask('selected_place_not_in_current_map')
                intent[phrase] = selected['entity_id']
        if 'destination_id' in merged:
            intent['issues'] = [i for i in intent['issues'] if i not in {'destination_required','concrete_destination_required'}]
        grounding = self._ground(intent, value.get('capture_key'))
        if 'object_id' in merged:
            chosen = [c for c in grounding['object_candidates'] if c['entity']['entity_id'] == merged['object_id']]
            if len(chosen) != 1:
                raise InvalidTask('selected_object_does_not_match_request')
            grounding['object_candidates'] = chosen
            grounding['issues'] = [i for i in grounding['issues'] if i != 'ambiguous_object']
            if not grounding['issues']:
                grounding['status'] = 'READY_TO_PLAN' if chosen[0]['fresh'] else 'REOBSERVE_REQUIRED'
        value.update(intent=intent, grounding=grounding, selection=merged,
                     status=grounding['status'], revision=value['revision']+1)
        self.store.save_record('intent', request_id, value)
        return value

    def _resolved(self, request_id):
        value = self.get(request_id)
        if value['task_request_id'] is not None or value['status'] in {'INTERRUPTED', 'CANCELLED'}:
            raise InvalidTask('request_already_finalized')
        grounding = self._ground(value['intent'], value.get('capture_key'))
        selected = value['selection'].get('object_id')
        if selected:
            matches = [c for c in grounding['object_candidates'] if c['entity']['entity_id'] == selected]
            if len(matches) != 1:
                raise InvalidTask('selected_object_changed')
            grounding['object_candidates'] = matches
            grounding['issues'] = [i for i in grounding['issues'] if i != 'ambiguous_object']
            grounding['status'] = ('READY_TO_PLAN' if matches[0]['fresh'] else 'REOBSERVE_REQUIRED') if not grounding['issues'] else 'NEEDS_CLARIFICATION'
        if grounding['status'] != 'READY_TO_PLAN':
            raise InvalidTask(grounding['status'].lower())
        if value['intent']['operation'] != 'fetch_object':
            raise InvalidTask('execution_supports_fetch_only_use_navigation_endpoint')
        objects, destinations = grounding['object_candidates'], grounding['destination_candidates']
        if len(objects) != 1 or len(destinations) != 1:
            raise InvalidTask('unambiguous_target_and_destination_required')
        target, destination = objects[0]['entity'], destinations[0]
        if target['source'] != 'simulation' or destination['source'] != 'simulation':
            raise InvalidTask('physical_execution_not_commissioned')
        if not target['position_m'] or not target['room_id'] or not target['support_id']:
            raise InvalidTask('target_location_required')
        return value, target, destination

    def _world(self):
        values = self.store.entities(self.context)
        rooms = {v['entity_id']: {'search_locations': []} for v in values if v['kind'] == 'room'}
        places = {}
        for v in values:
            if v['kind'] == 'surface' and v['room_id'] in rooms:
                places[v['entity_id']] = {'room': v['room_id'], 'label': v['label']}
                rooms[v['room_id']]['search_locations'].append(v['entity_id'])
        return dict(schema_version=1, transport_mode='carry_in_gripper', rooms=rooms, places=places,
                    objects={v['entity_id']: {'label': v['label']} for v in values if v['kind'] == 'object'})

    def preview(self, request_id):
        if self.household:
            return self.household_review(request_id)
        value, target, destination = self._resolved(request_id)
        if not self.app.pose_known:
            raise InvalidTask('localization_required')
        support = self.store.get(target['support_id'])
        if (not support or support['kind'] != 'surface' or support['geometry_kind'] != 'support_surface'
                or support['source'] != 'simulation' or not support['position_m']
                or support['room_id'] != target['room_id']):
            raise InvalidTask('observed_pick_support_required')
        Stamp.parse(support['stamp']).check(self.context, self.clock(), 60)
        support_height = finite(support['position_m'][2] - self.app.map.floor_z, 'support height', 0)
        # The synthetic application has 0.60 m of travel. Only one lift position reaches the search.
        fixed_height = preset_step(support_height, .6) / 100
        profile = PlacementProfile(0, .6, (fixed_height,), (.35, .5, .65), (-math.pi, -math.pi/2, 0, math.pi/2))
        feasibility = SyntheticFeasibility(self.app.map, self.app.xy, target['position_m'])
        candidates = PlacementPlanner(profile, feasibility).plan(target_m=target['position_m'],
            stamp=Stamp.parse(target['stamp']), context=self.context, now_s=self.clock(), maximum_age_s=60,
            scene_revision=self.context.map_revision, grasp_ids=['synthetic-top-grasp'],
            current_xy_yaw=(*self.app.xy, 0), current_height_m=0)
        request = FetchRequest(target['entity_id'], target['room_id'], destination['entity_id'])
        task = plan_fetch(request, self._world())
        fingerprint = hashlib.sha256(document({'request': value['intent'], 'selection': value['selection'],
            'target': target, 'support': support, 'pick_lift_m': fixed_height, 'destination': destination,
            'context': self.context.to_dict(), 'start_xy': list(self.app.xy),
            'interaction_policy': INTERACTION_POLICY}).encode()).hexdigest()
        preview = dict(request_id=request_id, preview_id=fingerprint, task=task, placement=candidates,
                       confirmation=dict(required=True, policy_id=INTERACTION_POLICY['policy_id'],
                           summary=f"{self._place_label(support)}에 있는 {target['label']}을 {self._place_label(destination)}에 옮깁니다.",
                           scope='one_complete_object_transfer'),
                       target=target, destination=destination, mode='simulation', executable=False,
                       physical_task_completed=False, hardware_commands=0,
                       note='합성 기능 결과로 기존 작업 실행기의 흐름을 검사합니다. 실물 IK·접촉 성공 검증이 아닙니다.')
        self.store.save_record('preview', request_id, preview)
        return preview

    def _place_label(self, entity):
        room = self.store.get(entity['room_id']) if entity.get('room_id') else None
        return (room['label']+' ' if room else '') + entity['label']

    def envelope(self, request_id, preview_id):
        if self.household:
            raise InvalidTask('physical_execution_not_commissioned')
        value = self.get(request_id)
        if value['status'] in {'INTERRUPTED', 'CANCELLED'}:
            raise InvalidTask('request_finalized_new_request_required')
        saved = self.store.record('preview', request_id)
        if not saved or saved['preview_id'] != preview_id:
            raise InvalidTask('reviewed_preview_required')
        current = self.preview(request_id)
        if current['preview_id'] != preview_id or current['placement']['status'] != 'CANDIDATES_READY':
            raise InvalidTask('preview_invalidated')
        if self.scan.active():
            raise InvalidTask('scan_active')
        if any(e['state'] in {'carried', 'released_unconfirmed'} for e in self.store.entities(self.context)):
            raise InvalidTask('payload_recovery_required')
        world = self._world()
        request = current['task']['request']
        envelope = dict(schema_version=1, operation='fetch_object', request_id='assistant-'+request_id,
                        request={'task': 'fetch_object', **request})
        identifier(envelope['request_id'], 'execution request_id')
        if self.app.lease.owner:
            raise InvalidTask('robot_busy')
        self.app.world = world
        return envelope

    def execution_requested(self, request_id, result):
        value = self.get(request_id)
        value.update(status='EXECUTION_REQUESTED', task_request_id=result['request_id'])
        self.store.save_record('intent', request_id, value)

    def on_tick(self, task_result):
        if not task_result:
            return
        for value in self.store.records('intent'):
            if value['task_request_id'] != task_result['request_id']:
                continue
            preview = self.store.record('preview', value['request_id'])
            if preview:
                target = self.store.get(preview['target']['entity_id'])
                completed = task_result['completed_steps']
                if target and target['source'] == 'simulation':
                    proof_stamp = Stamp(self.clock(), self.context, 'synthetic-'+value['request_id'])
                    proof = {'verified': True, 'entity_id': target['entity_id'],
                             'capture_id': proof_stamp.capture_id}
                    if completed >= 7 and target['state'] in {'observed', 'placed'} and not value.get('carry_recorded'):
                        target = self.store.mark_state(target['entity_id'], 'carried', proof_stamp,
                            expected_revision=target['revision'], evidence=proof)
                        value['carry_recorded'] = True
                    if completed >= 14 and target['state'] == 'carried':
                        target = self.store.mark_state(target['entity_id'], 'released_unconfirmed', proof_stamp,
                            expected_revision=target['revision'], evidence=proof)
                    if completed >= 15 and target['state'] == 'released_unconfirmed':
                        destination = preview['destination']
                        self.store.mark_state(target['entity_id'], 'placed', proof_stamp,
                            expected_revision=target['revision'], evidence=proof,
                            support_id=destination['entity_id'], position_m=destination['position_m'])
            value.update(status=task_result['status'], execution=task_result,
                         physical_task_completed=False, executable=False)
            self.store.save_record('intent', value['request_id'], value)

    def activate_scan(self, session_id, now_s):
        value = self.scan.get(session_id)
        if value['status'] != 'SAVED' or self.app.lease.owner or self.scan.active():
            raise InvalidTask('saved_idle_scan_required')
        nav = NavigationMap(self.scan.navigation_document(session_id))
        if self.travel_envelope is None:
            raise InvalidTask('current_travel_envelope_required')
        if value['navigation_build']['envelope'] != asdict(self.travel_envelope):
            raise InvalidTask('scan_robot_geometry_changed_rebuild_navigation_map')
        self.app.replace_map(nav, now_s)
        self.app.pose_known = False
        self.context = Context(nav.map_id, nav.revision, self.context.calibration_id, self.context.clock_id, nav.frame_id)
        self.scan.context = self.context
        self.furniture.context = self.context
        return {'status': 'LOCALIZATION_REQUIRED', 'context': self.context.to_dict(), 'motion_authorized': False}
