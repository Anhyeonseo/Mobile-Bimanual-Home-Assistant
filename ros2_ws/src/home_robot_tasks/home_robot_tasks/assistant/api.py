"""Authorized HTTP routing; called under the existing Gateway lock."""
from ..fetch import InvalidTask


def fields(body, expected):
    if not isinstance(body, dict) or set(body) != set(expected):
        raise InvalidTask('invalid endpoint fields')
    return body


def dispatch(gateway, method, path, body, authorization, permit):
    service = gateway.assistant
    if service is None:
        return 404, {'error': 'assistant_not_configured'}
    if method == 'GET' and path == '/v1/assistant/state':
        state = service.status()
        state['capabilities']['local_language_configured'] = gateway.language is not None
        state['language_jobs'] = service.store.records('language')
        return 200, state
    if method == 'GET' and path.startswith('/v1/assistant/household/evidence/'):
        if service.household is None:
            raise InvalidTask('household_not_configured')
        return 200, service.household.evidence(int(path.rsplit('/',1)[1]))
    if method == 'POST' and path == '/v1/assistant/household/names':
        fields(body, ('entity_id','label','aliases','expected_revision'))
        if service.household is None:
            raise InvalidTask('household_not_configured')
        result = service.household.name(**body)
        # Re-evaluate pending requests after a name choice; never execute.
        for value in service.store.records('intent'):
            if not value.get('task_request_id') and value['status'] not in {'CANCELLED','INTERRUPTED','SUCCEEDED','FAILED','CANCEL_REQUEST','NO_ACTIVE_TASK'}:
                service.refresh_request(value['request_id'])
        return 200, result
    if method == 'POST' and path == '/v1/assistant/object-captures':
        from .perception import ingest_object_capture
        return 200, ingest_object_capture(service, body)
    if service.household is not None and (path.startswith('/v1/assistant/scans') or
            path.startswith('/v1/assistant/surveys') or path.startswith('/v1/assistant/demo/') or
            path=='/v1/assistant/localize-simulation'):
        raise InvalidTask('recorded_home_review_only')
    if method == 'POST' and path == '/v1/assistant/language-requests':
        fields(body, ('request_id', 'utterance'))
        from .language import parse_request
        immediate = parse_request(body['utterance'], body['request_id'])
        if immediate['operation'] == 'cancel':
            if gateway.language is not None:
                gateway.language.cancel_pending()
            return dispatch(gateway, method, '/v1/assistant/requests', body, authorization, permit)
        if gateway.language is None:
            raise InvalidTask('local_language_not_configured')
        return 202, gateway.language.submit(body['utterance'], body['request_id'])
    pieces = path.strip('/').split('/')
    if len(pieces) in (4, 5) and pieces[:3] == ['v1', 'assistant', 'language-requests']:
        if gateway.language is None:
            raise InvalidTask('local_language_not_configured')
        if len(pieces) == 4 and method == 'GET':
            return 200, gateway.language.get(pieces[3])
        if len(pieces) == 5 and pieces[4] == 'cancel' and method == 'POST':
            fields(body, ())
            job = gateway.language.cancel(pieces[3])
            if job['status'] == 'READY':
                return dispatch(gateway, method, '/v1/assistant/requests/'+pieces[3]+'/cancel', body, authorization, permit)
            return 200, job
    if method == 'POST' and path == '/v1/assistant/demo/observe':
        fields(body, ())
        return 200, service.refresh_demo()
    if method == 'POST' and path == '/v1/assistant/furniture/observations':
        fields(body, ('observation',))
        return 200, service.observe_furniture(body['observation'])
    if method == 'POST' and path in {'/v1/assistant/requests', '/v1/assistant/interpreted-requests'}:
        interpreted = path.endswith('/interpreted-requests')
        fields(body, ('request_id', 'utterance', 'meaning') if interpreted else ('request_id', 'utterance'))
        if interpreted and not isinstance(body['meaning'], dict):
            raise InvalidTask('extracted meaning required')
        result = service.prepare(body['utterance'], body['request_id'],
                                 meaning=body['meaning'] if interpreted else None)
        if result['status'] == 'CANCEL_REQUEST':
            if gateway.language is not None:
                gateway.language.cancel_pending()
            if 'cancel_target_request_id' not in result:
                result['cancel_target_request_id'] = gateway.app.lease.owner
                service.store.save_record('intent', result['request_id'], result)
            owner = result['cancel_target_request_id']
            if owner:
                _, stopped = gateway._dispatch('POST', '/v1/requests/'+owner+'/cancel', authorization, {}, None)
                return 200, {**result, 'cancel_result': stopped}
            cancelled = []
            for pending in service.store.records('intent'):
                if pending['request_id']==result['request_id'] or pending.get('task_request_id') or pending.get('status') not in {'NEEDS_CLARIFICATION','SEARCH_REQUIRED','REOBSERVE_REQUIRED','READY_TO_PLAN'}:
                    continue
                pending.update(status='CANCELLED',executable=False)
                service.store.save_record('intent',pending['request_id'],pending)
                cancelled.append(pending['request_id'])
            result['status'] = 'CANCELLED' if cancelled else 'NO_ACTIVE_TASK'
            result['cancelled_pending_request_ids'] = cancelled
            service.store.save_record('intent', result['request_id'], result)
            return 200, result
        return 200, result
    if method == 'POST' and path == '/v1/assistant/scans':
        fields(body, ('session_id', 'name', 'mode'))
        return 201, service.scan.start(**body)
    if method == 'POST' and path == '/v1/assistant/surveys':
        fields(body, ('session_id', 'name'))
        previous = service.store.record('scan', body['session_id'])
        if previous is None:
            if not isinstance(permit, str) or gateway.permits.get(permit, 0) <= gateway.now():
                raise InvalidTask('fresh_request_permit_required')
            gateway.permits.pop(permit)
        return 201, service.survey.start(body['session_id'], body['name'], gateway.now())
    parts = path.strip('/').split('/')
    if len(parts) == 5 and parts[:3] == ['v1', 'assistant', 'requests'] and method == 'POST':
        request_id, action = parts[3:]
        if action == 'answer':
            fields(body, ('utterance',))
            from .language import parse_request
            if parse_request(body['utterance'],'answer-check')['operation']=='cancel':
                return dispatch(gateway,method,'/v1/assistant/requests/'+request_id+'/cancel',{},authorization,None)
            return 200, service.answer(request_id, body['utterance'])
        if action == 'refresh':
            fields(body, ())
            return 200, service.refresh_request(request_id)
        if action == 'clarify':
            fields(body, ('selection',))
            return 200, service.clarify(request_id, body['selection'])
        if action == 'preview':
            fields(body, ())
            return 200, service.preview(request_id)
        if action == 'execute':
            fields(body, ('preview_id',))
            record = service.get(request_id)
            rid = 'assistant-'+request_id
            previous = gateway.store.get(rid)
            saved = service.store.record('preview', request_id)
            if not saved or saved['preview_id'] != body['preview_id']:
                raise InvalidTask('reviewed_preview_required')
            if previous:
                expected = dict(schema_version=1, operation='fetch_object', request_id=rid,
                                request={'task': 'fetch_object', **saved['task']['request']})
                if previous[0] != expected:
                    raise InvalidTask('request_id_conflict')
                # Same persisted execution identity stays idempotent after a
                # restart or lost reply even when its old observation is stale.
                result = gateway.app.status(rid) if rid in gateway.app.tasks else previous[1]
                return 200, result
            envelope = service.envelope(request_id, body['preview_id'])
            code, result = gateway._dispatch('POST', '/v1/requests', authorization, envelope, permit)
            if code in {200, 202}:
                try:
                    service.execution_requested(request_id, result)
                except Exception:
                    gateway.latch_fault()
                    raise
            return code, result
        if action == 'cancel':
            fields(body, ())
            value = service.get(request_id)
            if value['task_request_id']:
                return gateway._dispatch('POST', '/v1/requests/'+value['task_request_id']+'/cancel', authorization, {}, None)
            value.update(status='CANCELLED', executable=False)
            service.store.save_record('intent', request_id, value)
            return 200, value
    if len(parts) == 5 and parts[:3] == ['v1', 'assistant', 'scans'] and method == 'POST':
        session_id, action = parts[3:]
        scan = service.scan.get(session_id)
        if scan.get('autonomous') and action in {'pause', 'resume', 'review', 'discard', 'save_exit'}:
            fields(body, ())
            if action == 'resume':
                if not isinstance(permit, str) or gateway.permits.get(permit, 0) <= gateway.now():
                    raise InvalidTask('fresh_request_permit_required')
                gateway.permits.pop(permit)
            return 200, service.survey.action(session_id, action, gateway.now())
        if scan.get('autonomous') and action in {'map', 'observe', 'demo', 'activate'}:
            raise InvalidTask('autonomous_scan_owns_observations')
        if action in {'pause', 'resume', 'review', 'save', 'discard'}:
            fields(body, ())
            return 200, getattr(service.scan, action)(session_id)
        if action == 'map':
            fields(body, ('map_document',))
            return 200, service.scan.ingest_map(session_id, body['map_document'])
        if action == 'observe':
            fields(body, ('observation',))
            return 200, service.scan.observe(session_id, body['observation'], now_s=service.clock())
        if action == 'label':
            fields(body, ('region_id', 'label'))
            return 200, service.scan.label(session_id, body['region_id'], body['label'])
        if action == 'activate':
            fields(body, ())
            return 200, service.activate_scan(session_id, gateway.now())
        if action == 'demo':
            fields(body, ())
            scan = service.scan.get(session_id)
            if scan['mode'] != 'simulation':
                raise InvalidTask('simulation_scan_required')
            service.scan.ingest_map(session_id, gateway.app.map._document)
            context = service.context.to_dict()
            return 200, service.scan.observe(session_id, {'stamp': {
                'observed_s': service.clock(), 'context': context, 'capture_id': 'demo-'+session_id},
                'xy_m': [3, 3], 'categories': ['sofa', 'coffee_table'],
                'surface_ids': ['coffee_table']}, now_s=service.clock())
    if method == 'POST' and path == '/v1/assistant/localize-simulation':
        fields(body, ('xy_m',))
        if not isinstance(body['xy_m'], list) or len(body['xy_m']) != 2:
            raise InvalidTask('XY pair required')
        gateway.app.localize_simulated(tuple(body['xy_m']), gateway.now())
        return 200, {'mode': 'simulation', 'localized': True}
    if method == 'POST' and path == '/v1/assistant/aliases':
        fields(body, ('entity_id', 'aliases', 'expected_revision'))
        if gateway.app.lease.owner:
            raise InvalidTask('robot_busy')
        return 200, service.store.aliases(body['entity_id'], body['aliases'], body['expected_revision'])
    return 404, {'error': 'unknown_assistant_endpoint'}
