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
        return 200, service.status()
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
            if 'cancel_target_request_id' not in result:
                result['cancel_target_request_id'] = gateway.app.lease.owner
                service.store.save_record('intent', result['request_id'], result)
            owner = result['cancel_target_request_id']
            if owner:
                _, stopped = gateway._dispatch('POST', '/v1/requests/'+owner+'/cancel', authorization, {}, None)
                return 200, {**result, 'cancel_result': stopped}
            result['status'] = 'NO_ACTIVE_TASK'
            service.store.save_record('intent', result['request_id'], result)
            return 200, result
        return 200, result
    if method == 'POST' and path == '/v1/assistant/scans':
        fields(body, ('session_id', 'name', 'mode'))
        return 201, service.scan.start(**body)
    parts = path.strip('/').split('/')
    if len(parts) == 5 and parts[:3] == ['v1', 'assistant', 'requests'] and method == 'POST':
        request_id, action = parts[3:]
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
