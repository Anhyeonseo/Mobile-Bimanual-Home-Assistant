"""Studio profile and reproducible scenarios over the composed synthetic runtime."""
from copy import deepcopy
import json
from pathlib import Path

from .fetch import FetchRequest, InvalidTask, plan_fetch
from .navigation import NavigateRequest, NavigationMap
from .offline_stack import OfflineStack


SCENARIOS = ('success', 'target_missing', 'sensor_loss', 'reboot', 'load_loss',
             *(f'cancel_{i:02}' for i in range(15)))


def load_profile(path):
    path = Path(path).resolve()
    data = json.loads(path.read_text(encoding='utf-8'))
    if data.get('mode') != 'simulation' or type(data.get('schema_version')) is not int or data['schema_version'] != 1:
        raise InvalidTask('simulation profile schema_version 1 required')
    template = (path.parent / data['stack_template']).resolve()
    if template.parent != path.parent:
        raise InvalidTask('stack template must be in the profile directory')
    stack = json.loads(template.read_text(encoding='utf-8'))
    world, map_data, request = data['world'], data['map'], data['request']
    if set(request) != {'schema_version', 'operation', 'request_id', 'request'} or request['operation'] != 'fetch_object':
        raise InvalidTask('fetch request envelope required')
    plan_fetch(FetchRequest.from_dict(request['request']), world)
    navigation = NavigationMap(map_data)
    catalog = stack['catalog']
    catalog.update(map_id=navigation.map_id, map_revision=navigation.revision, places={})
    if set(data['places']) != set(world['places']):
        raise InvalidTask('every semantic place requires an explicit stop configuration')
    for name, place in data['places'].items():
        approach = dict(schema_version=1, operation='navigate_to', request_id='approach-' + name,
                        map_id=navigation.map_id, map_revision=navigation.revision,
                        floor_id=navigation.floor_id, frame_id=navigation.frame_id,
                        point=place['point'], yaw_rad=place['yaw_rad'])
        navigation.plan(NavigateRequest.from_dict(approach), (1.5, 1.5))
        catalog['places'][name] = {
            'approach': approach, 'height_um': place['height_um'],
            'views': [{'id': name + '_front', 'regions': ['surface'],
                       'approach': deepcopy(approach), 'height_um': place['height_um']}]}
    stack['synthetic_scene'] = {key: request['request'][key] for key in ('object_id', 'destination_place')}
    return {'world': world, 'map': map_data, 'stack': stack, 'request': request,
            'profile_path': str(path), 'template_path': str(template)}


def run_scenario(profile, scenario, on_event=None):
    if scenario not in SCENARIOS:
        raise InvalidTask('unknown lab scenario: ' + scenario)
    stack = OfflineStack(deepcopy(profile['stack']), profile['world'], profile['map'])
    stack.hidden = scenario == 'target_missing'
    request = deepcopy(profile['request'])
    request['request_id'] = 'lab-' + scenario
    events, previous = [], None

    def record(plant, result):
        nonlocal previous
        state = {key: result[key] for key in
                 ('status', 'current_step_id', 'completed_steps', 'reason', 'control_owned')}
        state['runtime_fault'] = plant.runtime.fault
        if state != previous:
            event = {'event_id': len(events) + 1, 'at_s': plant.now,
                     'request_id': request['request_id'], 'evidence_source': 'synthetic', **state}
            events.append(event)
            if on_event is not None:
                on_event(event)
            previous = state

    expected = ('SUCCEEDED' if scenario == 'success' else 'FAILED' if scenario == 'target_missing'
                else 'CANCELLED' if scenario.startswith('cancel_') else 'STOP_UNCONFIRMED')
    result = stack.run(request,
                       cancel_step=int(scenario[7:]) if scenario.startswith('cancel_') else None,
                       fault=scenario if scenario in ('sensor_loss', 'reboot', 'load_loss') else None,
                       on_tick=record)
    owned_expected = expected == 'STOP_UNCONFIRMED'
    passed = (result['status'] == expected and result['control_owned'] == owned_expected
              and result['hardware_commands'] == 0 and result['physical_task_completed'] is False)
    if scenario == 'success':
        passed = passed and result['completed_steps'] == 15 and len(result['manipulation_stages']) == 11
    if scenario != 'success':
        passed = passed and result['stop_requests'] > 0
    return {'schema_version': 1, 'scenario': scenario, 'expected_status': expected,
            'passed': passed, 'mode': 'simulation', 'evidence_source': 'synthetic',
            'request': request, 'result': result, 'events': events}
