"""Offline S5 -> gateway -> composed S4 fetch runtime, with synthetic devices.

Unlike the small UI demo, this runs capture, search, payload, pick/place phases
and the shared stop coordinator. It neither imports a serial client nor opens
devices. Geometric reach-shell and sensor fixtures are deliberately identified.
"""
from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[2]
SCENARIOS = ('success', 'cancel_before_execute', 'cancel_carry',
             'sensor_loss', 'load_loss', 'reboot', 'pick_failure', 'place_failure')
UTTERANCE = '거실 소파에 있는 리모컨을 침실 협탁에 놓아줘'


def read(name):
    return json.loads((ROOT/'config'/name).read_text())


def seed_runtime_world(store, context, now):
    from home_robot_tasks.assistant.contracts import Stamp, make_entity
    stamp = Stamp(now, context, 'synthetic-language-scene')
    values = [
        make_entity('living_room', 'room', '거실', 'living_room', stamp, source='simulation'),
        make_entity('bedroom', 'room', '침실', 'bedroom', stamp, source='simulation'),
        make_entity('sofa', 'surface', '소파', 'sofa', stamp, aliases=['거실 소파'],
            room_id='living_room', position_m=[2.,2.,.8], extent_m=[.6,.6,.04],
            geometry_kind='support_surface', source='simulation'),
        make_entity('bedroom_drop_zone', 'surface', '협탁', 'nightstand', stamp,
            aliases=['침실 협탁'], room_id='bedroom', position_m=[6.,4.,.8],
            extent_m=[.6,.6,.04], geometry_kind='support_surface', source='simulation'),
        make_entity('remote_control', 'object', '리모컨', 'remote_control', stamp,
            room_id='living_room', support_id='sofa', position_m=[2.,2.,.82],
            extent_m=[.16,.04,.02], geometry_kind='bbox_center', source='simulation'),
    ]
    for value in values:
        store.put(value)


def rehearse(scenario='success', *, language_url=None, guarded_navigation=True):
    if scenario not in SCENARIOS:
        raise ValueError('unknown offline scenario')
    from home_robot_tasks.offline_stack import OfflineStack
    from home_robot_tasks.gateway import Gateway, GatewayStore
    from home_robot_tasks.assistant.contracts import Context
    from home_robot_tasks.assistant.service import AssistantService
    from home_robot_tasks.assistant.world import WorldStore
    from home_robot_tasks.skill_ports import PortResult

    # Inference is completed before creating the fake-time runtime. No control
    # loop waits for model inference; the production language queue is tested
    # separately by test_local_language.py.
    inference = None
    if language_url:
        from home_perception.language.local_model import LocalLanguageModel
        inference = LocalLanguageModel(language_url).extract(UTTERANCE, 'full-runtime')
    plant = OfflineStack(read('fetch_stack.simulation.json'), read('home.example.json'),
        read('navigation_map.simulation.json'), joint_placement=True,
        guarded_navigation=guarded_navigation)
    app = plant.runtime.app
    context = Context(app.map.map_id, app.map.revision, plant.camera.policy.calibration_id,
                      'offline-clock')
    world = WorldStore()
    service = AssistantService(app, world, context, plant.clock)
    seed_runtime_world(world, context, plant.now)
    gateway = Gateway(app, 'offline-rehearsal-key-'*3, GatewayStore(':memory:'), assistant=service)
    gateway.now = plant.clock
    auth = 'Bearer '+gateway.token
    trace = []
    try:
        if inference:
            from home_robot_tasks.assistant.local_language import meaning_from_slots
            prepared = service.prepare(UTTERANCE, 'full-runtime',
                meaning=meaning_from_slots(inference['extraction']), parser='local_language_v1')
        else:
            code, prepared = gateway.dispatch('POST', '/v1/assistant/requests', auth,
                {'request_id':'full-runtime', 'utterance':UTTERANCE})
            if code != 200:
                raise RuntimeError(f'request admission: {code}')
        preview = service.preview('full-runtime')
        if app.tasks or any(p.starts for p in plant.ports.values()):
            raise RuntimeError('preview started a device phase')
        if scenario == 'cancel_before_execute':
            gateway.dispatch('POST', '/v1/assistant/requests', auth,
                {'request_id':'cancel-rehearsal', 'utterance':'취소'})
            result = service.get('full-runtime')
            passed = result['status']=='CANCELLED' and not app.tasks
            execution = None
        else:
            if scenario in {'pick_failure', 'place_failure'}:
                jaw = plant.ports['gripper']
                original = jaw.poll
                action = 'close' if scenario=='pick_failure' else 'open'
                def fail_selected(goal):
                    entry = jaw.entries[goal]
                    # place_open must fail only after a successful pick.
                    if (entry['parameters']['action']==action and
                            (scenario=='pick_failure' or plant.held)):
                        entry['state']='FAILED'
                        return PortResult('FAILED', 'injected gripper failure')
                    return original(goal)
                jaw.poll = fail_selected
            permit = gateway.dispatch('GET', '/v1/permit', auth, None)[1]['permit']
            code, admitted = gateway.dispatch('POST',
                '/v1/assistant/requests/full-runtime/execute', auth,
                {'preview_id':preview['preview_id']}, permit)
            if code != 202:
                raise RuntimeError(f'execution admission: {code} {admitted}')
            def observe(value):
                service.on_tick(value)
                key = (value['status'], value['completed_steps'])
                if not trace or key != (trace[-1]['status'], trace[-1]['completed_steps']):
                    trace.append(dict(observed_s=plant.now, status=key[0], completed_steps=key[1]))
            execution = plant.run_submitted(admitted['request_id'],
                cancel_step=8 if scenario=='cancel_carry' else None,
                fault=scenario if scenario in {'sensor_loss','load_loss','reboot'} else None,
                on_tick=observe)
            result = service.get('full-runtime')
            expected = ('SUCCEEDED' if scenario=='success' else 'CANCELLED' if scenario=='cancel_carry'
                else 'FAILED' if scenario in {'pick_failure','place_failure'} else 'STOP_UNCONFIRMED')
            passed = result['status']==expected
        return dict(schema_version=1, scenario=scenario, passed=passed, mode='offline_composed_runtime',
            hardware_commands=0, physical_task_completed=False,
            confirmation_source='automated_simulation_test',
            utterance=UTTERANCE, inference=inference, prepared_status=prepared['status'],
            result=result, execution=execution, trace=trace, object=world.get('remote_control'),
            limitations=['synthetic RGB-D/detection/payload feedback',
                'synthetic reach-shell alignment and arm planning; not 5-DOF/STL qualification',
                'guarded navigation uses an in-memory device fixture; no Nav2 or controller connection'],
            guarded_navigation=guarded_navigation)
    finally:
        gateway.store.db.close()
        world.close()
