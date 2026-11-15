#!/usr/bin/env python3
"""Measure the Korean request parser on the fixed sentence set; text only, no robot."""
import argparse
import collections
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'ros2_ws/src/home_robot_tasks'))
from home_perception.language.task_parser import parse_reply


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


SLOTS = ('action', 'target', 'source', 'destination', 'location')


def normalise(value):
    return value.replace(' ', '') if isinstance(value, str) else value


def score(case, item):
    wrong = [name for name in SLOTS if normalise(getattr(item, name)) != normalise(case[name])]
    if [normalise(v) for v in item.excluded] != [normalise(v) for v in case['excluded']]:
        wrong.append('excluded')
    return wrong


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=Path, default=ROOT / 'config/perception_request_cases.json')
    parser.add_argument('--household-profile', type=Path)
    parser.add_argument('--household-source-map', type=Path)
    parser.add_argument('--endpoint', default='http://127.0.0.1:18767')
    parser.add_argument('--output', type=Path, default=ROOT / 'tmp/request-eval/report.json')
    args = parser.parse_args(argv)
    from home_perception.language.local_model import LocalLanguageModel
    from home_robot_tasks.assistant.local_language import meaning_from_slots
    from home_robot_tasks.assistant.meaning import intent_from_meaning, apply_interaction_policy
    model = LocalLanguageModel(args.endpoint)
    service = None
    if args.household_profile:
        from home_robot_tasks.assistant.context_labels import load_household_profile
        from home_robot_tasks.assistant.contracts import Context
        from home_robot_tasks.assistant.service import AssistantService
        from home_robot_tasks.assistant.world import WorldStore
        from home_robot_tasks.application import RobotApplication
        from home_robot_tasks.fake_robot import FakeRobot
        from home_robot_tasks.navigation import NavigationMap
        nav = NavigationMap(json.loads((ROOT/'config/navigation_map.simulation.json').read_text()))
        app = RobotApplication(json.loads((ROOT/'config/home.example.json').read_text()),nav,FakeRobot())
        service = AssistantService(app, WorldStore(),Context(nav.map_id,nav.revision,'simulation-calibration','language-review'),time.time)
        service.attach_household(load_household_profile(args.household_profile,source_map=args.household_source_map))
    rows, slots_total, slots_right, started = [], 0, 0, time.monotonic()
    by_group = collections.defaultdict(lambda: [0, 0])
    unsafe = []
    cases = json.loads(args.cases.read_text(encoding='utf-8'))
    if not cases:
        parser.error('at least one evaluation case required')
    for index, case in enumerate(cases):
        inference = model.extract(case['text'], f'case-{index}', 'eval')
        reply = inference['reply']
        item = parse_reply(reply, f'case-{index}', 'eval', case['text'])
        intent = apply_interaction_policy(intent_from_meaning(
            case['text'], f'case-{index}', meaning_from_slots(item.to_dict()), parser='local_language_v1'))
        grounding = service.prepare(case['text'],f'case-{index}',meaning=meaning_from_slots(item.to_dict()),parser='local_language_v1')['grounding'] if service else None
        if case['supported']:
            wrong = score(case, item)
            slots_total += len(SLOTS) + 1
            slots_right += len(SLOTS) + 1 - len(wrong)
        else:
            wrong = [] if item.action is None else ['executable']
            if item.action is not None:
                unsafe.append(case['text'])
        by_group[case['group']][0] += not wrong
        by_group[case['group']][1] += 1
        rows.append({'text': case['text'], 'group': case['group'], 'wrong': wrong, 'reply': reply, 'result': item.to_dict(), 'inference': inference, 'intent': intent,
                     'grounding': grounding,
                     'scope_wrong': ('expect_clear' in case and (not intent['issues']) != case['expect_clear'])})
        write_json(args.output, {'complete': False, 'rows': rows})
        if wrong:
            print('FAIL', case['group'], case['text'], wrong, json.dumps(item.to_dict(), ensure_ascii=False)[:150], flush=True)
    summary = {'sentences': len(rows), 'sentences_right': sum(not row['wrong'] for row in rows),
               'slot_accuracy': slots_right / slots_total if slots_total else None, 'unsupported_marked_executable': unsafe,
               'by_group': dict(by_group), 'seconds': time.monotonic() - started,
               'scope_cases': sum('expect_clear' in c for c in cases),
               'scope_wrong': sum(r['scope_wrong'] for r in rows),
               'mean_latency_s': sum(r['inference']['latency_s'] for r in rows)/len(rows),
               'note': 'Slot extraction score is not execution approval; S5 scope may require clarification.'}
    write_json(args.output, {'schema_version': 1, 'complete': True, 'summary': summary, 'rows': rows})
    if service:
        assert not service.app.tasks and service.app.admission() is False
        service.store.close()
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
