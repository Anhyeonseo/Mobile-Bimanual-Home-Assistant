import json

from home_perception.language.task_parser import parse_reply


def parse(reply, text='리모컨 집어줘'):
    return parse_reply(json.dumps(reply) if isinstance(reply, dict) else reply, 'r', 'u', text)


def test_valid_reply_fills_slots():
    item = parse({'target': '리모컨', 'source': '소파', 'destination': '협탁'}, '왼쪽 소파 위 리모컨을 협탁으로 옮겨줘')
    assert (item.action, item.target, item.source, item.destination, item.location) == ('move', '리모컨', '소파', '협탁', 'left')
    assert item.unresolved == []


def test_slots_missing_from_the_request_are_dropped():
    item = parse({'target': '계산기', 'source': '가정용 로봇'}, '가져다줘')
    assert item.target is None and item.source is None and item.unresolved == ['target']


def test_pronoun_is_not_a_target():
    assert parse({'target': '그거'}, '그거 집어줘').target is None


def test_position_inside_a_place_phrase_is_not_a_location_choice():
    assert parse({'target': '컵', 'source': '책상'}, '책상 앞 컵 집어줘').location is None


def test_code_fence_is_accepted():
    assert parse('```json\n{"target": "리모컨"}\n```').target == '리모컨'


def test_invalid_reply_is_unresolved_not_guessed():
    item = parse('모르겠습니다')
    assert item.action is None and 'invalid_reply' in item.unresolved


def test_motion_values_are_refused():
    item = parse({'action': 'pick', 'target': '컵', 'joint_angles': [0, 1]})
    assert item.action is None and 'invalid_reply' in item.unresolved


def test_request_without_a_supported_verb_has_no_action():
    item = parse({'target': '컵'}, '컵 예쁘다')
    assert item.action is None and 'action' in item.unresolved


def test_unsupported_text_overrides_the_model():
    item = parse({'action': 'pick', 'target': '물'}, '컵에 물 따라줘')
    assert item.action is None and 'unsupported_action' in item.unresolved


def test_multiple_actions_are_not_executed_as_one():
    item = parse({'action': 'pick', 'target': '컵'}, '컵 집고 나서 리모컨도 가져와')
    assert item.action is None and 'multiple_actions' in item.unresolved


def test_move_without_destination_is_unresolved():
    item = parse({'target': '컵'}, '컵 옮겨줘')
    assert item.action == 'move' and item.unresolved == ['destination']


def test_placeholder_values_become_none():
    item = parse({'target': '컵', 'source': 'null', 'destination': '소파|침대'}, '컵 집어줘')
    assert item.source is None and item.destination is None


def test_negation_order_overrides_a_swapped_reply():
    item = parse({'target': '수첩', 'excluded': ['충전기']}, '수첩 말고 충전기 가져와')
    assert item.target == '충전기' and item.excluded == ['수첩']


def test_negation_without_a_named_replacement_stays_unresolved():
    item = parse({'target': '수첩', 'excluded': []}, '수첩 말고 가져와')
    assert item.target is None and 'target' in item.unresolved

