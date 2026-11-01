"""L5 bounded Korean/English parser and L3B reference resolver.

No remote model or microphone is opened. Ambiguity/unsupported constraints are
returned as data; they never become invented objects, destinations or motion.
The production direction is API semantic extraction through meaning.py; this
parser is the offline development fallback, not the final language approach.
"""
from copy import deepcopy
import re

from ..fetch import InvalidTask
from ..navigation import identifier
from .contracts import text

NOUNS = {
    'marker': ('마커펜', '마커 펜', '매직펜', '마커', 'black marker pen', 'marker pen', 'marker'),
    'cup': ('컵', '머그컵', '머그', 'mug', 'cup'),
    'remote_control': ('리모컨', '리모콘', 'remote control', 'remote'),
    'bottle': ('물병', '병', 'bottle'),
    'scissors': ('가위', 'scissors'),
    'screwdriver': ('드라이버', 'screwdriver'),
}
COLORS = {
    'black': ('검은색', '검정색', '검은', '검정', 'black'),
    'white': ('흰색', '하얀색', '하얀', '흰', 'white'),
    'blue': ('파란색', '파란', '파랑', 'blue'),
    'red': ('빨간색', '빨간', '빨강', 'red'),
    'green': ('초록색', '초록', '녹색', 'green'),
}
OPERATIONS = {'fetch_object', 'find_object', 'navigate_to', 'cancel'}


def _matches(phrase, names):
    # Korean phrases allow particles directly after a noun; English uses words.
    p = phrase.lower()
    return [name for name in names if (name in p if re.search('[가-힣]', name)
            else re.search(r'(?<!\w)' + re.escape(name) + r'(?!\w)', p))]


def _remove_aliases(phrase, names):
    for name in sorted(set(names), key=len, reverse=True):
        pattern = re.escape(name) if re.search('[가-힣]', name) else r'(?<!\w)'+re.escape(name)+r'(?!\w)'
        phrase = re.sub(pattern, ' ', phrase, flags=re.I)
    return phrase


def parse_request(utterance, request_id):
    identifier(request_id, 'request_id')
    raw = text(utterance, 'utterance', 512)
    phrase = re.sub(r'\s+', ' ', raw.lower()).strip(' .!?')
    result = dict(schema_version=1, request_id=request_id, utterance=raw,
                  operation=None, category=None, attributes={}, source_phrase='',
                  destination_phrase='', issues=[], parser='bounded_ko_en_v1')
    if phrase in {'취소', '취소해', '취소해줘', '멈춰', '멈춰줘', '그만', 'stop', 'cancel'}:
        result['operation'] = 'cancel'
        return result
    if re.search(r'말고|말아|하지\s*마|놓지\s*마|집지\s*마|가져오지|않는|아닌|빼고|제외|그리고|다음에|둘 다|모두|두 개|세 개|\b(?:not|except|all|then)\b', phrase):
        result['issues'].append('unsupported_negation_or_multi_action')
        return result
    delivery = re.search(r'(가져다\s*줘|가져다\s*주세요|가져와\s*줘|가져와|옮겨\s*줘|옮겨\s*주세요|놓아\s*줘|놓아\s*주세요|놔\s*줘|\bbring\b|\bput\b|\bmove\b)', phrase)
    finding = re.search(r'찾아\s*줘|찾아\s*주세요|어디\s*있|\bfind\b', phrase)
    navigating = re.search(r'이동해\s*줘|이동해|가\s*줘|가\s*주세요|\bgo to\b', phrase)
    if delivery:
        result['operation'] = 'fetch_object'
    elif finding:
        result['operation'] = 'find_object'
    elif navigating:
        result['operation'] = 'navigate_to'
        result['destination_phrase'] = re.sub(r'(으로|로|에)?\s*(이동해\s*줘|이동해|가\s*줘|가\s*주세요)$', '', phrase).strip()
        result['destination_phrase'] = re.sub(r'^go to\s+', '', result['destination_phrase'])
        return result
    else:
        result['issues'].append('unsupported_action')
        return result
    object_part = phrase
    split = re.search(r'(.+?)(?:을|를)\s+(.+)', phrase)
    if split:
        object_part, rest = split.group(1), split.group(2)
        if result['operation'] == 'fetch_object':
            dest = re.search(r'^(.+?)(?:위에|으로|로|에)\s*(?:가져다|옮겨|놓아|놔)', rest)
            if dest:
                result['destination_phrase'] = dest.group(1).strip()
    elif result['operation'] == 'fetch_object':
        dest = re.search(r'\bto\s+(.+)$', phrase)
        if dest:
            result['destination_phrase'] = dest.group(1).strip()
            object_part = phrase[:dest.start()]
    source = re.split(r'(?:에\s*있는|위에\s*있는|에\s*놓인|위의)\s*', object_part, maxsplit=1)
    if len(source) == 2:
        result['source_phrase'], object_part = source[0].strip(), source[1].strip()
        if result['source_phrase'].endswith(' 위'):
            result['source_phrase'] = result['source_phrase'][:-2]
    categories = [key for key, names in NOUNS.items() if _matches(object_part, names)]
    # Prefer longer overlapping aliases (머그컵 and 컵 refer to the same class).
    if len(categories) != 1:
        result['issues'].append('object_category_required' if not categories else 'multiple_object_categories')
    else:
        result['category'] = categories[0]
    colors = [key for key, names in COLORS.items() if _matches(object_part, names)]
    if len(colors) > 1:
        result['issues'].append('ambiguous_color')
    elif colors:
        result['attributes']['color'] = colors[0]
    residual = _remove_aliases(object_part, [n for names in (*NOUNS.values(), *COLORS.values()) for n in names])
    residual = re.sub(r'(?:가져다|가져와|옮겨|놓아|놔|찾아)\s*(?:주세요|줘)?|어디\s*있(?:어|어요)?', ' ', residual)
    residual = re.sub(r'\b(?:please|the|a|an|bring|put|move|find)\b', ' ', residual)
    residual = re.sub(r'(?<![가-힣])(?:을|를|좀)(?![가-힣])', ' ', residual)
    if residual.strip(' .!?'):
        result['issues'].append('unsupported_object_constraint')
    if result['operation'] == 'fetch_object' and not result['destination_phrase']:
        result['issues'].append('destination_required')
    return result


def validate_intent(value):
    # Model adapters must use this exact contract, never tool calls or raw XYZ.
    fields = {'schema_version', 'request_id', 'utterance', 'operation', 'category',
              'attributes', 'source_phrase', 'destination_phrase', 'issues', 'parser'}
    if not isinstance(value, dict) or set(value) != fields or type(value['schema_version']) is not int or value['schema_version'] != 1:
        raise InvalidTask('invalid intent contract')
    identifier(value['request_id'], 'request_id')
    text(value['utterance'], 'utterance', 512)
    if value['operation'] is not None and (not isinstance(value['operation'], str) or value['operation'] not in OPERATIONS):
        raise InvalidTask('unsupported intent operation')
    if value['category'] is not None and (not isinstance(value['category'], str) or value['category'] not in NOUNS):
        raise InvalidTask('unsupported object category')
    if not isinstance(value['attributes'], dict) or set(value['attributes']) - {'color'}:
        raise InvalidTask('unsupported intent attribute')
    if 'color' in value['attributes'] and (not isinstance(value['attributes']['color'], str) or value['attributes']['color'] not in COLORS):
        raise InvalidTask('unsupported color')
    for key in ('source_phrase', 'destination_phrase'):
        if not isinstance(value[key], str) or len(value[key]) > 160 or any(ord(c) < 32 for c in value[key]):
            raise InvalidTask('invalid place phrase')
    if not isinstance(value['issues'], list) or len(value['issues']) > 16:
        raise InvalidTask('invalid clarification reasons')
    for issue in value['issues']:
        identifier(issue, 'issue')
    text(value['parser'], 'parser', 80)
    return deepcopy(value)


def resolve_place(phrase, entities, *, kind=None):
    if not phrase:
        return []
    matches = []
    recognized_names = []
    for value in entities:
        if value['kind'] not in {'room', 'surface'} or (kind and value['kind'] != kind):
            continue
        names = [value['label'], *value['aliases'], value['entity_id']]
        lengths = [len(n) for n in names if n.lower() in phrase.lower()]
        if lengths:
            matches.append((max(lengths), value))
            recognized_names.extend(names)
    # A recognized name does not authorize silently dropping 'beside', 'under',
    # or an unknown room. Only the supported room + surface phrase is accepted.
    residual = _remove_aliases(phrase, recognized_names)
    if re.sub(r'\bthe\b|위|에', ' ', residual).strip(' .'):
        return []
    # If a room is stated, it constrains surfaces with otherwise identical names.
    rooms = {v['entity_id'] for _, v in matches if v['kind'] == 'room'}
    surfaces = [(score, v) for score, v in matches if v['kind'] == 'surface' and
                (not rooms or v['room_id'] in rooms)]
    selected = surfaces if surfaces and kind != 'room' else [(s, v) for s, v in matches if v['kind'] == 'room']
    if not selected:
        return []
    best = max(score for score, _ in selected)
    return [deepcopy(v) for score, v in selected if score == best]


def ground_intent(intent, store, context, now_s):
    intent = validate_intent(intent)
    issues = list(intent['issues'])
    result = dict(intent=intent, status='NEEDS_CLARIFICATION', issues=issues,
                  object_candidates=[], destination_candidates=[], source_candidates=[],
                  executable=False, motion_authorized=False)
    if issues or intent['operation'] == 'cancel':
        if intent['operation'] == 'cancel' and not issues:
            result['status'] = 'CANCEL_REQUEST'
        return result
    values = store.entities(context)
    source = resolve_place(intent['source_phrase'], values)
    if any(v['state'] == 'unconfirmed' for v in source):
        issues.append('source_label_unconfirmed')
    if intent['source_phrase'] and len(source) != 1:
        issues.append('unknown_source' if not source else 'ambiguous_source')
    result['source_candidates'] = source
    destinations = resolve_place(intent['destination_phrase'], values)
    if any(v['state'] == 'unconfirmed' for v in destinations):
        issues.append('destination_label_unconfirmed')
    result['destination_candidates'] = destinations
    if intent['operation'] in {'fetch_object', 'navigate_to'}:
        if len(destinations) != 1:
            issues.append('unknown_destination' if not destinations else 'ambiguous_destination')
        elif intent['operation'] == 'fetch_object' and destinations[0]['kind'] != 'surface':
            issues.append('placement_surface_required')
    if intent['operation'] != 'navigate_to':
        src = source[0] if len(source) == 1 else None
        room = (src['entity_id'] if src['kind'] == 'room' else src['room_id']) if src else None
        support = src['entity_id'] if src and src['kind'] == 'surface' else None
        candidates = store.candidates(context=context, category=intent['category'],
            attributes=intent['attributes'], room_id=room, support_id=support, now_s=now_s)
        result['object_candidates'] = candidates
        if len(candidates) > 1:
            issues.append('ambiguous_object')
        elif not candidates:
            result['status'] = 'SEARCH_REQUIRED'
        elif not candidates[0]['fresh']:
            result['status'] = 'REOBSERVE_REQUIRED'
        else:
            result['status'] = 'READY_TO_PLAN'
    else:
        result['status'] = 'READY_TO_PLAN'
    if issues:
        result['status'] = 'NEEDS_CLARIFICATION'
    return result
