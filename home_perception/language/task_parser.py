"""Korean request text -> TaskRequest. The model proposes, this module decides.

The prompt never offers coordinates or joints, and a reply containing them is
refused. Actions and locations come from the request words, and an object slot
must appear in the request, so a small model cannot invent a task. Anything
unsupported stays unresolved rather than being mapped to the nearest action.
"""
# Adapted from yeoeun/perception-offline-lab @ 8da08b05ec65.
import json
import re

from home_perception.contracts import TaskRequest

FORBIDDEN_KEYS = re.compile(r'joint|coordinate|xyz|pose|motor|velocity|angle', re.IGNORECASE)
# Checked on the original text, so a model mistake cannot authorise these tasks.
UNSUPPORTED_REQUEST = re.compile(
    r'따라|붓|끓|요리|볶|데워|설거지|청소|빨래|쓸어|닦아|열어|닫아|잠가|켜줘|꺼줘|버려|부숴|던져|계단|2층|위층|춤|노래'
    r'|손에|건네|쥐어|먹여|물병|뜨거운|칼날|불 ')
NEGATED_ACTION = re.compile(r'(?:가져오|가져가|가져다주|옮기|집|잡|놓|움직이|실행하)지\s*(?:마|말)|하지\s*마')
MULTIPLE_ACTIONS = re.compile(r'(고 나서|ㄴ 다음|[은는던준온한]\s*다음|[은는던준온한]\s*뒤|다음에|그리고 |그러고|(?:집|잡|놓|찾|옮기|가져오|가져가)고\s+|둘\s*다|두\s*개|세\s*개|모두)')
PRONOUNS = ('그거', '저거', '이거', '그것', '저것', '이것')
ACTION_WORDS = (('find', r'찾|어디(?!로|든)'), ('pick', r'들어올'), ('move', r'옮기|옮겨|놓아|놔|올려|둬|두어'),
                ('fetch', r'가져|갖다'), ('pick', r'집어|집고|잡아|잡고|들어|집$|잡$'))
NEGATION = re.compile(r'^(.+?)\s*(?:말고|아니고|아니라)\s+(.+)$')
LOCATION_WORDS = (('left', '왼쪽'), ('right', '오른쪽'), ('front', '앞'), ('back', '뒤'))

PROMPT = """한국어 로봇 요청에서 물체와 장소를 JSON으로 뽑는다. 문장에 있는 단어만 쓰고, 없으면 null이다.
형식: {"target": 물체 또는 null, "source": 물체가 지금 있는 곳 또는 null, "destination": 놓을 곳 또는 null, "excluded": [제외할 물체]}
물체 이름 앞의 색이나 "내" 같은 꾸밈말은 물체에 포함한다. 왼쪽·오른쪽·앞·뒤 같은 위치 단어는 뺀다.
"나한테", "나에게"는 놓을 곳이 아니다. "나"를 destination에 쓰지 않는다.
excluded는 "말고/아니고/아니라/빼고/제외"로 실제 제외한 물체만 넣는다. 그 외에는 항상 []이다.
색깔이나 수량, "하지 마"는 excluded가 아니다. 색깔은 target에 붙여 보존한다.
source에는 물체가 놓인 가구나 장소만 넣는다. 왼쪽/오른쪽/앞/뒤는 장소가 아니며, 제외된 물체도 source가 아니다.
장소의 꾸밈말도 모두 보존한다. 예를 들어 "TV 옆 선반", "식탁 의자"는 각각 하나의 장소 이름이며, 앞부분을 source와 destination으로 나누지 않는다.
동사만 있는 요청은 target이 null이다.

요청: 책 집어줘
{"target": "책", "source": null, "destination": null, "excluded": []}
요청: 빨간 모자 가져와
{"target": "빨간 모자", "source": null, "destination": null, "excluded": []}
요청: 내 책 갖다줘
{"target": "내 책", "source": null, "destination": null, "excluded": []}
요청: 선풍기 위 모자 가져와
{"target": "모자", "source": "선풍기", "destination": null, "excluded": []}
요청: 나한테 모자 가져와
{"target": "모자", "source": null, "destination": null, "excluded": []}
요청: 모자를 의자로 옮겨줘
{"target": "모자", "source": null, "destination": "의자", "excluded": []}
요청: 의자에 있는 책을 선풍기 위에 놔줘
{"target": "책", "source": "의자", "destination": "선풍기", "excluded": []}
요청: 왼쪽 모자 집어줘
{"target": "모자", "source": null, "destination": null, "excluded": []}
요청: 모자 어디 뒀지
{"target": "모자", "source": null, "destination": null, "excluded": []}
요청: 모자 말고 책 가져다줘
{"target": "책", "source": null, "destination": null, "excluded": ["모자"]}
요청: 파란 모자 아니라 빨간 모자 가져와
{"target": "빨간 모자", "source": null, "destination": null, "excluded": ["파란 모자"]}
요청: 가져다줘
{"target": null, "source": null, "destination": null, "excluded": []}
요청: """


def _strip(text):
    return re.sub(r'^```(json)?|```$', '', text.strip(), flags=re.MULTILINE).strip()


def _clean(value):
    if value is None:
        return None
    if not isinstance(value, str):
        raise ValueError('slot must be text')
    value = value.strip()
    return None if value.lower() in ('', 'null', 'none') or '|' in value else value


def _in_text(value, text):
    """A slot the speaker never said is the model's invention."""
    if value is None or value.startswith(PRONOUNS):
        return None
    return value if value.replace(' ', '') in text.replace(' ', '') else None


def request_action(text):
    return next((name for name, words in ACTION_WORDS if re.search(words, text)), None)


def request_location(text):
    """Only a leading position word chooses among objects; "책상 앞" names a place."""
    for name, word in LOCATION_WORDS:
        if re.match(rf'{word}(쪽|에 있는)?\s', text):
            return name
    return None


def apply_negation(slots, excluded, text):
    """In "A 말고 B", the model may swap A and B; the sentence order decides."""
    match = NEGATION.match(text)
    if not match:
        return slots, excluded
    mentioned = [value for value in (*excluded, slots['target']) if value]
    before = [value for value in mentioned if value.replace(' ', '') in match[1].replace(' ', '')]
    after = [value for value in mentioned if value.replace(' ', '') in match[2].replace(' ', '')]
    return {**slots, 'target': max(after, key=len) if after else None}, before or excluded


def retain_literal_modifiers(slots, source_text):
    """Do not lose explicit ownership, size or colour next to a model noun."""
    target = slots['target']
    if not target:
        return slots
    matches = list(re.finditer(re.escape(target), source_text))
    if len(matches) != 1:
        return slots
    prefix = source_text[:matches[0].start()]
    modifier = re.search(r'(?<![가-힣])((?:(?:내|제|나의|저의|우리|큰|작은|검은색|검정색|검은|검정|흰색|하얀색|하얀|흰|빨간색|빨간|파란색|파란|초록색|초록)\s+)+)$', prefix)
    if modifier:
        slots['target'] = modifier[1]+target
    return slots


def normalize_place_roles(slots, source_text):
    """Repair a proposed place only when an explicit destination particle agrees.

    No place name is invented. Repeated places, compound commands and unclear
    attachment keep the model output for clarification/confirmation.
    """
    place, target = slots['source'], slots['target']
    if not target or MULTIPLE_ACTIONS.search(source_text):
        return slots
    if source_text.count(target) != 1:
        return slots
    before, after = source_text.split(target, 1)
    # Explicit "object를 [whole place]로 옮겨" binds the whole phrase.
    # A small model sometimes splits a compound furniture name into two roles.
    destination = re.match(r'^(?:을|를)?\s*(?:좀\s+)?(.{1,80}?)(?:\s*위에|\s*위로|으로|로|에)\s*(?:옮겨|놓아|놔|올려|둬|가져다)', after)
    if destination and not re.search(r'에서|에\s*있는|말고|아니라|그리고|^(?:과|와|하고|이랑|랑)\s|(?:을|를)(?:\s|$)|전부|모든',destination[1]):
        full = destination[1].strip()
        proposed = slots['destination']
        if proposed and full.endswith(proposed):
            slots['destination'] = full
            if place and place in full and place not in before:
                slots['source'] = None
    # Preserve source qualifiers only when they end at an explicit locative.
    source = re.fullmatch(r'\s*(.{1,80}?)(?:에\s*있는|\s*위(?:에\s*있는)?)\s*',before)
    if source and place and source[1].strip().endswith(place) and not re.match(r'왼쪽|오른쪽|앞쪽|뒤쪽',source[1].strip()):
        slots['source'] = source[1].strip()
    place = slots['source']
    if not place or source_text.count(place) != 1:
        return slots
    destination_use = re.search(re.escape(place)+r'\s*(?:위에|위로|으로|로|에)\s*(?:옮겨|놓아|놔|올려|둬|가져다)', after)
    if destination_use and slots['destination'] in (None, place):
        slots['source'], slots['destination'] = None, place
    return slots


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate reply field')
        result[key] = value
    return result


def parse_reply(text, request_id, user_id, source_text):
    """Validate one model reply. The text decides the action; the model fills objects."""
    try:
        if not isinstance(text, str) or len(text) > 8192:
            raise ValueError('bounded text reply required')
        data = json.loads(_strip(text), object_pairs_hook=_unique_pairs)
        if not isinstance(data, dict):
            raise ValueError('object required')
        if set(data) - {'target', 'source', 'destination', 'excluded', 'action'} or any(FORBIDDEN_KEYS.search(key) for key in data):
            raise ValueError('motion values are not allowed')
        if 'excluded' in data and (not isinstance(data['excluded'], list) or len(data['excluded']) > 8):
            raise ValueError('bounded excluded list required')
        slots = {name: _in_text(_clean(data.get(name)), source_text) for name in ('target', 'source', 'destination')}
        excluded = [item for item in (_in_text(_clean(value), source_text) for value in data.get('excluded') or []) if item]
        slots, excluded = apply_negation(slots, excluded, source_text)
        slots = normalize_place_roles(retain_literal_modifiers(slots, source_text), source_text)
    except (ValueError, TypeError, AttributeError):
        return TaskRequest(request_id, user_id, source_text, unresolved=['action', 'invalid_reply'])
    slots['action'], slots['location'] = request_action(source_text), request_location(source_text)
    unresolved = []
    if slots['source'] in {word+suffix for _, word in LOCATION_WORDS for suffix in ('', '쪽')}:
        slots['source'] = None
    if slots['destination'] in {'나', '저', '나한테', '나에게', '저에게', '저한테'}:
        slots['destination'] = None
    if slots['target'] and re.fullmatch(r'(?:집어|잡아|가져와|가져다|옮겨|찾아)(?:줘|주세요)?', slots['target']):
        slots['target'] = None
    if slots['location'] and slots['source']:
        # Leading left/right can modify the furniture, not the object.
        slots['location'] = None
        unresolved.append('ambiguous_location_scope')
    if NEGATED_ACTION.search(source_text):
        slots['action'] = None
        unresolved.append('negated_action')
    if UNSUPPORTED_REQUEST.search(source_text):
        slots['action'] = None
        unresolved.append('unsupported_action')
    if MULTIPLE_ACTIONS.search(source_text):
        slots['action'] = None
        unresolved.append('multiple_actions')
    if slots['action'] is None:
        unresolved.append('action')
    elif slots['target'] is None:
        unresolved.append('target')
    if slots['action'] == 'move' and slots['destination'] is None:
        unresolved.append('destination')
    return TaskRequest(request_id, user_id, source_text, excluded=excluded,
                       unresolved=list(dict.fromkeys(unresolved)), **slots)
