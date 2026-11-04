"""Korean request text -> TaskRequest. The model proposes, this module decides.

The prompt never offers coordinates or joints, and a reply containing them is
refused. Actions and locations come from the request words, and an object slot
must appear in the request, so a small model cannot invent a task. Anything
unsupported stays unresolved rather than being mapped to the nearest action.
"""
import json
import re

from home_perception.contracts import TaskRequest

FORBIDDEN_KEYS = re.compile(r'joint|coordinate|xyz|pose|motor|velocity|angle', re.IGNORECASE)
# Checked on the original text, so a model mistake cannot authorise these tasks.
UNSUPPORTED_REQUEST = re.compile(
    r'따라|붓|끓|요리|볶|데워|설거지|청소|빨래|쓸어|닦아|열어|닫아|잠가|켜줘|꺼줘|버려|부숴|던져|계단|2층|위층|춤|노래'
    r'|손에|건네|쥐어|먹여|물병|뜨거운|칼날|불 ')
MULTIPLE_ACTIONS = re.compile(r'(고 나서|ㄴ 다음|[은는던준온한]\s*다음|[은는던준온한]\s*뒤|다음에|그리고 |그러고)')
PRONOUNS = ('그거', '저거', '이거', '그것', '저것', '이것')
ACTION_WORDS = (('find', r'찾|어디'), ('pick', r'들어올'), ('move', r'옮기|옮겨|놓아|놔|올려|둬|두어'),
                ('fetch', r'가져|갖다'), ('pick', r'집어|집고|잡아|잡고|들어|집$|잡$'))
NEGATION = re.compile(r'^(.+?)\s*(?:말고|아니고|아니라)\s+(.+)$')
LOCATION_WORDS = (('left', '왼쪽'), ('right', '오른쪽'), ('front', '앞'), ('back', '뒤'))

PROMPT = """한국어 로봇 요청에서 물체와 장소를 JSON으로 뽑는다. 문장에 있는 단어만 쓰고, 없으면 null이다.
형식: {"target": 물체 또는 null, "source": 물체가 지금 있는 곳 또는 null, "destination": 놓을 곳 또는 null, "excluded": [제외할 물체]}
물체 이름 앞의 색이나 "내" 같은 꾸밈말은 물체에 포함한다. 왼쪽·오른쪽·앞·뒤 같은 위치 단어는 뺀다.
"나한테", "나에게"는 놓을 곳이 아니다.

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


def parse_reply(text, request_id, user_id, source_text):
    """Validate one model reply. The text decides the action; the model fills objects."""
    try:
        data = json.loads(_strip(text))
        if not isinstance(data, dict):
            raise ValueError('object required')
        if any(FORBIDDEN_KEYS.search(key) for key in data):
            raise ValueError('motion values are not allowed')
        slots = {name: _in_text(_clean(data.get(name)), source_text) for name in ('target', 'source', 'destination')}
        excluded = [item for item in (_in_text(_clean(value), source_text) for value in data.get('excluded') or []) if item]
        slots, excluded = apply_negation(slots, excluded, source_text)
    except (ValueError, TypeError, AttributeError):
        return TaskRequest(request_id, user_id, source_text, unresolved=['action', 'invalid_reply'])
    slots['action'], slots['location'] = request_action(source_text), request_location(source_text)
    if slots['source'] and slots['source'].startswith(tuple(word for _, word in LOCATION_WORDS)):
        slots['source'] = None
    unresolved = []
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
