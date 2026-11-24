"""Resolve a single pick target without substituting a different object."""
import json
import re
import unicodedata

from .object_vocabulary import COLORS, NOUNS

# Common phrases work offline even when the optional local translator is down.
OBJECTS = {alias: category.replace('_', ' ') for category, aliases in NOUNS.items()
           for alias in aliases}
OBJECTS.update({'리모콘': 'remote control', '머그': 'mug', '머그컵': 'mug',
                '물병': 'water bottle', '볼펜': 'ballpoint pen', '연필': 'pencil',
                '펜': 'pen', '지우개': 'eraser', '계산기': 'calculator', '책': 'book',
                '테이프': 'tape', '마스킹 테이프': 'masking tape', '노트': 'notebook',
                '숟가락': 'spoon', '사과': 'apple', '바나나': 'banana', '휴대폰': 'cell phone',
                '스펀지': 'sponge', '안경': 'eyeglasses', '열쇠': 'key',
                '박스': 'box', '상자': 'box', '수건': 'towel', '물통': 'water bottle'})
MODIFIERS = {alias: key for key, aliases in COLORS.items() for alias in aliases}
MODIFIERS.update({'노란': 'yellow', '노란색': 'yellow', '작은': 'small', '큰': 'large',
                  '분홍색': 'pink', '주황색': 'orange', '보라색': 'purple'})
ENGLISH = re.compile(r'[a-z][a-z0-9]*(?:[ -][a-z0-9]+)*')
PICK_KO = re.compile(r'\s*(?:좀\s*)?(?:집어|잡아|들어\s*올려)(?:\s*(?:주실래요|줄래요|줄래|주세요|줘요|줘))?$|\s*(?:집기|잡기)$')
UNSUPPORTED = re.compile(
    r'말고|빼고|아니라|하지\s*마|지\s*(?:마|말)|그리고|왼쪽|오른쪽|사이에|옆에|위에|아래에|에\s*있는|둘\s*다|두\s*개|모두|'
    r'(?:이랑|랑|하고)\s+|(?:옮겨|놓아|가져다|켜줘|꺼줘|찾아)|'
    r'\b(?:and|or|not|except|without|instead|between|left|right|next to|on|under|behind|beside|'
    r'near|then|both|all|two|find|move|put|drop|bring|stop|cancel|open|close|grab|pick|grasp)\b')
VAGUE = {'그거','이거','저거','물체','물건','그것','이것','저것','it','thing','something','object','this','that','up','pick','null','none','unknown'}
HELP = '집을 물체 하나의 이름이나 특징을 입력해 주세요. 예: 노란 테이프를 집어줘'


def _target_phrase(text):
    if not isinstance(text, str) or not 1 <= len(text.strip()) <= 160:
        raise ValueError(HELP)
    phrase = ' '.join(unicodedata.normalize('NFC', text).lower().split()).rstrip('.!?').strip()
    phrase = re.sub(r'^(?:please\s+)?(?:pick(?:\s+up)?|grab|grasp)\s+', '', phrase)
    phrase = re.sub(r'\s+up$', '', phrase) if re.match(r'^(?:please\s+)?pick\s', text.strip(), re.I) else phrase
    action = PICK_KO.search(phrase)
    if action:
        phrase = phrase[:action.start()].strip()
        phrase = re.sub(r'(?:을|를)$', '', phrase).strip()
    phrase = re.sub(r'^(?:the|a|an)\s+', '', phrase).strip()
    return phrase


def parse_pick_request(text):
    """Separate an explicit image-side selector from the detector noun phrase."""
    phrase = _target_phrase(text)
    match = re.match(r'^(?:(?:화면(?:의)?\s+)?(왼쪽|오른쪽)\s+|(left|right)(?:most)?\s+)(.+)$', phrase)
    location = None
    if match:
        location = {'왼쪽':'left', '오른쪽':'right'}.get(match[1], match[2])
        phrase = match[3]
    target, prompt = parse_target(phrase)
    return target, prompt, location


def parse_target(text):
    """Return the complete target phrase and an optional immediately usable English prompt."""
    phrase = _target_phrase(text)
    if (not phrase or phrase in VAGUE or UNSUPPORTED.search(phrase)
            or not re.fullmatch(r'[가-힣a-z0-9 -]+', phrase)):
        raise ValueError(HELP)
    if ENGLISH.fullmatch(phrase):
        return phrase, 'remote control' if phrase == 'remote' else phrase
    if phrase in OBJECTS:
        return phrase, OBJECTS[phrase]
    # Preserve all qualifiers. An unknown modifier belongs in the translation.
    for alias in sorted(MODIFIERS, key=len, reverse=True):
        if phrase.startswith(alias):
            noun = phrase[len(alias):].strip()
            if noun in OBJECTS or ENGLISH.fullmatch(noun):
                return phrase, MODIFIERS[alias]+' '+OBJECTS.get(noun, noun)
    return phrase, None


def translate_target(phrase):
    from home_perception.language.local_model import LocalLanguageModel
    schema = {'type':'object', 'additionalProperties':False, 'required':['prompt'],
              'properties':{'prompt':{'type':['string','null']}}}
    system = ('Translate one concrete object noun phrase into a short English detector caption. '
              'Retain every color, material, shape and distinguishing adjective. No article. '
              'Return null for an action, ambiguous object, multiple objects, negation, '
              'relative position, ownership or unsupported condition. No guessing. /no_think')
    try:
        reply, _ = LocalLanguageModel(timeout_s=30).complete_json(system, phrase, 'pick_target', schema)
        result = json.loads(reply)['prompt']
    except (OSError, ValueError, KeyError, IndexError, TypeError) as exc:
        raise ValueError('로컬 한영 모델 응답을 확인할 수 없습니다. 영어 물체 이름도 입력할 수 있습니다.') from exc
    if not isinstance(result, str) or len(result) > 120:
        raise ValueError(HELP)
    prompt = ' '.join(result.lower().split())
    if not ENGLISH.fullmatch(prompt) or prompt in VAGUE or UNSUPPORTED.search(prompt):
        raise ValueError(HELP)
    return prompt
