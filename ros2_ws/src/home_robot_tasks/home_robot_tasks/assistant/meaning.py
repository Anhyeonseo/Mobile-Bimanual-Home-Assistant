"""Text API boundary: extracted meaning is data, never an executable plan.

An independently scheduled API client submits this schema after interpreting
the user's text. No network request runs under the robot gateway/control lock.
The local rule parser remains an offline development fallback.
"""
from copy import deepcopy
import re

from ..fetch import InvalidTask
from .contracts import document, text
from .language import validate_intent, NOUNS, _matches


MEANING_FIELDS = {'operation', 'category', 'attributes', 'source_phrase',
                  'destination_phrase', 'issues'}

# User-agreed S5 scope. Model placement is independent of the meaning contract;
# moving inference from PC to Jetson must not change confirmation or execution.
INTERACTION_POLICY = dict(
    policy_id='s5-text-local-confirmed-v1', input_mode='text',
    inference_target='local_pc', future_inference_target='on_device',
    confirm_each_task=True, maximum_objects_per_task=1,
    explicit_destination_required=True, default_destination_enabled=False,
    handover_enabled=False, multi_task_enabled=False,
    cancel_requires_confirmation=False)


def apply_interaction_policy(intent):
    """Keep raw-text restrictions even if an extractor omits them.

    This is a scope check, not a replacement for model comprehension. Unsupported
    phrases remain questions; model output cannot enable the deferred features.
    """
    value = validate_intent(intent)
    raw = value['utterance'].strip().lower()
    issues = list(value['issues'])
    if re.search(r'손(?:에|으로)\s*(?:건네|줘|주|놓|쥐)|건네|\bhand\s+(?:it|me|over)\b', raw):
        issues.append('handover_not_supported')
    if re.search(r'그리고|고\s*나서|그\s*다음|다음에|둘\s*다|모두|두\s*개|세\s*개|\b(?:then|both)\b', raw):
        issues.append('single_task_required')
    named_kinds = [key for key,names in NOUNS.items() if _matches(raw, names)]
    aliases = sorted({name for names in NOUNS.values() for name in names}, key=len, reverse=True)
    mentions = list(re.finditer('|'.join(re.escape(name) for name in aliases), raw))
    joined = any(re.match(r'\s*(?:과|와|하고|이랑|랑|및|또는|혹은)(?:\s|$)', raw[a.end():b.start()])
                 for a,b in zip(mentions,mentions[1:]))
    if joined or re.search(r'(?<!\d)(?:[2-9]|\d{2,})\s*(?:개|잔|병|자루)|여러\s*개|전부|모든',raw):
        issues.append('single_task_required')
    if len(named_kinds) > 1 and re.search(r'(?:집|잡|놓|찾|옮기|가져오|가져가)고\s+', raw):
        issues.append('single_task_required')
    if re.search(r'(?:가져오|가져가|옮기|집|잡|놓|움직이|실행하)지\s*(?:마|말)|하지\s*마|\b(?:do\s+not|don.t)\b', raw):
        issues.append('negated_action_requires_clarification')
    if value['operation'] == 'fetch_object':
        destination = value['destination_phrase'].strip()
        if not destination:
            issues.append('destination_required')
        elif re.fullmatch(r'(?:나|저|내|제|사용자|사람)(?:한테|에게|쪽|\s*손(?:에|으로)?)?|여기|이쪽|거기|어디(?:로)?든|아무\s*데(?:나)?', destination):
            issues.append('concrete_destination_required')
    value['issues'] = list(dict.fromkeys(issues))
    return value


def intent_from_meaning(utterance, request_id, meaning, *, parser='external_semantic_api_v1'):
    """Attach trusted request identity; the model cannot choose IDs or XYZ."""
    if not isinstance(meaning, dict) or set(meaning) != MEANING_FIELDS:
        raise InvalidTask('invalid extracted meaning fields')
    value = dict(schema_version=1, request_id=request_id,
                 utterance=text(utterance, 'utterance', 512),
                 parser=parser, **deepcopy(meaning))
    value = validate_intent(value)
    issues = value['issues']
    if value['operation'] is None and not issues:
        issues.append('action_required')
    if value['operation'] in {'fetch_object', 'find_object'} and value['category'] is None:
        issues.append('object_category_required')
    if value['operation'] in {'fetch_object', 'navigate_to'} and not value['destination_phrase'].strip():
        issues.append('destination_required')
    value['issues'] = list(dict.fromkeys(issues))
    document(value)
    return value
