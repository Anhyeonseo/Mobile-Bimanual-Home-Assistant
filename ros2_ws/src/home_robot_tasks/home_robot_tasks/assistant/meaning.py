"""Text API boundary: extracted meaning is data, never an executable plan.

An independently scheduled API client submits this schema after interpreting
the user's text. No network request runs under the robot gateway/control lock.
The local rule parser remains an offline development fallback.
"""
from copy import deepcopy

from ..fetch import InvalidTask
from .contracts import document, text
from .language import validate_intent


MEANING_FIELDS = {'operation', 'category', 'attributes', 'source_phrase',
                  'destination_phrase', 'issues'}


def intent_from_meaning(utterance, request_id, meaning):
    """Attach trusted request identity; the model cannot choose IDs or XYZ."""
    if not isinstance(meaning, dict) or set(meaning) != MEANING_FIELDS:
        raise InvalidTask('invalid extracted meaning fields')
    value = dict(schema_version=1, request_id=request_id,
                 utterance=text(utterance, 'utterance', 512),
                 parser='external_semantic_api_v1', **deepcopy(meaning))
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
