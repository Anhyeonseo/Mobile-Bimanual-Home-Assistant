"""Versioned values shared by perception, memory, planning and the application."""
from copy import deepcopy
from dataclasses import asdict, dataclass
import json
import math

from ..fetch import InvalidTask
from ..navigation import identifier


def finite(value, name, minimum=None, maximum=None):
    if type(value) not in (int, float):
        raise InvalidTask(f'{name}: finite number required')
    try:
        value = float(value)
    except (OverflowError, ValueError) as error:
        raise InvalidTask(f'{name}: finite number required') from error
    if not math.isfinite(value) or (minimum is not None and value < minimum) or (maximum is not None and value > maximum):
        raise InvalidTask(f'{name}: outside bounds')
    return value


def text(value, name, maximum=160):
    if not isinstance(value, str) or not value.strip() or len(value) > maximum or any(ord(c) < 32 for c in value):
        raise InvalidTask(f'{name}: nonempty bounded text required')
    return value.strip()


def vector(value, size, name):
    if not isinstance(value, (tuple, list)) or len(value) != size:
        raise InvalidTask(f'{name}: {size} coordinates required')
    return tuple(finite(v, name, -10000, 10000) for v in value)


def document(value):
    """Canonical JSON also rejects NaN, unsupported objects and oversized input."""
    try:
        encoded = json.dumps(value, ensure_ascii=False, sort_keys=True, allow_nan=False, separators=(',', ':'))
    except (ValueError, TypeError, OverflowError) as error:
        raise InvalidTask('invalid JSON document') from error
    if len(encoded.encode()) > 2_000_000:
        raise InvalidTask('document too large')
    return encoded


@dataclass(frozen=True)
class Context:
    map_id: str
    map_revision: str
    calibration_id: str
    clock_id: str
    frame_id: str = 'map'

    def __post_init__(self):
        for key, value in asdict(self).items():
            identifier(value, key)

    @classmethod
    def parse(cls, value):
        if not isinstance(value, dict) or set(value) != set(cls.__dataclass_fields__):
            raise InvalidTask('complete context required')
        return cls(**value)

    def check(self, other):
        if self != other:
            raise InvalidTask('context_revision_or_clock_mismatch')

    def to_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class Stamp:
    observed_s: float
    context: Context
    capture_id: str

    def __post_init__(self):
        finite(self.observed_s, 'observation time', 0)
        if not isinstance(self.context, Context):
            raise InvalidTask('observation context required')
        identifier(self.capture_id, 'capture_id')

    def check(self, context, now, maximum_age):
        self.context.check(context)
        age = finite(now, 'now', 0) - self.observed_s
        if not 0 <= age <= finite(maximum_age, 'maximum age', 0):
            raise InvalidTask('stale_or_future_observation')

    def to_dict(self):
        return asdict(self)

    @classmethod
    def parse(cls, value):
        if not isinstance(value, dict) or set(value) != {'observed_s', 'context', 'capture_id'}:
            raise InvalidTask('complete observation stamp required')
        return cls(value['observed_s'], Context.parse(value['context']), value['capture_id'])


KINDS = {'room', 'surface', 'object'}
STATES = {'observed', 'unconfirmed', 'stale', 'occluded', 'carried', 'released_unconfirmed', 'placed'}
SOURCES = {'sensor', 'inferred', 'user', 'simulation'}


def entity(value):
    """Validate memory facts; surface XYZ is explicitly not a grasp pose."""
    fields = {'entity_id', 'kind', 'label', 'category', 'aliases', 'attributes', 'room_id',
              'support_id', 'position_m', 'extent_m', 'geometry_kind', 'state', 'source',
              'stamp', 'revision'}
    if not isinstance(value, dict) or set(value) != fields:
        raise InvalidTask('invalid entity fields')
    result = deepcopy(value)
    identifier(result['entity_id'], 'entity_id')
    if any(not isinstance(result[k], str) for k in ('kind', 'state', 'source')) or result['kind'] not in KINDS or result['state'] not in STATES or result['source'] not in SOURCES:
        raise InvalidTask('invalid entity kind/state/source')
    result['label'] = text(result['label'], 'entity label')
    result['category'] = text(result['category'], 'category').lower()
    aliases = result['aliases']
    if not isinstance(aliases, list) or len(aliases) > 24:
        raise InvalidTask('bounded aliases required')
    result['aliases'] = list(dict.fromkeys(text(v, 'alias') for v in aliases))
    attrs = result['attributes']
    if not isinstance(attrs, dict) or len(attrs) > 16:
        raise InvalidTask('bounded attributes required')
    result['attributes'] = {text(k, 'attribute', 40): text(v, 'attribute value') for k, v in attrs.items()}
    for key in ('room_id', 'support_id'):
        if result[key] is not None:
            identifier(result[key], key)
            if result[key] == result['entity_id']:
                raise InvalidTask('self relation')
    for key in ('position_m', 'extent_m'):
        if result[key] is not None:
            result[key] = list(vector(result[key], 3, key))
    if result['extent_m'] is not None and any(v <= 0 for v in result['extent_m']):
        raise InvalidTask('positive extent required')
    if not isinstance(result['geometry_kind'], str) or result['geometry_kind'] not in {'none', 'surface_point', 'bbox_center', 'support_surface'}:
        raise InvalidTask('invalid geometry kind')
    if (result['position_m'] is None) != (result['geometry_kind'] == 'none'):
        raise InvalidTask('geometry and position disagree')
    if result['kind'] == 'room' and (result['room_id'] or result['support_id']):
        raise InvalidTask('room cannot be contained by an entity')
    if result['kind'] == 'surface' and result['support_id']:
        raise InvalidTask('surface cannot have an object support')
    if type(result['revision']) is not int or result['revision'] < 1:
        raise InvalidTask('positive entity revision required')
    result['stamp'] = Stamp.parse(result['stamp']).to_dict()
    document(result)
    return result


def make_entity(entity_id, kind, label, category, stamp, *, aliases=(), attributes=None,
                room_id=None, support_id=None, position_m=None, extent_m=None,
                geometry_kind='none', state='observed', source='sensor', revision=1):
    return entity(dict(entity_id=entity_id, kind=kind, label=label, category=category,
        aliases=list(aliases), attributes=attributes or {}, room_id=room_id, support_id=support_id,
        position_m=position_m, extent_m=extent_m, geometry_kind=geometry_kind, state=state,
        source=source, stamp=stamp.to_dict(), revision=revision))
