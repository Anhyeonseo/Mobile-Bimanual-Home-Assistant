"""Choose one verified candidate from a parsed request. Geometry decides, not the model.

Left and right use image columns. Front means nearer the camera: the 3D Z when
every candidate has one, otherwise the box bottom edge on a table seen from above.
"""
from home_perception.contracts import AMBIGUOUS_TARGET, NOT_FOUND

LOCATIONS = ('left', 'right', 'front', 'back')


def _centre_x(item):
    return (item['box_xyxy'][0] + item['box_xyxy'][2]) / 2


def _nearness(item, use_depth):
    return -item['center_xyz_m'][2] if use_depth else item['box_xyxy'][3]


def select_target(candidates, location=None):
    """Return (candidate, failure). Never guesses between remaining candidates."""
    if location is not None and location not in LOCATIONS:
        raise ValueError(f'unknown location {location}')
    if not candidates:
        return None, NOT_FOUND
    if len(candidates) == 1:
        return candidates[0], None
    if location is None:
        return None, AMBIGUOUS_TARGET
    if location in ('left', 'right'):
        ordered = sorted(candidates, key=_centre_x)
        return (ordered[0] if location == 'left' else ordered[-1]), None
    use_depth = all(item.get('center_xyz_m') for item in candidates)
    ordered = sorted(candidates, key=lambda item: _nearness(item, use_depth))
    return (ordered[-1] if location == 'front' else ordered[0]), None
