"""Persistent object memory from accepted observations. Unseen objects are never deleted.

Association uses category and distance only. Same-looking objects that cannot be
told apart are recorded as unconfirmed instead of being merged by guess.
Appearance and registration across viewpoints are not used yet.
"""
import math

from home_perception.contracts import ACCEPTED, HELD, WorldEntity

SAME_PLACE_M = .05
HELD_UNCERTAINTY_M = .05
ACCEPTED_UNCERTAINTY_M = .01


def _position(item):
    return item.center_xyz_m or item.surface_xyz_m


class WorldMemory:
    def __init__(self, frame_id):
        self.frame_id = frame_id
        self.entities = {}
        self._next = 1

    def _category(self, kind):
        return [entity for entity in self.entities.values() if entity.kind == kind]

    def _create(self, item, state):
        entity = WorldEntity(f'object-{self._next}', item.label, state, 'observed', self.frame_id,
                             evidence=[item.capture_id])
        self._next += 1
        self.entities[entity.entity_id] = entity
        return self._update(entity, item, state)

    def _update(self, entity, item, state):
        if item.capture_id not in entity.evidence:
            entity.evidence.append(item.capture_id)
        entity.position_m, entity.geometry_kind = _position(item), 'visible_surface_center'
        entity.extent_m = item.obb['extent_m'] if item.obb else None
        entity.last_seen_utc = item.captured_at_utc
        entity.position_uncertainty_m = HELD_UNCERTAINTY_M if item.status == HELD else ACCEPTED_UNCERTAINTY_M
        entity.state, entity.revision = state, entity.revision + 1
        return entity

    def observe(self, observations):
        """Apply one capture. Every known object in view that was not matched goes stale."""
        usable = [item for item in observations if item.status in (ACCEPTED, HELD) and _position(item)]
        if any(item.frame_id != self.frame_id for item in usable):
            raise ValueError('observation frame does not match memory frame')
        matched = set()
        for kind in dict.fromkeys(item.label for item in usable):
            items = [item for item in usable if item.label == kind]
            known = self._category(kind)
            waiting = []
            # Stayed in place: nearest pairs first, so two close objects keep their identity.
            pairs = sorted((math.dist(_position(item), entity.position_m), index, entity.entity_id)
                           for index, item in enumerate(items) for entity in known)
            used = set()
            for distance, index, entity_id in pairs:
                if distance <= SAME_PLACE_M and index not in used and entity_id not in matched:
                    self._update(self.entities[entity_id], items[index], 'observed')
                    used.add(index)
                    matched.add(entity_id)
            waiting = [item for index, item in enumerate(items) if index not in used]
            free = [entity for entity in known if entity.entity_id not in matched]
            # Moved: only certain when exactly one object could have gone there.
            if len(waiting) == 1 and len(free) == 1:
                self._update(free[0], waiting[0], 'observed')
                matched.add(free[0].entity_id)
            else:
                for item in waiting:
                    state = 'unconfirmed' if free else 'observed'
                    matched.add(self._create(item, state).entity_id)
        for entity in self.entities.values():
            if entity.entity_id not in matched and entity.state not in ('carried', 'placed'):
                entity.state = 'stale'
        return matched
