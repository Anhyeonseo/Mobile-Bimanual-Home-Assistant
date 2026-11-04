import pytest

from home_perception.contracts import ACCEPTED, HELD, ObjectObservation, REJECTED
from home_perception.memory.world_memory import HELD_UNCERTAINTY_M, WorldMemory


def seen(capture, label, x, status=ACCEPTED):
    return ObjectObservation(capture, '2026-11-01T00:00:00+00:00', 'camera', label, [0., 0., 1., 1.], 'qwen', status,
                             center_xyz_m=None if status == REJECTED else [x, 0., .4],
                             reason=None if status == ACCEPTED else 'test')


@pytest.fixture
def memory():
    return WorldMemory('camera')


def test_same_place_keeps_one_entity_with_all_evidence(memory):
    memory.observe([seen('a', 'cup', 0.)])
    memory.observe([seen('b', 'cup', .01)])
    entity, = memory.entities.values()
    assert entity.evidence == ['a', 'b'] and entity.state == 'observed'


def test_moved_single_object_updates_position(memory):
    memory.observe([seen('a', 'remote', 0.)])
    memory.observe([seen('b', 'remote', .3)])
    entity, = memory.entities.values()
    assert entity.position_m[0] == .3


def test_unseen_object_goes_stale_and_is_kept(memory):
    memory.observe([seen('a', 'cup', 0.), seen('a', 'remote', .2)])
    memory.observe([seen('b', 'remote', .2)])
    states = {entity.kind: entity.state for entity in memory.entities.values()}
    assert states == {'cup': 'stale', 'remote': 'observed'}
    assert next(e for e in memory.entities.values() if e.kind == 'cup').position_m == [0., 0., .4]


def test_two_near_objects_keep_identity_by_nearest_pair(memory):
    memory.observe([seen('a', 'cup', 0.), seen('a', 'cup', .03)])
    first, second = memory.entities.values()
    memory.observe([seen('b', 'cup', .031), seen('b', 'cup', .001)])
    assert first.position_m[0] == .001 and second.position_m[0] == .031


def test_same_looking_new_object_is_unconfirmed(memory):
    memory.observe([seen('a', 'cup', 0.), seen('a', 'cup', .2)])
    memory.observe([seen('b', 'cup', .5), seen('b', 'cup', .8)])
    assert sum(entity.state == 'unconfirmed' for entity in memory.entities.values()) == 2


def test_rejected_observation_changes_nothing(memory):
    memory.observe([seen('a', 'cup', 0., REJECTED)])
    assert not memory.entities


def test_held_observation_has_wider_uncertainty(memory):
    memory.observe([seen('a', 'cup', 0., HELD)])
    entity, = memory.entities.values()
    assert entity.position_uncertainty_m == HELD_UNCERTAINTY_M


def test_carried_object_does_not_go_stale(memory):
    memory.observe([seen('a', 'cup', 0.)])
    entity, = memory.entities.values()
    entity.state = 'carried'
    memory.observe([])
    assert entity.state == 'carried'


def test_other_frame_is_refused(memory):
    item = seen('a', 'cup', 0.)
    item.frame_id = 'map'
    with pytest.raises(ValueError):
        memory.observe([item])
