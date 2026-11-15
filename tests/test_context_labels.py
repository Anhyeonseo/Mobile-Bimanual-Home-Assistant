from copy import deepcopy

import pytest

from home_robot_tasks.assistant.context_labels import (
    build_context_labels, rename_context_label, resolve_context_label)
from home_robot_tasks.assistant.world import WorldStore


def fixture():
    context = dict(map_id='home', map_revision='v1', calibration_id='cal',
                   clock_id='past', frame_id='map')
    report = dict(context=context, map_sha256='map-hash', instances=[], frames=[])
    review = dict(review_id='home-review', source_report_sha256='report-hash',
                  reviewed_views=[0, 1], zones=[dict(id='sleep', label='침실',
                  aliases=[], evidence_views=[0, 1])], entities=[], suppressed=[])
    for i in range(2):
        report['instances'].append(dict(instance_id=f'raw-{i}', context=context,
            category='wardrobe', observed_voxels=[[i, 1, 2], [i, 1, 3]],
            voxel_size_m=.05, last_observed_s=10+i))
        report['frames'].append(dict(image=f'view-{i:02}.jpg', capture={'capture_id':str(i)},
            mapping={'associations':[{'instance_id':f'raw-{i}'}]}))
        review['entities'].append(dict(id=f'table-{i}', label=['화장대', '침대 옆 탁자'][i],
            category='table', aliases=['탁자'], zone_id='sleep',
            member_instance_ids=[f'raw-{i}'], evidence_views=[i], display_views=[i]))
    return report, review


def build(report=None, review=None):
    if report is None:
        report, review = fixture()
    return build_context_labels(report, review, report_sha256='report-hash')


def test_context_names_do_not_change_sensor_labels_geometry_or_time(tmp_path):
    report, review = fixture(); original = deepcopy(report)
    result = build(report, review)
    assert report == original
    item = result['entities'][0]
    assert item['category'] == 'table' and item['raw_categories'] == ['wardrobe']
    assert item['last_observed_s'] == 10
    assert not item['navigation_goal_available'] and item['reobserve_required']
    assert not result['zones'][0]['room_boundary_verified']
    store = WorldStore(tmp_path/'names.sqlite')
    store.save_record('household_context_labels', result['review_id'], result)
    store.close()
    store = WorldStore(tmp_path/'names.sqlite')
    assert store.record('household_context_labels', result['review_id']) == result
    assert not store.entities()
    store.close()


def test_personal_room_name_resolves_without_changing_furniture_or_claiming_freshness():
    previous = build()
    updated = rename_context_label(previous, 'sleep', '안방', ['침실'], expected_revision=1)
    assert updated['entities'] == previous['entities']
    assert previous['zones'][0]['label'] == '침실'
    assert resolve_context_label(updated, '안방 화장대')['status'] == 'MATCH'
    assert resolve_context_label(updated, '침실화장대')['status'] == 'MATCH'
    assert resolve_context_label(updated, '탁자')['status'] == 'AMBIGUOUS'
    assert resolve_context_label(updated, '냉장고')['status'] == 'NOT_FOUND'
    with pytest.raises(ValueError, match='revision conflict'):
        rename_context_label(updated, 'sleep', '방', [], expected_revision=1)


@pytest.mark.parametrize('mutation,error', [
    (lambda r: r.update(source_report_sha256='other'), 'source hash'),
    (lambda r: r['entities'][0].update(evidence_views=[1]), 'without evidence|without referenced'),
    (lambda r: r['entities'][0].update(member_instance_ids=['missing']), 'unknown or reused'),
    (lambda r: r['entities'].pop(), 'unreviewed'),
    (lambda r: r.update(reviewed_views=[0]), 'complete declared'),
    (lambda r: r['entities'][0].update(display_views=[99]), 'display image'),
])
def test_source_and_image_provenance_rejected_on_mismatch(mutation, error):
    report, review = fixture(); mutation(review)
    with pytest.raises(ValueError, match=error):
        build(report, review)
