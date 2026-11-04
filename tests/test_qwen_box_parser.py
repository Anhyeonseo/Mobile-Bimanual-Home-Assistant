import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/diagnostics'))
from qwen3_vl_rgbd import parse_boxes


def test_well_formed_fenced_json_is_scaled_to_pixels():
    boxes = parse_boxes('```json\n[{"label": "cup", "bbox_2d": [100, 200, 300, 400]}]\n```', 640, 480)
    assert boxes == [{'label': 'cup', 'bbox_1000': [100, 200, 300, 400], 'bbox_xyxy': [64.0, 96.0, 192.0, 192.0]}]


def test_empty_array_means_nothing_found():
    assert parse_boxes('[]', 640, 480) == []


# Both strings are verbatim Qwen3-VL-4B outputs from the tabletop target-selection eval.
@pytest.mark.parametrize('raw,labels', [
    ('```json\n[\n\t{"label": "리모컨", "bbox_2d": [852, 227, 950, 497]\n}\n```', ['리모컨']),
    ('```json\n[\n\t{"label": "컵", "bbox_2d": [423, 277, 735, 770],\n\t"label": "컵", '
     '"bbox_2d": [454, 17, 670, 300]\n}\n```', ['컵', '컵']),
])
def test_unbalanced_4b_output_is_salvaged(raw, labels):
    assert [box['label'] for box in parse_boxes(raw, 1000, 1000)] == labels


def test_salvaged_boxes_are_still_validated():
    with pytest.raises(ValueError, match='Invalid'):
        parse_boxes('[{"label": "cup", "bbox_2d": [300, 200, 100, 400]}', 640, 480)


@pytest.mark.parametrize('raw', ['', 'I cannot see a cup.', '[{"label": "cup", "bbox_2d": [1, 2'])
def test_output_without_a_complete_box_is_rejected(raw):
    with pytest.raises(ValueError):
        parse_boxes(raw, 640, 480)
