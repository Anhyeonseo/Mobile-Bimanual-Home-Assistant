import numpy as np
import pytest

pytest.importorskip('cv2')
from home_perception.detect.base import Detection
from home_perception.detect.verify import (crop_box, missing_objects, parse_answer, question,
                                           question_english, resize_for_model, split_verified)


@pytest.mark.parametrize('text,expected', [
    ('예', True), ('네.', True), ('Yes', True), ('아니오', False), ('No', False),
    ('예 아니오', False), ('잘 모르겠습니다', False), ('', False), ('예시입니다', False),
])
def test_only_a_clear_yes_passes(text, expected):
    assert parse_answer(text) is expected


def test_crop_pads_box_and_stays_inside_image():
    image = np.zeros((100, 100, 3), np.uint8)
    assert crop_box(image, [40, 40, 60, 60]).shape[:2] == (31, 31)
    assert crop_box(image, [0, 0, 100, 100]).shape[:2] == (100, 100)


def test_small_crop_is_enlarged_and_large_crop_is_kept():
    assert min(resize_for_model(np.zeros((30, 60, 3), np.uint8)).shape[:2]) == 224
    assert resize_for_model(np.zeros((300, 300, 3), np.uint8)).shape[:2] == (300, 300)


def test_english_question_names_the_object_and_allows_no():
    assert 'cup' in question_english('cup') and 'answer no' in question_english('cup')


def test_question_names_the_object_and_allows_no():
    assert '리모컨' in question('리모컨') and '아니오' in question('리모컨')


def box(label):
    return Detection(label, [0., 0., 1., 1.], 'qwen')


def test_split_keeps_only_clear_yes_and_requires_one_answer_each():
    cup, remote = box('컵'), box('리모컨')
    assert split_verified([cup, remote], ['예', '아니오']) == ([cup], [remote])
    with pytest.raises(ValueError):
        split_verified([cup], [])


def test_missing_objects_matches_korean_or_english_labels():
    names = {'cup': '컵', 'remote control': '리모컨'}
    assert missing_objects(['cup', 'remote control'], [box('검은 컵')], names) == ['remote control']
    assert missing_objects(['cup', 'cup'], [box('cup')], names) == []
