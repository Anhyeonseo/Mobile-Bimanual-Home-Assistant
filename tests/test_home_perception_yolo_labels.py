import numpy as np
import pytest

pytest.importorskip('cv2')
from home_perception.eval.yolo_labels import label_line, mask_polygon


def test_polygon_is_normalised_largest_contour():
    mask = np.zeros((100, 200), bool)
    mask[10:30, 20:60] = True
    mask[50:52, 100:102] = True
    polygon = np.array(mask_polygon(mask)).reshape(-1, 2)
    assert polygon[:, 0].min() == pytest.approx(.1) and polygon[:, 0].max() < .31
    assert polygon[:, 1].min() == pytest.approx(.1) and polygon[:, 1].max() < .31


def test_empty_mask_has_no_polygon():
    assert mask_polygon(np.zeros((10, 10), bool)) is None


def test_label_line_starts_with_class():
    assert label_line(2, [.1, .2]) == '2 0.100000 0.200000'
