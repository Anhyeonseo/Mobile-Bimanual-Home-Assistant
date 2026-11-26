"""PointNet++ inference contracts needed by the CPU web station."""
import pytest

torch = pytest.importorskip('torch')
from home_perception.grasp import pointnet2_cpu as ops


def test_sampling_group_order_padding_and_rotated_cylinder():
    points = torch.tensor([[[0., 0., 1.], [0., 0.2, 1.], [.2, 0., 1.], [0., .1, 1.], [0., 0., 0.]]])
    assert ops.furthest_point_sampling(points, 2).tolist() == [[0, 1]]
    assert ops.ball_query(points[:, :1], points, .2, 3).tolist() == [[[0, 3, 0]]]
    rotation = torch.tensor([[[0., -1., 0., 1., 0., 0., 0., 0., 1.]]])
    assert ops.cylinder_query(points[:, :1], points, rotation, .02, .05, .25, 4).tolist() == [[[1, 3, 1, 1]]]
    assert ops.ball_query(points[:, :1] + 50, points, .2, 2).tolist() == [[[0, 0]]]


def test_feature_propagation_and_interpolation_across_batches():
    xyz = torch.tensor([[[0., 0., 0.], [1., 0., 0.], [3., 0., 0.]]]).expand(2, -1, -1)
    distances, indices = ops.three_nn(xyz[:, :1], xyz)
    assert distances.tolist() == [[[0., 1., 9.]]] * 2
    features = torch.tensor([[[3., 5., 7.]], [[11., 13., 15.]]])
    weights = torch.tensor([[[.5, 0., .5]]]).expand(2, -1, -1)
    assert ops.three_interpolate(features, indices, weights).tolist() == [[[5.]], [[13.]]]
    assert ops.gather_points(features, indices[:, 0, 1:]).tolist() == [[[5., 7.]], [[13., 15.]]]
