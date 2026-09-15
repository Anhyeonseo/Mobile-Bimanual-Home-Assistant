from types import SimpleNamespace as NS
import numpy as np
import pytest
from home_robot_tasks.ros_rgbd_source import image_array, quaternion_transform


def test_padded_rgb_and_big_endian_depth_are_decoded_without_unit_guess():
    rgb = NS(
        encoding="bgr8",
        width=2,
        height=1,
        step=8,
        is_bigendian=False,
        data=bytes([3, 2, 1, 6, 5, 4, 255, 255]),
    )
    assert image_array(rgb, depth_scale_m=0.001).tolist() == [[[1, 2, 3], [4, 5, 6]]]
    depth = NS(
        encoding="16UC1",
        width=2,
        height=1,
        step=6,
        is_bigendian=True,
        data=bytes([3, 232, 7, 208, 255, 255]),
    )
    np.testing.assert_allclose(image_array(depth, depth_scale_m=0.001), [[1, 2]])
    depth.data = depth.data[:-1]
    with pytest.raises(ValueError):
        image_array(depth, depth_scale_m=0.001)


def test_transform_quaternion_requires_real_rigid_rotation():
    t = NS(x=1, y=2, z=3)
    r = NS(x=0, y=0, z=0, w=1)
    assert quaternion_transform(t, r) == (
        (1, 0, 0, 1),
        (0, 1, 0, 2),
        (0, 0, 1, 3),
        (0, 0, 0, 1),
    )
    r.w = 0
    with pytest.raises(ValueError):
        quaternion_transform(t, r)
