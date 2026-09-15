"""ROS 2 registered/rectified RGB-D ingestion with exposure-time TF.

Subscribe to a rectified colour image and depth registered to that image. The
caller supplies explicit topics, depth scale, calibration identity and a motion
history provider. CameraInfo/P and every image frame must describe the same
optical frame. No fallback to the latest TF or to commanded robot pose.
"""

from dataclasses import replace
import numpy as np
from .capture_port import ImagePair, LatestRGBDSource
from .stationary_rgbd import RGBDFrame
from .perception import CameraIntrinsics
from .fetch import InvalidTask


def image_array(message, *, depth_scale_m):
    formats = {
        "rgb8": ("u1", 3),
        "bgr8": ("u1", 3),
        "16UC1": ("u2", 1),
        "32FC1": ("f4", 1),
    }
    if message.encoding not in formats:
        raise InvalidTask("unsupported image encoding")
    code, channels = formats[message.encoding]
    dtype = np.dtype((">" if message.is_bigendian else "<") + code)
    if (
        not 1 <= message.width <= 8192
        or not 1 <= message.height <= 8192
        or message.step < message.width * channels * dtype.itemsize
        or len(message.data) != message.step * message.height
    ):
        raise InvalidTask("invalid image stride or dimensions")
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(
        message.height, message.step
    )
    packed = raw[:, : message.width * channels * dtype.itemsize].copy().view(dtype)
    if channels == 3:
        result = packed.reshape(message.height, message.width, 3)
        return (
            result[:, :, ::-1].copy() if message.encoding == "bgr8" else result.copy()
        )
    result = packed.reshape(message.height, message.width).astype(np.float32)
    if message.encoding == "16UC1":
        result *= depth_scale_m
    return result


def quaternion_transform(translation, rotation):
    q = np.asarray([rotation.x, rotation.y, rotation.z, rotation.w], dtype=float)
    t = np.asarray([translation.x, translation.y, translation.z], dtype=float)
    if (
        not np.isfinite(q).all()
        or not np.isfinite(t).all()
        or abs(np.linalg.norm(q) - 1) > 1e-3
    ):
        raise InvalidTask("invalid transform quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return (
        (
            1 - 2 * (y * y + z * z),
            2 * (x * y - z * w),
            2 * (x * z + y * w),
            float(t[0]),
        ),
        (
            2 * (x * y + z * w),
            1 - 2 * (x * x + z * z),
            2 * (y * z - x * w),
            float(t[1]),
        ),
        (
            2 * (x * z - y * w),
            2 * (y * z + x * w),
            1 - 2 * (x * x + y * y),
            float(t[2]),
        ),
        (0, 0, 0, 1),
    )


class RosRGBDSource(LatestRGBDSource):
    def __init__(
        self,
        node,
        tf_buffer,
        policy,
        motion_at,
        clock,
        *,
        color_topic,
        aligned_depth_topic,
        camera_info_topic,
        depth_scale_m,
        maximum_skew_s=0.02,
    ):
        super().__init__()
        from sensor_msgs.msg import Image, CameraInfo
        from rclpy.qos import qos_profile_sensor_data

        if not np.isfinite(depth_scale_m) or not 0 < depth_scale_m <= 0.01:
            raise InvalidTask("explicit depth scale required")
        if not 0 < maximum_skew_s <= policy.maximum_tf_skew_s:
            raise InvalidTask("invalid image synchronization skew")
        self.node, self.tf, self.policy, self.motion_at, self.clock = (
            node,
            tf_buffer,
            policy,
            motion_at,
            clock,
        )
        self.scale, self.skew = depth_scale_m, maximum_skew_s
        self.color = self.depth = self.info = None
        self.sequence = 0
        self.last_stamp = None
        self.rejected = 0
        self.last_error = ""
        self.subscriptions = [
            node.create_subscription(
                Image, color_topic, self._color, qos_profile_sensor_data
            ),
            node.create_subscription(
                Image, aligned_depth_topic, self._depth, qos_profile_sensor_data
            ),
            node.create_subscription(
                CameraInfo, camera_info_topic, self._info, qos_profile_sensor_data
            ),
        ]

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

    def _color(self, message):
        self.color = message
        self._try_pair()

    def _depth(self, message):
        self.depth = message
        self._try_pair()

    def _info(self, message):
        self.info = message
        self._try_pair()

    def _try_pair(self):
        if self.color is None or self.depth is None or self.info is None:
            return
        color, depth, info = self.color, self.depth, self.info
        stamp = self.stamp(color)
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        try:
            from rclpy.time import Time
            from rclpy.duration import Duration

            p = self.policy
            if (
                abs(stamp - self.stamp(depth)) > self.skew
                or abs(stamp - self.stamp(info)) > self.skew
            ):
                return
            if any(m.header.frame_id != p.camera_frame for m in (color, depth, info)):
                raise InvalidTask("RGB-D optical frame mismatch")
            if any(
                (m.width, m.height) != (p.width, p.height) for m in (color, depth, info)
            ):
                raise InvalidTask("RGB-D dimensions mismatch")
            ros_now = self.node.get_clock().now().nanoseconds / 1e9
            now = self.clock()
            age = ros_now - stamp
            if not 0 <= age <= p.maximum_age_s:
                raise InvalidTask("stale or future ROS image")
            observed = now - age
            motion = self.motion_at(observed)
            at = Time.from_msg(color.header.stamp)
            if not self.tf.can_transform(
                p.root_frame, p.camera_frame, at, timeout=Duration()
            ):
                raise InvalidTask("exposure transform unavailable")
            transform = self.tf.lookup_transform(
                p.root_frame, p.camera_frame, at, timeout=Duration()
            ).transform
            intrinsics = CameraIntrinsics(
                float(info.p[0]), float(info.p[5]), float(info.p[2]), float(info.p[6])
            )
            self.sequence += 1
            frame = RGBDFrame(
                f"capture-{self.sequence}",
                observed,
                motion.pose_revision,
                p.camera_frame,
                p.calibration_id,
                p.width,
                p.height,
                intrinsics,
                p.root_frame,
                observed,
                quaternion_transform(transform.translation, transform.rotation),
                True,
            )
            rgb = image_array(color, depth_scale_m=self.scale)
            depth_m = image_array(depth, depth_scale_m=self.scale)
            if rgb.ndim != 3 or depth_m.ndim != 2:
                raise InvalidTask("wrong color/depth topic types")
            self.publish(
                ImagePair(frame, rgb, depth_m, now - (ros_now - self.stamp(depth)))
            )
            self.last_stamp = stamp
        except Exception as error:
            self.rejected += 1
            self.last_error = str(error)
