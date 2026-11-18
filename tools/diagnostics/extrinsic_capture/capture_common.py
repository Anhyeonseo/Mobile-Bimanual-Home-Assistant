"""Shared settings for the calibration capture tools (Jetson side). Read-only: no motor writes.

identity(sp, t, allow_latched_stop=False)
                   HELLO check that accepts the S3 stationary controller (0x24911) or the S4 unified
                   controller (0x2492D). Everything else is checked as stationary_pick.identity does:
                   protocol 2, 12 joints, both arm calibration hashes. A latched stop refuses unless
                   allow_latched_stop: torque-off encoder reads are unaffected by it (normal after a
                   tour or a STOP), so read-only shots record it in the HELLO instead of refusing.
open_camera(...)   Capture with the depth exposure fixed at DEPTH_EXPOSURE_US. D415 auto exposure
                   moved the measured table height by 6..9 mm on 2026-11-15; the runtime height model
                   uses the same 16 ms (home_robot_tasks.stationary_rgbd.HEIGHT_DEPTH_EXPOSURE_US).
                   Capture restores the previous exposure on exit.
settle(camera)     consume frames for SETTLE_S after the exposure change before a shot is taken.
"""
import inspect
import time
from dataclasses import asdict

DEPTH_EXPOSURE_US = 16000   # = home_robot_tasks.stationary_rgbd.HEIGHT_DEPTH_EXPOSURE_US
SETTLE_S = 3.0
SETTINGS = dict(depth_exposure_us=DEPTH_EXPOSURE_US, depth_auto_exposure=False, settle_s=SETTLE_S)


def allowed_firmware(sp):
    versions = {sp.StationaryPickAdapter.firmware_version}
    try:
        from so101_arm_bridge.unified_commissioning import FIRMWARE
        versions.add(FIRMWARE)
    except ImportError:
        pass
    return versions


def identity(sp, transport, *, allow_latched_stop=False):
    hello = transport.enter_binary_mode()
    _, _, checksum = sp.read_development_ranges(sp.CONFIG/'arm_ranges.development.json')
    expected = sp.calibration_hash(checksum)
    allowed = allowed_firmware(sp)
    if (hello.firmware_version not in allowed or hello.protocol_version != 2 or hello.joint_count != 12
            or hello.left_calibration_hash != expected or hello.right_calibration_hash != expected):
        raise ValueError(f'controller identity mismatch (allowed firmware {sorted(hex(v) for v in allowed)}): {hello}')
    if hello.stop_latched and not allow_latched_stop:
        raise ValueError(f'controller stop is latched: {hello}')
    return asdict(hello)


def open_camera(capture_cls, profile, reference):
    if 'depth_exposure_us' not in inspect.signature(capture_cls).parameters:
        raise ValueError('this release cannot fix the depth exposure; run from a release built from main f7ca16d or later')
    return capture_cls(profile, reference, retain_frame=True, depth_exposure_us=DEPTH_EXPOSURE_US)


def settle(camera, seconds=SETTLE_S):
    end = time.monotonic() + seconds
    frame = camera.next_frame()
    while time.monotonic() < end:
        frame = camera.next_frame()
    return frame
