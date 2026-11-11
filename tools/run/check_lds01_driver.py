#!/usr/bin/env python3
"""Exercise the installed LDS driver: virtual serial by default, explicit --port for hardware.

Reuses the official executable and navigation health gate. Only sensor start/pause
commands are sent; no base/controller interfaces exist here. Optional reviewed
sensor TFs and native RGB-D capture are stationary observation only.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/home_robot_tasks'))


def packet(index):
    data = bytearray(42)
    data[:2] = bytes((250, 160 + index))
    struct.pack_into('<H', data, 2, 3000)
    for offset in range(4, 40, 6):
        struct.pack_into('<HHH', data, offset, 100, 1000, 0)
    data[40] = data[41] = (255 - sum(data[:40])) & 255
    return bytes(data)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=8.)
    p.add_argument('--port', help='explicit CP210x port: starts the physical lidar')
    p.add_argument('--fault', choices=('none', 'missing_last', 'sensor_loss'), default='missing_last')
    p.add_argument('--driver-prefix', type=Path)
    p.add_argument('--sensor-frames', action='store_true', help='publish reviewed nominal sensor TFs in the isolated diagnostic domain')
    p.add_argument('--rgbd', action='store_true', help='also capture the actual D415, including unaligned native depth')
    a = p.parse_args()
    if not 4 <= a.seconds <= 120:
        p.error('seconds must be within 4..120')
    if a.port and a.fault != 'none':
        p.error('hardware capture requires --fault none')
    if a.rgbd and (not a.port or not a.sensor_frames):
        p.error('RGB-D capture requires an explicit lidar port and --sensor-frames')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        p.error('set ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST before this diagnostic')
    a.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from ament_index_python.packages import get_package_prefix
    from home_robot_tasks.navigation_health import NavigationHealth, NavigationHealthProfile
    prefix = a.driver_prefix or Path(get_package_prefix('hls_lfcd_lds_driver'))
    executable = prefix/'lib/hls_lfcd_lds_driver/hlds_laser_publisher'
    master = None
    if a.port:
        from serial.tools import list_ports
        port = str(Path(a.port).resolve(strict=True))
        matches = [s for s in list_ports.comports() if str(Path(s.device).resolve()) == port]
        if len(matches) != 1 or (matches[0].vid, matches[0].pid) != (0x10c4, 0xea60):
            p.error('identified CP210x adapter required')
        busy = subprocess.run(['fuser', port], capture_output=True, text=True, timeout=3)
        if busy.returncode != 1 or busy.stderr.strip():
            p.error('port ownership not available')
    else:
        master, slave = pty.openpty()
        port = os.ttyname(slave)
        os.close(slave)
        os.set_blocking(master, False)
    rclpy.init()
    node = rclpy.create_node('lds_cadence_probe', namespace='/lds_commissioning')
    health = NavigationHealth(NavigationHealthProfile('lidar_a'), time.monotonic)
    records = []

    def receive(m):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
        now = time.monotonic()
        observed = now - (node.get_clock().now().nanoseconds / 1e9 - stamp)
        values = list(m.ranges)
        try:
            health.scan(observed_s=observed, frame=m.header.frame_id, ranges=values,
                        minimum_range_m=m.range_min, maximum_range_m=m.range_max)
            error = None
        except ValueError as e:
            error = str(e)
        records.append(dict(arrival=now, stamp=stamp, observed_s=observed,
                            count=len(values), valid=sum(math.isfinite(v) and m.range_min <= v <= m.range_max for v in values),
                            ranges=[v if math.isfinite(v) else None for v in values],
                            frame=m.header.frame_id, angle_min=m.angle_min, angle_increment=m.angle_increment,
                            scan_time=m.scan_time, time_increment=m.time_increment,
                            health_error=error))

    node.create_subscription(LaserScan, 'scan', receive, qos_profile_sensor_data)
    log = (a.output/'driver.log').open('w')
    proc = None
    camera = None
    camera_records = []
    tf_buffer = None
    commands = bytearray()
    index = 0
    next_packet = None
    first_packet = None
    report = dict(mode='real_sensor' if a.port else 'virtual_serial', robot_motion_commands=0,
                  moving_mapping_qualified=False, driver_path=str(executable),
                  driver_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(), passed=False)
    try:
        if a.rgbd:
            import numpy as np
            from home_robot_tasks.s2_rgbd_acquisition import Capture
            profile = json.loads((ROOT/'config/home_assistant/s2_coordinates.development.json').read_text())
            reference = json.loads((ROOT/'config/home_assistant/camera_device.observed.json').read_text())
            camera = Capture(profile, reference_evidence=reference, retain_frame=True, retain_native_depth=True)
            camera.__enter__()
        if a.sensor_frames:
            from home_robot_tasks.s4_sensor_frames import load_sensor_frames
            from geometry_msgs.msg import TransformStamped
            from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
            metadata = camera.images[camera.buffer[-1]['capture_id']][2] if camera else None
            frame_report = load_sensor_frames(ROOT, depth_to_color=metadata['depth_to_color'] if metadata else None)
            tf_buffer = Buffer()
            listener = TransformListener(tf_buffer, node)
            broadcaster = StaticTransformBroadcaster(node)
            messages = []
            for entry in frame_report['transforms']:
                message = TransformStamped()
                message.header.stamp = node.get_clock().now().to_msg()
                message.header.frame_id, message.child_frame_id = entry['parent'], entry['child']
                for key, value in zip(('x', 'y', 'z'), entry['translation_m']):
                    setattr(message.transform.translation, key, value)
                for key, value in zip(('x', 'y', 'z', 'w'), entry['quaternion_xyzw']):
                    setattr(message.transform.rotation, key, value)
                messages.append(message)
            broadcaster.sendTransform(messages)
            report['sensor_frames'] = frame_report
        proc = subprocess.Popen([str(executable), '--ros-args', '-r', '__ns:=/lds_commissioning',
                                 '-p', 'port:='+port, '-p', 'frame_id:=lidar_a'], stdout=log, stderr=subprocess.STDOUT)
        started = time.monotonic()
        while time.monotonic() - started < a.seconds:
            if proc.poll() is not None:
                raise RuntimeError('driver exited before requested stop')
            if master is not None:
                if select.select([master], [], [], 0)[0]:
                    try:
                        data = os.read(master, 4096)
                        commands.extend(data)
                        if b'b' in data and next_packet is None:
                            first_packet = next_packet = time.monotonic() + .1
                    except OSError:
                        pass
                now = time.monotonic()
                active = a.fault != 'sensor_loss' or first_packet is None or now - first_packet < a.seconds / 2
                while active and next_packet is not None and now >= next_packet:
                    cycle, sector = divmod(index, 60)
                    if not (a.fault == 'missing_last' and cycle % 8 == 3 and sector == 59):
                        block = packet(sector)
                        if os.write(master, block) != len(block):
                            raise RuntimeError('partial virtual input write')
                    index += 1
                    next_packet += 1 / 300
            rclpy.spin_once(node, timeout_sec=.001)
            if camera:
                record, (color, aligned, metadata) = camera.next_frame()
                native = camera.native_depth_images[record['capture_id']]
                depth_m = native * metadata['depth_scale_m']
                valid = (depth_m >= .12) & (depth_m <= 4.)
                camera_records.append(dict(capture_id=record['capture_id'],
                    color_time_s=record['color_time_s'], depth_time_s=record['depth_time_s'],
                    native_valid_fraction=float(valid.mean()),
                    native_row_band_valid_fractions=[float(b.mean()) for b in np.array_split(valid, 3)]))
        if len(records) < 5:
            raise RuntimeError('insufficient scans')
        gaps = [b['observed_s'] - x['observed_s'] for x, b in zip(records, records[1:])]
        report.update(frames=len(records), gaps_over_035=sum(g > .35 for g in gaps),
                      maximum_gap_s=max(gaps), minimum_gap_s=min(gaps),
                      mean_gap_s=sum(gaps)/len(gaps), health=health.report(),
                      missing_sector_observed=any(r['ranges'][:6] == [None]*6 and r['valid'] == 354 for r in records))
        if a.fault == 'sensor_loss':
            report['passed'] = not report['health']['scan_ready'] and time.monotonic() - records[-1]['arrival'] > .5
        else:
            report['passed'] = report['gaps_over_035'] == 0 and report['health']['scan_ready']
            if a.fault == 'missing_last':
                report['passed'] &= report['missing_sector_observed']
        if tf_buffer is not None:
            import numpy as np
            from home_robot_tasks.ros_rgbd_source import quaternion_transform
            checks = []
            for entry in frame_report['transforms']:
                actual = tf_buffer.lookup_transform(entry['parent'], entry['child'], rclpy.time.Time()).transform
                error = float(np.max(np.abs(np.array(quaternion_transform(actual.translation, actual.rotation)) - entry['matrix'])))
                checks.append(dict(parent=entry['parent'], child=entry['child'], maximum_matrix_error=error))
            # Verify the composed floor-to-lidar path, not just each direct edge.
            actual = tf_buffer.lookup_transform('base_footprint', 'lidar_a', rclpy.time.Time()).transform
            report['tf_lookup_checks'] = checks
            report['scan_height_above_floor_m'] = actual.translation.z
            # SDK rotation entries are float32: quaternion conversion removes
            # their tiny orthogonality roundoff. This is not a metric tolerance.
            report['tf_matrix_roundoff_limit'] = 1e-7
            report['passed'] &= all(c['maximum_matrix_error'] < 1e-7 for c in checks)
        if camera:
            gaps = [b['depth_time_s']-x['depth_time_s'] for x, b in zip(camera_records, camera_records[1:])]
            report['rgbd'] = dict(frames=len(camera_records), native_shape=list(native.shape),
                maximum_depth_gap_s=max(gaps) if gaps else None,
                minimum_native_valid_fraction=min(r['native_valid_fraction'] for r in camera_records),
                calibration_equivalence=record['calibration_equivalence'],
                physical_sync_qualified=False, full_height_travel_coverage_qualified=False)
            report['passed'] &= len(camera_records) >= 5 and bool(gaps) and 0 < min(gaps) <= max(gaps) < .2
            np.savez_compressed(a.output/'rgbd-last.npz', color_bgr=color, aligned_depth=aligned, native_depth=native)
            (a.output/'rgbd-metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    except Exception as e:
        report['error'] = str(e)
        report['passed'] = False
    finally:
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill(); proc.wait()
        report['driver_exit_code'] = proc.returncode if proc else None
        report['passed'] &= report['driver_exit_code'] == 0
        if camera:
            try:
                camera.__exit__(None,None,None)
            except Exception as e:
                report['camera_cleanup_error'] = str(e)
                report['passed'] = False
            (a.output/'rgbd-frames.json').write_text(json.dumps(camera_records)+'\n')
        if master is not None:
            try:
                while True:
                    block = os.read(master, 4096)
                    if not block:
                        break
                    commands.extend(block)
            except OSError:
                pass
            os.close(master)
            report['virtual_commands_hex'] = commands.hex()
        log.close()
        node.destroy_node(); rclpy.try_shutdown()
        (a.output/'scans.json').write_text(json.dumps(records)+'\n')
        (a.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
