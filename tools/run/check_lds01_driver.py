#!/usr/bin/env python3
"""Exercise the installed LDS driver: virtual serial by default, explicit --port for hardware.

Reuses the official executable and navigation health gate. Only sensor start/pause
commands are sent; no base/controller interfaces exist here. Optional reviewed
sensor TFs are nominal. Optional RGB-D archives preserve moving observations
for later registration; capture alone never qualifies a map or navigation.
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
import shutil
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



class TeleopCompletion:
    """Observe one existing append-only log without owning the controller."""
    def __init__(self, path):
        self.path = Path(path)
        self.offset = 0
        self.pending = b''
        self.header = False
        self.finished = None
        self.poll()
        if not self.header or self.finished is not None:
            raise ValueError('an open identified teleop recording is required')

    def poll(self):
        with self.path.open('rb') as stream:
            if stream.seek(0, 2) < self.offset:
                raise ValueError('teleop recording was truncated')
            stream.seek(self.offset)
            data = stream.read(1_000_000)
            self.offset = stream.tell()
        lines = (self.pending + data).split(b'\n')
        self.pending = lines.pop()
        if len(self.pending) > 100_000:
            raise ValueError('oversized teleop record')
        for line in lines:
            row = json.loads(line)
            if not self.header:
                if (row.get('event') != 'session' or
                        row.get('firmware', {}).get('firmware') != 'mobile-base-hold-teleop-v7'):
                    raise ValueError('identified teleop session required')
                self.header = True
            if row.get('event') == 'finished':
                self.finished = row
        return self.finished


class RGBDArchive:
    """Sparse full-resolution RGB/native-depth assets with acquisition times."""
    def __init__(self, output, period):
        if not .5 <= period <= 10:
            raise ValueError('archive period must be within 0.5..10 seconds')
        self.folder = Path(output)/'rgbd'
        self.folder.mkdir()
        self.period, self.last, self.count = period, None, 0

    def add(self, record, color, native, metadata):
        import numpy as np
        stamp = record['depth_time_s']
        if self.last is not None and stamp - self.last < self.period:
            return False
        if shutil.disk_usage(self.folder).free < 1024**3 + color.nbytes + native.nbytes:
            raise RuntimeError('less than 1 GiB reserve for RGB-D archive')
        name = f'{self.count:06d}.npz'
        temporary = self.folder/(name+'.tmp')
        with temporary.open('xb') as stream:
            # No alignment, crop or depth quantization. Avoid compression in the sensor loop.
            np.savez(stream, color_bgr=color, native_depth=native)
        path = self.folder/name
        temporary.rename(path)
        if self.count == 0:
            (self.folder/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
        row = dict(file=name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                   capture_id=record['capture_id'],color_time_s=record['color_time_s'],
                   depth_time_s=stamp,host_saved_s=time.monotonic(),
                   color_frame=record['color_frame'],depth_frame=record['depth_frame'])
        with (self.folder/'frames.jsonl').open('a') as stream:
            stream.write(json.dumps(row)+'\n')
        self.last = stamp
        self.count += 1
        return True


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=8.)
    p.add_argument('--port', help='explicit CP210x port: starts the physical lidar')
    p.add_argument('--fault', choices=('none', 'missing_last', 'sensor_loss'), default='missing_last')
    p.add_argument('--driver-prefix', type=Path)
    p.add_argument('--sensor-frames', action='store_true', help='publish reviewed nominal sensor TFs in the isolated diagnostic domain')
    p.add_argument('--rgbd', action='store_true', help='also capture the actual D415, including unaligned native depth')
    p.add_argument('--require-receipt-timing', action='store_true', help='check host receipt estimate and reversed ray-time ordering')
    p.add_argument('--archive-rgbd-period', type=float, help='retain full native RGB/depth every N seconds (0.5..10)')
    p.add_argument('--teleop-record', type=Path, help='stop sensor capture when this existing teleop session finishes; never controls motors')
    a = p.parse_args()
    limit = 600 if a.teleop_record else 120
    if not 4 <= a.seconds <= limit:
        p.error(f'seconds must be within 4..{limit}')
    if a.archive_rgbd_period is not None and (not a.rgbd or not .5 <= a.archive_rgbd_period <= 10):
        p.error('archive requires --rgbd and a period within 0.5..10')
    if a.port and a.fault != 'none':
        p.error('hardware capture requires --fault none')
    if a.rgbd and (not a.port or not a.sensor_frames):
        p.error('RGB-D capture requires an explicit lidar port and --sensor-frames')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        p.error('set ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST before this diagnostic')
    completion = TeleopCompletion(a.teleop_record) if a.teleop_record else None
    a.output.mkdir(parents=True, exist_ok=False)
    archive = RGBDArchive(a.output, a.archive_rgbd_period) if a.archive_rgbd_period is not None else None
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
        next_log_check = started
        ready = False
        while time.monotonic() - started < a.seconds:
            if completion and time.monotonic() >= next_log_check:
                next_log_check = time.monotonic() + .25
                if completion.poll() is not None:
                    report['teleop_finished'] = completion.finished
                    report['capture_end_reason'] = 'teleop_finished'
                    break
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
                    color_frame=record['color_frame'], depth_frame=record['depth_frame'],
                    color_time_s=record['color_time_s'], depth_time_s=record['depth_time_s'],
                    native_valid_fraction=float(valid.mean()),
                    native_row_band_valid_fractions=[float(b.mean()) for b in np.array_split(valid, 3)]))
                if archive:
                    archive.add(record, color, native, metadata)
            if not ready and len(records) >= 5 and (not camera or len(camera_records) >= 5):
                ready = True
                (a.output/'ready.json').write_text(json.dumps(dict(ready=True, host_monotonic_s=time.monotonic(),
                    sensor_only=True, native_archive=archive is not None, teleop_record=str(a.teleop_record)))+'\n')
        report.setdefault('capture_end_reason', 'duration_limit')
        if len(records) < 5:
            raise RuntimeError('insufficient scans')
        gaps = [b['observed_s'] - x['observed_s'] for x, b in zip(records, records[1:])]
        report.update(frames=len(records), gaps_over_035=sum(g > .35 for g in gaps),
                      maximum_gap_s=max(gaps), minimum_gap_s=min(gaps),
                      mean_gap_s=sum(gaps)/len(gaps), health=health.report(),
                      missing_sector_observed=any(r['ranges'][:6] == [None]*6 and r['valid'] == 354 for r in records))
        if a.require_receipt_timing:
            timing_ok = all(r['time_increment'] < 0 and
                abs(360*r['time_increment']+r['scan_time']) < 1e-6 and
                0 <= r['arrival']-r['observed_s'] <= .3 for r in records)
            report['receipt_timing'] = dict(passed=timing_ok,
                basis='host serial receipt minus packet serialization; negative offsets in increasing-angle array',
                hardware_exposure_latency_calibrated=False,
                maximum_first_ray_age_s=max(r['arrival']-r['observed_s'] for r in records))
        if a.fault == 'sensor_loss':
            report['passed'] = not report['health']['scan_ready'] and time.monotonic() - records[-1]['arrival'] > .5
        else:
            report['passed'] = report['gaps_over_035'] == 0 and report['health']['scan_ready']
            if a.fault == 'missing_last':
                report['passed'] &= report['missing_sector_observed']
        if a.require_receipt_timing:
            report['passed'] &= timing_ok
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
                minimum_depth_gap_s=min(gaps) if gaps else None,
                maximum_depth_gap_s=max(gaps) if gaps else None,
                skipped_repeated_framesets=camera.repeated_framesets,
                skipped_repeated_stream_frames=camera.repeated_stream_frames,
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
        if archive:
            report['native_rgbd_archive'] = dict(frames=archive.count, period_s=archive.period,
                directory='rgbd', clock='SDK global_time converted to same-host monotonic',
                registration_applied=False, physical_sync_qualified=False)
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
