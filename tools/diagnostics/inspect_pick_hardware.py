#!/usr/bin/env python3
"""One-shot Jetson inventory and cached telemetry; never activates a robot.

Can be sent through SSH stdin. Optional --capture takes one D415 snapshot.
Does not deploy, renew a controller lease,
prepare modes, reference the lift, reset, STOP, or send motor commands. The
serial backend's exclusive ownership guard is still used for status queries.
"""
import argparse
import base64
import contextlib
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import sys
import tempfile
import time

DEFAULT_ROOT = Path('/home/hyper/robot/releases/s4-unified-pick-20261116')
DEVICE = '/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_005100393235511438363730-if02'
FILES = tuple('config/home_assistant/'+name for name in (
    's2_coordinates.development.json', 's2_coordinates.wide.development.json',
    'camera_height.calibrated.json', 'left_pad_gap.observed.json',
    'measured_calibration.development.json')) + (
    'tools/run/pick_service.py', 'tools/lib/pick_compute.py',
    'ros2_ws/src/home_robot_tasks/home_robot_tasks/pick_hardware.py',
    'ros2_ws/src/home_robot_tasks/home_robot_tasks/pick_capture.py')
MODULES = ('serial', 'pyrealsense2', 'numpy', 'scipy', 'cv2', 'PIL',
           'torch', 'torchvision', 'transformers', 'yaml', 'rclpy', 'moveit_msgs')


def installed_files(root):
    files = {}
    for name in FILES:
        path = root/name
        files[name] = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
    return dict(root=str(root), files_sha256=files,
                pick_service_installed=all(files[name] for name in FILES[5:]))


def camera_nodes(usb=Path('/sys/bus/usb/devices'), video=Path('/sys/class/video4linux')):
    nodes = set()
    for entry in usb.glob('*'):
        try:
            if 'D415' not in (entry/'product').read_text().upper():
                continue
            bus = int((entry/'busnum').read_text())
            dev = int((entry/'devnum').read_text())
            nodes.add(f'/dev/bus/usb/{bus:03d}/{dev:03d}')
            for camera in video.glob('video*'):
                if entry.resolve() in (camera/'device').resolve().parents:
                    nodes.add('/dev/'+camera.name)
        except (OSError, ValueError):
            continue
    return nodes


def process_inventory(device, proc=Path('/proc')):
    """Report matching executables and device owners, without terminating any."""
    result = dict(programs=[], serial_owners=[], camera_owners=[], unreadable_same_user=[])
    wanted = str(Path(device).resolve())
    camera_devices = camera_nodes()
    for entry in proc.iterdir():
        if not entry.name.isdecimal() or int(entry.name) == os.getpid():
            continue
        try:
            args = (entry/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace').strip()
            if any(name in args for name in ('unified_pick.py', 'unified_lift.py',
                                             'recover_supported_lift.py', 'pick_service.py')):
                result['programs'].append(dict(pid=int(entry.name), command=args[:1200]))
            owned = set()
            for fd in (entry/'fd').iterdir():
                try:
                    owned.add(os.readlink(fd))
                except FileNotFoundError:
                    pass
            if wanted in owned:
                result['serial_owners'].append(int(entry.name))
            if owned & camera_devices or (not camera_devices and any(p.startswith('/dev/video') for p in owned)):
                result['camera_owners'].append(int(entry.name))
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            try:
                if entry.stat().st_uid == os.getuid():
                    result['unreadable_same_user'].append(int(entry.name))
            except FileNotFoundError:
                pass
    return result


def cached_controller(transport):
    from so101_arm_bridge.unified_commissioning import CommissionClient, PICK_BOARDS, pick_board_scale
    from so101_arm_bridge.system_stop import SystemStopClient
    from so101_arm_bridge.lift_wire import LiftCommand, LiftStatus
    result = dict(errors={}, motion_commands=0, torque_commands=0)
    def read(name, operation):
        try:
            result[name] = operation()
            return result[name]
        except Exception as error:
            result['errors'][name] = str(error)
    hello = read('hello', lambda: asdict(transport.enter_binary_mode()))
    if hello is None:
        return result
    firmware = hello['firmware_version']
    # Use the installed host's explicit pick-board support table, not its
    # legacy commissioning default. This remains a query-only diagnostic.
    if (firmware not in PICK_BOARDS or hello.get('protocol_version') != 2
            or hello.get('joint_count') != 12):
        result['errors']['firmware'] = f'Unsupported telemetry identity: board={firmware:#x}'
        return result
    read('runtime', transport.stationary_runtime)
    read('motor_read_failures', transport.stationary_feedback_failures)
    read('joints', lambda: asdict(transport.get_feedback_snapshot()))
    commission = read('commission', lambda: asdict(CommissionClient(transport).query()))
    if commission is not None:
        read('lift_scale_um_per_revolution', lambda: pick_board_scale(firmware, commission['profile_crc32']))
    boot = (commission or {}).get('boot_id')
    if type(boot) is int and boot > 0:
        def proof():
            state, observed = SystemStopClient(transport, boot, time.monotonic).status()
            return dict(status=asdict(state), confirmed=state.confirmed, observed_host_s=observed)
        read('stop_proof', proof)
        def lift():
            command = LiftCommand(4, 1, 1, 0, boot)
            state = LiftStatus.decode(transport.exchange_mobile(command.encode()))
            if state.kind != 4 or state.request_sequence != 1 or state.boot_id != boot:
                raise ValueError('lift status identity mismatch')
            return asdict(state)
        read('lift', lift)
    else:
        result['errors']['boot'] = 'No current controller boot identity'
    return result


def inspect_serial(root, device, processes):
    # /proc may hide unrelated processes (including sshd). Cooperating robot
    # programs are serialized by acquire_backend_lease and the port lock below.
    if processes['serial_owners'] or processes['programs']:
        return dict(status='SKIPPED_BUSY_OR_OWNERSHIP_UNCERTAIN')
    if not Path(device).exists():
        return dict(status='DEVICE_MISSING')
    if not (root/'ros2_ws/src/so101_arm_bridge/so101_arm_bridge').is_dir():
        return dict(status='PROTOCOL_RUNTIME_MISSING', root=str(root))
    sys.path.insert(0, str(root/'ros2_ws/src/so101_arm_bridge'))
    try:
        import serial
        from so101_arm_bridge.backend_lease import acquire_backend_lease
        from so101_arm_bridge.serial_port import open_exclusive_serial
        from so101_arm_bridge.stream_transport_v2 import StreamValidationTransportV2
        with acquire_backend_lease('stm32', 189):
            with open_exclusive_serial(serial, device, 921600, .02) as port:
                state = cached_controller(StreamValidationTransportV2(port, response_timeout_s=1))
        return dict(status='CACHED_STATUS_READ' if not state['errors'] else 'PARTIAL_STATUS', **state)
    except Exception as error:
        return dict(status='QUERY_FAILED', error=str(error))


def inspect_camera(processes, *, capture=False):
    if processes['camera_owners'] or processes['programs']:
        return dict(status='SKIPPED_BUSY_OR_OWNERSHIP_UNCERTAIN', stream_started=False)
    try:
        import pyrealsense2 as rs
        cameras = []
        for device in rs.context().query_devices():
            record = {}
            for name in ('name', 'serial_number', 'firmware_version', 'usb_type_descriptor'):
                info = getattr(rs.camera_info, name)
                if device.supports(info):
                    record[name] = device.get_info(info)
            cameras.append(record)
        result = dict(status='ENUMERATED' if cameras else 'NO_CAMERA', devices=cameras, stream_started=False)
        if capture and cameras:
            result['snapshot_status'] = 'CAPTURE_ATTEMPTED'
            try:
                result.update(capture_snapshot())
            except Exception as error:
                result.update(snapshot_status='CAPTURE_FAILED', capture_error=str(error), stream_started=None)
        return result
    except Exception as error:
        return dict(status='QUERY_FAILED', error=str(error), stream_started=False)


SNAPSHOT_FILES = ('color.png', 'depth_aligned.png', 'metadata.json')


def capture_snapshot():
    from tools.diagnostics import realsense_viewer as viewer
    with tempfile.TemporaryDirectory(prefix='pick-d415-') as folder:
        args = viewer.parse_args(['--headless', '--frames', '3', '--save-on-exit',
                                  '--width', '640', '--height', '480', '--depth-width', '640',
                                  '--depth-height', '360', '--depth-filter', 'off', '--output', folder])
        with contextlib.redirect_stdout(sys.stderr):
            viewer.run(args)
        snapshots = list(Path(folder).glob('*/metadata.json'))
        if len(snapshots) != 1:
            raise ValueError('expected exactly one snapshot from this camera session')
        metadata = json.loads(snapshots[0].read_text())
        return dict(snapshot_status='RGBD_CAPTURED' if metadata['valid_depth_percent'] > 0
                    else 'NO_VALID_DEPTH', stream_started=True,
                    valid_depth_percent=metadata['valid_depth_percent'],
                    captured_at_utc=metadata['captured_at_utc'],
                    snapshot_files={name: base64.b64encode((snapshots[0].parent/name).read_bytes()).decode('ascii')
                                    for name in SNAPSHOT_FILES})


def collect(root, robot_home, device, *, capture=False):
    processes = process_inventory(device)
    installs = {root}
    for parent, pattern in ((robot_home/'releases', '*'), (robot_home/'shared', 'unified-lift-*')):
        if parent.is_dir():
            installs.update(p for p in parent.glob(pattern) if p.is_dir())
    missing = []
    for name in MODULES:
        try:
            if importlib.util.find_spec(name) is None:
                missing.append(name)
        except (ImportError, ValueError):
            missing.append(name)
    controller = inspect_serial(root, device, processes)
    camera = inspect_camera(processes, capture=capture)
    jazzy_setups = [str(p) for p in (robot_home/'shared/runtime/s4-jazzy/env.sh',
                                     Path('/opt/ros/jazzy/setup.bash')) if p.is_file()]
    return dict(schema_version=1, kind='pick_hardware_inspection',
                checked_at_utc=datetime.now(timezone.utc).isoformat(),
                host=platform.node(), architecture=platform.machine(), python=sys.executable,
                motion_commands=0, torque_commands=0, reset_commands=0, reference_commands=0,
                stream_started=camera['stream_started'], robot_execution_ready=False,
                processes=processes, missing_modules_in_this_python=missing,
                ros_jazzy_installed=bool(jazzy_setups), ros_jazzy_setup_files=jazzy_setups,
                installations=[installed_files(p) for p in sorted(installs) if p.is_dir()],
                serial_devices=[str(p) for p in Path('/dev/serial/by-id').glob('*')],
                baseline_present=(robot_home/'shared/diagnostics/s4-trial-20261114/baseline.json').is_file(),
                controller=controller, camera=camera,
                note='Inventory and cached telemetry only; not a fresh RGB-D/pose pair or motion admission.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    p.add_argument('--robot-home', type=Path, default=Path('/home/hyper/robot'))
    p.add_argument('--port', default=DEVICE)
    p.add_argument('--capture', action='store_true', help='also acquire and save a fresh D415 RGB-D snapshot')
    p.add_argument('--compare', type=Path, help='locally compare a saved report; no devices')
    args = p.parse_args()
    if args.compare:
        report = json.loads(args.compare.read_text())
        expected = installed_files(args.root)['files_sha256']
        for install in report['installations']:
            install['matches_local_files'] = {
                name: digest is not None and digest == install['files_sha256'].get(name)
                for name, digest in expected.items()}
        snapshot_files = report['camera'].pop('snapshot_files', None)
        if snapshot_files:
            directory = args.compare.parent/'cam'
            directory.mkdir(exist_ok=True)
            for name in SNAPSHOT_FILES:
                (directory/name).write_bytes(base64.b64decode(snapshot_files[name], validate=True))
            print('SNAPSHOT:', directory)
        target = args.compare.with_name('comparison.json')
        target.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print('HOST:', report['host'])
        processes = report.get('processes', {})
        programs = processes.get('programs', [])
        for program in programs:
            print('RUNNING_PROGRAM:', program['pid'], program['command'])
        if not programs:
            print('RUNNING_PROGRAM: no recognized pick controller found')
        print('SERIAL_OWNERS:', processes.get('serial_owners', []))
        print('CONTROLLER:', report['controller']['status'])
        controller = report['controller']
        if controller.get('hello'):
            print('CONTROLLER_FIRMWARE:', hex(controller['hello']['firmware_version']),
                  'PROFILE_CRC32:', (controller.get('commission') or {}).get('profile_crc32'))
            print('STOP_LATCHED:', controller['hello'].get('stop_latched'))
        print('CAMERA:', report['camera']['status'])
        if 'snapshot_status' in report['camera']:
            print('SNAPSHOT_STATUS:', report['camera']['snapshot_status'], report['camera'].get('capture_error', ''))
        if report['controller'].get('error') or report['controller'].get('errors'):
            print('CONTROLLER_DETAILS:', report['controller'].get('error') or report['controller']['errors'])
        print('MISSING_MODULES:', ', '.join(report['missing_modules_in_this_python']) or 'none')
        for install in report['installations']:
            if not install['pick_service_installed'] and not any(install['matches_local_files'].values()):
                continue
            print('INSTALLATION:', install['root'],
                  'pick_service='+str(install['pick_service_installed']),
                  'matching_files='+str(sum(install['matches_local_files'].values()))+'/'+str(len(FILES)))
        print('REPORT:', target)
        print('MOTION_COMMANDS: 0; ROBOT_EXECUTION_READY: false')
    else:
        print(json.dumps(collect(args.root, args.robot_home, args.port, capture=args.capture),
                         ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
