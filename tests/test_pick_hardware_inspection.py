from dataclasses import dataclass, field
from types import SimpleNamespace

import pytest

from tools.diagnostics.inspect_pick_hardware import (
    FILES, cached_controller, inspect_serial, installed_files, process_inventory,
)


@dataclass
class Status:
    protocol_version: int = 2
    joint_count: int = 12
    firmware_version: int = 0x2492d
    boot_id: int = 9
    confirmed: bool = True
    kind: int = 4
    request_sequence: int = 1
    positions: list = field(default_factory=list)
    profile_crc32: int = 2430209922


@pytest.mark.parametrize('firmware,profile_crc32', [
    (0x2492d,2430209922), (0x2492e,2696088964),
    (0x2492f,3399362976), (0x24930,3399362976), (0x24930,1),
])
def test_controller_only_reads_even_when_stop_latched(monkeypatch, firmware, profile_crc32):
    from so101_arm_bridge import unified_commissioning, system_stop, lift_wire
    events = []
    class Transport:
        def enter_binary_mode(self):
            events.append('hello'); return Status(firmware_version=firmware)
        def stationary_runtime(self):
            events.append('runtime'); return {'stop_latched': True}
        def stationary_feedback_failures(self):
            events.append('failures'); return {}
        def get_feedback_snapshot(self):
            events.append('joints'); return Status(positions=[1, 2])
        def exchange_mobile(self, packet):
            command = lift_wire.LiftCommand.decode(packet)
            assert command.kind == 4 and command.valid_until_ms == 0 and command.target_um == 0
            events.append('lift'); return b'status'
    class Client:
        def __init__(self, *_): pass
        def query(self): events.append('commission'); return Status(profile_crc32=profile_crc32)
    class Stop:
        def __init__(self, *_): pass
        def status(self): events.append('stop_query'); return Status(), 1.
        def request(self): raise AssertionError('must not stop another controller')
    monkeypatch.setattr(unified_commissioning, 'CommissionClient', Client)
    monkeypatch.setattr(system_stop, 'SystemStopClient', Stop)
    monkeypatch.setattr(lift_wire.LiftStatus, 'decode', lambda _: Status())
    result = cached_controller(Transport())
    if profile_crc32 == 1:
        assert set(result['errors']) == {'lift_scale_um_per_revolution'}
        assert 'lift_scale_um_per_revolution' not in result
    else:
        assert not result['errors']
        assert result['lift_scale_um_per_revolution'] == (84000 if firmware == 0x2492d else 87287)
    assert events == ['hello', 'runtime', 'failures', 'joints', 'commission', 'stop_query', 'lift']
    assert result['motion_commands'] == result['torque_commands'] == 0


def test_wrong_firmware_does_not_query_other_subsystems():
    result = cached_controller(SimpleNamespace(enter_binary_mode=lambda: Status(firmware_version=1)))
    assert 'firmware' in result['errors'] and 'lift' not in result


@pytest.mark.parametrize('busy', ['programs', 'serial_owners'])
def test_busy_does_not_even_load_protocol_or_open_device(tmp_path, busy):
    processes = dict(programs=[], serial_owners=[], unreadable_same_user=[])
    processes[busy] = [123]
    assert inspect_serial(tmp_path, '/no/device', processes)['status'] == 'SKIPPED_BUSY_OR_OWNERSHIP_UNCERTAIN'


def test_hidden_process_is_not_treated_as_a_known_device_owner(tmp_path):
    processes = dict(programs=[], serial_owners=[], unreadable_same_user=[123])
    assert inspect_serial(tmp_path, str(tmp_path/'absent'), processes)['status'] == 'DEVICE_MISSING'


def test_inventory_detects_process_and_open_descriptor(tmp_path, monkeypatch):
    monkeypatch.setattr('tools.diagnostics.inspect_pick_hardware.camera_nodes', lambda: {'/dev/video2'})
    proc = tmp_path/'proc'
    pid = proc/'123456789'
    (pid/'fd').mkdir(parents=True)
    (pid/'cmdline').write_bytes(b'python\0tools/run/unified_pick.py\0')
    (pid/'fd'/'5').symlink_to('/dev/ttyACM0')
    (pid/'fd'/'6').symlink_to('/dev/video2')
    result = process_inventory('/dev/ttyACM0', proc)
    assert result['serial_owners'] == result['camera_owners'] == [123456789]
    assert result['programs'][0]['pid'] == 123456789


def test_capture_reuses_viewer_and_preserves_one_snapshot(monkeypatch):
    import base64
    import json
    from pathlib import Path
    from tools import diagnostics
    from tools.diagnostics.inspect_pick_hardware import capture_snapshot

    def run(args):
        assert args[args.index('--frames') + 1] == '3' and '--headless' in args
        dest = Path(args[args.index('--output') + 1])/'fake-exposure'
        dest.mkdir()
        (dest/'metadata.json').write_text(json.dumps(dict(valid_depth_percent=51.3, captured_at_utc='sample')))
        (dest/'depth_aligned.png').write_bytes(b'depth-image')
        (dest/'color.png').write_bytes(b'color-image')
    monkeypatch.setattr(diagnostics, 'realsense_viewer', SimpleNamespace(parse_args=list, run=run), raising=False)
    capture = capture_snapshot()
    assert capture['snapshot_status'] == 'RGBD_CAPTURED' and capture['valid_depth_percent'] == 51.3
    assert base64.b64decode(capture['snapshot_files']['depth_aligned.png']) == b'depth-image'


def test_service_presence_requires_all_sampled_files(tmp_path):
    assert not installed_files(tmp_path)['pick_service_installed']
    for name in FILES[5:]:
        path = tmp_path/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('test')
    report = installed_files(tmp_path)
    assert report['pick_service_installed']
    assert report['files_sha256'][FILES[0]] is None
