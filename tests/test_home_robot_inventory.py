"""Passive S0 collection must not mistake metadata for verified hardware."""

import importlib.util
import json
from pathlib import Path


_path = Path(__file__).resolve().parents[1] / "tools/diagnostics/home_robot_inventory.py"
_spec = importlib.util.spec_from_file_location("home_robot_inventory", _path)
inventory = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(inventory)


def test_missing_metadata_stays_unknown(tmp_path):
    report = inventory.collect(tmp_path)
    assert report["usb"]["error"] == "FileNotFoundError"
    assert report["serial_links"]["/dev/serial/by-id"]["error"] == "FileNotFoundError"
    assert report["system"]["/etc/nv_tegra_release"]["status"] == "unavailable"
    assert report["verification"]["active_stm32_firmware"] is None
    assert not report["verification"]["physical_configuration_verified"]


def test_usb_and_serial_metadata_never_open_a_device(tmp_path, monkeypatch):
    usb = tmp_path / "sys/bus/usb/devices/1-1"
    usb.mkdir(parents=True)
    (usb / "idVendor").write_text("0483\n")
    (usb / "product").write_text("USB bridge\n")
    serial_dir = tmp_path / "dev/serial/by-id"
    serial_dir.mkdir(parents=True)
    (serial_dir / "example-bridge").symlink_to("../../ttyACM0")
    original_open = Path.open

    def guarded_open(path, *args, **kwargs):
        if path.is_relative_to(tmp_path / "dev"):
            raise AssertionError("S0 must not open device nodes")
        return original_open(path, *args, **kwargs)

    monkeypatch.setattr(Path, "open", guarded_open)
    report = inventory.collect(tmp_path)
    assert report["usb"]["entries"][0]["product"]["value"] == "USB bridge"
    assert report["serial_links"]["/dev/serial/by-id"]["entries"] == [
        {"name": "example-bridge", "target": "../../ttyACM0"}]
    assert not report["verification"]["installed_motor_ids_verified"]
    assert report["verification"]["active_stm32_firmware"] is None


def test_environment_capture_is_allowlisted(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBOT_API_KEY", "secret-not-for-inventory")
    monkeypatch.setenv("ROS_DISTRO", "test-distribution")
    report = inventory.collect(tmp_path)
    assert report["ros"]["environment"]["ROS_DISTRO"] == "test-distribution"
    assert "secret-not-for-inventory" not in json.dumps(report)


def test_bad_usb_descriptor_does_not_hide_other_device_metadata(tmp_path):
    usb=tmp_path/'sys/bus/usb/devices/1-1';usb.mkdir(parents=True)
    (usb/'idVendor').write_text('0483')
    (usb/'product').write_bytes(b'\xff')
    row=inventory.collect(tmp_path)['usb']['entries'][0]
    assert row['idVendor']['value']=='0483'
    assert row['product']=={'status':'unavailable','reason':'UnicodeDecodeError'}
