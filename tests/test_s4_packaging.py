import hashlib
import json
import tarfile
from tools.setup import package_stationary_pick as packaging


def test_s4_source_release_is_hashed_and_contains_no_implicit_flash(tmp_path,monkeypatch):
    root=tmp_path/'source';root.mkdir()
    paths={'ros2_ws/src/home_robot_tasks/home_robot_tasks/saved_map_node.py':'# source\n',
        'config/home_assistant/s4_commissioning.pending.json':'{}',
        'home_perception/contracts.py':'# perception\n','tools/lib/helper.py':'# shared\n',
        'ros2_ws/src/so101_interfaces/msg/State.msg':'bool ready\n','tests/native.c':'/* fixture */',
        'ros2_ws/src/hls_lfcd_lds_driver/src/driver.cpp':'// source',
        'ros2_ws/src/hls_lfcd_lds_driver/include/decoder.hpp':'// header',
        'ros2_ws/src/hls_lfcd_lds_driver/LICENSE':'BSD notice',
        'config/pytest.ini':'[pytest]\n','config/private.local.json':'private','tests/test_example.py':'# test\n',
        'firmware/stm32_g474_single_arm/Core/Src/main.c':'/* source */',
        'firmware/stm32_g474_single_arm/build/firmware.hex':'not packaged',
        'ros2_ws/src/home_robot_tasks/home_robot_tasks/__pycache__/bad.py':'not packaged'}
    for name,body in paths.items():
        p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(body)
    outside=tmp_path/'private.json';outside.write_text('private')
    (root/'config/external.json').symlink_to(outside)
    monkeypatch.setattr(packaging,'ROOT',root)
    folder=packaging.package_s4(tmp_path/'release')
    manifest=json.loads((folder/'release.json').read_text())
    assert not manifest['hardware_enabled'] and not manifest['jetson_deployed'] and manifest['no_firmware_binary']
    assert {'home_perception/contracts.py','tools/lib/helper.py','ros2_ws/src/so101_interfaces/msg/State.msg','tests/native.c'} <= manifest['files_sha256'].keys()
    assert 'config/pytest.ini' in manifest['files_sha256'] and 'tests/test_example.py' in manifest['files_sha256']
    for name in ('src/driver.cpp','include/decoder.hpp','LICENSE'):
        assert 'ros2_ws/src/hls_lfcd_lds_driver/'+name in manifest['files_sha256']
    for name,sha in manifest['files_sha256'].items():
        assert hashlib.sha256((folder/name).read_bytes()).hexdigest()==sha
        assert '__pycache__' not in name and '/build/' not in name and '.local.' not in name and name!='config/external.json'
    with tarfile.open(folder.parent/(folder.name+'.tar.gz')) as archive:
        assert all(n.startswith(folder.name+'/') or n==folder.name for n in archive.getnames())
    assert not (folder/'flash.sh').exists()


def test_s4_inspection_package_reuses_backup_and_has_no_automatic_install(tmp_path,monkeypatch):
    import os
    root=tmp_path/'source';build=root/'build/s1-lift-measurement';build.mkdir(parents=True)
    cache='MOBILE_BUS_INSPECTION_ONLY:BOOL=ON\nACTUATOR_MOBILE_FIRST_ID:STRING=7\nMOBILE_BOARD_PROFILE_HEADER:FILEPATH=\n'
    (build/'CMakeCache.txt').write_text(cache)
    paths=['tools/run/probe_mobile_bus.py','tools/diagnostics/analyze_wheel_capture.py',
           'ros2_ws/src/home_robot_tasks/home_robot_tasks/review.py',
           'ros2_ws/src/so101_arm_bridge/so101_arm_bridge/serial_port.py',
           'firmware/stm32_g474_single_arm/Core/Src/mobile_bus_inspection_app.c']
    for name in paths:
        p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('// fixture\n')
        os.utime(p,(1,1))
    for name in ('flash.sh','program.cfg','verify-backup.py'):
        p=root/'tools/setup/s2_deployment'/name;p.parent.mkdir(parents=True,exist_ok=True)
        p.write_bytes((packaging.ROOT/'tools/setup/s2_deployment'/name).read_bytes())
    (build/'stm32_g474_single_arm.hex').write_text(':00000001FF\n')
    monkeypatch.setattr(packaging,'ROOT',root)
    folder=packaging.package_s4_inspection(tmp_path/'out')
    manifest=json.loads((folder/'release.json').read_text())
    assert not manifest['motor_writes_permitted'] and not manifest['flashed'] and manifest['not_navigation_firmware']
    assert '--load-supported-motor-power-retained' in (folder/'flash.sh').read_text()
    installer=(folder/'flash.sh').read_text()
    assert installer.index('S4 inspection authentication')<installer.index('mkdir -p "$logs"')
    program=(folder/'program.cfg').read_text()
    assert program.index('dump_image')<program.index('Incomplete firmware backup')<program.index('flash write_image erase')
    assert 'boot-journal-after.bin' in program
    (build/'CMakeCache.txt').write_text(cache+'MOBILE_WHEEL_PULSE_ONLY:BOOL=ON\n')
    import pytest
    with pytest.raises(ValueError,match='another controller'):packaging.package_s4_inspection(tmp_path/'bad')
