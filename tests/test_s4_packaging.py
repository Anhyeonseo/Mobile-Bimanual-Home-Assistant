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
    for name,sha in manifest['files_sha256'].items():
        assert hashlib.sha256((folder/name).read_bytes()).hexdigest()==sha
        assert '__pycache__' not in name and '/build/' not in name and '.local.' not in name and name!='config/external.json'
    with tarfile.open(folder.parent/(folder.name+'.tar.gz')) as archive:
        assert all(n.startswith(folder.name+'/') or n==folder.name for n in archive.getnames())
    assert not (folder/'flash.sh').exists()
