"""Exercise the actual deployment artifact with no installed robot packages."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

import pytest
from tools.diagnostics.package_unified_lift import build, collect

ROOT=Path(__file__).resolve().parents[1]


def run_environment(entry, config_root, baseline, cwd):
    # A real isolated interpreter avoids accidentally passing via the developer
    # PYTHONPATH. A fake old release deliberately cannot supply the new module.
    script='''
import runpy, sys, serial
def forbidden(*args, **kwargs):
    raise AssertionError("SERIAL_DEVICE_OPENED")
serial.Serial = forbidden
entry, root, baseline = sys.argv[1:]
sys.argv = [entry, "--root", root, "--baseline", baseline, "--check-environment"]
runpy.run_path(entry, run_name="__main__")
'''
    return subprocess.run([sys.executable,'-I','-c',script,str(entry),str(config_root),str(baseline)],
                          cwd=cwd,capture_output=True,text=True,timeout=20)


@pytest.fixture
def packaged(tmp_path):
    result=build(ROOT,tmp_path/'built')
    destination=tmp_path/'jetson_shared';destination.mkdir()
    with tarfile.open(result['archive']) as archive:
        names=archive.getnames()
        assert all(not n.startswith('/') and '..' not in Path(n).parts for n in names)
        assert not any('/config/' in n or '/.git/' in n for n in names)
        archive.extractall(destination,filter='data')
    folder=destination/Path(result['folder']).name
    old=tmp_path/'old_release'
    for name in ('arm_ranges.development.json','lift_geometry.measured.json'):
        target=old/'config/home_assistant'/name;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/'config/home_assistant'/name,target)
    poison=old/'ros2_ws/src/home_robot_tasks/home_robot_tasks'
    poison.mkdir(parents=True)
    (poison/'__init__.py').write_text('raise RuntimeError("OLD_RELEASE_CODE_IMPORTED")\n')
    baseline=tmp_path/'baseline.json';baseline.write_text('{}')
    return folder,old,baseline


def test_dependency_closure_includes_lazy_probe_dependencies():
    modules,files,_=collect(ROOT)
    for module in ('home_robot_tasks.platform_preparation','home_robot_tasks.device_ports',
                   'home_robot_tasks.system_stop_backend','home_robot_tasks.execution',
                   'so101_arm_bridge.unified_arm_adapter','prepare_unified_controller'):
        assert modules[module] in files


def test_artifact_imports_all_dependencies_without_deployed_module_or_serial_access(packaged,tmp_path):
    folder,old,baseline=packaged
    result=run_environment(folder/'tools/run/unified_lift.py',old,baseline,tmp_path)
    assert result.returncode==0,result.stdout+result.stderr
    report=json.loads(result.stdout)
    manifest=json.loads((folder/'runtime_manifest.json').read_text())
    assert report['status']=='ENVIRONMENT_OK'
    assert report['bundled_module_count']==len(manifest['modules'])
    assert 'so101_arm_bridge.servo_tuning' in manifest['modules']
    assert report['serial_opened'] is False and report['motion_commands']==report['torque_commands']==0
    assert report['code_root']==str(folder) and report['config_root']==str(old)
    assert 'OLD_RELEASE_CODE_IMPORTED' not in result.stdout+result.stderr


@pytest.mark.parametrize('damage',['missing','changed'])
def test_incomplete_or_modified_bundle_stops_before_controller_access(packaged,tmp_path,damage):
    folder,old,baseline=packaged
    helper=folder/'ros2_ws/src/home_robot_tasks/home_robot_tasks/platform_preparation.py'
    if damage=='missing':helper.unlink()
    else:helper.write_text(helper.read_text()+'\n# changed after packaging\n')
    result=run_environment(folder/'tools/run/unified_lift.py',old,baseline,tmp_path)
    assert result.returncode==2
    assert ('missing bundled dependency' if damage=='missing' else 'bundled dependency changed') in result.stderr
    assert 'SERIAL_DEVICE_OPENED' not in result.stdout+result.stderr
