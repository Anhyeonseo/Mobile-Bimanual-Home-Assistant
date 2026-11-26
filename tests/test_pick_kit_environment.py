"""The kit must see the Jetson's user-installed ROS and both existing venvs."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import venv


def test_existing_jetson_environments_and_explicit_python(tmp_path):
    root = Path(__file__).resolve().parents[1]
    kit = tmp_path/'kit'
    for script in ('tools/run/pick_kit.sh', 'tools/lib/pick_ros_env.sh'):
        target = kit/script
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root/script, target)
    (kit/'SHA256SUMS').touch()
    (kit/'tools/run/pick_service.py').write_text(
        'import os, sys, json, importlib.util\n'
        'print(json.dumps([sys.executable, os.environ["PICK_TEST_ROS"], '
        'importlib.util.find_spec("control_only") is not None]))\n'
    )
    prefix = tmp_path/'jetson'
    runtime = prefix/'robot/shared/runtime/s4-jazzy'
    (runtime/'install').mkdir(parents=True)
    (runtime/'env.sh').write_text('export PICK_TEST_ROS=base\n')
    (runtime/'install/setup.bash').write_text('export PICK_TEST_ROS="$PICK_TEST_ROS/overlay"\n')
    vision = prefix/'realsense-grounding/.venv'
    venv.EnvBuilder(with_pip=False).create(vision)
    control = prefix/'robot/shared/venvs/s4-jazzy'
    control_site = control/f'lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages'
    control_site.mkdir(parents=True)
    (control_site/'control_only.py').touch()
    env = dict(os.environ, PICK_JETSON_HOME=str(prefix), VIRTUAL_ENV=str(control))
    env.pop('PICK_PYTHON', None)
    cmd = ['bash', str(kit/'tools/run/pick_kit.sh'), 'env']
    def run():
        return json.loads(subprocess.check_output(cmd, env=env, text=True))
    assert run() == [str(vision/'bin/python'), 'base/overlay', True]
    env['PICK_PYTHON'] = sys.executable
    assert run() == [sys.executable, 'base/overlay', False]
    env['PICK_PYTHON'] = str(tmp_path/'missing-python')
    assert subprocess.run(cmd, env=env, capture_output=True).returncode == 2
    env.update(PICK_PYTHON=sys.executable, PICK_WEB_PORT='18770')
    (kit/'tools/run/pick_service.py').write_text('import sys, json\nprint(json.dumps(sys.argv[1:]))\n')
    web_args = json.loads(subprocess.check_output(cmd[:-1]+['web'], env=env, text=True))
    assert web_args[web_args.index('--port')+1] == '18770'
    assert web_args[web_args.index('--mode')+1] == 'station'
    assert web_args[web_args.index('--stage-timeout-seconds')+1] == '0'
