"""Exercise an unpacked station without opening a camera, serial port or robot driver."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from urllib.request import ProxyHandler, Request, build_opener

ROOT = Path(__file__).resolve().parents[2]
TASK_PATHS = [ROOT, ROOT/'ros2_ws/src/home_robot_tasks', ROOT/'ros2_ws/src/so101_arm_bridge', ROOT/'tools/run']
sys.path[:0] = list(map(str, TASK_PATHS))


def demo_http(output):
    from home_robot_tasks.pick_demo import DemoBackend
    from home_robot_tasks.pick_service import PickStation
    from home_robot_tasks.pick_web import create_server
    station = PickStation(DemoBackend(step_seconds=0), output)
    with create_server(station, 0) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        station.start()
        try:
            url = f'http://127.0.0.1:{server.server_port}'
            opener = build_opener(ProxyHandler({}))
            def get(path):
                with opener.open(url+path, timeout=3) as resp:
                    return resp.read()
            token = re.search(rb"const token='([^']+)'", get('/')).group(1).decode()
            def ready():
                deadline = time.monotonic()+10
                while time.monotonic() < deadline:
                    status = json.loads(get('/api/status'))
                    if status['accepting']:
                        return status
                    time.sleep(.05)
                raise TimeoutError('demo did not return to READY')
            ready()
            for text, prompt in [('지우개 집어줘', 'eraser'), ('pick up the blue cup', 'blue cup')]:
                request = Request(url+'/api/requests', data=json.dumps({'text':text}).encode(),
                                  headers={'Content-Type':'application/json','X-Pick-Token':token})
                with opener.open(request, timeout=3) as resp:
                    run_id = json.load(resp)['id']
                status = ready()
                result = json.loads((output/run_id/'result.json').read_text())
                if (result['prompt'] != prompt or result['result']['status'] != 'SUCCEEDED' or result['result']['physical_success']
                        or len(result['stages']) != 6 or any(s['status'] != 'done' for s in result['stages'])):
                    raise ValueError('incomplete demo cycle')
            print('Two demo HTTP requests completed', flush=True)
        finally:
            server.shutdown()
            thread.join()
            station.close()


def model_description():
    from tools.diagnostics.run_s3_stationary_lab import model
    from home_robot_tasks import carry_model, s2_geometry
    from home_robot_tasks.camera_evidence import load_capture_profile
    # Import the live backend as on the target, without creating it or preparing hardware.
    from home_robot_tasks.pick_hardware import UnifiedPickHardware
    import pyrealsense2
    import serial
    import rclpy
    from moveit_msgs.srv import GetPlanningScene
    from ament_index_python.packages import get_package_prefix
    from home_robot_tasks.pick_calibration import verify_installed
    print('Verified field calibration', verify_installed(ROOT))
    profile, _ = load_capture_profile(ROOT, 'wide')
    from so101_arm_bridge.unified_commissioning import check_pick_calibration
    from home_robot_tasks.unified_carry_inputs import controller_lift_height_m
    observed_board = (0x2492E, 2696088964)
    check_pick_calibration(*observed_board, profile)
    assert abs(controller_lift_height_m(10000, profile, board=observed_board) - .010) < 1e-7
    geometry = s2_geometry.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
    params = carry_model.parameters(ROOT, profile, geometry,
                                    {side:model(0, side, profile=profile) for side in ('left','right')},
                                    ROOT/'assets/mobile_meshes')
    for name in ('moveit_ros_move_group','moveit_kinematics','moveit_planners_ompl','robot_state_publisher'):
        get_package_prefix(name)
    if params['allow_trajectory_execution']:
        raise ValueError('simulation has trajectory execution enabled')
    print('Calibrated URDF/SRDF constructed', profile['calibration_id'])
    return params


def planner(output):
    from moveit_msgs.msg import RobotState
    from home_robot_tasks.s2_geometry import JOINTS
    from tools.diagnostics.carry_moveit_session import PersistentSession
    from moveit_msgs.srv import GetPositionFK
    params = model_description()
    state = RobotState()
    state.joint_state.name = ['s2_lift_joint']+[side+'_model_'+j for side in ('left','right') for j in JOINTS]
    state.joint_state.position = [0.0]*len(state.joint_state.name)
    pool = PersistentSession()
    try:
        for _ in range(2):
            with pool(params, state, output) as call:
                query = GetPositionFK.Request(robot_state=state, fk_link_names=['left_tcp'])
                query.header.frame_id = 'base_link'
                response = call('compute_fk', query)
                if response.error_code.val != 1 or not response.pose_stamped:
                    raise RuntimeError('calibrated synthetic FK failed')
        if pool.starts != 1:
            raise RuntimeError('expected one resident planner')
    finally:
        pool.close()


def run_step(command, log, timeout):
    with log.open('w') as stream:
        proc = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, cwd=ROOT,
                                start_new_session=True)
        try:
            return proc.wait(timeout)
        finally:
            # ROS probes launch children. Interrupt and timeout must release them as well.
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--probe', choices=['geometry','web','planner'])
    args = parser.parse_args()
    if args.probe:
        if args.probe == 'geometry':
            model_description()
        elif args.probe == 'web':
            demo_http(args.output)
        else:
            planner(args.output)
        return 0
    logs = ROOT/'output/preflight'
    logs.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ-'), dir=logs))
    print('REPORT', output/'report.json', flush=True)
    os.environ.update(PYTHONPATH=os.pathsep.join([*map(str,TASK_PATHS), os.environ.get('PYTHONPATH','')]),
                      HF_HUB_OFFLINE='1', ROS_DOMAIN_ID='197', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(output/'ros'))
    (output/'ros').mkdir()
    python = sys.executable
    probe = [python, str(Path(__file__).resolve())]
    checks = [
        ('files', ['sha256sum','--quiet','-c','SHA256SUMS'],180),
        ('dependencies', [python,'tools/run/pick_service.py','--mode','live','--check',
                          '--candidate-source','graspnet','--mobile-mesh-dir','assets/mobile_meshes'],30),
        ('calibration',[python,'tools/diagnostics/extrinsic_capture/apply_measured_calibration.py','--check-only'],60),
        ('geometry',probe+['--probe','geometry'],90),
        ('perception',[python,'tools/diagnostics/check_pick_models.py','--snapshot','examples/remote/cam',
                       '--candidate-source','graspnet','--artifacts-dir','artifacts','--output',str(output/'models'),'--device','cpu'],600),
        ('web',probe+['--probe','web','--output',str(output/'demo')],60),
        ('resident_moveit',probe+['--probe','planner','--output',str(output/'resident')],150),
        ('collision_planning',[python,'tools/run/check_moveit_planning.py','--retain-moveit-plugin',
                               '--output',str(output/'planning')],180),
    ]
    results = {}
    for name, command, timeout in checks:
        logfile = output/(name+'.log')
        try:
            if name in ('web','resident_moveit','collision_planning'):
                # Surface a sandbox/namespace socket denial before spawning DDS.
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
                    listener.bind(('127.0.0.1',0))
            code = run_step(command, logfile, timeout)
            if code:
                raise RuntimeError(f'exit code {code}')
            results[name] = {'status':'PASS'}
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            if not logfile.exists():
                logfile.write_text(str(exc)+'\n')
            results[name] = {'status':'INCOMPLETE','error':str(exc)}
        print(name, results[name]['status'], logfile, flush=True)
        if name == 'files' and results[name]['status'] != 'PASS':
            break
    passed = len(results) == len(checks) and all(v['status']=='PASS' for v in results.values())
    status = 'OFFLINE_PREFLIGHT_PASSED' if passed else 'OFFLINE_PREFLIGHT_INCOMPLETE'
    (output/'report.json').write_text(json.dumps(dict(status=status, checks=results, hardware_commands=0,
                                                       physical_pick_verified=False), indent=2)+'\n')
    print(status, flush=True)
    return 0 if passed else 2


if __name__ == '__main__':
    raise SystemExit(main())
